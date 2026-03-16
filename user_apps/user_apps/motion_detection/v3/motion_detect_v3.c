/*
 * motion_detect_v3.c - Multi-stream RTSP motion detection + YOLO + SQLite
 *
 * v3: Builds on v2 multi-stream (RKMPP HW decode, pthreads).
 *     Adds: color live preview, RKNN NPU YOLO object classification,
 *     bounding box overlay, SQLite detection logging.
 *
 * YOLO inference is gated by motion detection with a window:
 *   - IDLE: no YOLO, buffering last N NV12 frames in ring buffer
 *   - Motion triggers ACTIVE: retroactively infer on buffered frames (DB only),
 *     then run YOLO on every processed frame (display + DB)
 *   - Motion stops → COOLDOWN: continue YOLO for N more frames, then → IDLE
 *
 * Usage:
 *   ./motion_detect_v3 [--url rtsp://...] [--url rtsp://...]
 *                      [--model path.rknn] [--threshold 20]
 *                      [--framestep 5] [--trigger 1.0]
 *                      [--yolo-window 5] [--max-box-pct 80]
 *                      [--db path.db] [--sw]
 */

#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/hwcontext.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>

#include <rknn_api.h>
#include <sqlite3.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>

#include "yolo_postprocess.h"
#include "detection_db.h"

#define DISPLAY_W    960
#define DISPLAY_H    540
#define MODEL_W      640
#define MODEL_H      640
#define MAX_STREAMS  8
#define MAX_YOLO_WINDOW 16

typedef enum {
	YOLO_IDLE,       /* no motion — buffering frames, no inference */
	YOLO_ACTIVE,     /* motion ongoing — YOLO every processed frame */
	YOLO_COOLDOWN,   /* motion stopped — YOLO for N more frames */
} yolo_state_t;

/* Ring buffer entry: packed NV12 frame data (no stride padding) */
typedef struct {
	uint8_t *data;   /* packed Y + UV: size = src_w * src_h * 3/2 */
	int valid;
} nv12_ring_entry_t;

typedef struct {
	int id;
	const char *url;

	/* FFmpeg */
	AVFormatContext *fmt_ctx;
	AVCodecContext  *dec_ctx;
	AVBufferRef    *hw_device_ctx;
	struct SwsContext *sws_display;  /* NV12 → RGB for display */
	struct SwsContext *sws_model;    /* NV12 → RGB for RKNN 640x640 */
	int video_idx;
	int hw_decode;
	AVFrame *sw_frame;

	/* Motion detection */
	uint8_t *curr_gray;
	uint8_t *prev_gray;
	int      src_w, src_h;
	int      gray_size;
	int      have_prev;

	/* Color display */
	uint8_t *rgb_display;  /* RGB24 DISPLAY_W x DISPLAY_H */
	Display *x_display;
	Window   window;
	GC       gc;
	XImage  *ximage;
	uint8_t *xbuf;         /* BGRA DISPLAY_W x DISPLAY_H */

	/* RKNN */
	rknn_context rknn_ctx;
	rknn_input_output_num io_num;
	rknn_tensor_attr *output_attrs;
	int model_channel, model_width, model_height;
	uint8_t *rgb_model;    /* RGB24 MODEL_W x MODEL_H */

	/* Letterbox params */
	float lb_scale;
	int   lb_pad_w;
	int   lb_pad_h;

	/* YOLO window state machine */
	yolo_state_t yolo_state;
	int cooldown_remaining;
	nv12_ring_entry_t frame_ring[MAX_YOLO_WINDOW];
	int ring_head;          /* next write position */
	int ring_count;         /* valid entries (up to yolo_window) */
	int nv12_frame_size;    /* src_w * src_h * 3/2 */

	/* Settings (read-only after init) */
	int threshold;
	int framestep;
	int force_sw;
	float motion_trigger;
	int yolo_window;        /* frames before/after motion */
	float max_box_pct;      /* max box area as % of frame (filter) */
	const char *model_path;
	sqlite3 *db;

	/* Stats */
	int frame_count;
	int motion_pixels;
	int total_pixels;
} StreamCtx;

static volatile sig_atomic_t running = 1;

static void sigint_handler(int sig)
{
	(void)sig;
	running = 0;
}

/* ------------------------------------------------------------------ */
/* Motion detection                                                    */
/* ------------------------------------------------------------------ */

static void frame_difference(const uint8_t *curr, const uint8_t *prev,
                             int size, int threshold, int *motion_count)
{
	int count = 0;
	for (int i = 0; i < size; i++) {
		int diff = abs((int)curr[i] - (int)prev[i]);
		if (diff > threshold)
			count++;
	}
	*motion_count = count;
}

/* ------------------------------------------------------------------ */
/* NV12 frame handling                                                 */
/* ------------------------------------------------------------------ */

static AVFrame *get_nv12_frame(StreamCtx *s, AVFrame *frame)
{
	if (s->hw_decode && frame->format == AV_PIX_FMT_DRM_PRIME) {
		av_frame_unref(s->sw_frame);
		int ret = av_hwframe_transfer_data(s->sw_frame, frame, 0);
		if (ret < 0) {
			char errbuf[128];
			av_strerror(ret, errbuf, sizeof(errbuf));
			fprintf(stderr, "[Stream %d] HW transfer failed: %s\n",
			        s->id, errbuf);
			return NULL;
		}
		return s->sw_frame;
	}
	return frame;
}

static void extract_y_plane(const AVFrame *nv12, int w, int h, uint8_t *gray)
{
	const uint8_t *src = nv12->data[0];
	int stride = nv12->linesize[0];
	for (int y = 0; y < h; y++)
		memcpy(gray + y * w, src + y * stride, w);
}

static void frame_to_display_rgb(StreamCtx *s, const AVFrame *nv12)
{
	uint8_t *dst[1] = { s->rgb_display };
	int dst_stride[1] = { DISPLAY_W * 3 };
	sws_scale(s->sws_display,
	          (const uint8_t *const *)nv12->data, nv12->linesize,
	          0, s->src_h, dst, dst_stride);
}

/*
 * Convert NV12 data to model RGB with letterbox.
 * Accepts raw data pointers + linesizes (works for both AVFrame and ring buffer).
 */
static void nv12_to_model_rgb(StreamCtx *s,
                               const uint8_t *y_data, int y_stride,
                               const uint8_t *uv_data, int uv_stride)
{
	memset(s->rgb_model, 128, MODEL_W * MODEL_H * 3);

	uint8_t *dst_start = s->rgb_model + (s->lb_pad_h * MODEL_W + s->lb_pad_w) * 3;
	uint8_t *dst[1] = { dst_start };
	int dst_stride[1] = { MODEL_W * 3 };

	const uint8_t *src_data[2] = { y_data, uv_data };
	int src_linesize[2] = { y_stride, uv_stride };

	sws_scale(s->sws_model, src_data, src_linesize,
	          0, s->src_h, dst, dst_stride);
}

static void frame_to_model_rgb(StreamCtx *s, const AVFrame *nv12)
{
	nv12_to_model_rgb(s, nv12->data[0], nv12->linesize[0],
	                  nv12->data[1], nv12->linesize[1]);
}

/* ------------------------------------------------------------------ */
/* NV12 ring buffer                                                    */
/* ------------------------------------------------------------------ */

static void ring_buffer_init(StreamCtx *s)
{
	s->nv12_frame_size = s->src_w * s->src_h * 3 / 2;
	for (int i = 0; i < s->yolo_window; i++) {
		s->frame_ring[i].data = (uint8_t *)malloc(s->nv12_frame_size);
		s->frame_ring[i].valid = 0;
	}
	s->ring_head = 0;
	s->ring_count = 0;
}

static void ring_buffer_free(StreamCtx *s)
{
	for (int i = 0; i < s->yolo_window; i++)
		free(s->frame_ring[i].data);
}

/*
 * Save NV12 frame to ring buffer (compact copy, no stride padding).
 */
static void ring_buffer_push(StreamCtx *s, const AVFrame *nv12)
{
	nv12_ring_entry_t *entry = &s->frame_ring[s->ring_head];
	uint8_t *dst = entry->data;

	/* Copy Y plane row by row (strip stride padding) */
	const uint8_t *y_src = nv12->data[0];
	int y_stride = nv12->linesize[0];
	for (int y = 0; y < s->src_h; y++)
		memcpy(dst + y * s->src_w, y_src + y * y_stride, s->src_w);

	/* Copy UV plane row by row */
	const uint8_t *uv_src = nv12->data[1];
	int uv_stride = nv12->linesize[1];
	uint8_t *uv_dst = dst + s->src_w * s->src_h;
	for (int y = 0; y < s->src_h / 2; y++)
		memcpy(uv_dst + y * s->src_w, uv_src + y * uv_stride, s->src_w);

	entry->valid = 1;
	s->ring_head = (s->ring_head + 1) % s->yolo_window;
	if (s->ring_count < s->yolo_window)
		s->ring_count++;
}

/* ------------------------------------------------------------------ */
/* Display helpers                                                     */
/* ------------------------------------------------------------------ */

static void rgb_to_xbuf(const uint8_t *rgb, uint8_t *xbuf, int w, int h)
{
	for (int i = 0; i < w * h; i++) {
		xbuf[i * 4 + 0] = rgb[i * 3 + 2]; /* B */
		xbuf[i * 4 + 1] = rgb[i * 3 + 1]; /* G */
		xbuf[i * 4 + 2] = rgb[i * 3 + 0]; /* R */
		xbuf[i * 4 + 3] = 0;
	}
}

static void draw_rect(uint8_t *xbuf, int bw, int bh,
                      int x1, int y1, int x2, int y2,
                      uint8_t r, uint8_t g, uint8_t b, int thickness)
{
	for (int t = 0; t < thickness; t++) {
		for (int x = x1; x <= x2; x++) {
			int yt = y1 + t, yb = y2 - t;
			if (x >= 0 && x < bw) {
				if (yt >= 0 && yt < bh) {
					int off = (yt * bw + x) * 4;
					xbuf[off] = b; xbuf[off+1] = g; xbuf[off+2] = r;
				}
				if (yb >= 0 && yb < bh) {
					int off = (yb * bw + x) * 4;
					xbuf[off] = b; xbuf[off+1] = g; xbuf[off+2] = r;
				}
			}
		}
		for (int y = y1; y <= y2; y++) {
			int xl = x1 + t, xr = x2 - t;
			if (y >= 0 && y < bh) {
				if (xl >= 0 && xl < bw) {
					int off = (y * bw + xl) * 4;
					xbuf[off] = b; xbuf[off+1] = g; xbuf[off+2] = r;
				}
				if (xr >= 0 && xr < bw) {
					int off = (y * bw + xr) * 4;
					xbuf[off] = b; xbuf[off+1] = g; xbuf[off+2] = r;
				}
			}
		}
	}
}

static void draw_boxes(StreamCtx *s, const detection_group_t *dets)
{
	float sx = (float)DISPLAY_W / s->src_w;
	float sy = (float)DISPLAY_H / s->src_h;

	for (int i = 0; i < dets->count; i++) {
		const detection_t *d = &dets->dets[i];
		int x1 = (int)(d->box.left * sx);
		int y1 = (int)(d->box.top * sy);
		int x2 = (int)(d->box.right * sx);
		int y2 = (int)(d->box.bottom * sy);

		draw_rect(s->xbuf, DISPLAY_W, DISPLAY_H, x1, y1, x2, y2,
		          0, 255, 0, 2);

		char label[48];
		snprintf(label, sizeof(label), "%s %.0f%%",
		         d->name, d->confidence * 100.0f);
		XDrawString(s->x_display, s->window, s->gc,
		            x1, y1 > 12 ? y1 - 4 : y1 + 14,
		            label, (int)strlen(label));
	}
}

/* ------------------------------------------------------------------ */
/* Initialization                                                      */
/* ------------------------------------------------------------------ */

static int init_x11(StreamCtx *s)
{
	s->x_display = XOpenDisplay(NULL);
	if (!s->x_display) {
		fprintf(stderr, "[Stream %d] Cannot open X display\n", s->id);
		return -1;
	}

	int screen = DefaultScreen(s->x_display);
	int x_offset = s->id * DISPLAY_W;

	s->window = XCreateSimpleWindow(
		s->x_display, RootWindow(s->x_display, screen),
		x_offset, 0, DISPLAY_W, DISPLAY_H, 0,
		BlackPixel(s->x_display, screen),
		BlackPixel(s->x_display, screen));

	char title[64];
	snprintf(title, sizeof(title), "Stream %d - Motion+YOLO", s->id);
	XStoreName(s->x_display, s->window, title);
	XSelectInput(s->x_display, s->window, ExposureMask | KeyPressMask);
	XMapWindow(s->x_display, s->window);

	s->gc = XCreateGC(s->x_display, s->window, 0, NULL);
	XSetForeground(s->x_display, s->gc,
	               WhitePixel(s->x_display, screen));

	s->xbuf = (uint8_t *)calloc(DISPLAY_W * DISPLAY_H * 4, 1);
	s->ximage = XCreateImage(
		s->x_display, DefaultVisual(s->x_display, screen),
		DefaultDepth(s->x_display, screen), ZPixmap, 0,
		(char *)s->xbuf, DISPLAY_W, DISPLAY_H, 32, 0);

	s->rgb_display = (uint8_t *)malloc(DISPLAY_W * DISPLAY_H * 3);

	return 0;
}

static int init_ffmpeg(StreamCtx *s)
{
	AVDictionary *opts = NULL;
	av_dict_set(&opts, "rtsp_transport", "tcp", 0);
	av_dict_set(&opts, "stimeout", "5000000", 0);

	int ret = avformat_open_input(&s->fmt_ctx, s->url, NULL, &opts);
	av_dict_free(&opts);
	if (ret < 0) {
		char errbuf[128];
		av_strerror(ret, errbuf, sizeof(errbuf));
		fprintf(stderr, "[Stream %d] Cannot open %s: %s\n",
		        s->id, s->url, errbuf);
		return -1;
	}

	if (avformat_find_stream_info(s->fmt_ctx, NULL) < 0) {
		fprintf(stderr, "[Stream %d] Cannot find stream info\n", s->id);
		return -1;
	}

	s->video_idx = -1;
	for (unsigned i = 0; i < s->fmt_ctx->nb_streams; i++) {
		if (s->fmt_ctx->streams[i]->codecpar->codec_type ==
		    AVMEDIA_TYPE_VIDEO) {
			s->video_idx = (int)i;
			break;
		}
	}
	if (s->video_idx < 0) {
		fprintf(stderr, "[Stream %d] No video stream found\n", s->id);
		return -1;
	}

	AVCodecParameters *par = s->fmt_ctx->streams[s->video_idx]->codecpar;

	const AVCodec *codec = NULL;
	if (!s->force_sw) {
		codec = avcodec_find_decoder_by_name("hevc_rkmpp");
		if (codec) {
			printf("[Stream %d] Found RKMPP hardware decoder: %s\n",
			       s->id, codec->name);
			s->hw_decode = 1;
		} else {
			printf("[Stream %d] RKMPP not available, using software\n",
			       s->id);
		}
	}

	if (!codec) {
		codec = avcodec_find_decoder(par->codec_id);
		if (!codec) {
			fprintf(stderr, "[Stream %d] No decoder for codec %d\n",
			        s->id, par->codec_id);
			return -1;
		}
		s->hw_decode = 0;
	}

	s->dec_ctx = avcodec_alloc_context3(codec);
	avcodec_parameters_to_context(s->dec_ctx, par);

	if (s->hw_decode) {
		ret = av_hwdevice_ctx_create(&s->hw_device_ctx,
		                             AV_HWDEVICE_TYPE_RKMPP,
		                             NULL, NULL, 0);
		if (ret < 0) {
			char errbuf[128];
			av_strerror(ret, errbuf, sizeof(errbuf));
			fprintf(stderr, "[Stream %d] RKMPP device failed: %s, "
			        "falling back to software\n", s->id, errbuf);

			avcodec_free_context(&s->dec_ctx);
			codec = avcodec_find_decoder(par->codec_id);
			if (!codec) {
				fprintf(stderr, "[Stream %d] No software decoder\n",
				        s->id);
				return -1;
			}
			s->dec_ctx = avcodec_alloc_context3(codec);
			avcodec_parameters_to_context(s->dec_ctx, par);
			s->hw_decode = 0;
		} else {
			s->dec_ctx->hw_device_ctx = av_buffer_ref(s->hw_device_ctx);
		}
	}

	if (!s->hw_decode)
		s->dec_ctx->thread_count = 4;

	if (avcodec_open2(s->dec_ctx, codec, NULL) < 0) {
		fprintf(stderr, "[Stream %d] Cannot open decoder\n", s->id);
		return -1;
	}

	s->src_w = s->dec_ctx->width;
	s->src_h = s->dec_ctx->height;
	s->gray_size = s->src_w * s->src_h;
	s->total_pixels = s->gray_size;

	printf("[Stream %d] %dx%d %s (%s)\n", s->id,
	       s->src_w, s->src_h, codec->name,
	       s->hw_decode ? "hardware RKMPP" : "software");

	if (s->hw_decode)
		s->sw_frame = av_frame_alloc();

	s->curr_gray = (uint8_t *)malloc(s->gray_size);
	s->prev_gray = (uint8_t *)malloc(s->gray_size);
	memset(s->prev_gray, 0, s->gray_size);

	s->sws_display = sws_getContext(
		s->src_w, s->src_h, AV_PIX_FMT_NV12,
		DISPLAY_W, DISPLAY_H, AV_PIX_FMT_RGB24,
		SWS_FAST_BILINEAR, NULL, NULL, NULL);

	float scale_w = (float)MODEL_W / s->src_w;
	float scale_h = (float)MODEL_H / s->src_h;
	s->lb_scale = (scale_w < scale_h) ? scale_w : scale_h;
	int new_w = (int)(s->src_w * s->lb_scale);
	int new_h = (int)(s->src_h * s->lb_scale);
	s->lb_pad_w = (MODEL_W - new_w) / 2;
	s->lb_pad_h = (MODEL_H - new_h) / 2;

	s->sws_model = sws_getContext(
		s->src_w, s->src_h, AV_PIX_FMT_NV12,
		new_w, new_h, AV_PIX_FMT_RGB24,
		SWS_FAST_BILINEAR, NULL, NULL, NULL);

	s->rgb_model = (uint8_t *)malloc(MODEL_W * MODEL_H * 3);
	memset(s->rgb_model, 128, MODEL_W * MODEL_H * 3);

	/* Allocate ring buffer now that we know frame dimensions */
	ring_buffer_init(s);

	return 0;
}

static int init_rknn(StreamCtx *s)
{
	FILE *fp = fopen(s->model_path, "rb");
	if (!fp) {
		fprintf(stderr, "[Stream %d] Cannot open model: %s\n",
		        s->id, s->model_path);
		return -1;
	}
	fseek(fp, 0, SEEK_END);
	long model_size = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	unsigned char *model_data = (unsigned char *)malloc(model_size);
	if (fread(model_data, 1, model_size, fp) != (size_t)model_size) {
		fprintf(stderr, "[Stream %d] Failed to read model\n", s->id);
		free(model_data);
		fclose(fp);
		return -1;
	}
	fclose(fp);

	int ret = rknn_init(&s->rknn_ctx, model_data, model_size, 0, NULL);
	free(model_data);
	if (ret < 0) {
		fprintf(stderr, "[Stream %d] rknn_init failed: %d\n", s->id, ret);
		return -1;
	}

	ret = rknn_query(s->rknn_ctx, RKNN_QUERY_IN_OUT_NUM,
	                 &s->io_num, sizeof(s->io_num));
	if (ret < 0) {
		fprintf(stderr, "[Stream %d] rknn_query IO failed: %d\n",
		        s->id, ret);
		return -1;
	}

	rknn_tensor_attr input_attr;
	memset(&input_attr, 0, sizeof(input_attr));
	input_attr.index = 0;
	rknn_query(s->rknn_ctx, RKNN_QUERY_INPUT_ATTR,
	           &input_attr, sizeof(input_attr));

	if (input_attr.fmt == RKNN_TENSOR_NCHW) {
		s->model_channel = input_attr.dims[1];
		s->model_height  = input_attr.dims[2];
		s->model_width   = input_attr.dims[3];
	} else {
		s->model_height  = input_attr.dims[1];
		s->model_width   = input_attr.dims[2];
		s->model_channel = input_attr.dims[3];
	}

	printf("[Stream %d] RKNN model: %dx%dx%d, %d inputs, %d outputs\n",
	       s->id, s->model_width, s->model_height, s->model_channel,
	       s->io_num.n_input, s->io_num.n_output);

	s->output_attrs = (rknn_tensor_attr *)calloc(s->io_num.n_output,
	                                              sizeof(rknn_tensor_attr));
	for (uint32_t i = 0; i < s->io_num.n_output; i++) {
		s->output_attrs[i].index = i;
		rknn_query(s->rknn_ctx, RKNN_QUERY_OUTPUT_ATTR,
		           &s->output_attrs[i], sizeof(rknn_tensor_attr));
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* RKNN inference                                                      */
/* ------------------------------------------------------------------ */

static int run_inference(StreamCtx *s, detection_group_t *dets)
{
	rknn_input inputs[1];
	memset(inputs, 0, sizeof(inputs));
	inputs[0].index = 0;
	inputs[0].type = RKNN_TENSOR_UINT8;
	inputs[0].size = s->model_width * s->model_height * s->model_channel;
	inputs[0].fmt = RKNN_TENSOR_NHWC;
	inputs[0].pass_through = 0;
	inputs[0].buf = s->rgb_model;

	int ret = rknn_inputs_set(s->rknn_ctx, s->io_num.n_input, inputs);
	if (ret < 0)
		return -1;

	ret = rknn_run(s->rknn_ctx, NULL);
	if (ret < 0)
		return -1;

	rknn_output outputs[s->io_num.n_output];
	memset(outputs, 0, sizeof(outputs));
	for (uint32_t i = 0; i < s->io_num.n_output; i++) {
		outputs[i].index = i;
		outputs[i].want_float = 0;
	}

	ret = rknn_outputs_get(s->rknn_ctx, s->io_num.n_output, outputs, NULL);
	if (ret < 0)
		return -1;

	int32_t qnt_zps[3];
	float qnt_scales[3];
	for (int i = 0; i < 3 && i < (int)s->io_num.n_output; i++) {
		qnt_zps[i] = s->output_attrs[i].zp;
		qnt_scales[i] = s->output_attrs[i].scale;
	}

	yolo_postprocess(
		(int8_t *)outputs[0].buf,
		(int8_t *)outputs[1].buf,
		(int8_t *)outputs[2].buf,
		s->model_height, s->model_width,
		BOX_THRESH, NMS_THRESH,
		s->lb_pad_w, s->lb_pad_h,
		s->lb_scale, s->lb_scale,
		qnt_zps, qnt_scales, dets);

	rknn_outputs_release(s->rknn_ctx, s->io_num.n_output, outputs);

	return 0;
}

/* ------------------------------------------------------------------ */
/* Detection filtering                                                 */
/* ------------------------------------------------------------------ */

/*
 * Remove detections where bounding box area exceeds max_pct% of frame area.
 * Filters in-place, returns new count.
 */
static int filter_large_boxes(detection_group_t *dets, int src_w, int src_h,
                              float max_pct)
{
	int frame_area = src_w * src_h;
	float max_area = max_pct / 100.0f * frame_area;
	int kept = 0;

	for (int i = 0; i < dets->count; i++) {
		const detection_t *d = &dets->dets[i];
		int bw = d->box.right - d->box.left;
		int bh = d->box.bottom - d->box.top;
		int area = bw * bh;

		if ((float)area <= max_area) {
			if (kept != i)
				dets->dets[kept] = dets->dets[i];
			kept++;
		}
	}
	dets->count = kept;
	return kept;
}

/*
 * Run inference + filter on model RGB buffer already prepared.
 * Logs detections to DB. Returns detection count.
 */
static int infer_and_log(StreamCtx *s, float motion_pct)
{
	detection_group_t dets = {0};
	if (run_inference(s, &dets) < 0)
		return 0;

	filter_large_boxes(&dets, s->src_w, s->src_h, s->max_box_pct);

	for (int i = 0; i < dets.count; i++) {
		const detection_t *d = &dets.dets[i];
		db_insert(s->db, s->id, d->name, d->confidence,
		          d->box.left, d->box.top,
		          d->box.right, d->box.bottom, motion_pct);
	}

	return dets.count;
}

/* ------------------------------------------------------------------ */
/* Frame processing with YOLO window state machine                     */
/* ------------------------------------------------------------------ */

static void process_frame(StreamCtx *s, AVFrame *frame)
{
	s->frame_count++;
	if (s->frame_count % s->framestep != 0)
		return;

	AVFrame *nv12 = get_nv12_frame(s, frame);
	if (!nv12)
		return;

	/* Extract Y plane for motion detection */
	extract_y_plane(nv12, s->src_w, s->src_h, s->curr_gray);

	/* Motion detection */
	float motion_pct = 0.0f;
	int motion_detected = 0;
	if (s->have_prev) {
		frame_difference(s->curr_gray, s->prev_gray,
		                 s->gray_size, s->threshold,
		                 &s->motion_pixels);
		motion_pct = 100.0f * s->motion_pixels / s->total_pixels;
		motion_detected = (motion_pct >= s->motion_trigger);
	}

	/* Color preview: NV12 → RGB → BGRA → XImage */
	frame_to_display_rgb(s, nv12);
	rgb_to_xbuf(s->rgb_display, s->xbuf, DISPLAY_W, DISPLAY_H);

	/* YOLO state machine */
	int run_yolo = 0;
	int dets_count = 0;
	const char *state_tag = "IDLE";

	switch (s->yolo_state) {
	case YOLO_IDLE:
		/* Buffer frame for retroactive inference */
		ring_buffer_push(s, nv12);

		if (motion_detected) {
			/* Process buffered frames retroactively (DB only) */
			int retroactive = 0;
			for (int j = 0; j < s->ring_count; j++) {
				int idx = (s->ring_head - s->ring_count + j +
				           s->yolo_window) % s->yolo_window;
				nv12_ring_entry_t *e = &s->frame_ring[idx];
				if (!e->valid)
					continue;

				/* Convert buffered NV12 → model RGB */
				uint8_t *y_ptr = e->data;
				uint8_t *uv_ptr = e->data + s->src_w * s->src_h;
				nv12_to_model_rgb(s, y_ptr, s->src_w,
				                  uv_ptr, s->src_w);
				retroactive += infer_and_log(s, motion_pct);
			}
			if (retroactive > 0)
				printf("[Stream %d] Retroactive: %d detections "
				       "from %d buffered frames\n",
				       s->id, retroactive, s->ring_count);

			s->yolo_state = YOLO_ACTIVE;
			run_yolo = 1;
		}
		break;

	case YOLO_ACTIVE:
		state_tag = "ACTIVE";
		run_yolo = 1;
		if (!motion_detected) {
			s->cooldown_remaining = s->yolo_window;
			s->yolo_state = YOLO_COOLDOWN;
		}
		break;

	case YOLO_COOLDOWN:
		state_tag = "COOL";
		run_yolo = 1;
		s->cooldown_remaining--;
		if (motion_detected) {
			s->yolo_state = YOLO_ACTIVE;
			state_tag = "ACTIVE";
		} else if (s->cooldown_remaining <= 0) {
			s->yolo_state = YOLO_IDLE;
			s->ring_count = 0;  /* reset ring buffer */
		}
		break;
	}

	/* Run YOLO on current frame if state machine says so */
	if (run_yolo) {
		frame_to_model_rgb(s, nv12);

		detection_group_t dets = {0};
		if (run_inference(s, &dets) == 0) {
			filter_large_boxes(&dets, s->src_w, s->src_h,
			                   s->max_box_pct);
			dets_count = dets.count;

			if (dets.count > 0) {
				draw_boxes(s, &dets);
				for (int i = 0; i < dets.count; i++) {
					const detection_t *d = &dets.dets[i];
					db_insert(s->db, s->id, d->name,
					          d->confidence,
					          d->box.left, d->box.top,
					          d->box.right, d->box.bottom,
					          motion_pct);
				}
			}
		}
	}

	/* Display */
	XPutImage(s->x_display, s->window, s->gc, s->ximage,
	          0, 0, 0, 0, DISPLAY_W, DISPLAY_H);
	XFlush(s->x_display);

	/* Stats */
	printf("\033[%d;0H[Stream %d] Frame %5d | Motion: %5.1f%% | "
	       "Dets: %d | %-6s",
	       s->id + 1, s->id, s->frame_count, motion_pct,
	       dets_count, state_tag);
	fflush(stdout);

	/* Swap gray buffers */
	uint8_t *tmp = s->prev_gray;
	s->prev_gray = s->curr_gray;
	s->curr_gray = tmp;
	s->have_prev = 1;
}

/* ------------------------------------------------------------------ */
/* Cleanup                                                             */
/* ------------------------------------------------------------------ */

static void cleanup_stream(StreamCtx *s)
{
	if (s->ximage) {
		s->ximage->data = NULL;
		XDestroyImage(s->ximage);
	}
	free(s->xbuf);
	free(s->rgb_display);
	if (s->gc)
		XFreeGC(s->x_display, s->gc);
	if (s->window)
		XDestroyWindow(s->x_display, s->window);
	if (s->x_display)
		XCloseDisplay(s->x_display);

	if (s->rknn_ctx)
		rknn_destroy(s->rknn_ctx);
	free(s->output_attrs);
	free(s->rgb_model);

	ring_buffer_free(s);

	free(s->curr_gray);
	free(s->prev_gray);
	av_frame_free(&s->sw_frame);
	sws_freeContext(s->sws_display);
	sws_freeContext(s->sws_model);
	av_buffer_unref(&s->hw_device_ctx);
	avcodec_free_context(&s->dec_ctx);
	avformat_close_input(&s->fmt_ctx);
}

/* ------------------------------------------------------------------ */
/* Stream thread                                                       */
/* ------------------------------------------------------------------ */

static void *stream_thread(void *arg)
{
	StreamCtx *s = (StreamCtx *)arg;

	printf("[Stream %d] Starting: %s\n", s->id, s->url);

	if (init_x11(s) < 0) {
		fprintf(stderr, "[Stream %d] X11 init failed\n", s->id);
		return NULL;
	}

	if (init_ffmpeg(s) < 0) {
		fprintf(stderr, "[Stream %d] FFmpeg init failed\n", s->id);
		cleanup_stream(s);
		return NULL;
	}

	if (init_rknn(s) < 0) {
		fprintf(stderr, "[Stream %d] RKNN init failed\n", s->id);
		cleanup_stream(s);
		return NULL;
	}

	AVPacket *pkt = av_packet_alloc();
	AVFrame *frame = av_frame_alloc();

	while (running) {
		int ret = av_read_frame(s->fmt_ctx, pkt);
		if (ret < 0) {
			if (ret == AVERROR_EOF)
				printf("[Stream %d] End of stream\n", s->id);
			else
				fprintf(stderr, "[Stream %d] Read error\n", s->id);
			break;
		}

		if (pkt->stream_index != s->video_idx) {
			av_packet_unref(pkt);
			continue;
		}

		ret = avcodec_send_packet(s->dec_ctx, pkt);
		av_packet_unref(pkt);
		if (ret < 0)
			continue;

		while (ret >= 0) {
			ret = avcodec_receive_frame(s->dec_ctx, frame);
			if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
				break;
			if (ret < 0)
				break;

			process_frame(s, frame);
			av_frame_unref(frame);
		}

		while (XPending(s->x_display)) {
			XEvent ev;
			XNextEvent(s->x_display, &ev);
			if (ev.type == KeyPress)
				running = 0;
		}
	}

	av_packet_free(&pkt);
	av_frame_free(&frame);
	cleanup_stream(s);

	printf("[Stream %d] Stopped\n", s->id);
	return NULL;
}

/* ------------------------------------------------------------------ */
/* Main                                                                */
/* ------------------------------------------------------------------ */

int main(int argc, char *argv[])
{
	const char *urls[MAX_STREAMS];
	int num_streams = 0;
	int threshold = 20;
	int framestep = 5;
	int force_sw = 0;
	float motion_trigger = 1.0f;
	int yolo_window = 5;
	float max_box_pct = 80.0f;
	const char *model_path = "./model/yolov5s-640-640.rknn";
	const char *db_path = "detections.db";

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "--url") == 0 && i + 1 < argc) {
			if (num_streams >= MAX_STREAMS) {
				fprintf(stderr, "Max %d streams\n", MAX_STREAMS);
				return 1;
			}
			urls[num_streams++] = argv[++i];
		} else if (strcmp(argv[i], "--threshold") == 0 && i + 1 < argc) {
			threshold = atoi(argv[++i]);
		} else if (strcmp(argv[i], "--framestep") == 0 && i + 1 < argc) {
			framestep = atoi(argv[++i]);
		} else if (strcmp(argv[i], "--trigger") == 0 && i + 1 < argc) {
			motion_trigger = (float)atof(argv[++i]);
		} else if (strcmp(argv[i], "--yolo-window") == 0 && i + 1 < argc) {
			yolo_window = atoi(argv[++i]);
			if (yolo_window > MAX_YOLO_WINDOW)
				yolo_window = MAX_YOLO_WINDOW;
			if (yolo_window < 1)
				yolo_window = 1;
		} else if (strcmp(argv[i], "--max-box-pct") == 0 && i + 1 < argc) {
			max_box_pct = (float)atof(argv[++i]);
		} else if (strcmp(argv[i], "--model") == 0 && i + 1 < argc) {
			model_path = argv[++i];
		} else if (strcmp(argv[i], "--db") == 0 && i + 1 < argc) {
			db_path = argv[++i];
		} else if (strcmp(argv[i], "--sw") == 0) {
			force_sw = 1;
		} else if (strcmp(argv[i], "-h") == 0 ||
		           strcmp(argv[i], "--help") == 0) {
			printf("Usage: %s [OPTIONS]\n\n"
			       "  --url URL          RTSP stream (repeat for multiple, max %d)\n"
			       "  --model PATH       RKNN model file (default: %s)\n"
			       "  --db PATH          SQLite database (default: %s)\n"
			       "  --threshold N      Motion pixel diff threshold (default: 20)\n"
			       "  --framestep N      Process every Nth frame (default: 5)\n"
			       "  --trigger PCT      Motion %% to trigger YOLO (default: 1.0)\n"
			       "  --yolo-window N    Frames before/after motion for YOLO (default: 5)\n"
			       "  --max-box-pct PCT  Max bounding box area %% of frame (default: 80)\n"
			       "  --sw               Force software decode\n"
			       "  -h, --help         Show this help\n\n"
			       "Default: video1 + video2 from 192.168.12.142\n",
			       argv[0], MAX_STREAMS, model_path, db_path);
			return 0;
		}
	}

	if (num_streams == 0) {
		urls[0] = "rtsp://192.168.12.142:554/video1";
		urls[1] = "rtsp://192.168.12.142:554/video2";
		num_streams = 2;
	}

	signal(SIGINT, sigint_handler);

	sqlite3 *db = db_open(db_path);
	if (!db) {
		fprintf(stderr, "Failed to open database: %s\n", db_path);
		return 1;
	}

	printf("Motion Detection v3 - RKMPP + YOLO + SQLite\n");
	printf("  Streams:     %d\n", num_streams);
	for (int i = 0; i < num_streams; i++)
		printf("    [%d] %s\n", i, urls[i]);
	printf("  Model:       %s\n", model_path);
	printf("  Database:    %s\n", db_path);
	printf("  Threshold:   %d\n", threshold);
	printf("  Framestep:   %d (%.0f ms @ 50fps)\n",
	       framestep, framestep * 20.0);
	printf("  Trigger:     %.1f%% motion\n", motion_trigger);
	printf("  YOLO window: %d frames before/after\n", yolo_window);
	printf("  Max box:     %.0f%% of frame area\n", max_box_pct);
	if (force_sw)
		printf("  Decode:      software (forced)\n");
	printf("  Ctrl+C to quit\n\n");

	StreamCtx streams[MAX_STREAMS];
	memset(streams, 0, sizeof(streams));
	pthread_t threads[MAX_STREAMS];

	for (int i = 0; i < num_streams; i++) {
		streams[i].id = i;
		streams[i].url = urls[i];
		streams[i].threshold = threshold;
		streams[i].framestep = framestep;
		streams[i].force_sw = force_sw;
		streams[i].motion_trigger = motion_trigger;
		streams[i].yolo_window = yolo_window;
		streams[i].max_box_pct = max_box_pct;
		streams[i].model_path = model_path;
		streams[i].db = db;
		streams[i].yolo_state = YOLO_IDLE;
	}

	for (int i = 0; i < num_streams; i++)
		printf("\n");

	for (int i = 0; i < num_streams; i++) {
		int err = pthread_create(&threads[i], NULL, stream_thread,
		                         &streams[i]);
		if (err) {
			fprintf(stderr, "Failed to create thread %d: %s\n",
			        i, strerror(err));
			running = 0;
			break;
		}
	}

	for (int i = 0; i < num_streams; i++)
		pthread_join(threads[i], NULL);

	printf("\033[%d;0H\nAll streams stopped.\n", num_streams + 1);

	db_close(db);
	printf("Database closed: %s\n", db_path);

	return 0;
}
