/*
 * motion_detect_v4.c - Motion Detection Debug GUI
 *
 * GTK3 GUI for real-time tuning of motion detection parameters.
 * Dual view: color preview + grid-based motion debug visualization.
 * Interactive settings panel for parameter adjustment.
 *
 * Features:
 *   - 3x3 box blur before differencing (suppresses HEVC compression noise)
 *   - Background model (EMA) replaces frame-to-frame differencing
 *   - Grid-based motion: divides frame into cells, tracks per-cell motion %
 *   - Max cell % used as trigger (sensitive to localized motion)
 *   - Interactive GTK3 GUI with real-time parameter tuning
 *
 * Usage:
 *   ./motion_detect_v4 [--url rtsp://...] [--sw] [--grid N]
 */

#include <gtk/gtk.h>
#include <gdk/gdk.h>

#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/hwcontext.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <time.h>

#define MAX_GRID 16

static volatile sig_atomic_t sigint_flag = 0;
static void sigint_handler(int sig) { (void)sig; sigint_flag = 1; }

typedef struct {
	/* Stream */
	const char *url;
	int src_w, src_h, gray_size;
	int disp_w, disp_h, disp_size; /* 16:9 display resolution */

	/* FFmpeg */
	AVFormatContext *fmt_ctx;
	AVCodecContext  *dec_ctx;
	AVBufferRef    *hw_device_ctx;
	struct SwsContext *sws_color;
	int video_idx, hw_decode, force_sw;
	AVFrame *sw_frame;
	const char *decoder_name;

	/* Motion detection */
	uint8_t *curr_gray;
	uint8_t *blurred;
	float   *bg_model;
	int      bg_initialized;

	/* Shared display buffers (mutex protected) */
	pthread_mutex_t buf_mutex;
	uint8_t *color_rgb;     /* worker writes */
	uint8_t *motion_rgb;    /* worker writes */
	float   shared_motion_pct;
	int     shared_motion_pixels;
	float   shared_grid_pcts[MAX_GRID * MAX_GRID];
	int     shared_grid_cols, shared_grid_rows;
	float   shared_grid_max_pct;
	int     shared_grid_max_col, shared_grid_max_row;
	float   shared_fps;
	int     buf_updated;

	/* Draw-local (main thread only) */
	uint8_t *draw_color;
	uint8_t *draw_motion;
	float   draw_grid_pcts[MAX_GRID * MAX_GRID];
	int     draw_grid_cols, draw_grid_rows;
	float   draw_grid_max_pct;
	float   draw_motion_pct;
	int     draw_motion_pixels;

	/* Worker-local (worker thread only, avoids per-frame malloc) */
	uint8_t *work_color;
	uint8_t *work_motion;

	/* Settings (written by GTK callbacks, read by worker) */
	int threshold;
	float alpha;
	float trigger;
	int framestep;
	int grid_size;  /* number of columns; rows computed for ~square cells */

	/* Timing */
	int frame_count;
	double last_time;
	float fps;

	/* GTK widgets */
	GtkWidget *win;
	GtkWidget *color_area;
	GtkWidget *motion_area;
	GtkWidget *lbl_motion;
	GtkWidget *lbl_pixels;
	GtkWidget *lbl_max_cell;
	GtkWidget *lbl_max_loc;
	GtkWidget *lbl_state;
	GtkWidget *lbl_fps;
	GtkWidget *lbl_url;
	GtkWidget *lbl_res;
	GtkWidget *lbl_dec;

	/* Thread control */
	pthread_t worker;
	volatile int running;
	int worker_started;
} AppCtx;

/* ================================================================ */
/* Motion detection: box blur + background model + grid              */
/* ================================================================ */

static void box_blur_3x3(const uint8_t *src, uint8_t *dst, int w, int h)
{
	/* Interior: 3x3 average */
	for (int y = 1; y < h - 1; y++) {
		for (int x = 1; x < w - 1; x++) {
			int sum = 0;
			for (int dy = -1; dy <= 1; dy++)
				for (int dx = -1; dx <= 1; dx++)
					sum += src[(y + dy) * w + (x + dx)];
			dst[y * w + x] = (uint8_t)(sum / 9);
		}
	}
	/* Edges: copy */
	for (int x = 0; x < w; x++) {
		dst[x] = src[x];
		dst[(h - 1) * w + x] = src[(h - 1) * w + x];
	}
	for (int y = 1; y < h - 1; y++) {
		dst[y * w] = src[y * w];
		dst[y * w + w - 1] = src[y * w + w - 1];
	}
}

/*
 * Grid-based motion detection.
 * Divides the frame into grid_cols x grid_rows cells and computes
 * per-cell motion percentage. Returns the max cell percentage.
 * Also accumulates total_count for overall frame motion stats.
 */
static float bg_diff_grid(const uint8_t *blurred, const float *bg,
                          int w, int h, int threshold,
                          int grid_cols, int grid_rows,
                          float *cell_pcts, int *total_count,
                          int *max_col, int *max_row)
{
	float max_pct = 0;
	*max_col = 0;
	*max_row = 0;
	*total_count = 0;

	for (int gr = 0; gr < grid_rows; gr++) {
		int y0 = gr * h / grid_rows;
		int y1 = (gr + 1) * h / grid_rows;
		for (int gc = 0; gc < grid_cols; gc++) {
			int x0 = gc * w / grid_cols;
			int x1 = (gc + 1) * w / grid_cols;

			int count = 0;
			int total = 0;
			for (int y = y0; y < y1; y++) {
				for (int x = x0; x < x1; x++) {
					int i = y * w + x;
					int diff = abs((int)blurred[i] -
					               (int)(bg[i] + 0.5f));
					if (diff > threshold)
						count++;
					total++;
				}
			}

			float pct = total > 0 ? 100.0f * count / total : 0;
			cell_pcts[gr * grid_cols + gc] = pct;
			*total_count += count;

			if (pct > max_pct) {
				max_pct = pct;
				*max_col = gc;
				*max_row = gr;
			}
		}
	}

	return max_pct;
}

/*
 * Produce motion debug visualization at display resolution (subsampled).
 *   Green = above threshold (detected motion)
 *   Gray  = below threshold (amplified diff shows noise/sub-threshold)
 */
static void bg_diff_visualize(const uint8_t *blurred, const float *bg,
                              int src_w, int src_h, int threshold,
                              int disp_w, int disp_h, uint8_t *vis_rgb)
{
	for (int dy = 0; dy < disp_h; dy++) {
		int sy = dy * src_h / disp_h;
		for (int dx = 0; dx < disp_w; dx++) {
			int sx = dx * src_w / disp_w;
			int si = sy * src_w + sx;
			int di = (dy * disp_w + dx) * 3;
			int diff = abs((int)blurred[si] - (int)(bg[si] + 0.5f));
			if (diff > threshold) {
				vis_rgb[di + 0] = 0;
				vis_rgb[di + 1] = (uint8_t)(128 + (diff > 127 ? 127 : diff));
				vis_rgb[di + 2] = 0;
			} else {
				uint8_t v = (uint8_t)(diff * 12 > 255 ? 255 : diff * 12);
				vis_rgb[di + 0] = v;
				vis_rgb[di + 1] = v;
				vis_rgb[di + 2] = v;
			}
		}
	}
}

static void bg_update(float *bg, const uint8_t *blurred, int size, float alpha)
{
	float inv = 1.0f - alpha;
	for (int i = 0; i < size; i++)
		bg[i] = alpha * (float)blurred[i] + inv * bg[i];
}

static void bg_init(float *bg, const uint8_t *blurred, int size)
{
	for (int i = 0; i < size; i++)
		bg[i] = (float)blurred[i];
}

/* ================================================================ */
/* NV12 helpers                                                      */
/* ================================================================ */

static AVFrame *get_nv12(AppCtx *c, AVFrame *frame)
{
	if (c->hw_decode && frame->format == AV_PIX_FMT_DRM_PRIME) {
		av_frame_unref(c->sw_frame);
		if (av_hwframe_transfer_data(c->sw_frame, frame, 0) < 0)
			return NULL;
		return c->sw_frame;
	}
	return frame;
}

static void extract_y(const AVFrame *nv12, int w, int h, uint8_t *gray)
{
	const uint8_t *s = nv12->data[0];
	int stride = nv12->linesize[0];
	for (int y = 0; y < h; y++)
		memcpy(gray + y * w, s + y * stride, w);
}

static void nv12_to_color_rgb(AppCtx *c, const AVFrame *nv12, uint8_t *rgb)
{
	uint8_t *dst[1] = { rgb };
	int stride[1] = { c->disp_w * 3 };
	sws_scale(c->sws_color,
	          (const uint8_t *const *)nv12->data, nv12->linesize,
	          0, c->src_h, dst, stride);
}

/* ================================================================ */
/* FFmpeg init                                                       */
/* ================================================================ */

static int init_ffmpeg(AppCtx *c)
{
	AVDictionary *opts = NULL;
	av_dict_set(&opts, "rtsp_transport", "tcp", 0);
	av_dict_set(&opts, "stimeout", "5000000", 0);

	int ret = avformat_open_input(&c->fmt_ctx, c->url, NULL, &opts);
	av_dict_free(&opts);
	if (ret < 0) {
		fprintf(stderr, "Cannot open %s\n", c->url);
		return -1;
	}

	if (avformat_find_stream_info(c->fmt_ctx, NULL) < 0)
		return -1;

	c->video_idx = -1;
	for (unsigned i = 0; i < c->fmt_ctx->nb_streams; i++) {
		if (c->fmt_ctx->streams[i]->codecpar->codec_type ==
		    AVMEDIA_TYPE_VIDEO) {
			c->video_idx = (int)i;
			break;
		}
	}
	if (c->video_idx < 0)
		return -1;

	AVCodecParameters *par = c->fmt_ctx->streams[c->video_idx]->codecpar;
	const AVCodec *codec = NULL;

	if (!c->force_sw) {
		codec = avcodec_find_decoder_by_name("hevc_rkmpp");
		if (codec)
			c->hw_decode = 1;
	}
	if (!codec) {
		codec = avcodec_find_decoder(par->codec_id);
		if (!codec)
			return -1;
		c->hw_decode = 0;
	}

	c->dec_ctx = avcodec_alloc_context3(codec);
	avcodec_parameters_to_context(c->dec_ctx, par);

	if (c->hw_decode) {
		ret = av_hwdevice_ctx_create(&c->hw_device_ctx,
		                             AV_HWDEVICE_TYPE_RKMPP,
		                             NULL, NULL, 0);
		if (ret < 0) {
			avcodec_free_context(&c->dec_ctx);
			codec = avcodec_find_decoder(par->codec_id);
			if (!codec)
				return -1;
			c->dec_ctx = avcodec_alloc_context3(codec);
			avcodec_parameters_to_context(c->dec_ctx, par);
			c->hw_decode = 0;
		} else {
			c->dec_ctx->hw_device_ctx = av_buffer_ref(c->hw_device_ctx);
		}
	}

	if (!c->hw_decode)
		c->dec_ctx->thread_count = 4;

	if (avcodec_open2(c->dec_ctx, codec, NULL) < 0)
		return -1;

	c->src_w = c->dec_ctx->width;
	c->src_h = c->dec_ctx->height;
	c->gray_size = c->src_w * c->src_h;

	/* 16:9 display resolution derived from source width */
	c->disp_w = c->src_w;
	c->disp_h = c->src_w * 9 / 16;
	c->disp_size = c->disp_w * c->disp_h * 3;
	c->decoder_name = codec->name;

	printf("Stream: %dx%d -> display %dx%d (16:9) %s (%s)\n",
	       c->src_w, c->src_h, c->disp_w, c->disp_h,
	       codec->name, c->hw_decode ? "hardware" : "software");

	if (c->hw_decode)
		c->sw_frame = av_frame_alloc();

	/* Motion buffers (full source resolution for accuracy) */
	c->curr_gray = (uint8_t *)malloc(c->gray_size);
	c->blurred   = (uint8_t *)malloc(c->gray_size);
	c->bg_model  = (float *)malloc(c->gray_size * sizeof(float));

	/* sws: NV12 source -> RGB24 at 16:9 display resolution */
	c->sws_color = sws_getContext(
		c->src_w, c->src_h, AV_PIX_FMT_NV12,
		c->disp_w, c->disp_h, AV_PIX_FMT_RGB24,
		SWS_FAST_BILINEAR, NULL, NULL, NULL);

	/* Shared + draw + worker buffers (all at display resolution) */
	c->color_rgb     = (uint8_t *)calloc(c->disp_size, 1);
	c->motion_rgb    = (uint8_t *)calloc(c->disp_size, 1);
	c->draw_color    = (uint8_t *)malloc(c->disp_size);
	c->draw_motion   = (uint8_t *)malloc(c->disp_size);
	c->work_color    = (uint8_t *)malloc(c->disp_size);
	c->work_motion   = (uint8_t *)malloc(c->disp_size);

	return 0;
}

/* ================================================================ */
/* GTK idle callback for thread-safe redraw                          */
/* ================================================================ */

static gboolean redraw_idle(gpointer data)
{
	AppCtx *c = (AppCtx *)data;
	if (c->color_area)
		gtk_widget_queue_draw(c->color_area);
	if (c->motion_area)
		gtk_widget_queue_draw(c->motion_area);
	return G_SOURCE_REMOVE;
}

/* ================================================================ */
/* Frame processing (runs in worker thread)                          */
/* ================================================================ */

static void process_frame(AppCtx *c, AVFrame *frame)
{
	c->frame_count++;
	if (c->frame_count % c->framestep != 0)
		return;

	/* FPS tracking */
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	double now = ts.tv_sec + ts.tv_nsec * 1e-9;
	if (c->last_time > 0) {
		double dt = now - c->last_time;
		c->fps = c->fps * 0.9f + (float)(1.0 / dt) * 0.1f;
	}
	c->last_time = now;

	AVFrame *nv12 = get_nv12(c, frame);
	if (!nv12)
		return;

	/* Y plane -> blur -> bg model diff */
	extract_y(nv12, c->src_w, c->src_h, c->curr_gray);
	box_blur_3x3(c->curr_gray, c->blurred, c->src_w, c->src_h);

	/* Local copies of settings */
	int thresh = c->threshold;
	float alpha = c->alpha;
	int gs = c->grid_size;
	if (gs < 1) gs = 1;
	if (gs > MAX_GRID) gs = MAX_GRID;

	/* Compute grid dimensions: cols = grid_size, rows for ~square cells on 16:9 display */
	int gcols = gs;
	int grows = (gs * c->disp_h + c->disp_w / 2) / c->disp_w;
	if (grows < 1) grows = 1;
	if (grows > MAX_GRID) grows = MAX_GRID;

	int motion_pixels = 0;
	float motion_pct = 0.0f;
	float grid_max_pct = 0.0f;
	int grid_max_col = 0, grid_max_row = 0;
	float cell_pcts[MAX_GRID * MAX_GRID];
	memset(cell_pcts, 0, sizeof(cell_pcts));

	if (!c->bg_initialized) {
		bg_init(c->bg_model, c->blurred, c->gray_size);
		c->bg_initialized = 1;
		memset(c->work_motion, 0, c->disp_size);
	} else {
		grid_max_pct = bg_diff_grid(c->blurred, c->bg_model,
		                            c->src_w, c->src_h, thresh,
		                            gcols, grows,
		                            cell_pcts, &motion_pixels,
		                            &grid_max_col, &grid_max_row);
		motion_pct = 100.0f * motion_pixels / c->gray_size;
		bg_diff_visualize(c->blurred, c->bg_model,
		                  c->src_w, c->src_h, thresh,
		                  c->disp_w, c->disp_h, c->work_motion);
	}

	bg_update(c->bg_model, c->blurred, c->gray_size, alpha);

	/* Color preview */
	nv12_to_color_rgb(c, nv12, c->work_color);

	/* Copy to shared buffers */
	pthread_mutex_lock(&c->buf_mutex);
	memcpy(c->color_rgb, c->work_color, c->disp_size);
	memcpy(c->motion_rgb, c->work_motion, c->disp_size);
	c->shared_motion_pct = motion_pct;
	c->shared_motion_pixels = motion_pixels;
	c->shared_grid_cols = gcols;
	c->shared_grid_rows = grows;
	c->shared_grid_max_pct = grid_max_pct;
	c->shared_grid_max_col = grid_max_col;
	c->shared_grid_max_row = grid_max_row;
	memcpy(c->shared_grid_pcts, cell_pcts,
	       gcols * grows * sizeof(float));
	c->shared_fps = c->fps;
	c->buf_updated = 1;
	pthread_mutex_unlock(&c->buf_mutex);

	/* Schedule GTK redraw */
	g_idle_add(redraw_idle, c);
}

/* ================================================================ */
/* Worker thread                                                     */
/* ================================================================ */

static void *worker_thread(void *arg)
{
	AppCtx *c = (AppCtx *)arg;
	AVPacket *pkt = av_packet_alloc();
	AVFrame *frame = av_frame_alloc();
	int read_errors = 0;

	printf("Worker thread started\n");
	fflush(stdout);

	while (c->running) {
		int ret = av_read_frame(c->fmt_ctx, pkt);
		if (ret < 0) {
			read_errors++;
			if (ret == AVERROR_EOF || read_errors > 50) {
				fprintf(stderr, "Stream ended (errors=%d)\n",
				        read_errors);
				break;
			}
			usleep(10000); /* 10ms backoff on transient error */
			continue;
		}
		read_errors = 0;

		if (pkt->stream_index != c->video_idx) {
			av_packet_unref(pkt);
			continue;
		}

		ret = avcodec_send_packet(c->dec_ctx, pkt);
		av_packet_unref(pkt);
		if (ret < 0)
			continue;

		while (ret >= 0 && c->running) {
			ret = avcodec_receive_frame(c->dec_ctx, frame);
			if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
				break;
			if (ret < 0)
				break;
			process_frame(c, frame);
			av_frame_unref(frame);
		}
	}

	av_packet_free(&pkt);
	av_frame_free(&frame);
	printf("Worker thread exiting (running=%d)\n", c->running);
	fflush(stdout);
	c->running = 0;
	return NULL;
}

/* ================================================================ */
/* GTK draw callbacks                                                */
/* ================================================================ */

static gboolean on_draw_color(GtkWidget *widget, cairo_t *cr, gpointer data)
{
	AppCtx *c = (AppCtx *)data;
	int aw = gtk_widget_get_allocated_width(widget);
	int ah = gtk_widget_get_allocated_height(widget);

	/* Black background */
	cairo_set_source_rgb(cr, 0, 0, 0);
	cairo_paint(cr);

	if (c->disp_w == 0)
		return FALSE;

	/* Copy shared data */
	int gc, gr;
	float grid_max;
	pthread_mutex_lock(&c->buf_mutex);
	memcpy(c->draw_color, c->color_rgb, c->disp_size);
	gc = c->shared_grid_cols;
	gr = c->shared_grid_rows;
	grid_max = c->shared_grid_max_pct;
	memcpy(c->draw_grid_pcts, c->shared_grid_pcts,
	       gc * gr * sizeof(float));
	pthread_mutex_unlock(&c->buf_mutex);

	/* Create pixbuf from display-resolution RGB */
	GdkPixbuf *pb = gdk_pixbuf_new_from_data(
		c->draw_color, GDK_COLORSPACE_RGB, FALSE, 8,
		c->disp_w, c->disp_h, c->disp_w * 3, NULL, NULL);

	/* Stretch to fill entire widget area */
	double scx = (double)aw / c->disp_w;
	double scy = (double)ah / c->disp_h;

	cairo_save(cr);
	cairo_scale(cr, scx, scy);
	gdk_cairo_set_source_pixbuf(cr, pb, 0, 0);
	cairo_pattern_set_filter(cairo_get_source(cr), CAIRO_FILTER_BILINEAR);
	cairo_paint(cr);
	cairo_restore(cr);
	g_object_unref(pb);

	/* Highlight active grid cells */
	float trig = c->trigger;
	if (gc > 0 && gr > 0) {
		double cw = (double)aw / gc;
		double ch = (double)ah / gr;

		for (int row = 0; row < gr; row++) {
			for (int col = 0; col < gc; col++) {
				float pct = c->draw_grid_pcts[row * gc + col];
				if (pct >= trig) {
					/* Red tint proportional to motion */
					double a = 0.1 + 0.3 * (pct / 100.0);
					if (a > 0.4) a = 0.4;
					cairo_set_source_rgba(cr, 1, 0, 0, a);
					cairo_rectangle(cr, col * cw, row * ch,
					                cw, ch);
					cairo_fill(cr);

					/* Border */
					cairo_set_source_rgba(cr, 1, 0.3, 0, 0.7);
					cairo_set_line_width(cr, 2.0);
					cairo_rectangle(cr, col * cw, row * ch,
					                cw, ch);
					cairo_stroke(cr);
				}
			}
		}
	}

	/* Overlay info text at bottom */
	cairo_set_source_rgba(cr, 0, 0, 0, 0.5);
	cairo_rectangle(cr, 0, ah - 24, aw, 24);
	cairo_fill(cr);

	cairo_set_source_rgb(cr, 1, 1, 1);
	cairo_set_font_size(cr, 13);
	char info[128];
	snprintf(info, sizeof(info), "Color Preview | Max Cell: %.1f%%",
	         grid_max);
	cairo_move_to(cr, 8, ah - 7);
	cairo_show_text(cr, info);

	return FALSE;
}

static gboolean on_draw_motion(GtkWidget *widget, cairo_t *cr, gpointer data)
{
	AppCtx *c = (AppCtx *)data;
	int aw = gtk_widget_get_allocated_width(widget);
	int ah = gtk_widget_get_allocated_height(widget);

	cairo_set_source_rgb(cr, 0, 0, 0);
	cairo_paint(cr);

	if (c->disp_w == 0)
		return FALSE;

	/* Copy shared data */
	float motion_pct;
	int motion_pixels;
	int gc, gr;
	float grid_pcts[MAX_GRID * MAX_GRID];

	pthread_mutex_lock(&c->buf_mutex);
	memcpy(c->draw_motion, c->motion_rgb, c->disp_size);
	motion_pct = c->shared_motion_pct;
	motion_pixels = c->shared_motion_pixels;
	gc = c->shared_grid_cols;
	gr = c->shared_grid_rows;
	memcpy(grid_pcts, c->shared_grid_pcts, gc * gr * sizeof(float));
	pthread_mutex_unlock(&c->buf_mutex);

	GdkPixbuf *pb = gdk_pixbuf_new_from_data(
		c->draw_motion, GDK_COLORSPACE_RGB, FALSE, 8,
		c->disp_w, c->disp_h, c->disp_w * 3, NULL, NULL);

	/* Stretch to fill entire widget area */
	cairo_save(cr);
	cairo_scale(cr, (double)aw / c->disp_w, (double)ah / c->disp_h);
	gdk_cairo_set_source_pixbuf(cr, pb, 0, 0);
	cairo_pattern_set_filter(cairo_get_source(cr), CAIRO_FILTER_BILINEAR);
	cairo_paint(cr);
	cairo_restore(cr);
	g_object_unref(pb);

	/* Grid overlay */
	float trig = c->trigger;
	if (gc > 0 && gr > 0) {
		double cw = (double)aw / gc;
		double ch = (double)ah / gr;

		/* Grid lines */
		cairo_set_source_rgba(cr, 1, 1, 1, 0.3);
		cairo_set_line_width(cr, 1.0);
		for (int i = 1; i < gc; i++) {
			cairo_move_to(cr, i * cw, 0);
			cairo_line_to(cr, i * cw, ah);
		}
		for (int i = 1; i < gr; i++) {
			cairo_move_to(cr, 0, i * ch);
			cairo_line_to(cr, aw, i * ch);
		}
		cairo_stroke(cr);

		/* Per-cell percentage text */
		double font_sz = cw / 5.0;
		if (font_sz < 8) font_sz = 8;
		if (font_sz > 14) font_sz = 14;
		cairo_set_font_size(cr, font_sz);

		for (int row = 0; row < gr; row++) {
			for (int col = 0; col < gc; col++) {
				float pct = grid_pcts[row * gc + col];
				double cx = col * cw + cw / 2;
				double cy = row * ch + ch / 2;

				char buf[8];
				snprintf(buf, sizeof(buf), "%.0f", pct);
				cairo_text_extents_t ext;
				cairo_text_extents(cr, buf, &ext);

				/* Text background for readability */
				cairo_set_source_rgba(cr, 0, 0, 0, 0.4);
				cairo_rectangle(cr,
					cx - ext.width / 2 - 2,
					cy - ext.height / 2 - 2,
					ext.width + 4, ext.height + 4);
				cairo_fill(cr);

				/* Color: yellow if triggered, dim white otherwise */
				if (pct >= trig)
					cairo_set_source_rgb(cr, 1, 1, 0);
				else if (pct > 0)
					cairo_set_source_rgba(cr, 1, 1, 1, 0.6);
				else
					cairo_set_source_rgba(cr, 1, 1, 1, 0.2);

				cairo_move_to(cr, cx - ext.width / 2,
				              cy + ext.height / 2);
				cairo_show_text(cr, buf);
			}
		}
	}

	/* Overlay info */
	cairo_set_source_rgba(cr, 0, 0, 0, 0.5);
	cairo_rectangle(cr, 0, ah - 24, aw, 24);
	cairo_fill(cr);

	cairo_set_source_rgb(cr, 1, 1, 1);
	cairo_set_font_size(cr, 13);
	char info[128];
	snprintf(info, sizeof(info),
	         "Motion Debug | Overall: %.1f%% (%d px) | Grid: %dx%d",
	         motion_pct, motion_pixels, gc, gr);
	cairo_move_to(cr, 8, ah - 7);
	cairo_show_text(cr, info);

	return FALSE;
}

/* ================================================================ */
/* GTK settings callbacks                                            */
/* ================================================================ */

static void on_threshold_changed(GtkSpinButton *spin, gpointer data)
{
	((AppCtx *)data)->threshold = gtk_spin_button_get_value_as_int(spin);
}

static void on_alpha_changed(GtkSpinButton *spin, gpointer data)
{
	((AppCtx *)data)->alpha = (float)gtk_spin_button_get_value(spin);
}

static void on_trigger_changed(GtkSpinButton *spin, gpointer data)
{
	((AppCtx *)data)->trigger = (float)gtk_spin_button_get_value(spin);
}

static void on_framestep_changed(GtkSpinButton *spin, gpointer data)
{
	((AppCtx *)data)->framestep = gtk_spin_button_get_value_as_int(spin);
}

static void on_grid_changed(GtkSpinButton *spin, gpointer data)
{
	((AppCtx *)data)->grid_size = gtk_spin_button_get_value_as_int(spin);
}

/* ================================================================ */
/* Stats update (periodic timer)                                     */
/* ================================================================ */

static gboolean update_stats(gpointer data)
{
	AppCtx *c = (AppCtx *)data;

	if (!c->running)
		return G_SOURCE_REMOVE;

	/* Check SIGINT */
	if (sigint_flag) {
		c->running = 0;
		gtk_main_quit();
		return G_SOURCE_REMOVE;
	}

	/* Read stats under mutex */
	pthread_mutex_lock(&c->buf_mutex);
	float motion = c->shared_motion_pct;
	int pixels = c->shared_motion_pixels;
	float grid_max = c->shared_grid_max_pct;
	int max_col = c->shared_grid_max_col;
	int max_row = c->shared_grid_max_row;
	float fps = c->shared_fps;
	pthread_mutex_unlock(&c->buf_mutex);

	char buf[64];

	snprintf(buf, sizeof(buf), "%.1f%%", motion);
	gtk_label_set_text(GTK_LABEL(c->lbl_motion), buf);

	snprintf(buf, sizeof(buf), "%d / %d", pixels, c->gray_size);
	gtk_label_set_text(GTK_LABEL(c->lbl_pixels), buf);

	snprintf(buf, sizeof(buf), "%.1f%%", grid_max);
	gtk_label_set_text(GTK_LABEL(c->lbl_max_cell), buf);

	snprintf(buf, sizeof(buf), "(%d, %d)", max_col, max_row);
	gtk_label_set_text(GTK_LABEL(c->lbl_max_loc), buf);

	const char *state = (grid_max >= c->trigger) ? "MOTION" : "IDLE";
	gtk_label_set_text(GTK_LABEL(c->lbl_state), state);

	snprintf(buf, sizeof(buf), "%.1f", fps);
	gtk_label_set_text(GTK_LABEL(c->lbl_fps), buf);

	return G_SOURCE_CONTINUE;
}

/* ================================================================ */
/* Build GTK GUI                                                     */
/* ================================================================ */

static GtkWidget *make_label(const char *text, int bold)
{
	GtkWidget *lbl = gtk_label_new(NULL);
	if (bold) {
		char markup[128];
		snprintf(markup, sizeof(markup), "<b>%s</b>", text);
		gtk_label_set_markup(GTK_LABEL(lbl), markup);
	} else {
		gtk_label_set_text(GTK_LABEL(lbl), text);
	}
	gtk_label_set_xalign(GTK_LABEL(lbl), 0.0);
	return lbl;
}

static GtkWidget *make_spin_int(int val, int lo, int hi, int step,
                                GCallback cb, AppCtx *c)
{
	GtkAdjustment *adj = gtk_adjustment_new(val, lo, hi, step, step, 0);
	GtkWidget *spin = gtk_spin_button_new(adj, 1.0, 0);
	g_signal_connect(spin, "value-changed", cb, c);
	return spin;
}

static GtkWidget *make_spin_float(double val, double lo, double hi,
                                  double step, int digits,
                                  GCallback cb, AppCtx *c)
{
	GtkAdjustment *adj = gtk_adjustment_new(val, lo, hi, step, step, 0);
	GtkWidget *spin = gtk_spin_button_new(adj, 1.0, digits);
	g_signal_connect(spin, "value-changed", cb, c);
	return spin;
}

static void build_gui(AppCtx *c)
{
	c->win = gtk_window_new(GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title(GTK_WINDOW(c->win),
	                     "Motion Detect v4 - Grid Debug");
	gtk_window_set_default_size(GTK_WINDOW(c->win), 1280, 780);
	g_signal_connect(c->win, "destroy", G_CALLBACK(gtk_main_quit), NULL);

	/* Main horizontal box: video | settings */
	GtkWidget *hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 4);
	gtk_container_add(GTK_CONTAINER(c->win), hbox);

	/* Left: stacked video views */
	GtkWidget *vbox_video = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
	gtk_box_pack_start(GTK_BOX(hbox), vbox_video, TRUE, TRUE, 0);

	/* Color preview */
	GtkWidget *frame_color = gtk_frame_new("Color Preview");
	gtk_box_pack_start(GTK_BOX(vbox_video), frame_color, TRUE, TRUE, 0);
	c->color_area = gtk_drawing_area_new();
	gtk_widget_set_size_request(c->color_area, 640, 360);
	gtk_widget_set_hexpand(c->color_area, TRUE);
	gtk_widget_set_vexpand(c->color_area, TRUE);
	gtk_container_add(GTK_CONTAINER(frame_color), c->color_area);
	g_signal_connect(c->color_area, "draw",
	                 G_CALLBACK(on_draw_color), c);

	/* Motion debug view */
	GtkWidget *frame_motion = gtk_frame_new(
		"Motion Debug (Grid + BG Model)");
	gtk_box_pack_start(GTK_BOX(vbox_video), frame_motion, TRUE, TRUE, 0);
	c->motion_area = gtk_drawing_area_new();
	gtk_widget_set_size_request(c->motion_area, 640, 360);
	gtk_widget_set_hexpand(c->motion_area, TRUE);
	gtk_widget_set_vexpand(c->motion_area, TRUE);
	gtk_container_add(GTK_CONTAINER(frame_motion), c->motion_area);
	g_signal_connect(c->motion_area, "draw",
	                 G_CALLBACK(on_draw_motion), c);

	/* Right: settings panel */
	GtkWidget *vbox_settings = gtk_box_new(GTK_ORIENTATION_VERTICAL, 8);
	gtk_widget_set_size_request(vbox_settings, 280, -1);
	gtk_box_pack_start(GTK_BOX(hbox), vbox_settings, FALSE, FALSE, 4);

	/* --- Settings frame --- */
	GtkWidget *frame_set = gtk_frame_new("Settings");
	gtk_box_pack_start(GTK_BOX(vbox_settings), frame_set,
	                   FALSE, FALSE, 0);

	GtkWidget *grid_set = gtk_grid_new();
	gtk_grid_set_row_spacing(GTK_GRID(grid_set), 6);
	gtk_grid_set_column_spacing(GTK_GRID(grid_set), 8);
	gtk_widget_set_margin_start(grid_set, 8);
	gtk_widget_set_margin_end(grid_set, 8);
	gtk_widget_set_margin_top(grid_set, 4);
	gtk_widget_set_margin_bottom(grid_set, 4);
	gtk_container_add(GTK_CONTAINER(frame_set), grid_set);

	int row = 0;
	gtk_grid_attach(GTK_GRID(grid_set), make_label("Threshold", 0),
	                0, row, 1, 1);
	gtk_grid_attach(GTK_GRID(grid_set),
	                make_spin_int(c->threshold, 1, 100, 1,
	                              G_CALLBACK(on_threshold_changed), c),
	                1, row++, 1, 1);

	gtk_grid_attach(GTK_GRID(grid_set), make_label("BG Alpha", 0),
	                0, row, 1, 1);
	gtk_grid_attach(GTK_GRID(grid_set),
	                make_spin_float(c->alpha, 0.001, 0.5, 0.005, 3,
	                                G_CALLBACK(on_alpha_changed), c),
	                1, row++, 1, 1);

	gtk_grid_attach(GTK_GRID(grid_set), make_label("Trigger %", 0),
	                0, row, 1, 1);
	gtk_grid_attach(GTK_GRID(grid_set),
	                make_spin_float(c->trigger, 0.0, 50.0, 0.5, 1,
	                                G_CALLBACK(on_trigger_changed), c),
	                1, row++, 1, 1);

	gtk_grid_attach(GTK_GRID(grid_set), make_label("Framestep", 0),
	                0, row, 1, 1);
	gtk_grid_attach(GTK_GRID(grid_set),
	                make_spin_int(c->framestep, 1, 20, 1,
	                              G_CALLBACK(on_framestep_changed), c),
	                1, row++, 1, 1);

	gtk_grid_attach(GTK_GRID(grid_set), make_label("Grid Size", 0),
	                0, row, 1, 1);
	gtk_grid_attach(GTK_GRID(grid_set),
	                make_spin_int(c->grid_size, 1, MAX_GRID, 1,
	                              G_CALLBACK(on_grid_changed), c),
	                1, row++, 1, 1);

	/* --- Stats frame --- */
	GtkWidget *frame_stats = gtk_frame_new("Stats");
	gtk_box_pack_start(GTK_BOX(vbox_settings), frame_stats,
	                   FALSE, FALSE, 0);

	GtkWidget *grid_stats = gtk_grid_new();
	gtk_grid_set_row_spacing(GTK_GRID(grid_stats), 4);
	gtk_grid_set_column_spacing(GTK_GRID(grid_stats), 8);
	gtk_widget_set_margin_start(grid_stats, 8);
	gtk_widget_set_margin_end(grid_stats, 8);
	gtk_widget_set_margin_top(grid_stats, 4);
	gtk_widget_set_margin_bottom(grid_stats, 4);
	gtk_container_add(GTK_CONTAINER(frame_stats), grid_stats);

	row = 0;
	gtk_grid_attach(GTK_GRID(grid_stats), make_label("Overall:", 1),
	                0, row, 1, 1);
	c->lbl_motion = make_label("0.0%", 0);
	gtk_grid_attach(GTK_GRID(grid_stats), c->lbl_motion, 1, row++, 1, 1);

	gtk_grid_attach(GTK_GRID(grid_stats), make_label("Pixels:", 1),
	                0, row, 1, 1);
	c->lbl_pixels = make_label("0 / 0", 0);
	gtk_grid_attach(GTK_GRID(grid_stats), c->lbl_pixels, 1, row++, 1, 1);

	gtk_grid_attach(GTK_GRID(grid_stats), make_label("Max Cell:", 1),
	                0, row, 1, 1);
	c->lbl_max_cell = make_label("0.0%", 0);
	gtk_grid_attach(GTK_GRID(grid_stats), c->lbl_max_cell,
	                1, row++, 1, 1);

	gtk_grid_attach(GTK_GRID(grid_stats), make_label("Max Loc:", 1),
	                0, row, 1, 1);
	c->lbl_max_loc = make_label("(0, 0)", 0);
	gtk_grid_attach(GTK_GRID(grid_stats), c->lbl_max_loc,
	                1, row++, 1, 1);

	gtk_grid_attach(GTK_GRID(grid_stats), make_label("State:", 1),
	                0, row, 1, 1);
	c->lbl_state = make_label("IDLE", 0);
	gtk_grid_attach(GTK_GRID(grid_stats), c->lbl_state, 1, row++, 1, 1);

	gtk_grid_attach(GTK_GRID(grid_stats), make_label("FPS:", 1),
	                0, row, 1, 1);
	c->lbl_fps = make_label("0.0", 0);
	gtk_grid_attach(GTK_GRID(grid_stats), c->lbl_fps, 1, row++, 1, 1);

	/* --- Stream info frame --- */
	GtkWidget *frame_info = gtk_frame_new("Stream");
	gtk_box_pack_start(GTK_BOX(vbox_settings), frame_info,
	                   FALSE, FALSE, 0);

	GtkWidget *grid_info = gtk_grid_new();
	gtk_grid_set_row_spacing(GTK_GRID(grid_info), 4);
	gtk_grid_set_column_spacing(GTK_GRID(grid_info), 8);
	gtk_widget_set_margin_start(grid_info, 8);
	gtk_widget_set_margin_end(grid_info, 8);
	gtk_widget_set_margin_top(grid_info, 4);
	gtk_widget_set_margin_bottom(grid_info, 4);
	gtk_container_add(GTK_CONTAINER(frame_info), grid_info);

	row = 0;
	gtk_grid_attach(GTK_GRID(grid_info), make_label("URL:", 1),
	                0, row, 1, 1);
	c->lbl_url = make_label(c->url, 0);
	gtk_label_set_ellipsize(GTK_LABEL(c->lbl_url), PANGO_ELLIPSIZE_END);
	gtk_label_set_max_width_chars(GTK_LABEL(c->lbl_url), 24);
	gtk_grid_attach(GTK_GRID(grid_info), c->lbl_url, 1, row++, 1, 1);

	char res_buf[64];
	snprintf(res_buf, sizeof(res_buf), "%dx%d -> %dx%d",
	         c->src_w, c->src_h, c->disp_w, c->disp_h);
	gtk_grid_attach(GTK_GRID(grid_info), make_label("Resolution:", 1),
	                0, row, 1, 1);
	c->lbl_res = make_label(res_buf, 0);
	gtk_grid_attach(GTK_GRID(grid_info), c->lbl_res, 1, row++, 1, 1);

	gtk_grid_attach(GTK_GRID(grid_info), make_label("Decoder:", 1),
	                0, row, 1, 1);
	c->lbl_dec = make_label(
		c->decoder_name ? c->decoder_name : "N/A", 0);
	gtk_grid_attach(GTK_GRID(grid_info), c->lbl_dec, 1, row++, 1, 1);

	/* --- Legend frame --- */
	GtkWidget *frame_legend = gtk_frame_new("Motion Debug Legend");
	gtk_box_pack_start(GTK_BOX(vbox_settings), frame_legend,
	                   FALSE, FALSE, 0);

	GtkWidget *legend_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 2);
	gtk_widget_set_margin_start(legend_box, 8);
	gtk_widget_set_margin_end(legend_box, 8);
	gtk_widget_set_margin_top(legend_box, 4);
	gtk_widget_set_margin_bottom(legend_box, 4);
	gtk_container_add(GTK_CONTAINER(frame_legend), legend_box);

	gtk_box_pack_start(GTK_BOX(legend_box),
		make_label("Green = above threshold", 0), FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(legend_box),
		make_label("Gray = below threshold (12x amp)", 0),
		FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(legend_box),
		make_label("Yellow % = cell above trigger", 0),
		FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(legend_box),
		make_label("Red overlay = motion on color", 0),
		FALSE, FALSE, 0);

	/* Stats timer: update labels every 200ms */
	g_timeout_add(200, update_stats, c);

	gtk_widget_show_all(c->win);
}

/* ================================================================ */
/* Cleanup                                                           */
/* ================================================================ */

static void cleanup(AppCtx *c)
{
	free(c->curr_gray);
	free(c->blurred);
	free(c->bg_model);

	free(c->color_rgb);
	free(c->motion_rgb);
	free(c->draw_color);
	free(c->draw_motion);
	free(c->work_color);
	free(c->work_motion);

	av_frame_free(&c->sw_frame);
	sws_freeContext(c->sws_color);
	av_buffer_unref(&c->hw_device_ctx);
	avcodec_free_context(&c->dec_ctx);
	avformat_close_input(&c->fmt_ctx);
}

/* ================================================================ */
/* Main                                                              */
/* ================================================================ */

int main(int argc, char *argv[])
{
	gtk_init(&argc, &argv);

	AppCtx ctx;
	memset(&ctx, 0, sizeof(ctx));
	pthread_mutex_init(&ctx.buf_mutex, NULL);

	/* Defaults */
	ctx.url        = "rtsp://192.168.12.142:554/video1";
	ctx.threshold  = 12;
	ctx.alpha      = 0.05f;
	ctx.trigger    = 5.0f;   /* per-cell trigger - higher than whole-frame */
	ctx.framestep  = 5;
	ctx.grid_size  = 8;
	ctx.running    = 1;

	/* Parse args (after gtk_init consumed GTK args) */
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "--url") == 0 && i + 1 < argc)
			ctx.url = argv[++i];
		else if (strcmp(argv[i], "--threshold") == 0 && i + 1 < argc)
			ctx.threshold = atoi(argv[++i]);
		else if (strcmp(argv[i], "--alpha") == 0 && i + 1 < argc)
			ctx.alpha = (float)atof(argv[++i]);
		else if (strcmp(argv[i], "--trigger") == 0 && i + 1 < argc)
			ctx.trigger = (float)atof(argv[++i]);
		else if (strcmp(argv[i], "--framestep") == 0 && i + 1 < argc)
			ctx.framestep = atoi(argv[++i]);
		else if (strcmp(argv[i], "--grid") == 0 && i + 1 < argc)
			ctx.grid_size = atoi(argv[++i]);
		else if (strcmp(argv[i], "--sw") == 0)
			ctx.force_sw = 1;
		else if (strcmp(argv[i], "-h") == 0 ||
		         strcmp(argv[i], "--help") == 0) {
			printf("Usage: %s [OPTIONS]\n\n"
			       "  --url URL          RTSP stream URL\n"
			       "  --threshold N      Pixel diff threshold (default: 12)\n"
			       "  --alpha F          BG model learning rate (default: 0.05)\n"
			       "  --trigger PCT      Max cell %% to trigger (default: 5.0)\n"
			       "  --framestep N      Process every Nth frame (default: 5)\n"
			       "  --grid N           Grid columns (default: 8, max: %d)\n"
			       "  --sw               Force software decode\n",
			       argv[0], MAX_GRID);
			return 0;
		}
	}

	signal(SIGINT, sigint_handler);

	printf("Connecting to %s ...\n", ctx.url);
	if (init_ffmpeg(&ctx) < 0) {
		fprintf(stderr, "FFmpeg init failed\n");
		return 1;
	}

	build_gui(&ctx);

	/* Start worker thread */
	if (pthread_create(&ctx.worker, NULL, worker_thread, &ctx) == 0) {
		ctx.worker_started = 1;
	} else {
		fprintf(stderr, "Failed to create worker thread\n");
		ctx.running = 0;
	}

	/* GTK main loop */
	gtk_main();

	/* Shutdown */
	ctx.running = 0;
	if (ctx.worker_started)
		pthread_join(ctx.worker, NULL);

	cleanup(&ctx);
	pthread_mutex_destroy(&ctx.buf_mutex);
	printf("Done.\n");
	return 0;
}
