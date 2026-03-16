/*
 * core/clip.c - Event-triggered clip creation
 *
 * Ring buffer stores last ~25s of raw HEVC packets per camera.
 * Clip tracker state machine captures event windows (15s pre + 15s post).
 * Transcoder thread decodes, scales to 960x540, HW encodes H.265 to MP4.
 */
#include "clip.h"
#include "database.h"
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

static double clip_get_time(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

/* ---- Ring Buffer ---- */

void ring_buf_init(RingBuf *rb)
{
    memset(rb, 0, sizeof(*rb));
}

void ring_buf_free(RingBuf *rb)
{
    for (int i = 0; i < RING_BUF_MAX_PKTS; i++)
        free(rb->pkts[i].data);
    free(rb->extradata);
    memset(rb, 0, sizeof(*rb));
}

void ring_buf_set_stream_params(RingBuf *rb, const AVCodecParameters *par,
                                AVRational time_base)
{
    free(rb->extradata);
    rb->extradata = NULL;
    rb->extradata_size = 0;

    if (par->extradata_size > 0 && par->extradata) {
        rb->extradata = malloc(par->extradata_size);
        if (rb->extradata) {
            memcpy(rb->extradata, par->extradata, par->extradata_size);
            rb->extradata_size = par->extradata_size;
        }
    }
    rb->src_w = par->width;
    rb->src_h = par->height;
    rb->time_base = time_base;
}

void ring_buf_push(RingBuf *rb, const AVPacket *pkt, double wall_time)
{
    RingPacket *slot = &rb->pkts[rb->head];

    /* Evict old packet data */
    free(slot->data);
    slot->data = NULL;

    /* Deep copy packet */
    slot->data = malloc(pkt->size);
    if (!slot->data) return;
    memcpy(slot->data, pkt->data, pkt->size);
    slot->size = pkt->size;
    slot->pts = pkt->pts;
    slot->dts = pkt->dts;
    slot->flags = pkt->flags;
    slot->wall_time = wall_time;

    rb->head = (rb->head + 1) % RING_BUF_MAX_PKTS;
    if (rb->count < RING_BUF_MAX_PKTS)
        rb->count++;
}

/* Find nearest keyframe index at or before target wall_time.
 * Returns index into ring buffer or -1. */
static int ring_buf_find_keyframe(RingBuf *rb, double target)
{
    int tail = (rb->head - rb->count + RING_BUF_MAX_PKTS) % RING_BUF_MAX_PKTS;
    int best = -1;

    for (int i = 0; i < rb->count; i++) {
        int idx = (tail + i) % RING_BUF_MAX_PKTS;
        RingPacket *rp = &rb->pkts[idx];
        if (rp->wall_time <= target && (rp->flags & AV_PKT_FLAG_KEY))
            best = i; /* offset from tail */
        if (rp->wall_time > target)
            break;
    }
    return best;
}

/* Extract packets for [start_wall, end_wall] from ring buffer.
 * Seeks back to nearest keyframe before start_wall.
 * Returns heap-allocated deep-copied array, sets *out_count.
 * Caller must free each pkt data and the array. */
static RingPacket *ring_buf_extract(RingBuf *rb, double start_wall,
                                    double end_wall, int *out_count,
                                    int *out_snapshot_offset)
{
    int tail = (rb->head - rb->count + RING_BUF_MAX_PKTS) % RING_BUF_MAX_PKTS;

    /* Find keyframe at or before start */
    int kf = ring_buf_find_keyframe(rb, start_wall);
    if (kf < 0) {
        /* No keyframe before start — try first keyframe in buffer */
        for (int i = 0; i < rb->count; i++) {
            int idx = (tail + i) % RING_BUF_MAX_PKTS;
            if (rb->pkts[idx].flags & AV_PKT_FLAG_KEY) {
                kf = i;
                break;
            }
        }
    }
    if (kf < 0) {
        *out_count = 0;
        return NULL;
    }

    /* Count packets from kf to end_wall */
    int n = 0;
    int snapshot_off = -1;
    for (int i = kf; i < rb->count; i++) {
        int idx = (tail + i) % RING_BUF_MAX_PKTS;
        if (rb->pkts[idx].wall_time > end_wall)
            break;
        /* Track the first packet at or after start_wall (for snapshot) */
        if (snapshot_off < 0 && rb->pkts[idx].wall_time >= start_wall + CLIP_PRE_SEC)
            snapshot_off = n;
        n++;
    }

    if (n == 0) {
        *out_count = 0;
        return NULL;
    }

    /* Deep copy */
    RingPacket *arr = calloc(n, sizeof(RingPacket));
    if (!arr) {
        *out_count = 0;
        return NULL;
    }

    for (int i = 0; i < n; i++) {
        int idx = (tail + kf + i) % RING_BUF_MAX_PKTS;
        RingPacket *src = &rb->pkts[idx];
        arr[i].data = malloc(src->size);
        if (arr[i].data) {
            memcpy(arr[i].data, src->data, src->size);
            arr[i].size = src->size;
        }
        arr[i].pts = src->pts;
        arr[i].dts = src->dts;
        arr[i].flags = src->flags;
        arr[i].wall_time = src->wall_time;
    }

    *out_count = n;
    if (out_snapshot_offset)
        *out_snapshot_offset = (snapshot_off >= 0) ? snapshot_off : 0;
    return arr;
}

/* ---- Clip Tracker ---- */

void clip_tracker_init(ClipTracker *ct)
{
    memset(ct, 0, sizeof(*ct));
    ct->state = CLIP_IDLE;
}

void clip_tracker_event(ClipTracker *ct, RingBuf *rb, int event_id,
                        double wall_time)
{
    (void)rb;

    if (ct->state == CLIP_IDLE) {
        ct->state = CLIP_RECORDING;
        ct->first_event_time = wall_time;
        ct->first_event_id = event_id;
        ct->num_event_ids = 0;
        printf("[clip] Event window started (event %d)\n", event_id);
    }

    ct->last_event_time = wall_time;
    if (ct->num_event_ids < CLIP_MAX_EVENTS)
        ct->event_ids[ct->num_event_ids++] = event_id;
}

int clip_tracker_tick(ClipTracker *ct, RingBuf *rb, double wall_time,
                      ClipJob **out_job, int camera_id,
                      const char *camera_name, sqlite3 *db)
{
    *out_job = NULL;

    if (ct->state != CLIP_RECORDING)
        return 0;

    /* Check if post-event window has expired */
    if (wall_time < ct->last_event_time + CLIP_POST_SEC)
        return 0;

    /* Window closed — build clip job */
    double clip_start = ct->first_event_time - CLIP_PRE_SEC;
    double clip_end = ct->last_event_time + CLIP_POST_SEC;

    int num_pkts = 0;
    int snapshot_off = 0;
    RingPacket *pkts = ring_buf_extract(rb, clip_start, clip_end,
                                        &num_pkts, &snapshot_off);
    if (!pkts || num_pkts == 0) {
        fprintf(stderr, "[clip] No packets for clip window (cam %d)\n",
                camera_id);
        ct->state = CLIP_IDLE;
        ct->num_event_ids = 0;
        return 0;
    }

    ClipJob *job = calloc(1, sizeof(ClipJob));
    if (!job) {
        for (int i = 0; i < num_pkts; i++) free(pkts[i].data);
        free(pkts);
        ct->state = CLIP_IDLE;
        ct->num_event_ids = 0;
        return 0;
    }

    job->pkts = pkts;
    job->num_pkts = num_pkts;
    job->src_w = rb->src_w;
    job->src_h = rb->src_h;
    job->time_base = rb->time_base;

    if (rb->extradata && rb->extradata_size > 0) {
        job->extradata = malloc(rb->extradata_size);
        if (job->extradata) {
            memcpy(job->extradata, rb->extradata, rb->extradata_size);
            job->extradata_size = rb->extradata_size;
        }
    }

    job->camera_id = camera_id;
    snprintf(job->camera_name, sizeof(job->camera_name), "%s",
             camera_name ? camera_name : "unknown");
    job->trigger_time = time(NULL) - (time_t)(wall_time - ct->first_event_time);
    job->snapshot_idx = snapshot_off;
    job->db = db;

    memcpy(job->event_ids, ct->event_ids,
           ct->num_event_ids * sizeof(int));
    job->num_event_ids = ct->num_event_ids;

    printf("[clip] Window closed: cam %d, %d events, %d pkts, %.1fs\n",
           camera_id, ct->num_event_ids, num_pkts,
           clip_end - clip_start);

    /* Reset tracker */
    ct->state = CLIP_IDLE;
    ct->num_event_ids = 0;

    *out_job = job;
    return 1;
}

/* ---- Job cleanup ---- */

static void clip_job_free(ClipJob *job)
{
    if (!job) return;
    for (int i = 0; i < job->num_pkts; i++)
        free(job->pkts[i].data);
    free(job->pkts);
    free(job->extradata);
    free(job);
}

/* ---- JPEG snapshot ---- */

static int save_jpeg(const char *path, AVFrame *frame, int w, int h)
{
    const AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_MJPEG);
    if (!codec) return -1;

    AVCodecContext *ctx = avcodec_alloc_context3(codec);
    if (!ctx) return -1;
    ctx->width = w;
    ctx->height = h;
    ctx->time_base = (AVRational){1, 1};
    ctx->pix_fmt = AV_PIX_FMT_YUVJ420P;

    /* Convert input to YUVJ420P if needed */
    struct SwsContext *sws = NULL;
    AVFrame *jpeg_frame = NULL;

    if (frame->format != AV_PIX_FMT_YUVJ420P) {
        sws = sws_getContext(w, h, frame->format,
                             w, h, AV_PIX_FMT_YUVJ420P,
                             SWS_FAST_BILINEAR, NULL, NULL, NULL);
        if (!sws) {
            avcodec_free_context(&ctx);
            return -1;
        }
        jpeg_frame = av_frame_alloc();
        jpeg_frame->format = AV_PIX_FMT_YUVJ420P;
        jpeg_frame->width = w;
        jpeg_frame->height = h;
        av_frame_get_buffer(jpeg_frame, 0);
        sws_scale(sws, (const uint8_t *const *)frame->data,
                  frame->linesize, 0, h,
                  jpeg_frame->data, jpeg_frame->linesize);
    } else {
        jpeg_frame = frame;
    }

    if (avcodec_open2(ctx, codec, NULL) < 0) {
        avcodec_free_context(&ctx);
        if (sws) sws_freeContext(sws);
        if (jpeg_frame != frame) av_frame_free(&jpeg_frame);
        return -1;
    }

    jpeg_frame->pts = 0;
    avcodec_send_frame(ctx, jpeg_frame);
    avcodec_send_frame(ctx, NULL);

    AVPacket *pkt = av_packet_alloc();
    int ret = -1;
    if (avcodec_receive_packet(ctx, pkt) == 0) {
        FILE *f = fopen(path, "wb");
        if (f) {
            fwrite(pkt->data, 1, pkt->size, f);
            fclose(f);
            ret = 0;
        }
    }

    av_packet_free(&pkt);
    avcodec_free_context(&ctx);
    if (sws) sws_freeContext(sws);
    if (jpeg_frame != frame) av_frame_free(&jpeg_frame);
    return ret;
}

/* ---- Transcoder pipeline ---- */

static int transcode_clip(ClipJob *job)
{
    int out_w = 960, out_h = 540;

    /* Create output directory */
    char dir[512];
    snprintf(dir, sizeof(dir), "events/cam_%d", job->camera_id);
    mkdir("events", 0755);
    mkdir(dir, 0755);

    /* Build output filenames */
    struct tm tm;
    localtime_r(&job->trigger_time, &tm);
    char mp4_path[512], jpg_path[512];
    snprintf(mp4_path, sizeof(mp4_path),
             "%s/clip_%04d%02d%02d_%02d%02d%02d.mp4", dir,
             tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
             tm.tm_hour, tm.tm_min, tm.tm_sec);
    snprintf(jpg_path, sizeof(jpg_path),
             "%s/clip_%04d%02d%02d_%02d%02d%02d.jpg", dir,
             tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
             tm.tm_hour, tm.tm_min, tm.tm_sec);

    /* Start/end timestamps for DB */
    char start_ts[32], end_ts[32];
    {
        time_t st = job->trigger_time - (time_t)CLIP_PRE_SEC;
        struct tm stm;
        localtime_r(&st, &stm);
        strftime(start_ts, sizeof(start_ts), "%Y-%m-%dT%H:%M:%S", &stm);

        double dur_est = 0;
        if (job->num_pkts > 1)
            dur_est = job->pkts[job->num_pkts - 1].wall_time -
                      job->pkts[0].wall_time;
        time_t et = st + (time_t)dur_est;
        struct tm etm;
        localtime_r(&et, &etm);
        strftime(end_ts, sizeof(end_ts), "%Y-%m-%dT%H:%M:%S", &etm);
    }

    /* Init SW HEVC decoder */
    const AVCodec *dec = avcodec_find_decoder(AV_CODEC_ID_HEVC);
    if (!dec) {
        fprintf(stderr, "[clip] No HEVC decoder found\n");
        return -1;
    }
    AVCodecContext *dec_ctx = avcodec_alloc_context3(dec);
    dec_ctx->width = job->src_w;
    dec_ctx->height = job->src_h;
    dec_ctx->thread_count = 2;

    if (job->extradata && job->extradata_size > 0) {
        dec_ctx->extradata = av_mallocz(job->extradata_size +
                                        AV_INPUT_BUFFER_PADDING_SIZE);
        memcpy(dec_ctx->extradata, job->extradata, job->extradata_size);
        dec_ctx->extradata_size = job->extradata_size;
    }

    if (avcodec_open2(dec_ctx, dec, NULL) < 0) {
        fprintf(stderr, "[clip] Failed to open HEVC decoder\n");
        avcodec_free_context(&dec_ctx);
        return -1;
    }

    /* Init encoder: try HW first, then SW fallback */
    const AVCodec *enc = avcodec_find_encoder_by_name("hevc_rkmpp");
    int hw_enc = (enc != NULL);
    if (!enc) {
        enc = avcodec_find_encoder(AV_CODEC_ID_HEVC);
        hw_enc = 0;
    }
    if (!enc) {
        fprintf(stderr, "[clip] No HEVC encoder found\n");
        avcodec_free_context(&dec_ctx);
        return -1;
    }

    AVCodecContext *enc_ctx = avcodec_alloc_context3(enc);
    enc_ctx->width = out_w;
    enc_ctx->height = out_h;
    enc_ctx->time_base = (AVRational){1, 50};
    enc_ctx->framerate = (AVRational){50, 1};
    enc_ctx->gop_size = 100;
    enc_ctx->max_b_frames = 0;
    enc_ctx->pix_fmt = hw_enc ? AV_PIX_FMT_NV12 : AV_PIX_FMT_YUV420P;

    if (hw_enc) {
        av_opt_set(enc_ctx->priv_data, "quality", "medium", 0);
    }

    if (avcodec_open2(enc_ctx, enc, NULL) < 0) {
        if (hw_enc) {
            fprintf(stderr, "[clip] HW encoder failed, trying SW fallback\n");
            avcodec_free_context(&enc_ctx);
            enc = avcodec_find_encoder(AV_CODEC_ID_HEVC);
            hw_enc = 0;
            if (!enc) {
                avcodec_free_context(&dec_ctx);
                return -1;
            }
            enc_ctx = avcodec_alloc_context3(enc);
            enc_ctx->width = out_w;
            enc_ctx->height = out_h;
            enc_ctx->time_base = (AVRational){1, 50};
            enc_ctx->framerate = (AVRational){50, 1};
            enc_ctx->gop_size = 100;
            enc_ctx->max_b_frames = 0;
            enc_ctx->pix_fmt = AV_PIX_FMT_YUV420P;
            if (avcodec_open2(enc_ctx, enc, NULL) < 0) {
                fprintf(stderr, "[clip] SW encoder also failed\n");
                avcodec_free_context(&enc_ctx);
                avcodec_free_context(&dec_ctx);
                return -1;
            }
        } else {
            fprintf(stderr, "[clip] Encoder open failed\n");
            avcodec_free_context(&enc_ctx);
            avcodec_free_context(&dec_ctx);
            return -1;
        }
    }

    printf("[clip] Encoding with %s (%s, %dx%d)\n",
           enc->name, hw_enc ? "HW" : "SW", out_w, out_h);

    /* SWS created lazily after first decoded frame */
    struct SwsContext *sws = NULL;

    /* Open MP4 output */
    AVFormatContext *ofmt = NULL;
    avformat_alloc_output_context2(&ofmt, NULL, "mp4", mp4_path);
    if (!ofmt) {
        fprintf(stderr, "[clip] Failed to create MP4 output context\n");
        avcodec_free_context(&enc_ctx);
        avcodec_free_context(&dec_ctx);
        return -1;
    }

    AVStream *ost = avformat_new_stream(ofmt, enc);
    avcodec_parameters_from_context(ost->codecpar, enc_ctx);
    ost->time_base = enc_ctx->time_base;

    if (!(ofmt->oformat->flags & AVFMT_NOFILE)) {
        if (avio_open(&ofmt->pb, mp4_path, AVIO_FLAG_WRITE) < 0) {
            fprintf(stderr, "[clip] Cannot open %s for writing\n", mp4_path);
            avformat_free_context(ofmt);
            avcodec_free_context(&enc_ctx);
            avcodec_free_context(&dec_ctx);
            return -1;
        }
    }

    if (avformat_write_header(ofmt, NULL) < 0) {
        fprintf(stderr, "[clip] Failed to write MP4 header\n");
        goto cleanup;
    }

    /* Allocate frames */
    AVFrame *frame = av_frame_alloc();
    AVFrame *scaled = av_frame_alloc();
    scaled->format = enc_ctx->pix_fmt;
    scaled->width = out_w;
    scaled->height = out_h;
    av_frame_get_buffer(scaled, 0);

    AVPacket *out_pkt = av_packet_alloc();
    int64_t out_pts = 0;
    int jpeg_saved = 0;
    int frames_encoded = 0;

    /* Process packets */
    for (int i = 0; i < job->num_pkts; i++) {
        RingPacket *rp = &job->pkts[i];
        if (!rp->data || rp->size <= 0) continue;

        AVPacket in_pkt;
        memset(&in_pkt, 0, sizeof(in_pkt));
        in_pkt.data = rp->data;
        in_pkt.size = rp->size;
        in_pkt.pts = rp->pts;
        in_pkt.dts = rp->dts;
        in_pkt.flags = rp->flags;

        int ret = avcodec_send_packet(dec_ctx, &in_pkt);
        if (ret < 0) continue;

        while (ret >= 0) {
            ret = avcodec_receive_frame(dec_ctx, frame);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) break;
            if (ret < 0) break;

            /* Lazy SWS init */
            if (!sws && frame->format != AV_PIX_FMT_NONE) {
                sws = sws_getContext(
                    job->src_w, job->src_h, frame->format,
                    out_w, out_h, enc_ctx->pix_fmt,
                    SWS_BILINEAR, NULL, NULL, NULL);
                if (!sws) {
                    fprintf(stderr, "[clip] SWS init failed\n");
                    av_frame_unref(frame);
                    goto cleanup;
                }
            }

            if (!sws) {
                av_frame_unref(frame);
                continue;
            }

            /* Scale */
            sws_scale(sws, (const uint8_t *const *)frame->data,
                      frame->linesize, 0, job->src_h,
                      scaled->data, scaled->linesize);

            /* JPEG snapshot */
            if (!jpeg_saved && i >= job->snapshot_idx) {
                if (save_jpeg(jpg_path, scaled, out_w, out_h) == 0)
                    printf("[clip] Saved snapshot %s\n", jpg_path);
                jpeg_saved = 1;
            }

            /* Encode */
            scaled->pts = out_pts++;
            ret = avcodec_send_frame(enc_ctx, scaled);
            if (ret < 0) {
                av_frame_unref(frame);
                continue;
            }

            while (1) {
                ret = avcodec_receive_packet(enc_ctx, out_pkt);
                if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) break;
                if (ret < 0) break;
                av_packet_rescale_ts(out_pkt, enc_ctx->time_base,
                                     ost->time_base);
                out_pkt->stream_index = ost->index;
                av_interleaved_write_frame(ofmt, out_pkt);
                av_packet_unref(out_pkt);
                frames_encoded++;
            }

            av_frame_unref(frame);
        }
    }

    /* Flush encoder */
    avcodec_send_frame(enc_ctx, NULL);
    while (1) {
        int ret = avcodec_receive_packet(enc_ctx, out_pkt);
        if (ret < 0) break;
        av_packet_rescale_ts(out_pkt, enc_ctx->time_base, ost->time_base);
        out_pkt->stream_index = ost->index;
        av_interleaved_write_frame(ofmt, out_pkt);
        av_packet_unref(out_pkt);
        frames_encoded++;
    }

    /* Finalize MP4 */
    av_write_trailer(ofmt);

    double duration = (double)out_pts / 50.0;
    int file_size = 0;
    if (ofmt->pb)
        file_size = (int)avio_tell(ofmt->pb);

    /* Insert DB record */
    if (job->db) {
        db_clip_insert(job->db, job->camera_id, mp4_path, jpg_path,
                       start_ts, end_ts, duration, file_size,
                       out_w, out_h,
                       job->event_ids, job->num_event_ids);
    }

    printf("[clip] Saved %s (%.1fs, %d frames, %d events, %.0f KB)\n",
           mp4_path, duration, frames_encoded, job->num_event_ids,
           file_size / 1024.0);

cleanup:
    if (!(ofmt->oformat->flags & AVFMT_NOFILE))
        avio_closep(&ofmt->pb);
    avformat_free_context(ofmt);
    if (sws) sws_freeContext(sws);
    av_frame_free(&frame);
    av_frame_free(&scaled);
    av_packet_free(&out_pkt);
    avcodec_free_context(&dec_ctx);
    avcodec_free_context(&enc_ctx);

    return (frames_encoded > 0) ? 0 : -1;
}

/* ---- Job Queue ---- */

static void clip_queue_init(ClipQueue *q)
{
    memset(q, 0, sizeof(*q));
    pthread_mutex_init(&q->mutex, NULL);
    pthread_cond_init(&q->cond, NULL);
}

static void clip_queue_destroy(ClipQueue *q)
{
    /* Free any remaining jobs */
    for (int i = 0; i < q->count; i++) {
        int idx = (q->head + i) % CLIP_QUEUE_MAX;
        clip_job_free(q->jobs[idx]);
    }
    pthread_mutex_destroy(&q->mutex);
    pthread_cond_destroy(&q->cond);
}

static void clip_queue_push(ClipQueue *q, ClipJob *job)
{
    pthread_mutex_lock(&q->mutex);

    /* If full, drop oldest job to avoid blocking camera thread */
    if (q->count >= CLIP_QUEUE_MAX) {
        fprintf(stderr, "[clip] Queue full, dropping oldest job\n");
        clip_job_free(q->jobs[q->head]);
        q->head = (q->head + 1) % CLIP_QUEUE_MAX;
        q->count--;
    }

    q->jobs[q->tail] = job;
    q->tail = (q->tail + 1) % CLIP_QUEUE_MAX;
    q->count++;
    pthread_cond_signal(&q->cond);
    pthread_mutex_unlock(&q->mutex);
}

static ClipJob *clip_queue_pop(ClipQueue *q)
{
    pthread_mutex_lock(&q->mutex);
    while (q->count == 0 && !q->shutdown)
        pthread_cond_wait(&q->cond, &q->mutex);

    if (q->shutdown && q->count == 0) {
        pthread_mutex_unlock(&q->mutex);
        return NULL;
    }

    ClipJob *job = q->jobs[q->head];
    q->head = (q->head + 1) % CLIP_QUEUE_MAX;
    q->count--;
    pthread_mutex_unlock(&q->mutex);
    return job;
}

/* ---- Transcoder Thread ---- */

static void *transcoder_thread(void *arg)
{
    ClipTranscoder *tc = (ClipTranscoder *)arg;
    printf("[clip] Transcoder thread started\n");

    while (1) {
        ClipJob *job = clip_queue_pop(&tc->queue);
        if (!job) break;

        printf("[clip] Transcoding: cam_%d, %d pkts\n",
               job->camera_id, job->num_pkts);

        double t0 = clip_get_time();
        int rc = transcode_clip(job);
        double elapsed = clip_get_time() - t0;

        if (rc < 0)
            fprintf(stderr, "[clip] Transcode failed for cam_%d\n",
                    job->camera_id);
        else
            printf("[clip] Transcode done in %.1fs\n", elapsed);

        clip_job_free(job);
    }

    printf("[clip] Transcoder thread exiting\n");
    return NULL;
}

/* ---- Public API ---- */

void clip_transcoder_init(ClipTranscoder *tc)
{
    memset(tc, 0, sizeof(*tc));
    clip_queue_init(&tc->queue);
}

void clip_transcoder_start(ClipTranscoder *tc)
{
    if (tc->started) return;
    if (pthread_create(&tc->thread, NULL, transcoder_thread, tc) != 0) {
        fprintf(stderr, "[clip] Failed to start transcoder thread\n");
        return;
    }
    tc->started = 1;
}

void clip_transcoder_stop(ClipTranscoder *tc)
{
    if (!tc->started) return;

    pthread_mutex_lock(&tc->queue.mutex);
    tc->queue.shutdown = 1;
    pthread_cond_broadcast(&tc->queue.cond);
    pthread_mutex_unlock(&tc->queue.mutex);

    pthread_join(tc->thread, NULL);
    tc->started = 0;
    clip_queue_destroy(&tc->queue);
}

void clip_transcoder_submit(ClipTranscoder *tc, ClipJob *job)
{
    if (!tc || !tc->started || !job) {
        clip_job_free(job);
        return;
    }
    clip_queue_push(&tc->queue, job);
}
