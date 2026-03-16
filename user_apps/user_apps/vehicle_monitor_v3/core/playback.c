/*
 * core/playback.c - DVR playback engine
 *
 * Uses software decode to avoid RKMPP contention with live camera threads.
 * Reads recording segments from disk, decodes, scales to RGB, shares via mutex.
 * Supports seek, speed control (0.5x-8x), and cross-segment playback.
 */
#include "playback.h"
#include "database.h"
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

static double pb_get_time(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

/* Parse ISO timestamp to epoch */
static double ts_to_epoch(const char *ts)
{
    if (!ts || !ts[0]) return 0;
    struct tm tm;
    memset(&tm, 0, sizeof(tm));
    sscanf(ts, "%d-%d-%dT%d:%d:%d",
           &tm.tm_year, &tm.tm_mon, &tm.tm_mday,
           &tm.tm_hour, &tm.tm_min, &tm.tm_sec);
    tm.tm_year -= 1900;
    tm.tm_mon -= 1;
    tm.tm_isdst = -1;
    return (double)mktime(&tm);
}

/* Format epoch to ISO timestamp */
static void epoch_to_ts(double epoch, char *buf, int buflen)
{
    time_t t = (time_t)epoch;
    struct tm tm;
    localtime_r(&t, &tm);
    strftime(buf, buflen, "%Y-%m-%dT%H:%M:%S", &tm);
}

double playback_speed_multiplier(PlaybackSpeed s)
{
    switch (s) {
    case SPEED_HALF: return 0.5;
    case SPEED_1X:   return 1.0;
    case SPEED_2X:   return 2.0;
    case SPEED_4X:   return 4.0;
    case SPEED_8X:   return 8.0;
    default:         return 1.0;
    }
}

const char *playback_speed_label(PlaybackSpeed s)
{
    switch (s) {
    case SPEED_HALF: return "0.5x";
    case SPEED_1X:   return "1x";
    case SPEED_2X:   return "2x";
    case SPEED_4X:   return "4x";
    case SPEED_8X:   return "8x";
    default:         return "1x";
    }
}

PlaybackCtx *playback_create(AppCtx *app, int camera_id)
{
    PlaybackCtx *pb = calloc(1, sizeof(PlaybackCtx));
    pb->app = app;
    pb->camera_id = camera_id;
    pb->speed = SPEED_1X;
    pb->paused = 1;
    pb->aspect_w = 16;
    pb->aspect_h = 9;
    pb->scale = 1.0;

    /* Match live camera display settings */
    for (int i = 0; i < app->num_cameras; i++) {
        if (app->cameras[i].id == camera_id) {
            if (app->cameras[i].aspect_w > 0 && app->cameras[i].aspect_h > 0) {
                pb->aspect_w = app->cameras[i].aspect_w;
                pb->aspect_h = app->cameras[i].aspect_h;
            }
            if (app->cameras[i].scale >= 0.1)
                pb->scale = app->cameras[i].scale;
            break;
        }
    }

    pthread_mutex_init(&pb->mutex, NULL);
    return pb;
}

static void pb_close_file(PlaybackCtx *pb)
{
    if (pb->sws_ctx) {
        sws_freeContext(pb->sws_ctx);
        pb->sws_ctx = NULL;
    }
    if (pb->dec_ctx) {
        avcodec_free_context(&pb->dec_ctx);
        pb->dec_ctx = NULL;
    }
    if (pb->fmt_ctx) {
        avformat_close_input(&pb->fmt_ctx);
        pb->fmt_ctx = NULL;
    }
    if (pb->raw_file) {
        fclose((FILE *)pb->raw_file);
        pb->raw_file = NULL;
    }
    pb->raw_mode = 0;
    pb->raw_rec_id = 0;
    pb->current_file[0] = '\0';
}

/* Open a raw HEVC file: read header, set up decoder */
static int pb_open_raw_hevc(PlaybackCtx *pb, const char *filepath)
{
    FILE *f = fopen(filepath, "rb");
    if (!f) return -1;

    /* Read and validate header */
    char magic[4];
    uint32_t version, width, height, extsize;
    if (fread(magic, 1, 4, f) != 4 || memcmp(magic, "HEVC", 4) != 0) {
        fclose(f);
        return -1;
    }
    if (fread(&version, 4, 1, f) != 1 || version != 1) {
        fclose(f);
        return -1;
    }
    if (fread(&width, 4, 1, f) != 1 || fread(&height, 4, 1, f) != 1 ||
        fread(&extsize, 4, 1, f) != 1) {
        fclose(f);
        return -1;
    }

    /* Read extradata (VPS/SPS/PPS) */
    uint8_t *extradata = NULL;
    if (extsize > 0 && extsize < 4096) {
        extradata = malloc(extsize);
        if (fread(extradata, 1, extsize, f) != extsize) {
            free(extradata);
            fclose(f);
            return -1;
        }
    }

    /* Create HEVC decoder */
    const AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_HEVC);
    if (!codec) {
        free(extradata);
        fclose(f);
        return -1;
    }

    pb->dec_ctx = avcodec_alloc_context3(codec);
    pb->dec_ctx->width = (int)width;
    pb->dec_ctx->height = (int)height;
    pb->dec_ctx->thread_count = 2;

    if (extradata && extsize > 0) {
        pb->dec_ctx->extradata = av_mallocz(extsize + AV_INPUT_BUFFER_PADDING_SIZE);
        memcpy(pb->dec_ctx->extradata, extradata, extsize);
        pb->dec_ctx->extradata_size = (int)extsize;
        free(extradata);
    }

    if (avcodec_open2(pb->dec_ctx, codec, NULL) < 0) {
        avcodec_free_context(&pb->dec_ctx);
        fclose(f);
        return -1;
    }

    pb->raw_file = f;
    pb->raw_mode = 1;
    pb->src_w = (int)width;
    pb->src_h = (int)height;
    pb->pts_to_sec = 1.0 / 1000000.0;
    return 0;
}

static int pb_open_file(PlaybackCtx *pb, const char *filepath,
                        double seg_start, double seg_end, int rec_id)
{
    pb_close_file(pb);

    /* Detect file type by extension */
    const char *ext = strrchr(filepath, '.');
    int is_hevc = (ext && strcmp(ext, ".hevc") == 0);

    if (is_hevc) {
        if (pb_open_raw_hevc(pb, filepath) < 0) {
            fprintf(stderr, "[playback] Cannot open raw HEVC %s\n", filepath);
            return -1;
        }
        pb->raw_rec_id = rec_id;
    } else {
        /* Legacy MP4 path */
        int ret = avformat_open_input(&pb->fmt_ctx, filepath, NULL, NULL);
        if (ret < 0) {
            fprintf(stderr, "[playback] Cannot open %s\n", filepath);
            return -1;
        }

        if (avformat_find_stream_info(pb->fmt_ctx, NULL) < 0) {
            avformat_close_input(&pb->fmt_ctx);
            return -1;
        }

        pb->video_idx = -1;
        for (unsigned i = 0; i < pb->fmt_ctx->nb_streams; i++) {
            if (pb->fmt_ctx->streams[i]->codecpar->codec_type ==
                AVMEDIA_TYPE_VIDEO) {
                pb->video_idx = (int)i;
                break;
            }
        }
        if (pb->video_idx < 0) {
            avformat_close_input(&pb->fmt_ctx);
            return -1;
        }

        AVCodecParameters *par = pb->fmt_ctx->streams[pb->video_idx]->codecpar;
        const AVCodec *codec = avcodec_find_decoder(par->codec_id);
        if (!codec) {
            avformat_close_input(&pb->fmt_ctx);
            return -1;
        }

        pb->dec_ctx = avcodec_alloc_context3(codec);
        avcodec_parameters_to_context(pb->dec_ctx, par);
        pb->dec_ctx->thread_count = 2;

        if (avcodec_open2(pb->dec_ctx, codec, NULL) < 0) {
            avcodec_free_context(&pb->dec_ctx);
            avformat_close_input(&pb->fmt_ctx);
            return -1;
        }

        pb->src_w = pb->dec_ctx->width;
        pb->src_h = pb->dec_ctx->height;
        pb->raw_mode = 0;
        AVStream *st = pb->fmt_ctx->streams[pb->video_idx];
        pb->pts_to_sec = av_q2d(st->time_base);
    }

    /* Display size: match live camera (scale + aspect ratio) */
    double sc = pb->scale;
    if (sc < 0.1) sc = 1.0;
    if (sc > 4.0) sc = 4.0;
    pb->disp_w = (int)(pb->src_w * sc);
    pb->disp_h = pb->disp_w * pb->aspect_h / pb->aspect_w;
    if (pb->disp_w < 64) pb->disp_w = 64;
    if (pb->disp_h < 64) pb->disp_h = 64;
    pb->disp_size = pb->disp_w * pb->disp_h * 3;

    /* SWS created lazily on first decoded frame (pix_fmt unknown until then) */

    /* Allocate shared frame buffer */
    pthread_mutex_lock(&pb->mutex);
    free(pb->frame_rgb);
    pb->frame_rgb = calloc(pb->disp_size, 1);
    pthread_mutex_unlock(&pb->mutex);

    snprintf(pb->current_file, sizeof(pb->current_file), "%s", filepath);
    pb->seg_start_epoch = seg_start;
    pb->seg_end_epoch = seg_end;

    printf("[playback] Opened %s (%dx%d -> %dx%d, %s)\n",
           filepath, pb->src_w, pb->src_h, pb->disp_w, pb->disp_h,
           pb->raw_mode ? "raw HEVC" : "mp4");
    return 0;
}

static int pb_open_segment_at(PlaybackCtx *pb, double epoch)
{
    char ts[32];
    epoch_to_ts(epoch, ts, sizeof(ts));

    DbRecording rec;
    if (!db_recording_find_at_time(pb->app->db, pb->camera_id, ts, &rec)) {
        fprintf(stderr, "[playback] No recording at %s for cam %d\n",
                ts, pb->camera_id);
        return -1;
    }

    double seg_start = ts_to_epoch(rec.start_time);
    double seg_end = rec.end_time[0] ? ts_to_epoch(rec.end_time) : seg_start + 600;

    return pb_open_file(pb, rec.filepath, seg_start, seg_end, rec.id);
}

static int pb_open_next_segment(PlaybackCtx *pb)
{
    /* Find the segment that starts after current segment end */
    char ts[32];
    epoch_to_ts(pb->seg_end_epoch + 1, ts, sizeof(ts));

    /* Search for recording at or after end time */
    char ts_end[32];
    epoch_to_ts(pb->seg_end_epoch + 86400, ts_end, sizeof(ts_end));

    DbRecording recs[8];
    char ts_start[32];
    epoch_to_ts(pb->seg_end_epoch, ts_start, sizeof(ts_start));
    int n = db_recording_load_range(pb->app->db, pb->camera_id,
                                    ts_start, ts_end, recs, 8);
    for (int i = 0; i < n; i++) {
        double start = ts_to_epoch(recs[i].start_time);
        if (start >= pb->seg_end_epoch - 1 &&
            strcmp(recs[i].filepath, pb->current_file) != 0) {
            double end = recs[i].end_time[0] ?
                ts_to_epoch(recs[i].end_time) : start + 600;
            return pb_open_file(pb, recs[i].filepath, start, end, recs[i].id);
        }
    }
    return -1;
}

/* Read one frame from raw HEVC file into AVPacket.
 * Returns 0 on success, -1 on EOF/error. */
static int pb_read_raw_frame(PlaybackCtx *pb, AVPacket *pkt)
{
    FILE *f = (FILE *)pb->raw_file;
    uint32_t size, flags;
    int64_t pts_us;

    if (fread(&size, 4, 1, f) != 1) return -1;
    if (fread(&flags, 4, 1, f) != 1) return -1;
    if (fread(&pts_us, 8, 1, f) != 1) return -1;

    if (size > 16 * 1024 * 1024) return -1; /* sanity limit: 16 MB */

    if (av_new_packet(pkt, (int)size) < 0) return -1;
    if (fread(pkt->data, 1, size, f) != size) {
        av_packet_unref(pkt);
        return -1;
    }

    pkt->pts = pts_us;
    pkt->dts = pts_us;
    if (flags & 1) pkt->flags |= AV_PKT_FLAG_KEY;
    return 0;
}

static void *playback_thread(void *arg)
{
    PlaybackCtx *pb = (PlaybackCtx *)arg;
    AVPacket *pkt = av_packet_alloc();
    AVFrame *frame = av_frame_alloc();

    printf("[playback] Thread started for cam %d\n", pb->camera_id);

    while (pb->running) {
        /* Check seek request */
        pthread_mutex_lock(&pb->mutex);
        int do_seek = pb->seek_requested;
        double seek_target = pb->seek_target;
        if (do_seek) pb->seek_requested = 0;
        int is_paused = pb->paused;
        double spd = playback_speed_multiplier(pb->speed);
        pthread_mutex_unlock(&pb->mutex);

        if (do_seek) {
            /* Check if seek target is in current segment */
            int have_file = pb->raw_mode ? (pb->raw_file != NULL)
                                         : (pb->fmt_ctx != NULL);
            int need_new_seg = 0;
            if (!have_file ||
                seek_target < pb->seg_start_epoch ||
                seek_target > pb->seg_end_epoch) {
                need_new_seg = 1;
            }

            if (need_new_seg) {
                if (pb_open_segment_at(pb, seek_target) < 0) {
                    usleep(100000);
                    continue;
                }
            }

            /* Seek within file */
            double offset = seek_target - pb->seg_start_epoch;
            if (offset < 0) offset = 0;

            if (pb->raw_mode && pb->raw_file) {
                /* Raw HEVC: use keyframe index */
                int64_t seek_pts_us = (int64_t)(offset * 1000000.0);
                int64_t kf_offset, kf_pts;
                if (db_rec_index_find_keyframe(pb->app->db, pb->raw_rec_id,
                                               seek_pts_us,
                                               &kf_offset, &kf_pts) == 0) {
                    fseek((FILE *)pb->raw_file, kf_offset, SEEK_SET);
                    if (pb->dec_ctx)
                        avcodec_flush_buffers(pb->dec_ctx);
                }
            } else if (pb->fmt_ctx) {
                /* Legacy MP4: FFmpeg seek */
                AVStream *st = pb->fmt_ctx->streams[pb->video_idx];
                int64_t seek_ts = (int64_t)(offset / av_q2d(st->time_base));
                av_seek_frame(pb->fmt_ctx, pb->video_idx, seek_ts,
                              AVSEEK_FLAG_BACKWARD);
                if (pb->dec_ctx)
                    avcodec_flush_buffers(pb->dec_ctx);
            }

            /* Reset pacing */
            pb->wall_base = pb_get_time();
            pb->video_base = seek_target;

            pthread_mutex_lock(&pb->mutex);
            pb->position_sec = seek_target;
            pthread_mutex_unlock(&pb->mutex);
        }

        /* If paused, just sleep */
        if (is_paused) {
            usleep(50000);
            continue;
        }

        /* Need an open file */
        int have_file = pb->raw_mode ? (pb->raw_file != NULL)
                                     : (pb->fmt_ctx != NULL);
        if (!have_file) {
            usleep(100000);
            continue;
        }

        /* Read a packet */
        int ret;
        if (pb->raw_mode) {
            ret = pb_read_raw_frame(pb, pkt);
        } else {
            ret = av_read_frame(pb->fmt_ctx, pkt);
        }

        if (ret < 0) {
            /* EOF or error - try next segment */
            if (pb_open_next_segment(pb) < 0) {
                pthread_mutex_lock(&pb->mutex);
                pb->paused = 1;
                pthread_mutex_unlock(&pb->mutex);
                continue;
            }
            pb->wall_base = pb_get_time();
            pb->video_base = pb->seg_start_epoch;
            continue;
        }

        /* In legacy mode, skip non-video streams */
        if (!pb->raw_mode && pkt->stream_index != pb->video_idx) {
            av_packet_unref(pkt);
            continue;
        }

        ret = avcodec_send_packet(pb->dec_ctx, pkt);
        av_packet_unref(pkt);
        if (ret < 0) continue;

        while (ret >= 0 && pb->running) {
            ret = avcodec_receive_frame(pb->dec_ctx, frame);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                break;
            if (ret < 0)
                break;

            /* Calculate frame epoch time */
            double pts_sec = frame->pts * pb->pts_to_sec;
            double frame_epoch = pb->seg_start_epoch + pts_sec;

            /* Pacing: wait based on speed */
            double video_elapsed = frame_epoch - pb->video_base;
            double wall_elapsed = pb_get_time() - pb->wall_base;
            double target_wall = video_elapsed / spd;
            double wait = target_wall - wall_elapsed;
            if (wait > 0 && wait < 2.0) {
                usleep((useconds_t)(wait * 1000000));
            } else if (wait < -1.0) {
                pb->wall_base = pb_get_time();
                pb->video_base = frame_epoch;
            }

            /* Lazy SWS creation on first decoded frame */
            if (!pb->sws_ctx && frame->format != AV_PIX_FMT_NONE) {
                pb->sws_ctx = sws_getContext(
                    pb->src_w, pb->src_h, frame->format,
                    pb->disp_w, pb->disp_h, AV_PIX_FMT_RGB24,
                    SWS_BILINEAR, NULL, NULL, NULL);
                if (pb->sws_ctx)
                    printf("[playback] SWS created: fmt=%d %dx%d -> %dx%d RGB24\n",
                           frame->format, pb->src_w, pb->src_h,
                           pb->disp_w, pb->disp_h);
            }

            if (pb->sws_ctx && frame->format != AV_PIX_FMT_NONE) {
                uint8_t *dst[1];
                int dst_stride[1];

                pthread_mutex_lock(&pb->mutex);
                if (pb->frame_rgb) {
                    dst[0] = pb->frame_rgb;
                    dst_stride[0] = pb->disp_w * 3;
                    sws_scale(pb->sws_ctx,
                              (const uint8_t *const *)frame->data,
                              frame->linesize, 0, pb->src_h,
                              dst, dst_stride);
                    pb->frame_updated = 1;
                    pb->frame_time = frame_epoch;
                    pb->position_sec = frame_epoch;
                }
                pthread_mutex_unlock(&pb->mutex);
            }

            av_frame_unref(frame);
        }
    }

    av_packet_free(&pkt);
    av_frame_free(&frame);
    pb_close_file(pb);
    printf("[playback] Thread exiting for cam %d\n", pb->camera_id);
    return NULL;
}

int playback_start(PlaybackCtx *pb, double epoch_time)
{
    if (pb->started)
        playback_stop(pb);

    pb->running = 1;
    pb->paused = 0;
    pb->seek_requested = 1;
    pb->seek_target = epoch_time;
    pb->position_sec = epoch_time;
    pb->wall_base = pb_get_time();
    pb->video_base = epoch_time;

    if (pthread_create(&pb->thread, NULL, playback_thread, pb) != 0) {
        pb->running = 0;
        return -1;
    }
    pb->started = 1;
    return 0;
}

void playback_stop(PlaybackCtx *pb)
{
    if (!pb->started)
        return;
    pb->running = 0;
    pthread_join(pb->thread, NULL);
    pb->started = 0;
}

void playback_destroy(PlaybackCtx *pb)
{
    if (!pb) return;
    playback_stop(pb);
    pthread_mutex_destroy(&pb->mutex);
    free(pb->frame_rgb);
    free(pb);
}

void playback_toggle_pause(PlaybackCtx *pb)
{
    pthread_mutex_lock(&pb->mutex);
    pb->paused = !pb->paused;
    if (!pb->paused) {
        /* Reset pacing on unpause */
        pb->wall_base = pb_get_time();
        pb->video_base = pb->position_sec;
    }
    pthread_mutex_unlock(&pb->mutex);
}

void playback_set_speed(PlaybackCtx *pb, PlaybackSpeed speed)
{
    pthread_mutex_lock(&pb->mutex);
    pb->speed = speed;
    /* Reset pacing */
    pb->wall_base = pb_get_time();
    pb->video_base = pb->position_sec;
    pthread_mutex_unlock(&pb->mutex);
}

void playback_seek(PlaybackCtx *pb, double epoch_time)
{
    pthread_mutex_lock(&pb->mutex);
    pb->seek_requested = 1;
    pb->seek_target = epoch_time;
    pthread_mutex_unlock(&pb->mutex);
}

double playback_get_position(PlaybackCtx *pb)
{
    pthread_mutex_lock(&pb->mutex);
    double pos = pb->position_sec;
    pthread_mutex_unlock(&pb->mutex);
    return pos;
}

int playback_is_paused(PlaybackCtx *pb)
{
    pthread_mutex_lock(&pb->mutex);
    int p = pb->paused;
    pthread_mutex_unlock(&pb->mutex);
    return p;
}

PlaybackSpeed playback_get_speed(PlaybackCtx *pb)
{
    pthread_mutex_lock(&pb->mutex);
    PlaybackSpeed s = pb->speed;
    pthread_mutex_unlock(&pb->mutex);
    return s;
}
