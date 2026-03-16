/*
 * core/camera.c - Camera decode threads and shared buffers
 *
 * Adapted from motion_detect_v4.c worker thread pattern.
 * One thread per camera: FFmpeg decode → Y extract → blur → grid motion → RGB.
 * Triple buffer: worker-local → shared (mutex) → draw-local (GTK draw callback).
 */
#include "camera.h"
#include "motion.h"
#include "database.h"
#include "undistort.h"
#include "clip.h"
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/hwcontext.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libavutil/opt.h>
#include <time.h>
#include <math.h>
#include <sys/stat.h>

#ifdef __aarch64__
#include <arm_neon.h>
#endif

/* ---- Birdview pre-rotation (NEON on aarch64, scalar fallback) ---- */

/*
 * 90° CW rotation of RGB image: (w,h) → (h,w)
 * Output pixel (dx,dy) = input pixel (y, w-1-dx) where dx=h-1-y, dy=x
 * NEON: process 8 output pixels per iteration using vld3 gather.
 */
static void bv_rotate_90_cw(const uint8_t *src, int w, int h, uint8_t *dst)
{
    int dw = h;  /* output width = input height */
#ifdef __aarch64__
    /* Tile-based: for each output row (= input column), process 8 pixels */
    for (int x = 0; x < w; x++) {
        const int dy = x;
        int y = h - 1;
        uint8_t *drow = dst + dy * dw * 3;
        /* Process 8 pixels at a time from bottom to top of input column */
        int dx = 0;
        for (; dx + 7 < h; dx += 8, y -= 8) {
            /* Load 8 pixels from input column x, rows y..y-7 */
            uint8_t tmp[24];
            for (int k = 0; k < 8; k++) {
                int si = ((y - k) * w + x) * 3;
                tmp[k * 3]     = src[si];
                tmp[k * 3 + 1] = src[si + 1];
                tmp[k * 3 + 2] = src[si + 2];
            }
            /* Store 8 pixels contiguously */
            uint8_t *dp = drow + dx * 3;
            uint8x8x3_t v = vld3_u8(tmp);
            vst3_u8(dp, v);
        }
        /* Scalar tail */
        for (; dx < h; dx++, y--) {
            int si = (y * w + x) * 3;
            uint8_t *dp = drow + dx * 3;
            dp[0] = src[si]; dp[1] = src[si + 1]; dp[2] = src[si + 2];
        }
    }
#else
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            int si = (y * w + x) * 3;
            int dx = h - 1 - y, dy = x;
            int di = (dy * dw + dx) * 3;
            dst[di] = src[si]; dst[di+1] = src[si+1]; dst[di+2] = src[si+2];
        }
#endif
}

/*
 * 90° CCW rotation of RGB image: (w,h) → (h,w)
 */
static void bv_rotate_90_ccw(const uint8_t *src, int w, int h, uint8_t *dst)
{
    int dw = h;
#ifdef __aarch64__
    for (int x = 0; x < w; x++) {
        const int dy = w - 1 - x;
        uint8_t *drow = dst + dy * dw * 3;
        int dx = 0;
        for (int y = 0; dx + 7 < h; dx += 8, y += 8) {
            uint8_t tmp[24];
            for (int k = 0; k < 8; k++) {
                int si = ((y + k) * w + x) * 3;
                tmp[k * 3]     = src[si];
                tmp[k * 3 + 1] = src[si + 1];
                tmp[k * 3 + 2] = src[si + 2];
            }
            uint8_t *dp = drow + dx * 3;
            uint8x8x3_t v = vld3_u8(tmp);
            vst3_u8(dp, v);
        }
        for (int y = h - (h % 8); dx < h; dx++, y++) {
            int si = (y * w + x) * 3;
            uint8_t *dp = drow + dx * 3;
            dp[0] = src[si]; dp[1] = src[si + 1]; dp[2] = src[si + 2];
        }
    }
#else
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            int si = (y * w + x) * 3;
            int dx = y, dy = w - 1 - x;
            int di = (dy * dw + dx) * 3;
            dst[di] = src[si]; dst[di+1] = src[si+1]; dst[di+2] = src[si+2];
        }
#endif
}

/*
 * 180° rotation of RGB image (same dimensions).
 * NEON: process 16 pixels per iteration, reversing order.
 */
static void bv_rotate_180(const uint8_t *src, int w, int h, uint8_t *dst)
{
    int total = w * h;
#ifdef __aarch64__
    int i = 0, j = total - 1;
    for (; i + 15 < total; i += 16, j -= 16) {
        /* Load 16 forward pixels */
        uint8x16x3_t fwd = vld3q_u8(src + i * 3);
        /* Reverse each channel */
        fwd.val[0] = vrev64q_u8(fwd.val[0]);
        fwd.val[1] = vrev64q_u8(fwd.val[1]);
        fwd.val[2] = vrev64q_u8(fwd.val[2]);
        /* Swap high/low 64-bit halves to complete 128-bit reverse */
        fwd.val[0] = vextq_u8(fwd.val[0], fwd.val[0], 8);
        fwd.val[1] = vextq_u8(fwd.val[1], fwd.val[1], 8);
        fwd.val[2] = vextq_u8(fwd.val[2], fwd.val[2], 8);
        vst3q_u8(dst + (j - 15) * 3, fwd);
    }
    for (; i < total; i++) {
        int k = total - 1 - i;
        dst[k * 3]     = src[i * 3];
        dst[k * 3 + 1] = src[i * 3 + 1];
        dst[k * 3 + 2] = src[i * 3 + 2];
    }
#else
    for (int i = 0; i < total; i++) {
        int j = total - 1 - i;
        dst[j * 3]     = src[i * 3];
        dst[j * 3 + 1] = src[i * 3 + 1];
        dst[j * 3 + 2] = src[i * 3 + 2];
    }
#endif
}

/*
 * Pre-rotate color frame for birdview based on camera position.
 * Called from worker thread after nv12_to_rgb.
 * Writes to cam->bv_rotated under mutex.
 */
/*
 * Pre-rotate into thread-local buffer. Returns pointer to rotated
 * data and sets *out_w, *out_h. Returns NULL if no birdview pos.
 */
static const uint8_t *bv_prerotate(CameraCtx *cam, int *out_w, int *out_h)
{
    if (cam->birdview_pos == 0 || cam->disp_w == 0)
        return NULL;

    int sw = cam->disp_w, sh = cam->disp_h;
    int rw, rh;

    switch (cam->birdview_pos) {
    case BIRDVIEW_LEFT:   rw = sh; rh = sw; break;
    case BIRDVIEW_RIGHT:  rw = sh; rh = sw; break;
    case BIRDVIEW_FRONT:  rw = sw; rh = sh; break;
    case BIRDVIEW_REAR:   rw = sw; rh = sh; break;
    default: return NULL;
    }

    int needed = rw * rh * 3;

    static __thread uint8_t *rot_work;
    static __thread int rot_work_size;
    if (rot_work_size < needed) {
        free(rot_work);
        rot_work = (uint8_t *)malloc(needed);
        rot_work_size = needed;
    }

    switch (cam->birdview_pos) {
    case BIRDVIEW_LEFT:
        bv_rotate_90_ccw(cam->work_color, sw, sh, rot_work);
        break;
    case BIRDVIEW_RIGHT:
        bv_rotate_90_cw(cam->work_color, sw, sh, rot_work);
        break;
    case BIRDVIEW_FRONT:
        bv_rotate_180(cam->work_color, sw, sh, rot_work);
        break;
    case BIRDVIEW_REAR:
        memcpy(rot_work, cam->work_color, needed);
        break;
    }

    *out_w = rw;
    *out_h = rh;
    return rot_work;
}

/* Thread argument: camera + app pointer for DB access */
typedef struct {
    CameraCtx *cam;
    AppCtx    *app;
} WorkerArg;

/* Forward declaration - defined after camera_free */
static void camera_close_stream(CameraCtx *cam);

static double get_time(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

/* FFmpeg interrupt callback: returns 1 to abort blocking I/O when stopping */
static int interrupt_cb(void *opaque)
{
    CameraCtx *cam = (CameraCtx *)opaque;
    return !cam->running;
}

static AVFrame *get_nv12(CameraCtx *cam, AVFrame *frame)
{
    if (cam->hw_decode && frame->format == AV_PIX_FMT_DRM_PRIME) {
        av_frame_unref(cam->sw_frame);
        if (av_hwframe_transfer_data(cam->sw_frame, frame, 0) < 0)
            return NULL;
        return cam->sw_frame;
    }
    return frame;
}

static void extract_y(const AVFrame *nv12, int w, int h, uint8_t *gray)
{
    const uint8_t *s = nv12->data[0];
    int stride = nv12->linesize[0];
    if (stride == w) {
        memcpy(gray, s, w * h);
    } else {
        for (int y = 0; y < h; y++)
            memcpy(gray + y * w, s + y * stride, w);
    }
}

static void nv12_to_rgb(CameraCtx *cam, const AVFrame *nv12, uint8_t *rgb)
{
    uint8_t *dst[1] = { rgb };
    int stride[1] = { cam->disp_w * 3 };
    sws_scale(cam->sws_color,
              (const uint8_t *const *)nv12->data, nv12->linesize,
              0, cam->src_h, dst, stride);
}

/* ---- Recording helpers: raw HEVC + SQLite keyframe index ---- */

/*
 * File format:
 *   Header: "HEVC" (4 bytes) | version (uint32) | width (uint32) |
 *           height (uint32) | extradata_size (uint32) | extradata (N bytes)
 *   Frames: size (uint32) | flags (uint32, bit0=key) | pts_us (int64) |
 *           payload (size bytes)
 */

static void rec_open_segment(CameraCtx *cam, AppCtx *app)
{
    /* Create recordings directory */
    mkdir("recordings", 0755);
    snprintf(cam->rec_dir, sizeof(cam->rec_dir), "recordings/cam_%d", cam->id);
    mkdir(cam->rec_dir, 0755);

    /* Filename from current time */
    time_t now = time(NULL);
    cam->rec_start_time = now;
    struct tm tm_now;
    localtime_r(&now, &tm_now);
    char fname[512];
    snprintf(fname, sizeof(fname), "%s/rec_%04d%02d%02d_%02d%02d%02d.hevc",
             cam->rec_dir,
             tm_now.tm_year + 1900, tm_now.tm_mon + 1, tm_now.tm_mday,
             tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec);

    FILE *f = fopen(fname, "wb");
    if (!f) {
        fprintf(stderr, "[%s] rec: cannot open %s\n", cam->name, fname);
        return;
    }

    /* Write file header */
    AVCodecParameters *par = cam->fmt_ctx->streams[cam->video_idx]->codecpar;
    uint32_t version = 1;
    uint32_t width = (uint32_t)cam->src_w;
    uint32_t height = (uint32_t)cam->src_h;
    uint32_t extsize = (uint32_t)par->extradata_size;

    fwrite("HEVC", 1, 4, f);
    fwrite(&version, 4, 1, f);
    fwrite(&width, 4, 1, f);
    fwrite(&height, 4, 1, f);
    fwrite(&extsize, 4, 1, f);
    if (extsize > 0)
        fwrite(par->extradata, 1, extsize, f);

    cam->rec_file = f;
    cam->rec_first_pts = AV_NOPTS_VALUE;

    /* Insert DB record */
    char ts[32];
    strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%S", &tm_now);
    cam->rec_id = db_recording_insert(app->db, cam->id, fname, ts,
                                      cam->src_w, cam->src_h);

    printf("[%s] rec: segment opened %s (db_id=%d)\n",
           cam->name, fname, cam->rec_id);
}

static void rec_close_segment(CameraCtx *cam, AppCtx *app)
{
    FILE *f = (FILE *)cam->rec_file;
    if (!f)
        return;

    fflush(f);
    long fsize = ftell(f);
    if (fsize < 0) fsize = 0;
    fclose(f);
    cam->rec_file = NULL;

    /* Update DB record */
    time_t now = time(NULL);
    struct tm tm_now;
    localtime_r(&now, &tm_now);
    char ts[32];
    strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%S", &tm_now);
    db_recording_finish(app->db, cam->rec_id, ts, (int)fsize);

    printf("[%s] rec: segment closed (size=%d, db_id=%d)\n",
           cam->name, (int)fsize, cam->rec_id);
    cam->rec_id = 0;
}

static void rec_write_packet(CameraCtx *cam, AppCtx *app, AVPacket *pkt)
{
    if (!cam->recording_enabled || !cam->recording)
        return;

    int is_key = (pkt->flags & AV_PKT_FLAG_KEY) != 0;

    /* Need keyframe to start first segment */
    if (!cam->rec_file) {
        if (!is_key)
            return;
        rec_open_segment(cam, app);
        if (!cam->rec_file)
            return;
    }

    /* Segment rotation: on keyframe after N seconds */
    if (is_key && cam->rec_start_time > 0) {
        time_t now = time(NULL);
        if (now - cam->rec_start_time >= cam->rec_segment_sec) {
            rec_close_segment(cam, app);
            rec_open_segment(cam, app);
            if (!cam->rec_file)
                return;
        }
    }

    /* Calculate PTS in microseconds relative to segment start */
    AVStream *st = cam->fmt_ctx->streams[cam->video_idx];
    int64_t pts = pkt->pts;
    if (pts == AV_NOPTS_VALUE) pts = pkt->dts;
    if (pts == AV_NOPTS_VALUE) pts = 0;

    if (cam->rec_first_pts == AV_NOPTS_VALUE)
        cam->rec_first_pts = pts;

    int64_t rel_pts = pts - cam->rec_first_pts;
    if (rel_pts < 0) rel_pts = 0;
    int64_t pts_us = av_rescale_q(rel_pts, st->time_base,
                                  (AVRational){1, 1000000});

    FILE *f = (FILE *)cam->rec_file;

    /* Record file offset before writing (for keyframe index) */
    long frame_offset = ftell(f);

    /* Write frame header: size, flags, pts_us */
    uint32_t size = (uint32_t)pkt->size;
    uint32_t flags = is_key ? 1 : 0;
    fwrite(&size, 4, 1, f);
    fwrite(&flags, 4, 1, f);
    fwrite(&pts_us, 8, 1, f);

    /* Write payload */
    fwrite(pkt->data, 1, pkt->size, f);

    /* Index keyframes in SQLite for seeking */
    if (is_key && cam->rec_id > 0)
        db_rec_index_insert(app->db, cam->rec_id, frame_offset, pts_us);
}

/* Check zone-based triggers and insert events */
static void execute_actions(int actions, const char *group_name,
                            const char *cam_name)
{
    if (actions & ACTION_BEEP) {
        printf("\a");
        fflush(stdout);
    }
    if (actions & ACTION_FLASH_LED) {
        /* Non-blocking sysfs LED write (best-effort) */
        FILE *f = fopen("/sys/class/leds/work-led/brightness", "w");
        if (f) { fprintf(f, "1"); fclose(f); }
    }
    if (actions & ACTION_EMAIL) {
        printf("[ACTION] Email trigger: cam=%s group=%s\n",
               cam_name, group_name);
        fflush(stdout);
    }
}

static void check_zone_triggers(CameraCtx *cam, AppCtx *app,
                                float *cell_pcts, int gcols, int grows,
                                float overall_pct)
{
    if (!app->db || cam->id <= 0)
        return;

    double now = get_time();
    int min_blob = cam->min_blob_cells;
    if (min_blob < 1) min_blob = 3;

    /* Full-grid blob detection for triggering */
    int blob_col = 0, blob_row = 0;
    int blob_cells = motion_find_blobs(cell_pcts, gcols, grows,
                                       cam->trigger_pct,
                                       &blob_col, &blob_row);

    /* Load zones for this camera */
    DbZone zones[MAX_GRID * MAX_GRID];
    int nz = db_zone_load_camera(app->db, cam->id, zones,
                                 MAX_GRID * MAX_GRID);

    if (nz == 0) {
        /* No zones: trigger on largest blob in entire grid */
        if (blob_cells >= min_blob) {
            if (now - cam->last_trigger_time[0] >= ZONE_COOLDOWN_SEC) {
                cam->last_trigger_time[0] = now;
                int ev_id = db_event_insert(app->db, cam->id, "default",
                                overall_pct, (double)blob_cells,
                                blob_col, blob_row);
                if (ev_id > 0 && cam->clip_tracker && cam->ring_buf)
                    clip_tracker_event(cam->clip_tracker, cam->ring_buf,
                                       ev_id, now);
                DbActionGroup def_groups[1];
                if (db_action_group_load_all(app->db, def_groups, 1) > 0)
                    execute_actions(def_groups[0].actions, "default",
                                    cam->name);
            }
        }
        return;
    }

    /* With zones: find blobs, check which zone groups they overlap.
     * Build a label map of connected components, then for each group
     * find the largest blob that overlaps any of the group's cells. */

    /* Label map: assign each active cell to a blob ID (1-based) */
    uint8_t visited[MAX_GRID * MAX_GRID];
    int labels[MAX_GRID * MAX_GRID];
    int blob_sizes[MAX_GRID * MAX_GRID / 2 + 1]; /* blob_id → size */
    int stack[MAX_GRID * MAX_GRID];
    int total = gcols * grows;
    memset(visited, 0, total);
    memset(labels, 0, total * sizeof(int));
    int num_blobs = 0;

    static const int dr[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    static const int dc[] = { 0, 0, 1,-1,  1, -1, 1,-1};

    for (int r = 0; r < grows; r++) {
        for (int c = 0; c < gcols; c++) {
            int idx = r * gcols + c;
            if (visited[idx] || cell_pcts[idx] < cam->trigger_pct)
                continue;

            num_blobs++;
            int bid = num_blobs;
            int sp = 0;
            stack[sp++] = idx;
            visited[idx] = 1;
            int size = 0;

            while (sp > 0) {
                int ci = stack[--sp];
                int cr_ = ci / gcols;
                int cc = ci % gcols;
                labels[ci] = bid;
                size++;

                for (int d = 0; d < 8; d++) {
                    int nr = cr_ + dr[d];
                    int nc = cc + dc[d];
                    if (nr < 0 || nr >= grows || nc < 0 || nc >= gcols)
                        continue;
                    int ni = nr * gcols + nc;
                    if (!visited[ni] && cell_pcts[ni] >= cam->trigger_pct) {
                        visited[ni] = 1;
                        stack[sp++] = ni;
                    }
                }
            }
            blob_sizes[bid] = size;
        }
    }

    /* For each action group, find largest blob overlapping its zone cells */
    DbActionGroup groups[MAX_ACTION_GROUPS];
    int ng = db_action_group_load_all(app->db, groups, MAX_ACTION_GROUPS);

    for (int g = 0; g < ng; g++) {
        /* Track which blob IDs overlap this group's cells */
        int best_blob_size = 0;

        for (int z = 0; z < nz; z++) {
            if (zones[z].action_group_id != groups[g].id)
                continue;
            int c = zones[z].cell_col;
            int r = zones[z].cell_row;
            if (c < 0 || c >= gcols || r < 0 || r >= grows)
                continue;
            int lid = labels[r * gcols + c];
            if (lid > 0 && blob_sizes[lid] > best_blob_size)
                best_blob_size = blob_sizes[lid];
        }

        if (best_blob_size >= min_blob) {
            int gidx = groups[g].id;
            if (gidx >= 0 && gidx < MAX_ACTION_GROUPS) {
                if (now - cam->last_trigger_time[gidx] >= ZONE_COOLDOWN_SEC) {
                    cam->last_trigger_time[gidx] = now;
                    int ev_id = db_event_insert(app->db, cam->id,
                                    groups[g].name,
                                    overall_pct, (double)best_blob_size,
                                    blob_col, blob_row);
                    if (ev_id > 0 && cam->clip_tracker && cam->ring_buf)
                        clip_tracker_event(cam->clip_tracker, cam->ring_buf,
                                           ev_id, now);
                    execute_actions(groups[g].actions, groups[g].name,
                                    cam->name);
                }
            }
        }
    }
}

/* Load lens undistortion params from DB and build LUT */
static void camera_load_undistort(CameraCtx *cam, AppCtx *app)
{
    DbLensParams lp;
    if (db_lens_params_load(app->db, cam->id, &lp) != 0)
        return;

    if (lp.k1 == 0.0 && lp.k2 == 0.0)
        return;

    LensParams p = {
        .k1 = lp.k1, .k2 = lp.k2,
        .cx_frac = lp.cx_frac, .cy_frac = lp.cy_frac
    };

    if (cam->undistort_lut.entries)
        warp_lut_free(&cam->undistort_lut);

    if (undistort_lut_build(&p, cam->disp_w, cam->disp_h,
                            &cam->undistort_lut) == 0) {
        cam->undistort_valid = 1;
        printf("[%s] Undistort LUT loaded: k1=%.4f k2=%.4f\n",
               cam->name, lp.k1, lp.k2);
    }
}

static void process_frame(CameraCtx *cam, AppCtx *app, AVFrame *frame)
{
    cam->frame_count++;

    /* FPS tracking (every frame) */
    double now = get_time();
    if (cam->last_time > 0) {
        double dt = now - cam->last_time;
        cam->fps = cam->fps * 0.9f + (float)(1.0 / dt) * 0.1f;
    }
    cam->last_time = now;

    /* NV12 transfer (every frame for responsive display) */
    AVFrame *nv12 = get_nv12(cam, frame);
    if (!nv12)
        return;

    /* Color preview (every frame) */
    nv12_to_rgb(cam, nv12, cam->work_color);

    /* Apply lens undistortion if calibrated */
    if (cam->undistort_valid && cam->work_undistorted) {
        warp_lut_apply(&cam->undistort_lut, cam->work_color,
                        cam->work_undistorted);
        memcpy(cam->work_color, cam->work_undistorted, cam->disp_size);
    }

    int do_motion = (cam->frame_count % cam->framestep == 0);

    /* Motion detection pipeline (at framestep rate) */
    int motion_pixels = 0;
    float motion_pct = 0.0f;
    float grid_max_pct = 0.0f;
    int grid_max_col = 0, grid_max_row = 0;
    int blob_cells = 0, blob_col = 0, blob_row = 0;
    float cell_pcts[MAX_GRID * MAX_GRID];
    memset(cell_pcts, 0, sizeof(cell_pcts));

    int gcols = cam->grid_cols;
    int grows = cam->grid_rows;
    if (gcols < 1) gcols = 1;
    if (gcols > MAX_GRID) gcols = MAX_GRID;
    if (grows < 1) grows = 1;
    if (grows > MAX_GRID) grows = MAX_GRID;

    if (do_motion) {
        extract_y(nv12, cam->src_w, cam->src_h, cam->curr_gray);
        motion_blur_3x3(cam->curr_gray, cam->blurred, cam->src_w, cam->src_h);

        int thresh = cam->threshold;
        float alpha = cam->alpha;

        if (!cam->bg_initialized) {
            motion_bg_init(cam->bg_model, cam->blurred, cam->gray_size);
            cam->bg_initialized = 1;
            memset(cam->work_motion, 0, cam->disp_size);
        } else {
            grid_max_pct = motion_bg_diff_grid(
                cam->blurred, cam->bg_model,
                cam->src_w, cam->src_h, thresh,
                gcols, grows,
                cell_pcts, &motion_pixels,
                &grid_max_col, &grid_max_row);
            motion_pct = 100.0f * motion_pixels / cam->gray_size;

            blob_cells = motion_find_blobs(cell_pcts, gcols, grows,
                                           cam->trigger_pct,
                                           &blob_col, &blob_row);

            motion_visualize(cam->blurred, cam->bg_model,
                             cam->src_w, cam->src_h, thresh,
                             cam->disp_w, cam->disp_h, cam->work_motion);

            check_zone_triggers(cam, app, cell_pcts, gcols, grows, motion_pct);

            if (cam->clip_tracker && cam->ring_buf) {
                ClipJob *clip_job = NULL;
                if (clip_tracker_tick(cam->clip_tracker, cam->ring_buf, now,
                                       &clip_job, cam->id, cam->name, app->db))
                    clip_transcoder_submit(app->clip_transcoder, clip_job);
            }
        }

        motion_bg_update(cam->bg_model, cam->blurred, cam->gray_size, alpha);
    }

    /* Pre-rotate for birdview (worker-local, no mutex needed) */
    const uint8_t *bv_data = NULL;
    int bv_rw = 0, bv_rh = 0;
    if (app->birdview_visible)
        bv_data = bv_prerotate(cam, &bv_rw, &bv_rh);

    /* Copy to shared buffers */
    pthread_mutex_lock(&cam->buf_mutex);
    if (bv_data) {
        int bv_size = bv_rw * bv_rh * 3;
        if (!cam->bv_rotated || cam->bv_rot_w != bv_rw || cam->bv_rot_h != bv_rh) {
            free(cam->bv_rotated);
            cam->bv_rotated = (uint8_t *)malloc(bv_size);
            cam->bv_rot_w = bv_rw;
            cam->bv_rot_h = bv_rh;
        }
        memcpy(cam->bv_rotated, bv_data, bv_size);
    }
    memcpy(cam->color_rgb, cam->work_color, cam->disp_size);
    if (do_motion) {
        memcpy(cam->motion_rgb, cam->work_motion, cam->disp_size);
        cam->shared_motion_pct = motion_pct;
        cam->shared_motion_pixels = motion_pixels;
        cam->shared_grid_cols = gcols;
        cam->shared_grid_rows = grows;
        cam->shared_grid_max_pct = grid_max_pct;
        cam->shared_grid_max_col = grid_max_col;
        cam->shared_grid_max_row = grid_max_row;
        cam->shared_blob_cells = blob_cells;
        cam->shared_blob_col = blob_col;
        cam->shared_blob_row = blob_row;
        memcpy(cam->shared_grid_pcts, cell_pcts,
               gcols * grows * sizeof(float));
    }
    cam->shared_fps = cam->fps;
    cam->buf_updated = 1;
    pthread_mutex_unlock(&cam->buf_mutex);
}

static void *worker_thread(void *arg)
{
    WorkerArg *wa = (WorkerArg *)arg;
    CameraCtx *cam = wa->cam;
    AppCtx *app = wa->app;
    free(wa);

    printf("[%s] Worker thread started\n", cam->name);
    fflush(stdout);

    /* Outer loop: connect → decode → reconnect on stream loss */
    while (cam->running) {

        /* Connect with retry (non-blocking to UI) */
        if (!cam->fmt_ctx) {
            while (cam->running) {
                cam->connect_state = CAM_CONNECTING;
                printf("[%s] Connecting to %s\n", cam->name, cam->url);
                fflush(stdout);
                if (camera_init(cam) == 0)
                    break;
                cam->connect_state = CAM_FAILED;
                fprintf(stderr, "[%s] Connection failed, retrying in 5s\n",
                        cam->name);
                fflush(stderr);
                /* Sleep 5s, checking running every 100ms */
                for (int w = 0; w < 50 && cam->running; w++)
                    usleep(100000);
            }
            if (!cam->running)
                break;
        }

        cam->connect_state = CAM_CONNECTED;

        /* Load lens undistortion if calibrated */
        camera_load_undistort(cam, app);

        /* Decode loop */
        AVPacket *pkt = av_packet_alloc();
        AVFrame *frame = av_frame_alloc();
        int read_errors = 0;

        while (cam->running) {
            int ret = av_read_frame(cam->fmt_ctx, pkt);
            if (ret < 0) {
                read_errors++;
                if (ret == AVERROR_EOF || read_errors > 50) {
                    fprintf(stderr, "[%s] Stream ended (errors=%d)\n",
                            cam->name, read_errors);
                    break;
                }
                usleep(10000);
                continue;
            }
            read_errors = 0;

            if (pkt->stream_index != cam->video_idx) {
                av_packet_unref(pkt);
                continue;
            }

            /* Passthrough recording (before decode) */
            rec_write_packet(cam, app, pkt);

            /* Push to clip ring buffer */
            if (cam->ring_buf)
                ring_buf_push(cam->ring_buf, pkt, get_time());

            ret = avcodec_send_packet(cam->dec_ctx, pkt);
            av_packet_unref(pkt);
            if (ret < 0)
                continue;

            while (ret >= 0 && cam->running) {
                ret = avcodec_receive_frame(cam->dec_ctx, frame);
                if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                    break;
                if (ret < 0)
                    break;
                process_frame(cam, app, frame);
                av_frame_unref(frame);
            }
        }

        /* Close recording segment if open */
        if (cam->rec_file)
            rec_close_segment(cam, app);

        av_packet_free(&pkt);
        av_frame_free(&frame);

        if (!cam->running)
            break;

        /* Stream lost - close FFmpeg, keep display buffers, reconnect */
        printf("[%s] Stream lost, reconnecting in 3s...\n", cam->name);
        fflush(stdout);
        camera_close_stream(cam);
        cam->connect_state = CAM_FAILED;

        for (int w = 0; w < 30 && cam->running; w++)
            usleep(100000);
    }

    cam->connect_state = CAM_IDLE;
    cam->running = 0;
    printf("[%s] Worker thread exiting\n", cam->name);
    fflush(stdout);
    return NULL;
}

int camera_init(CameraCtx *cam)
{
    if (!cam->url[0]) {
        fprintf(stderr, "[%s] No URL configured\n", cam->name);
        return -1;
    }

    /* RTSP options */
    AVDictionary *opts = NULL;
    av_dict_set(&opts, "rtsp_transport", "tcp", 0);
    av_dict_set(&opts, "stimeout", "5000000", 0);

    /* Pre-allocate context with interrupt callback for cancellation */
    cam->fmt_ctx = avformat_alloc_context();
    cam->fmt_ctx->interrupt_callback.callback = interrupt_cb;
    cam->fmt_ctx->interrupt_callback.opaque = cam;

    int ret = avformat_open_input(&cam->fmt_ctx, cam->url, NULL, &opts);
    av_dict_free(&opts);
    if (ret < 0) {
        fprintf(stderr, "[%s] Cannot open %s\n", cam->name, cam->url);
        return -1;
    }

    if (avformat_find_stream_info(cam->fmt_ctx, NULL) < 0) {
        avformat_close_input(&cam->fmt_ctx);
        return -1;
    }

    /* Find video stream */
    cam->video_idx = -1;
    for (unsigned i = 0; i < cam->fmt_ctx->nb_streams; i++) {
        if (cam->fmt_ctx->streams[i]->codecpar->codec_type ==
            AVMEDIA_TYPE_VIDEO) {
            cam->video_idx = (int)i;
            break;
        }
    }
    if (cam->video_idx < 0) {
        avformat_close_input(&cam->fmt_ctx);
        return -1;
    }

    AVCodecParameters *par =
        cam->fmt_ctx->streams[cam->video_idx]->codecpar;
    const AVCodec *codec = NULL;

    /* Try hardware decode first */
    if (!cam->force_sw) {
        codec = avcodec_find_decoder_by_name("hevc_rkmpp");
        if (codec)
            cam->hw_decode = 1;
    }

    if (!codec) {
        codec = avcodec_find_decoder(par->codec_id);
        if (!codec) {
            avformat_close_input(&cam->fmt_ctx);
            return -1;
        }
        cam->hw_decode = 0;
    }

    cam->dec_ctx = avcodec_alloc_context3(codec);
    avcodec_parameters_to_context(cam->dec_ctx, par);

    if (cam->hw_decode) {
        ret = av_hwdevice_ctx_create(&cam->hw_device_ctx,
                                     AV_HWDEVICE_TYPE_RKMPP,
                                     NULL, NULL, 0);
        if (ret < 0) {
            avcodec_free_context(&cam->dec_ctx);
            codec = avcodec_find_decoder(par->codec_id);
            if (!codec) {
                avformat_close_input(&cam->fmt_ctx);
                return -1;
            }
            cam->dec_ctx = avcodec_alloc_context3(codec);
            avcodec_parameters_to_context(cam->dec_ctx, par);
            cam->hw_decode = 0;
        } else {
            cam->dec_ctx->hw_device_ctx = av_buffer_ref(cam->hw_device_ctx);
        }
    }

    if (!cam->hw_decode)
        cam->dec_ctx->thread_count = 4;

    if (avcodec_open2(cam->dec_ctx, codec, NULL) < 0) {
        avcodec_free_context(&cam->dec_ctx);
        avformat_close_input(&cam->fmt_ctx);
        return -1;
    }

    cam->src_w = cam->dec_ctx->width;
    cam->src_h = cam->dec_ctx->height;
    cam->gray_size = cam->src_w * cam->src_h;

    /* Display resolution: aspect ratio + scale */
    int old_disp_size = cam->disp_size;
    int ar_w = cam->aspect_w;
    int ar_h = cam->aspect_h;
    if (ar_w <= 0 || ar_h <= 0) {
        ar_w = cam->src_w;
        ar_h = cam->src_h;
    }
    double sc = cam->scale;
    if (sc < 0.1) sc = 1.0;
    if (sc > 4.0) sc = 4.0;
    int new_disp_w = (int)(cam->src_w * sc);
    int new_disp_h = new_disp_w * ar_h / ar_w;
    if (new_disp_w < 64) new_disp_w = 64;
    if (new_disp_h < 64) new_disp_h = 64;
    if (new_disp_w > cam->src_w * 2) new_disp_w = cam->src_w * 2;
    if (new_disp_h > cam->src_h * 2) new_disp_h = cam->src_h * 2;
    cam->disp_w = new_disp_w;
    cam->disp_h = new_disp_h;
    cam->disp_size = cam->disp_w * cam->disp_h * 3;
    cam->decoder_name = codec->name;

    /* Update ring buffer stream params for clip system */
    if (cam->ring_buf)
        ring_buf_set_stream_params(cam->ring_buf, par,
                                    cam->fmt_ctx->streams[cam->video_idx]->time_base);

    printf("[%s] Stream: %dx%d -> display %dx%d (%s, %s)\n",
           cam->name, cam->src_w, cam->src_h,
           cam->disp_w, cam->disp_h,
           codec->name, cam->hw_decode ? "hardware" : "software");

    if (cam->hw_decode)
        cam->sw_frame = av_frame_alloc();

    /* Motion buffers (worker-only, always reallocated) */
    cam->curr_gray = (uint8_t *)malloc(cam->gray_size);
    cam->blurred   = (uint8_t *)malloc(cam->gray_size);
    cam->bg_model  = (float *)malloc(cam->gray_size * sizeof(float));
    cam->work_color      = (uint8_t *)malloc(cam->disp_size);
    cam->work_motion     = (uint8_t *)malloc(cam->disp_size);
    cam->work_undistorted = (uint8_t *)malloc(cam->disp_size);

    /* SWS: NV12 → RGB24 at display resolution */
    cam->sws_color = sws_getContext(
        cam->src_w, cam->src_h, AV_PIX_FMT_NV12,
        cam->disp_w, cam->disp_h, AV_PIX_FMT_RGB24,
        SWS_FAST_BILINEAR, NULL, NULL, NULL);

    /* Display/shared buffers: reuse on reconnect if dimensions match */
    if (!cam->color_rgb || cam->disp_size != old_disp_size) {
        if (cam->color_rgb) {
            /* Dimensions changed - signal draw callback, wait, then free */
            pthread_mutex_lock(&cam->buf_mutex);
            cam->disp_w = 0;
            cam->disp_h = 0;
            cam->buf_updated = 0;
            pthread_mutex_unlock(&cam->buf_mutex);
            usleep(50000);
            free(cam->color_rgb);
            free(cam->motion_rgb);
            free(cam->draw_color);
            free(cam->draw_motion);
        }
        cam->color_rgb   = (uint8_t *)calloc(cam->disp_size, 1);
        cam->motion_rgb  = (uint8_t *)calloc(cam->disp_size, 1);
        cam->draw_color  = (uint8_t *)malloc(cam->disp_size);
        cam->draw_motion = (uint8_t *)malloc(cam->disp_size);
        memset(cam->draw_color, 0, cam->disp_size);
        memset(cam->draw_motion, 0, cam->disp_size);
        /* Restore actual dimensions */
        cam->disp_w = new_disp_w;
        cam->disp_h = new_disp_h;
    }

    cam->bg_initialized = 0;
    cam->frame_count = 0;
    cam->fps = 0;
    cam->last_time = 0;

    return 0;
}

int camera_start(CameraCtx *cam, AppCtx *app)
{
    if (cam->started || !cam->enabled || !cam->url[0])
        return -1;

    /* Allocate clip system buffers (persist across reconnects) */
    if (!cam->ring_buf) {
        cam->ring_buf = calloc(1, sizeof(RingBuf));
        if (cam->ring_buf) ring_buf_init(cam->ring_buf);
    }
    if (!cam->clip_tracker) {
        cam->clip_tracker = calloc(1, sizeof(ClipTracker));
        if (cam->clip_tracker) clip_tracker_init(cam->clip_tracker);
    }

    /* Connection happens asynchronously in the worker thread */
    cam->running = 1;
    cam->connect_state = CAM_CONNECTING;

    WorkerArg *wa = (WorkerArg *)malloc(sizeof(WorkerArg));
    wa->cam = cam;
    wa->app = app;

    if (pthread_create(&cam->thread, NULL, worker_thread, wa) != 0) {
        free(wa);
        cam->running = 0;
        cam->connect_state = CAM_IDLE;
        return -1;
    }

    cam->started = 1;
    return 0;
}

void camera_stop(CameraCtx *cam)
{
    if (!cam->started)
        return;
    cam->running = 0;
    pthread_join(cam->thread, NULL);
    cam->started = 0;
}

/* Close FFmpeg + worker-only buffers. Keeps display/shared buffers for
 * reconnect (GTK draw thread may still be reading them). */
static void camera_close_stream(CameraCtx *cam)
{
    if (cam->sws_color) { sws_freeContext(cam->sws_color); cam->sws_color = NULL; }
    if (cam->sw_frame) { av_frame_free(&cam->sw_frame); cam->sw_frame = NULL; }
    if (cam->hw_device_ctx) { av_buffer_unref(&cam->hw_device_ctx); cam->hw_device_ctx = NULL; }
    if (cam->dec_ctx) { avcodec_free_context(&cam->dec_ctx); cam->dec_ctx = NULL; }
    if (cam->fmt_ctx) { avformat_close_input(&cam->fmt_ctx); cam->fmt_ctx = NULL; }

    free(cam->curr_gray);  cam->curr_gray = NULL;
    free(cam->blurred);    cam->blurred = NULL;
    free(cam->bg_model);   cam->bg_model = NULL;
    free(cam->work_color); cam->work_color = NULL;
    free(cam->work_motion); cam->work_motion = NULL;
    free(cam->work_undistorted); cam->work_undistorted = NULL;
    cam->undistort_valid = 0;
    if (cam->undistort_lut.entries)
        warp_lut_free(&cam->undistort_lut);
    cam->bg_initialized = 0;
}

void camera_free(CameraCtx *cam)
{
    camera_stop(cam);
    camera_close_stream(cam);

    /* Free clip system */
    if (cam->ring_buf) {
        ring_buf_free(cam->ring_buf);
        free(cam->ring_buf);
        cam->ring_buf = NULL;
    }
    if (cam->clip_tracker) {
        free(cam->clip_tracker);
        cam->clip_tracker = NULL;
    }

    free(cam->color_rgb);   cam->color_rgb = NULL;
    free(cam->motion_rgb);  cam->motion_rgb = NULL;
    free(cam->draw_color);  cam->draw_color = NULL;
    free(cam->draw_motion); cam->draw_motion = NULL;
    free(cam->bv_rotated);  cam->bv_rotated = NULL;
    cam->bv_rot_w = 0;
    cam->bv_rot_h = 0;

    cam->disp_w = 0;
    cam->disp_h = 0;
    cam->disp_size = 0;
    cam->connect_state = CAM_IDLE;

    pthread_mutex_destroy(&cam->buf_mutex);
}

void camera_rec_start(CameraCtx *cam, AppCtx *app)
{
    (void)app;
    cam->recording = 1;
    printf("[%s] Recording enabled\n", cam->name);
}

void camera_rec_stop(CameraCtx *cam, AppCtx *app)
{
    (void)app;
    cam->recording = 0;
    printf("[%s] Recording disabled\n", cam->name);
}

int camera_find(AppCtx *app, int db_id)
{
    for (int i = 0; i < app->num_cameras; i++) {
        if (app->cameras[i].id == db_id)
            return i;
    }
    return -1;
}

void camera_start_all(AppCtx *app)
{
    for (int i = 0; i < app->num_cameras; i++) {
        if (app->cameras[i].enabled && !app->cameras[i].started) {
            camera_start(&app->cameras[i], app);
        }
    }
}

void camera_stop_all(AppCtx *app)
{
    for (int i = 0; i < app->num_cameras; i++)
        camera_stop(&app->cameras[i]);
}
