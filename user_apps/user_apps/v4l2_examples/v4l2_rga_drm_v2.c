/*
 * v4l2_rga_drm_v2.c — V4L2 → RGA → DRM zero-copy composite PoC (v2)
 *
 * Multi-camera V4L2 → RGA → DRM zero-copy composite.
 * Per-channel format config via ch_cfg[] — edit to match physical camera placement.
 * RGA rescales all sources to uniform quadrants on the display.
 *
 * Build:  gcc -O2 -o v4l2_rga_drm_v2 v4l2_rga_drm_v2.c \
 *             $(pkg-config --cflags --libs librga libdrm)
 *
 * Usage:
 *   sudo systemctl stop lightdm
 *   sudo ./v4l2_rga_drm_v2 [video_base]    # default 11
 *   sudo systemctl start lightdm
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <poll.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <linux/videodev2.h>

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>

#include <rga/im2d.h>
#include <rga/rga.h>

/* ──────────────────────── Config ──────────────────────── */

#define MAX_CAMS        4
#define V4L2_BUF_CNT    4
#define DRM_CARD        "/dev/dri/card1"

/* NVP6324 sysfs path */
#define NVP_CHANNEL_FMT "/sys/devices/platform/2ac70000.i2c/i2c-4/4-0031/channel_fmt"

/* DV timing indices */
#define DV_IDX_CVBS_NTSC 0
#define DV_IDX_CVBS_PAL  1
#define DV_IDX_1080P25   6

/* Per-channel format: ch2=AHD 1080p25 (camera), ch0/1/3=CVBS NTSC (color bars) */
static const struct { int dv_idx; int width; int height; } ch_cfg[MAX_CAMS] = {
    { DV_IDX_CVBS_NTSC,  720,  480 },   /* ch0: CVBS NTSC (no camera) */
    { DV_IDX_CVBS_NTSC,  720,  480 },   /* ch1: CVBS NTSC (no camera) */
    { DV_IDX_1080P25,   1920, 1080 },   /* ch2: AHD 1080p25 (camera) */
    { DV_IDX_CVBS_NTSC,  720,  480 },   /* ch3: CVBS NTSC (no camera) */
};

/* ──────────────────────── Types ──────────────────────── */

struct cam_buf {
    void                   *start;
    size_t                  length;
    int                     dmabuf_fd;
    rga_buffer_handle_t     rga_handle;
    rga_buffer_t            rga;
};

struct cam {
    int                     fd;
    int                     active;
    char                    path[32];
    uint32_t                buf_type;
    int                     is_mplane;
    int                     width, height;
    int                     src_height;  /* visible lines (height/2 for interlaced CVBS) */
    int                     buf_count;
    struct cam_buf          bufs[V4L2_BUF_CNT];
    im_rect                 quad;
};

struct display {
    int                     fd;
    uint32_t                conn_id, crtc_id;
    drmModeModeInfo         mode;
    drmModeCrtcPtr          saved_crtc;
    uint32_t                fb_id, gem_handle;
    uint32_t                pitch;
    uint64_t                size;
    void                   *map;
    int                     dmabuf_fd;
    rga_buffer_handle_t     rga_handle;
    rga_buffer_t            rga;
};

/* ──────────────────────── Globals ──────────────────────── */

static volatile sig_atomic_t quit = 0;
static void on_signal(int s) { (void)s; quit = 1; }

/* ──────────────────────── NVP6324 + media setup ──────────────────────── */

static int setup_nvp6324(void)
{
    /* Set per-channel formats: ch0=AHD 1080p, ch1-3=CVBS NTSC */
    int fd = open(NVP_CHANNEL_FMT, O_WRONLY);
    if (fd < 0) {
        fprintf(stderr, "NVP6324: cannot open %s: %s\n",
                NVP_CHANNEL_FMT, strerror(errno));
        return -1;
    }

    for (int ch = 0; ch < MAX_CAMS; ch++) {
        char cmd[16];
        int len = snprintf(cmd, sizeof(cmd), "%d %d", ch, ch_cfg[ch].dv_idx);
        if (write(fd, cmd, len) < 0)
            fprintf(stderr, "NVP6324: ch%d set failed: %s\n", ch, strerror(errno));
    }
    close(fd);

    /* Verify */
    fd = open(NVP_CHANNEL_FMT, O_RDONLY);
    if (fd >= 0) {
        char buf[512];
        int n = read(fd, buf, sizeof(buf) - 1);
        if (n > 0) { buf[n] = 0; printf("NVP6324:\n%s", buf); }
        close(fd);
    }

    return 0;
}

static int setup_media_pipeline(void)
{
    /* Find largest resolution across all channels */
    int max_w = 0, max_h = 0;
    for (int i = 0; i < MAX_CAMS; i++) {
        if (ch_cfg[i].width > max_w)  max_w = ch_cfg[i].width;
        if (ch_cfg[i].height > max_h) max_h = ch_cfg[i].height;
    }

    char cmd[3][256];
    snprintf(cmd[0], sizeof(cmd[0]),
        "media-ctl -d /dev/media1 --set-v4l2 "
        "'\"m00_b_jaguar2 4-0031\":0[fmt:UYVY8_2X8/%dx%d field:none]'",
        max_w, max_h);
    snprintf(cmd[1], sizeof(cmd[1]),
        "media-ctl -d /dev/media1 --set-v4l2 "
        "'\"rockchip-csi2-dphy0\":0[fmt:UYVY8_2X8/%dx%d field:none]'",
        max_w, max_h);
    snprintf(cmd[2], sizeof(cmd[2]),
        "media-ctl -d /dev/media1 --set-v4l2 "
        "'\"rockchip-csi2-dphy0\":1[fmt:UYVY8_2X8/%dx%d field:none]'",
        max_w, max_h);

    for (int i = 0; i < 3; i++) {
        int ret = system(cmd[i]);
        if (ret != 0)
            fprintf(stderr, "media-ctl cmd %d returned %d\n", i, ret);
    }

    printf("Media pipeline: %dx%d UYVY (set for largest stream)\n", max_w, max_h);
    return 0;
}

/* ──────────────────────── DRM ──────────────────────── */

static int drm_init(struct display *d)
{
    memset(d, 0, sizeof(*d));
    d->fd = -1;
    d->dmabuf_fd = -1;

    d->fd = open(DRM_CARD, O_RDWR | O_CLOEXEC);
    if (d->fd < 0) {
        fprintf(stderr, "DRM: open %s: %s\n", DRM_CARD, strerror(errno));
        return -1;
    }

    if (drmSetMaster(d->fd) < 0) {
        fprintf(stderr, "DRM: setMaster: %s (stop lightdm first)\n", strerror(errno));
        goto err;
    }

    drmModeResPtr res = drmModeGetResources(d->fd);
    if (!res) { fprintf(stderr, "DRM: no resources\n"); goto err; }

    /* Find HDMI connector */
    drmModeConnectorPtr conn = NULL;
    for (int i = 0; i < res->count_connectors; i++) {
        conn = drmModeGetConnector(d->fd, res->connectors[i]);
        if (conn && conn->connection == DRM_MODE_CONNECTED &&
            (conn->connector_type == DRM_MODE_CONNECTOR_HDMIA ||
             conn->connector_type == DRM_MODE_CONNECTOR_HDMIB))
            break;
        if (conn) { drmModeFreeConnector(conn); conn = NULL; }
    }
    if (!conn) {
        fprintf(stderr, "DRM: no HDMI\n");
        drmModeFreeResources(res);
        goto err;
    }
    d->conn_id = conn->connector_id;

    /* Preferred mode */
    drmModeModeInfo *best = &conn->modes[0];
    for (int i = 0; i < conn->count_modes; i++)
        if (conn->modes[i].type & DRM_MODE_TYPE_PREFERRED)
            { best = &conn->modes[i]; break; }
    d->mode = *best;

    /* Encoder → CRTC */
    d->crtc_id = 0;
    if (conn->encoder_id) {
        drmModeEncoderPtr enc = drmModeGetEncoder(d->fd, conn->encoder_id);
        if (enc) { d->crtc_id = enc->crtc_id; drmModeFreeEncoder(enc); }
    }
    if (!d->crtc_id) {
        for (int e = 0; e < conn->count_encoders && !d->crtc_id; e++) {
            drmModeEncoderPtr enc = drmModeGetEncoder(d->fd, conn->encoders[e]);
            if (!enc) continue;
            for (int c = 0; c < res->count_crtcs; c++)
                if (enc->possible_crtcs & (1u << c))
                    { d->crtc_id = res->crtcs[c]; break; }
            drmModeFreeEncoder(enc);
        }
    }
    drmModeFreeConnector(conn);
    drmModeFreeResources(res);
    if (!d->crtc_id) { fprintf(stderr, "DRM: no CRTC\n"); goto err; }

    d->saved_crtc = drmModeGetCrtc(d->fd, d->crtc_id);

    printf("DRM: conn=%u crtc=%u mode=%ux%u@%uHz\n",
           d->conn_id, d->crtc_id,
           d->mode.hdisplay, d->mode.vdisplay, d->mode.vrefresh);

    /* Dumb buffer */
    struct drm_mode_create_dumb cr = {
        .width  = d->mode.hdisplay,
        .height = d->mode.vdisplay,
        .bpp    = 32,
    };
    if (drmIoctl(d->fd, DRM_IOCTL_MODE_CREATE_DUMB, &cr))
        { perror("DRM create_dumb"); goto err; }
    d->gem_handle = cr.handle;
    d->pitch      = cr.pitch;
    d->size       = cr.size;

    if (drmModeAddFB(d->fd, d->mode.hdisplay, d->mode.vdisplay,
                     24, 32, d->pitch, d->gem_handle, &d->fb_id))
        { perror("DRM addFB"); goto err; }

    /* mmap + clear */
    struct drm_mode_map_dumb mr = { .handle = d->gem_handle };
    if (drmIoctl(d->fd, DRM_IOCTL_MODE_MAP_DUMB, &mr))
        { perror("DRM map_dumb"); goto err; }
    d->map = mmap(NULL, d->size, PROT_READ | PROT_WRITE,
                  MAP_SHARED, d->fd, mr.offset);
    if (d->map == MAP_FAILED) { perror("DRM mmap"); d->map = NULL; goto err; }
    memset(d->map, 0, d->size);

    /* DMA-BUF for RGA */
    if (drmPrimeHandleToFD(d->fd, d->gem_handle,
                           DRM_CLOEXEC | DRM_RDWR, &d->dmabuf_fd))
        { perror("DRM primeHandleToFD"); goto err; }

    /* Activate */
    if (drmModeSetCrtc(d->fd, d->crtc_id, d->fb_id, 0, 0,
                       &d->conn_id, 1, &d->mode))
        { perror("DRM setCrtc"); goto err; }

    printf("DRM: FB %u (%ux%u pitch=%u)\n",
           d->fb_id, d->mode.hdisplay, d->mode.vdisplay, d->pitch);
    return 0;

err:
    if (d->dmabuf_fd >= 0) close(d->dmabuf_fd);
    if (d->map)      munmap(d->map, d->size);
    if (d->fb_id)    drmModeRmFB(d->fd, d->fb_id);
    if (d->gem_handle) {
        struct drm_mode_destroy_dumb dd = { .handle = d->gem_handle };
        drmIoctl(d->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dd);
    }
    if (d->saved_crtc) drmModeFreeCrtc(d->saved_crtc);
    if (d->fd >= 0) { drmDropMaster(d->fd); close(d->fd); d->fd = -1; }
    return -1;
}

static void drm_cleanup(struct display *d)
{
    if (d->fd < 0) return;
    if (d->saved_crtc) {
        drmModeSetCrtc(d->fd, d->saved_crtc->crtc_id,
                       d->saved_crtc->buffer_id,
                       d->saved_crtc->x, d->saved_crtc->y,
                       &d->conn_id, 1, &d->saved_crtc->mode);
        drmModeFreeCrtc(d->saved_crtc);
    }
    if (d->rga_handle) releasebuffer_handle(d->rga_handle);
    if (d->dmabuf_fd >= 0) close(d->dmabuf_fd);
    if (d->map)      munmap(d->map, d->size);
    if (d->fb_id)    drmModeRmFB(d->fd, d->fb_id);
    if (d->gem_handle) {
        struct drm_mode_destroy_dumb dd = { .handle = d->gem_handle };
        drmIoctl(d->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dd);
    }
    drmDropMaster(d->fd);
    close(d->fd);
    d->fd = -1;
}

/* ──────────────────────── V4L2 ──────────────────────── */

static int xioctl(int fd, unsigned long req, void *arg)
{
    int r;
    do { r = ioctl(fd, req, arg); } while (r < 0 && errno == EINTR);
    return r;
}

static int cam_init(struct cam *c, int ch_idx)
{
    for (int i = 0; i < V4L2_BUF_CNT; i++) {
        c->bufs[i].dmabuf_fd = -1;
        c->bufs[i].start = NULL;
        c->bufs[i].rga_handle = 0;
    }

    c->fd = open(c->path, O_RDWR | O_NONBLOCK);
    if (c->fd < 0) return -1;

    struct v4l2_capability cap;
    if (xioctl(c->fd, VIDIOC_QUERYCAP, &cap) < 0) goto err;

    c->is_mplane = !!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE);
    if (!c->is_mplane && !(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
        goto err;
    c->buf_type = c->is_mplane
        ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
        : V4L2_BUF_TYPE_VIDEO_CAPTURE;

    /* Set per-channel format */
    int req_w = ch_cfg[ch_idx].width;
    int req_h = ch_cfg[ch_idx].height;
    struct v4l2_format fmt = { .type = c->buf_type };
    if (c->is_mplane) {
        fmt.fmt.pix_mp.width       = req_w;
        fmt.fmt.pix_mp.height      = req_h;
        fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_UYVY;
        fmt.fmt.pix_mp.num_planes  = 1;
        fmt.fmt.pix_mp.field       = V4L2_FIELD_NONE;
    } else {
        fmt.fmt.pix.width       = req_w;
        fmt.fmt.pix.height      = req_h;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
        fmt.fmt.pix.field       = V4L2_FIELD_NONE;
    }
    if (xioctl(c->fd, VIDIOC_S_FMT, &fmt) < 0) {
        fprintf(stderr, "V4L2: %s S_FMT: %s\n", c->path, strerror(errno));
        goto err;
    }
    c->width  = c->is_mplane ? fmt.fmt.pix_mp.width  : fmt.fmt.pix.width;
    c->height = c->is_mplane ? fmt.fmt.pix_mp.height : fmt.fmt.pix.height;

    if (c->width != req_w || c->height != req_h)
        fprintf(stderr, "V4L2: %s wanted %dx%d, got %dx%d\n",
                c->path, req_w, req_h, c->width, c->height);

    /* CVBS interlaced: CIF delivers single field (half height) in full buffer */
    int dv = ch_cfg[ch_idx].dv_idx;
    if (dv == DV_IDX_CVBS_NTSC || dv == DV_IDX_CVBS_PAL)
        c->src_height = c->height / 2;
    else
        c->src_height = c->height;

    printf("V4L2: %s → %dx%d UYVY (src_h=%d)\n",
           c->path, c->width, c->height, c->src_height);

    /* Request MMAP buffers */
    struct v4l2_requestbuffers req = {
        .count  = V4L2_BUF_CNT,
        .type   = c->buf_type,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (xioctl(c->fd, VIDIOC_REQBUFS, &req) < 0) {
        fprintf(stderr, "V4L2: %s REQBUFS: %s\n", c->path, strerror(errno));
        goto err;
    }
    c->buf_count = req.count;

    for (int i = 0; i < c->buf_count; i++) {
        struct v4l2_buffer vb = {
            .type   = c->buf_type,
            .memory = V4L2_MEMORY_MMAP,
            .index  = i,
        };
        struct v4l2_plane planes[1] = {};
        if (c->is_mplane) { vb.m.planes = planes; vb.length = 1; }

        if (xioctl(c->fd, VIDIOC_QUERYBUF, &vb) < 0) goto err;

        uint32_t off = c->is_mplane ? planes[0].m.mem_offset : vb.m.offset;
        size_t   len = c->is_mplane ? planes[0].length       : vb.length;

        c->bufs[i].start = mmap(NULL, len, PROT_READ | PROT_WRITE,
                                MAP_SHARED, c->fd, off);
        if (c->bufs[i].start == MAP_FAILED) {
            c->bufs[i].start = NULL;
            goto err;
        }
        c->bufs[i].length = len;

        struct v4l2_exportbuffer eb = {
            .type  = c->buf_type,
            .index = i,
            .plane = 0,
            .flags = O_RDWR | O_CLOEXEC,
        };
        if (xioctl(c->fd, VIDIOC_EXPBUF, &eb) < 0) {
            fprintf(stderr, "V4L2: %s EXPBUF %d: %s\n", c->path, i, strerror(errno));
            goto err;
        }
        c->bufs[i].dmabuf_fd = eb.fd;
    }

    /* Queue + stream on */
    for (int i = 0; i < c->buf_count; i++) {
        struct v4l2_buffer vb = {
            .type   = c->buf_type,
            .memory = V4L2_MEMORY_MMAP,
            .index  = i,
        };
        struct v4l2_plane planes[1] = {};
        if (c->is_mplane) { vb.m.planes = planes; vb.length = 1; }
        if (xioctl(c->fd, VIDIOC_QBUF, &vb) < 0) goto err;
    }

    int type_on = c->buf_type;
    if (xioctl(c->fd, VIDIOC_STREAMON, &type_on) < 0) {
        fprintf(stderr, "V4L2: %s STREAMON: %s\n", c->path, strerror(errno));
        goto err;
    }

    c->active = 1;
    return 0;

err:
    for (int i = 0; i < V4L2_BUF_CNT; i++) {
        if (c->bufs[i].dmabuf_fd >= 0) { close(c->bufs[i].dmabuf_fd); c->bufs[i].dmabuf_fd = -1; }
        if (c->bufs[i].start)          { munmap(c->bufs[i].start, c->bufs[i].length); c->bufs[i].start = NULL; }
    }
    if (c->fd >= 0) { close(c->fd); c->fd = -1; }
    return -1;
}

static void cam_cleanup(struct cam *c)
{
    if (!c->active) return;
    int type = c->buf_type;
    xioctl(c->fd, VIDIOC_STREAMOFF, &type);
    for (int i = 0; i < c->buf_count; i++) {
        if (c->bufs[i].rga_handle) releasebuffer_handle(c->bufs[i].rga_handle);
        if (c->bufs[i].dmabuf_fd >= 0) close(c->bufs[i].dmabuf_fd);
        if (c->bufs[i].start) munmap(c->bufs[i].start, c->bufs[i].length);
    }
    close(c->fd);
    c->fd = -1;
    c->active = 0;
}

/* ──────────────────────── RGA ──────────────────────── */

static int rga_setup(struct cam cams[], int ncams, struct display *d)
{
    printf("RGA: %s\n", querystring(RGA_VERSION));

    int w = d->mode.hdisplay;
    int h = d->mode.vdisplay;
    int wstride = d->pitch / 4;

    im_handle_param_t dp = {
        .width  = (uint32_t)wstride,
        .height = (uint32_t)h,
        .format = RK_FORMAT_BGRX_8888,
    };
    d->rga_handle = importbuffer_fd(d->dmabuf_fd, &dp);
    if (!d->rga_handle) {
        fprintf(stderr, "RGA: import composite failed\n");
        return -1;
    }
    d->rga = wrapbuffer_handle(d->rga_handle, w, h,
                               RK_FORMAT_BGRX_8888, wstride, h);
    printf("RGA: composite %dx%d stride=%d\n", w, h, wstride);

    int qw = w / 2, qh = h / 2;
    int pos[4][2] = { {0,0}, {qw,0}, {0,qh}, {qw,qh} };

    for (int i = 0; i < ncams; i++) {
        struct cam *c = &cams[i];
        if (!c->active) continue;

        c->quad = (im_rect){ pos[i][0], pos[i][1], qw, qh };
        printf("RGA: cam%d %dx%d → quad (%d,%d)+%dx%d\n",
               i, c->width, c->height,
               c->quad.x, c->quad.y, qw, qh);

        im_handle_param_t sp = {
            .width  = (uint32_t)c->width,
            .height = (uint32_t)c->height,
            .format = RK_FORMAT_UYVY_422,
        };
        for (int b = 0; b < c->buf_count; b++) {
            c->bufs[b].rga_handle = importbuffer_fd(c->bufs[b].dmabuf_fd, &sp);
            if (!c->bufs[b].rga_handle) {
                fprintf(stderr, "RGA: import cam%d buf%d failed\n", i, b);
                return -1;
            }
            c->bufs[b].rga = wrapbuffer_handle(c->bufs[b].rga_handle,
                                                c->width, c->height,
                                                RK_FORMAT_UYVY_422);
            c->bufs[b].rga.color_space_mode = IM_YUV_TO_RGB_BT601_LIMIT;
        }
    }
    return 0;
}

/* ──────────────────────── Main ──────────────────────── */

int main(int argc, char **argv)
{
    int video_base = 11;
    if (argc > 1) video_base = atoi(argv[1]);

    /* Line-buffered stdout for log files */
    setvbuf(stdout, NULL, _IOLBF, 0);

    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    /* ── NVP6324 + media pipeline ── */
    printf("=== Configuring NVP6324: ch2=AHD 1080p25, others=CVBS NTSC ===\n");
    if (setup_nvp6324() < 0)
        return 1;

    printf("=== Configuring media pipeline ===\n");
    setup_media_pipeline();

    /* Brief settle time for NVP6324 reconfiguration */
    usleep(500000);

    /* ── DRM ── */
    struct display disp;
    if (drm_init(&disp) < 0)
        return 1;

    /* ── Cameras ── */
    struct cam cams[MAX_CAMS];
    int active = 0;
    for (int i = 0; i < MAX_CAMS; i++) {
        memset(&cams[i], 0, sizeof(cams[i]));
        cams[i].fd = -1;
        snprintf(cams[i].path, sizeof(cams[i].path),
                 "/dev/video%d", video_base + i);
        if (cam_init(&cams[i], i) == 0)
            active++;
        else
            fprintf(stderr, "Skip %s\n", cams[i].path);
    }
    if (!active) {
        fprintf(stderr, "No cameras\n");
        drm_cleanup(&disp);
        return 1;
    }
    printf("%d camera(s) active\n", active);

    /* ── RGA ── */
    if (rga_setup(cams, MAX_CAMS, &disp) < 0) {
        for (int i = 0; i < MAX_CAMS; i++) cam_cleanup(&cams[i]);
        drm_cleanup(&disp);
        return 1;
    }

    rga_buffer_t pat_empty;
    memset(&pat_empty, 0, sizeof(pat_empty));
    im_rect prect_empty = {0, 0, 0, 0};

    struct timespec t0;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    unsigned long frames = 0;
    unsigned long per_cam[MAX_CAMS] = {0};

    /* ── Main loop ── */
    printf("Running... Ctrl+C to stop\n");
    while (!quit) {
        struct pollfd pfds[MAX_CAMS];
        int pfd_to_cam[MAX_CAMS];
        int nfds = 0;
        for (int i = 0; i < MAX_CAMS; i++) {
            if (!cams[i].active) continue;
            pfds[nfds].fd      = cams[i].fd;
            pfds[nfds].events  = POLLIN;
            pfds[nfds].revents = 0;
            pfd_to_cam[nfds]   = i;
            nfds++;
        }

        int ret = poll(pfds, nfds, 100);
        if (ret < 0) {
            if (errno == EINTR) continue;
            perror("poll");
            break;
        }
        if (ret == 0) continue;

        for (int p = 0; p < nfds; p++) {
            if (!(pfds[p].revents & POLLIN)) continue;
            int ci = pfd_to_cam[p];
            struct cam *c = &cams[ci];

            struct v4l2_buffer vb = {
                .type   = c->buf_type,
                .memory = V4L2_MEMORY_MMAP,
            };
            struct v4l2_plane planes[1] = {};
            if (c->is_mplane) { vb.m.planes = planes; vb.length = 1; }

            if (xioctl(c->fd, VIDIOC_DQBUF, &vb) < 0) {
                if (errno == EAGAIN) continue;
                fprintf(stderr, "DQBUF cam%d: %s\n", ci, strerror(errno));
                continue;
            }

            int bi = vb.index;
            im_rect src_rect = { 0, 0, c->width, c->src_height };

            IM_STATUS st = improcess(c->bufs[bi].rga, disp.rga, pat_empty,
                                     src_rect, c->quad, prect_empty, IM_SYNC);
            if (st != IM_STATUS_SUCCESS && st != IM_STATUS_NOERROR)
                fprintf(stderr, "RGA cam%d: %s (%d)\n",
                        ci, imStrError_t(st), st);

            if (xioctl(c->fd, VIDIOC_QBUF, &vb) < 0)
                fprintf(stderr, "QBUF cam%d: %s\n", ci, strerror(errno));

            frames++;
            per_cam[ci]++;
        }

        /* FPS report every 5s */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        double elapsed = (now.tv_sec - t0.tv_sec)
                       + (now.tv_nsec - t0.tv_nsec) / 1e9;
        if (elapsed >= 5.0) {
            printf("%.1f fps total [", frames / elapsed);
            for (int i = 0; i < MAX_CAMS; i++) {
                if (i) printf(" ");
                printf("cam%d:%.1f", i,
                       cams[i].active ? per_cam[i] / elapsed : 0.0);
                per_cam[i] = 0;
            }
            printf("] (%lu frames / %.1fs)\n", frames, elapsed);
            t0 = now;
            frames = 0;
        }
    }

    printf("\nShutting down...\n");
    for (int i = 0; i < MAX_CAMS; i++)
        cam_cleanup(&cams[i]);
    drm_cleanup(&disp);
    printf("Done.\n");
    return 0;
}
