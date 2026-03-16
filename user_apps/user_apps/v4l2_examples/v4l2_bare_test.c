/* V4L2 + DRM + RGA + GPU test — find what kills camera */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <poll.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <stdint.h>
#include <linux/videodev2.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <rga/im2d.h>
#include <rga/rga.h>
#include <gbm.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <drm_fourcc.h>

static volatile int quit = 0;
static void sig(int s) { quit = 1; }

static int xioctl(int fd, unsigned long req, void *arg) {
    int r;
    do { r = ioctl(fd, req, arg); } while (r < 0 && errno == EINTR);
    return r;
}

#define NBUF 4

int main(int argc, char **argv)
{
    const char *dev = argc > 1 ? argv[1] : "/dev/video13";
    int w = 1920, h = 1080;
    int do_gpu = getenv("TEST_GPU") != NULL;

    signal(SIGINT, sig);
    signal(SIGTERM, sig);
    setvbuf(stdout, NULL, _IOLBF, 0);

    /* ── NVP6324 + media setup (same as quad tool) ── */
    if (getenv("TEST_NVP")) {
        printf("NVP: configuring...\n");
        system("v4l2-ctl -d /dev/v4l-subdev2 --set-dv-bt-timings index=6");
        system("echo '0 6' > /sys/bus/i2c/devices/4-0031/channel_fmt");
        system("echo '1 6' > /sys/bus/i2c/devices/4-0031/channel_fmt");
        system("echo '2 6' > /sys/bus/i2c/devices/4-0031/channel_fmt");
        system("echo '3 6' > /sys/bus/i2c/devices/4-0031/channel_fmt");
        system("media-ctl -d /dev/media1 --set-v4l2 "
               "'\"m00_b_jaguar2 4-0031\":0[fmt:UYVY8_2X8/1920x1080 field:none]'");
        system("media-ctl -d /dev/media1 --set-v4l2 "
               "'\"rockchip-csi2-dphy0\":0[fmt:UYVY8_2X8/1920x1080 field:none]'");
        system("media-ctl -d /dev/media1 --set-v4l2 "
               "'\"rockchip-csi2-dphy0\":1[fmt:UYVY8_2X8/1920x1080 field:none]'");
        usleep(500000);
        printf("NVP: done\n");
    }

    /* ── DRM ── */
    int drm_fd = open("/dev/dri/card1", O_RDWR | O_CLOEXEC);
    if (drm_fd < 0) { perror("DRM"); return 1; }
    drmSetMaster(drm_fd);

    drmModeResPtr res = drmModeGetResources(drm_fd);
    drmModeConnectorPtr conn = NULL;
    for (int i = 0; i < res->count_connectors; i++) {
        conn = drmModeGetConnector(drm_fd, res->connectors[i]);
        if (conn && conn->connection == DRM_MODE_CONNECTED) break;
        if (conn) { drmModeFreeConnector(conn); conn = NULL; }
    }
    drmModeModeInfo mode = conn->modes[0];
    drmModeEncoderPtr enc = drmModeGetEncoder(drm_fd, conn->encoders[0]);
    uint32_t crtc_id = enc->crtc_id;
    drmModeFreeEncoder(enc);
    int dw = mode.hdisplay, dh = mode.vdisplay;

    uint32_t fb_id[2], gem[2];
    int drm_dmabuf[2];
    for (int b = 0; b < 2; b++) {
        struct drm_mode_create_dumb cr = { .width = dw, .height = dh, .bpp = 32 };
        drmIoctl(drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &cr);
        gem[b] = cr.handle;
        drmModeAddFB(drm_fd, dw, dh, 24, 32, cr.pitch, cr.handle, &fb_id[b]);
        drmPrimeHandleToFD(drm_fd, cr.handle, DRM_CLOEXEC | DRM_RDWR, &drm_dmabuf[b]);
    }
    uint32_t conn_id = conn->connector_id;
    drmModeSetCrtc(drm_fd, crtc_id, fb_id[0], 0, 0, &conn_id, 1, &mode);
    drmModeFreeConnector(conn);
    drmModeFreeResources(res);
    printf("DRM: %dx%d@%d\n", dw, dh, mode.vrefresh);

    /* ── V4L2 ── */
    int fd = open(dev, O_RDWR | O_NONBLOCK);
    struct v4l2_capability cap;
    xioctl(fd, VIDIOC_QUERYCAP, &cap);
    int mp = !!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE);
    int type = mp ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_CAPTURE;
    struct v4l2_format fmt = { .type = type };
    if (mp) {
        fmt.fmt.pix_mp.width = w; fmt.fmt.pix_mp.height = h;
        fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_UYVY;
        fmt.fmt.pix_mp.num_planes = 1; fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
    }
    xioctl(fd, VIDIOC_S_FMT, &fmt);
    struct v4l2_requestbuffers req = { .count = NBUF, .type = type, .memory = V4L2_MEMORY_MMAP };
    xioctl(fd, VIDIOC_REQBUFS, &req);

    int dmabuf_fds[NBUF];
    for (int i = 0; i < NBUF; i++) {
        struct v4l2_buffer vb = { .type = type, .memory = V4L2_MEMORY_MMAP, .index = i };
        struct v4l2_plane planes[1] = {};
        if (mp) { vb.m.planes = planes; vb.length = 1; }
        xioctl(fd, VIDIOC_QUERYBUF, &vb);
        uint32_t off = mp ? planes[0].m.mem_offset : vb.m.offset;
        size_t len = mp ? planes[0].length : vb.length;
        mmap(NULL, len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, off);
        struct v4l2_exportbuffer eb = { .type = type, .index = i, .plane = 0, .flags = O_RDWR|O_CLOEXEC };
        xioctl(fd, VIDIOC_EXPBUF, &eb);
        dmabuf_fds[i] = eb.fd;
        xioctl(fd, VIDIOC_QBUF, &vb);
    }
    xioctl(fd, VIDIOC_STREAMON, &type);
    printf("V4L2: streaming %dx%d\n", w, h);

    /* ── RGA ── */
    im_handle_param_t dp = { .width = (uint32_t)(dw), .height = (uint32_t)dh, .format = RK_FORMAT_BGRX_8888 };
    rga_buffer_t drm_rga[2];
    for (int b = 0; b < 2; b++) {
        int rh = importbuffer_fd(drm_dmabuf[b], &dp);
        drm_rga[b] = wrapbuffer_handle(rh, dw, dh, RK_FORMAT_BGRX_8888, dw, dh);
    }
    im_handle_param_t sp = { .width = (uint32_t)w, .height = (uint32_t)h, .format = RK_FORMAT_UYVY_422 };
    rga_buffer_t cam_rga[NBUF];
    for (int i = 0; i < NBUF; i++) {
        int rh = importbuffer_fd(dmabuf_fds[i], &sp);
        cam_rga[i] = wrapbuffer_handle(rh, w, h, RK_FORMAT_UYVY_422);
        cam_rga[i].color_space_mode = IM_YUV_TO_RGB_BT601_LIMIT;
    }
    printf("RGA: imported\n");

    /* ── GPU (optional) ── */
    if (do_gpu) {
        printf("GPU: initializing EGL/GLES...\n");
        struct gbm_device *gbm = gbm_create_device(drm_fd);
        PFNEGLGETPLATFORMDISPLAYEXTPROC GetPlatformDisplay =
            (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");
        EGLDisplay egl_dpy;
        if (GetPlatformDisplay)
            egl_dpy = GetPlatformDisplay(EGL_PLATFORM_GBM_KHR, gbm, NULL);
        else
            egl_dpy = eglGetDisplay((EGLNativeDisplayType)gbm);
        eglInitialize(egl_dpy, NULL, NULL);
        printf("GPU: EGL %s\n", eglQueryString(egl_dpy, EGL_VERSION));

        static const EGLint cfg_attr[] = {
            EGL_SURFACE_TYPE, 0,
            EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
            EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8,
            EGL_NONE
        };
        EGLConfig cfg;
        EGLint ncfg;
        eglChooseConfig(egl_dpy, cfg_attr, &cfg, 1, &ncfg);

        static const EGLint ctx_attr[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE };
        EGLContext ctx = eglCreateContext(egl_dpy, cfg, EGL_NO_CONTEXT, ctx_attr);
        eglMakeCurrent(egl_dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, ctx);

        printf("GPU: %s\n", glGetString(GL_RENDERER));

        /* Create FBO backed by DRM dumb buffer dmabuf */
        PFNEGLCREATEIMAGEKHRPROC eglCreateImageKHR =
            (PFNEGLCREATEIMAGEKHRPROC)eglGetProcAddress("eglCreateImageKHR");
        PFNGLEGLIMAGETARGETTEXTURE2DOESPROC glEGLImageTargetTexture2DOES =
            (PFNGLEGLIMAGETARGETTEXTURE2DOESPROC)eglGetProcAddress("glEGLImageTargetTexture2DOES");

        GLuint tex[2], fbo[2];
        glGenTextures(2, tex);
        glGenFramebuffers(2, fbo);
        for (int b = 0; b < 2; b++) {
            EGLint img_attr[] = {
                EGL_WIDTH, dw, EGL_HEIGHT, dh,
                EGL_LINUX_DRM_FOURCC_EXT, DRM_FORMAT_XRGB8888,
                EGL_DMA_BUF_PLANE0_FD_EXT, drm_dmabuf[b],
                EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0,
                EGL_DMA_BUF_PLANE0_PITCH_EXT, dw * 4,
                EGL_NONE
            };
            EGLImageKHR img = eglCreateImageKHR(egl_dpy, EGL_NO_CONTEXT,
                                                 EGL_LINUX_DMA_BUF_EXT, NULL, img_attr);
            glBindTexture(GL_TEXTURE_2D, tex[b]);
            glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, img);
            glBindFramebuffer(GL_FRAMEBUFFER, fbo[b]);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex[b], 0);
            GLenum st = glCheckFramebufferStatus(GL_FRAMEBUFFER);
            printf("GPU: FBO%d status=%x\n", b, st);
        }

        /* Compile a simple shader */
        const char *vs = "attribute vec2 a_pos; void main() { gl_Position = vec4(a_pos, 0.0, 1.0); }";
        const char *fs = "precision mediump float; void main() { gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0); }";
        GLuint v = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(v, 1, &vs, NULL);
        glCompileShader(v);
        GLuint f = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(f, 1, &fs, NULL);
        glCompileShader(f);
        GLuint prog = glCreateProgram();
        glAttachShader(prog, v);
        glAttachShader(prog, f);
        glLinkProgram(prog);
        printf("GPU: shader compiled & linked\n");
        glFinish();
        printf("GPU: init done\n");
    }

    /* TEST: delay after STREAMON to simulate quad tool's self-test timing */
    if (getenv("TEST_DELAY")) {
        int delay = atoi(getenv("TEST_DELAY"));
        printf("DELAY: sleeping %d seconds (camera streaming but not consuming)...\n", delay);
        sleep(delay);
        /* Drain any queued buffers */
        for (int d = 0; d < NBUF + 2; d++) {
            struct v4l2_buffer dvb = { .type = type, .memory = V4L2_MEMORY_MMAP };
            struct v4l2_plane dp[1] = {};
            if (mp) { dvb.m.planes = dp; dvb.length = 1; }
            if (xioctl(fd, VIDIOC_DQBUF, &dvb) == 0)
                xioctl(fd, VIDIOC_QBUF, &dvb);
        }
        printf("DELAY: done, starting capture loop\n");
    }

    /* ── Capture loop ── */
    rga_buffer_t pat_empty;
    memset(&pat_empty, 0, sizeof(pat_empty));
    im_rect prect_empty = {0, 0, 0, 0};
    im_rect src_rect = {0, 0, w, h};
    im_rect dst_rect = {0, 0, dw/2, dh/2};
    int bb = 0;

    struct timespec t0;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    unsigned long frames = 0, total = 0;

    while (!quit) {
        struct pollfd pfd = { .fd = fd, .events = POLLIN };
        int ret = poll(&pfd, 1, 200);
        if (ret < 0) { if (errno == EINTR) continue; break; }
        if (ret == 0) { printf("poll timeout (total=%lu)\n", total); continue; }

        struct v4l2_buffer vb = { .type = type, .memory = V4L2_MEMORY_MMAP };
        struct v4l2_plane planes[1] = {};
        if (mp) { vb.m.planes = planes; vb.length = 1; }
        if (xioctl(fd, VIDIOC_DQBUF, &vb) < 0) {
            if (errno == EAGAIN) continue;
            break;
        }

        improcess(cam_rga[vb.index], drm_rga[bb], pat_empty,
                  src_rect, dst_rect, prect_empty, IM_SYNC);

        drmModeSetCrtc(drm_fd, crtc_id, fb_id[bb], 0, 0, &conn_id, 1, &mode);
        bb = 1 - bb;

        xioctl(fd, VIDIOC_QBUF, &vb);
        frames++; total++;

        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        double elapsed = (now.tv_sec - t0.tv_sec) + (now.tv_nsec - t0.tv_nsec) / 1e9;
        if (elapsed >= 5.0) {
            printf("%.1f fps (total=%lu)\n", frames / elapsed, total);
            frames = 0; t0 = now;
        }
    }

    printf("done, total=%lu\n", total);
    return 0;
}
