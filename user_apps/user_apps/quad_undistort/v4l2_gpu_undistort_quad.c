/*
 * v4l2_gpu_undistort_quad.c — 4-stream GPU undistort quad composite
 *
 * Captures all 4 NVP6324 camera channels, applies per-channel barrel/pincushion
 * distortion correction via GPU (EGL/GLES2), and composites into a 2×2 quad
 * displayed fullscreen on HDMI via DRM.
 *
 * Two modes:
 *   MODE_QUAD   — 2×2 grid preview with blinking selection border
 *   MODE_SINGLE — fullscreen calibration for selected channel
 *
 * Per-channel calibration loaded/saved to /root/lens_calib_ch{0..3}.conf
 *
 * Build (on board):
 *   gcc -O2 -o v4l2_gpu_undistort_quad v4l2_gpu_undistort_quad.c \
 *       $(pkg-config --cflags --libs librga libdrm gbm egl glesv2) -lm -lpthread
 *
 * Usage:
 *   sudo systemctl stop lightdm
 *   sudo ./v4l2_gpu_undistort_quad [video_base]   # default 11
 *   sudo systemctl start lightdm
 *
 * Keyboard (MODE_QUAD):
 *   Left/Right  = cycle selected channel
 *   Enter       = enter fullscreen calibration for selected channel
 *   Q           = quit
 *
 * Keyboard (MODE_SINGLE):
 *   Left/Right  = adjust k1 (barrel/pincushion), step +/-0.01
 *   Up/Down     = adjust k2 (4th-order term), step +/-0.005
 *   C/V         = adjust cx (optical center X), step +/-0.01
 *   B/N         = adjust cy (optical center Y), step +/-0.01
 *   G           = toggle grid overlay
 *   Space       = toggle bypass (compare original vs corrected)
 *   S           = save parameters to config file
 *   L           = load parameters from config file
 *   R           = reset all to defaults (k1=k2=0, cx=cy=0.5)
 *   +/-         = zoom
 *   ESC         = return to quad view
 *   Q           = quit
 *
 * Mouse calibration (MODE_SINGLE):
 *   Mouse move  = move crosshair cursor
 *   Left click  = place calibration point on current line
 *   Tab         = start new calibration line
 *   A           = auto-detect edges → fit k1/k2 (Sobel edge detection)
 *   D           = delete last calibration line
 *   F           = auto-fit k1/k2 from placed points
 *   X           = clear all calibration points
 *   Backspace   = undo last point
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <poll.h>
#include <time.h>
#include <termios.h>
#include <dirent.h>
#include <pthread.h>
#include <math.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include <linux/videodev2.h>
#include <linux/input.h>

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>

#include <rga/im2d.h>
#include <rga/rga.h>

#include <gbm.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>

/* ─────────────────── Config ─────────────────── */

#define MAX_CAMS        4
#define V4L2_BUF_CNT    4
#define DRM_CARD        "/dev/dri/card1"

/* NVP6324 sysfs paths */
#define NVP_CHANNEL_FMT "/sys/devices/platform/2ac70000.i2c/i2c-4/4-0031/channel_fmt"

/* DV timing indices */
#define DV_IDX_CVBS_NTSC 0
#define DV_IDX_CVBS_PAL  1
#define DV_IDX_1080P25   6

/* Per-channel format: all 1080p25 to avoid MIPI CSI errors from mixed formats.
 * ch2 has a real AHD camera, ch0/1/3 will show color bars at 1080p. */
static const struct { int dv_idx; int width; int height; int skip; } ch_cfg[MAX_CAMS] = {
    { DV_IDX_1080P25,   1920, 1080, 0 },   /* ch0: AHD 1080p25 */
    { DV_IDX_1080P25,   1920, 1080, 0 },   /* ch1: AHD 1080p25 */
    { DV_IDX_1080P25,   1920, 1080, 0 },   /* ch2: AHD 1080p25 */
    { DV_IDX_1080P25,   1920, 1080, 0 },   /* ch3: AHD 1080p25 */
};

/* Per-channel calibration */
struct calib {
    double k1, k2, cx, cy;
    int zoom;  /* in 1/10ths: 10=1.0x, 15=1.5x, etc. */
};

/* ─────────────────── 8x8 VGA Font (ASCII 32-126) ─────────────────── */

static const uint8_t font8x8[95][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},  /* 32 ' ' */
    {0x18,0x3c,0x3c,0x18,0x18,0x00,0x18,0x00},  /* 33 '!' */
    {0x66,0x66,0x24,0x00,0x00,0x00,0x00,0x00},  /* 34 '"' */
    {0x6c,0x6c,0xfe,0x6c,0xfe,0x6c,0x6c,0x00},  /* 35 '#' */
    {0x18,0x3e,0x60,0x3c,0x06,0x7c,0x18,0x00},  /* 36 '$' */
    {0x00,0xc6,0xcc,0x18,0x30,0x66,0xc6,0x00},  /* 37 '%' */
    {0x38,0x6c,0x38,0x76,0xdc,0xcc,0x76,0x00},  /* 38 '&' */
    {0x18,0x18,0x30,0x00,0x00,0x00,0x00,0x00},  /* 39 '\'' */
    {0x0c,0x18,0x30,0x30,0x30,0x18,0x0c,0x00},  /* 40 '(' */
    {0x30,0x18,0x0c,0x0c,0x0c,0x18,0x30,0x00},  /* 41 ')' */
    {0x00,0x66,0x3c,0xff,0x3c,0x66,0x00,0x00},  /* 42 '*' */
    {0x00,0x18,0x18,0x7e,0x18,0x18,0x00,0x00},  /* 43 '+' */
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x30},  /* 44 ',' */
    {0x00,0x00,0x00,0x7e,0x00,0x00,0x00,0x00},  /* 45 '-' */
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00},  /* 46 '.' */
    {0x06,0x0c,0x18,0x30,0x60,0xc0,0x80,0x00},  /* 47 '/' */
    {0x38,0x6c,0xc6,0xd6,0xc6,0x6c,0x38,0x00},  /* 48 '0' */
    {0x18,0x38,0x18,0x18,0x18,0x18,0x7e,0x00},  /* 49 '1' */
    {0x7c,0xc6,0x06,0x1c,0x30,0x66,0xfe,0x00},  /* 50 '2' */
    {0x7c,0xc6,0x06,0x3c,0x06,0xc6,0x7c,0x00},  /* 51 '3' */
    {0x1c,0x3c,0x6c,0xcc,0xfe,0x0c,0x1e,0x00},  /* 52 '4' */
    {0xfe,0xc0,0xc0,0xfc,0x06,0xc6,0x7c,0x00},  /* 53 '5' */
    {0x38,0x60,0xc0,0xfc,0xc6,0xc6,0x7c,0x00},  /* 54 '6' */
    {0xfe,0xc6,0x0c,0x18,0x30,0x30,0x30,0x00},  /* 55 '7' */
    {0x7c,0xc6,0xc6,0x7c,0xc6,0xc6,0x7c,0x00},  /* 56 '8' */
    {0x7c,0xc6,0xc6,0x7e,0x06,0x0c,0x78,0x00},  /* 57 '9' */
    {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x00},  /* 58 ':' */
    {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x30},  /* 59 ';' */
    {0x06,0x0c,0x18,0x30,0x18,0x0c,0x06,0x00},  /* 60 '<' */
    {0x00,0x00,0x7e,0x00,0x00,0x7e,0x00,0x00},  /* 61 '=' */
    {0x60,0x30,0x18,0x0c,0x18,0x30,0x60,0x00},  /* 62 '>' */
    {0x7c,0xc6,0x0c,0x18,0x18,0x00,0x18,0x00},  /* 63 '?' */
    {0x7c,0xc6,0xde,0xde,0xde,0xc0,0x78,0x00},  /* 64 '@' */
    {0x38,0x6c,0xc6,0xfe,0xc6,0xc6,0xc6,0x00},  /* 65 'A' */
    {0xfc,0x66,0x66,0x7c,0x66,0x66,0xfc,0x00},  /* 66 'B' */
    {0x3c,0x66,0xc0,0xc0,0xc0,0x66,0x3c,0x00},  /* 67 'C' */
    {0xf8,0x6c,0x66,0x66,0x66,0x6c,0xf8,0x00},  /* 68 'D' */
    {0xfe,0x62,0x68,0x78,0x68,0x62,0xfe,0x00},  /* 69 'E' */
    {0xfe,0x62,0x68,0x78,0x68,0x60,0xf0,0x00},  /* 70 'F' */
    {0x3c,0x66,0xc0,0xc0,0xce,0x66,0x3a,0x00},  /* 71 'G' */
    {0xc6,0xc6,0xc6,0xfe,0xc6,0xc6,0xc6,0x00},  /* 72 'H' */
    {0x3c,0x18,0x18,0x18,0x18,0x18,0x3c,0x00},  /* 73 'I' */
    {0x1e,0x0c,0x0c,0x0c,0xcc,0xcc,0x78,0x00},  /* 74 'J' */
    {0xe6,0x66,0x6c,0x78,0x6c,0x66,0xe6,0x00},  /* 75 'K' */
    {0xf0,0x60,0x60,0x60,0x62,0x66,0xfe,0x00},  /* 76 'L' */
    {0xc6,0xee,0xfe,0xfe,0xd6,0xc6,0xc6,0x00},  /* 77 'M' */
    {0xc6,0xe6,0xf6,0xde,0xce,0xc6,0xc6,0x00},  /* 78 'N' */
    {0x7c,0xc6,0xc6,0xc6,0xc6,0xc6,0x7c,0x00},  /* 79 'O' */
    {0xfc,0x66,0x66,0x7c,0x60,0x60,0xf0,0x00},  /* 80 'P' */
    {0x7c,0xc6,0xc6,0xc6,0xc6,0xce,0x7c,0x0e},  /* 81 'Q' */
    {0xfc,0x66,0x66,0x7c,0x6c,0x66,0xe6,0x00},  /* 82 'R' */
    {0x3c,0x66,0x30,0x18,0x0c,0x66,0x3c,0x00},  /* 83 'S' */
    {0x7e,0x7e,0x5a,0x18,0x18,0x18,0x3c,0x00},  /* 84 'T' */
    {0xc6,0xc6,0xc6,0xc6,0xc6,0xc6,0x7c,0x00},  /* 85 'U' */
    {0xc6,0xc6,0xc6,0xc6,0xc6,0x6c,0x38,0x00},  /* 86 'V' */
    {0xc6,0xc6,0xc6,0xd6,0xd6,0xfe,0x6c,0x00},  /* 87 'W' */
    {0xc6,0xc6,0x6c,0x38,0x6c,0xc6,0xc6,0x00},  /* 88 'X' */
    {0x66,0x66,0x66,0x3c,0x18,0x18,0x3c,0x00},  /* 89 'Y' */
    {0xfe,0xc6,0x8c,0x18,0x32,0x66,0xfe,0x00},  /* 90 'Z' */
    {0x3c,0x30,0x30,0x30,0x30,0x30,0x3c,0x00},  /* 91 '[' */
    {0xc0,0x60,0x30,0x18,0x0c,0x06,0x02,0x00},  /* 92 '\\' */
    {0x3c,0x0c,0x0c,0x0c,0x0c,0x0c,0x3c,0x00},  /* 93 ']' */
    {0x10,0x38,0x6c,0xc6,0x00,0x00,0x00,0x00},  /* 94 '^' */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff},  /* 95 '_' */
    {0x30,0x18,0x0c,0x00,0x00,0x00,0x00,0x00},  /* 96 '`' */
    {0x00,0x00,0x78,0x0c,0x7c,0xcc,0x76,0x00},  /* 97 'a' */
    {0xe0,0x60,0x7c,0x66,0x66,0x66,0xdc,0x00},  /* 98 'b' */
    {0x00,0x00,0x7c,0xc6,0xc0,0xc6,0x7c,0x00},  /* 99 'c' */
    {0x1c,0x0c,0x7c,0xcc,0xcc,0xcc,0x76,0x00},  /* 100 'd' */
    {0x00,0x00,0x7c,0xc6,0xfe,0xc0,0x7c,0x00},  /* 101 'e' */
    {0x3c,0x66,0x60,0xf8,0x60,0x60,0xf0,0x00},  /* 102 'f' */
    {0x00,0x00,0x76,0xcc,0xcc,0x7c,0x0c,0xf8},  /* 103 'g' */
    {0xe0,0x60,0x6c,0x76,0x66,0x66,0xe6,0x00},  /* 104 'h' */
    {0x18,0x00,0x38,0x18,0x18,0x18,0x3c,0x00},  /* 105 'i' */
    {0x06,0x00,0x06,0x06,0x06,0x66,0x66,0x3c},  /* 106 'j' */
    {0xe0,0x60,0x66,0x6c,0x78,0x6c,0xe6,0x00},  /* 107 'k' */
    {0x38,0x18,0x18,0x18,0x18,0x18,0x3c,0x00},  /* 108 'l' */
    {0x00,0x00,0xec,0xfe,0xd6,0xd6,0xd6,0x00},  /* 109 'm' */
    {0x00,0x00,0xdc,0x66,0x66,0x66,0x66,0x00},  /* 110 'n' */
    {0x00,0x00,0x7c,0xc6,0xc6,0xc6,0x7c,0x00},  /* 111 'o' */
    {0x00,0x00,0xdc,0x66,0x66,0x7c,0x60,0xf0},  /* 112 'p' */
    {0x00,0x00,0x76,0xcc,0xcc,0x7c,0x0c,0x1e},  /* 113 'q' */
    {0x00,0x00,0xdc,0x76,0x60,0x60,0xf0,0x00},  /* 114 'r' */
    {0x00,0x00,0x7e,0xc0,0x7c,0x06,0xfc,0x00},  /* 115 's' */
    {0x30,0x30,0xfc,0x30,0x30,0x36,0x1c,0x00},  /* 116 't' */
    {0x00,0x00,0xcc,0xcc,0xcc,0xcc,0x76,0x00},  /* 117 'u' */
    {0x00,0x00,0xc6,0xc6,0xc6,0x6c,0x38,0x00},  /* 118 'v' */
    {0x00,0x00,0xc6,0xd6,0xd6,0xfe,0x6c,0x00},  /* 119 'w' */
    {0x00,0x00,0xc6,0x6c,0x38,0x6c,0xc6,0x00},  /* 120 'x' */
    {0x00,0x00,0xc6,0xc6,0xc6,0x7e,0x06,0xfc},  /* 121 'y' */
    {0x00,0x00,0x7e,0x4c,0x18,0x32,0x7e,0x00},  /* 122 'z' */
    {0x0e,0x18,0x18,0x70,0x18,0x18,0x0e,0x00},  /* 123 '{' */
    {0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x00},  /* 124 '|' */
    {0x70,0x18,0x18,0x0e,0x18,0x18,0x70,0x00},  /* 125 '}' */
    {0x76,0xdc,0x00,0x00,0x00,0x00,0x00,0x00},  /* 126 '~' */
};

/* ─────────────────── Globals ─────────────────── */

static volatile sig_atomic_t quit = 0;
static void on_signal(int s) { (void)s; quit = 1; }

/* Mode state machine */
enum { MODE_QUAD, MODE_SINGLE };
static volatile int g_mode = MODE_QUAD;
static volatile int g_selected = 0;       /* 0-3: selected cam in quad mode */

/* Calibration parameters (for MODE_SINGLE, accessed from input thread) */
static volatile double g_k1 = 0.0;
static volatile double g_k2 = 0.0;
static volatile double g_cx = 0.5;
static volatile double g_cy = 0.5;
static volatile int g_grid = 0;
static volatile int g_bypass = 0;
static volatile int g_save_req = 0;
static volatile int g_load_req = 0;
static volatile int g_reset_req = 0;
static volatile int g_fit_req = 0;
static volatile int g_autodetect_req = 0;
static volatile int g_enter_single = 0;   /* quad→single transition request */
static volatile int g_exit_single = 0;    /* single→quad transition request */
static pthread_mutex_t param_lock = PTHREAD_MUTEX_INITIALIZER;

/* Mouse cursor state */
static volatile int g_cursor_x, g_cursor_y;
static volatile int g_cursor_visible = 0;
static volatile int g_zoom = 10;  /* zoom in 1/10ths: 10=1.0x, 15=1.5x, 20=2.0x */
#define MIN_ZOOM 10
#define MAX_ZOOM 80  /* 8.0x */
static int g_disp_w, g_disp_h;
static int g_src_w, g_src_h;
static volatile int g_crop_x, g_crop_y, g_crop_w, g_crop_h;

/* Calibration points */
#define MAX_CALIB_LINES  8
#define MAX_CALIB_PTS    64

static int calib_line = 0;
static volatile int calib_npts = 0;
static struct {
    int line_idx;
    int dx, dy;
    double px, py;
} calib_pts[MAX_CALIB_PTS];

/* Terminal state */
static struct termios orig_tio;
static int tio_saved = 0;

/* ─────────────────── Config file save/load ─────────────────── */

static void calib_path(char *buf, size_t sz, int ch)
{
    snprintf(buf, sz, "/root/lens_calib_ch%d.conf", ch);
}

static int save_params(const char *path, double k1, double k2, double cx, double cy, int zoom)
{
    FILE *fp = fopen(path, "w");
    if (!fp) {
        fprintf(stderr, "Cannot save to %s: %s\n", path, strerror(errno));
        return -1;
    }
    fprintf(fp, "k1=%.6f\nk2=%.6f\ncx=%.6f\ncy=%.6f\nzoom=%d\n", k1, k2, cx, cy, zoom);
    fclose(fp);
    printf("Saved: k1=%.4f k2=%.4f cx=%.3f cy=%.3f zoom=%.1fx -> %s\n",
           k1, k2, cx, cy, zoom / 10.0, path);
    return 0;
}

static int load_params(const char *path, struct calib *cal)
{
    FILE *fp = fopen(path, "r");
    if (!fp) return -1;
    char line[128];
    while (fgets(line, sizeof(line), fp)) {
        double val; int ival;
        if (sscanf(line, "k1=%lf", &val) == 1) cal->k1 = val;
        else if (sscanf(line, "k2=%lf", &val) == 1) cal->k2 = val;
        else if (sscanf(line, "cx=%lf", &val) == 1) cal->cx = val;
        else if (sscanf(line, "cy=%lf", &val) == 1) cal->cy = val;
        else if (sscanf(line, "zoom=%d", &ival) == 1) cal->zoom = ival;
    }
    fclose(fp);
    printf("Loaded: k1=%.4f k2=%.4f cx=%.3f cy=%.3f zoom=%.1fx <- %s\n",
           cal->k1, cal->k2, cal->cx, cal->cy, cal->zoom / 10.0, path);
    return 0;
}

/* ─────────────────── Types ─────────────────── */

struct cam_buf {
    void               *start;
    size_t              length;
    int                 dmabuf_fd;
    rga_buffer_handle_t rga_handle;
    rga_buffer_t        rga;
};

struct cam {
    int             fd;
    int             active;
    char            path[32];
    uint32_t        buf_type;
    int             is_mplane;
    int             width, height;
    int             src_height;     /* visible lines (half for interlaced CVBS) */
    int             buf_count;
    struct cam_buf  bufs[V4L2_BUF_CNT];
    im_rect         quad;           /* destination quadrant on display */
    struct calib    cal;            /* per-channel distortion params */

    /* GPU intermediate buffer (per-channel, source resolution) */
    uint32_t        inter_gem;
    uint32_t        inter_pitch;
    uint64_t        inter_size;
    int             inter_dmabuf_fd;
    rga_buffer_handle_t inter_rga_handle;
    rga_buffer_t    inter_rga;
    EGLImageKHR     src_img;
    GLuint          src_tex;
};

struct display {
    int                 fd;
    uint32_t            conn_id, crtc_id;
    drmModeModeInfo     mode;
    drmModeCrtcPtr      saved_crtc;
    uint32_t            fb_id[2], gem_handle[2];
    uint32_t            pitch;
    uint64_t            size;
    void               *map[2];
    int                 dmabuf_fd[2];
    rga_buffer_handle_t rga_handle[2];
    rga_buffer_t        rga[2];
    int                 back;
};

typedef struct {
    struct gbm_device *gbm;
    EGLDisplay dpy;
    EGLContext ctx;
    GLuint prog;

    GLint loc_tex, loc_center, loc_res, loc_max_r, loc_k1, loc_k2, loc_fit_scale, loc_crop;

    /* Double-buffered FBO backed by display DRM dumb buffers */
    EGLImageKHR dst_img[2];
    GLuint dst_tex[2];
    GLuint fbo[2];

    PFNEGLCREATEIMAGEKHRPROC CreateImage;
    PFNEGLDESTROYIMAGEKHRPROC DestroyImage;
    PFNGLEGLIMAGETARGETTEXTURE2DOESPROC ImageTargetTex2D;

    int dst_w, dst_h;
} gpu_ctx;

/* ─────────────────── NVP6324 + media setup ─────────────────── */

static int setup_nvp6324(void)
{
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
    printf("Media pipeline: %dx%d UYVY\n", max_w, max_h);
    return 0;
}

/* ─────────────────── DRM ─────────────────── */

static int drm_init(struct display *d)
{
    memset(d, 0, sizeof(*d));
    d->fd = -1;
    d->dmabuf_fd[0] = d->dmabuf_fd[1] = -1;

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

    drmModeModeInfo *best = &conn->modes[0];
    for (int i = 0; i < conn->count_modes; i++)
        if (conn->modes[i].type & DRM_MODE_TYPE_PREFERRED)
            { best = &conn->modes[i]; break; }
    d->mode = *best;

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

    /* Double-buffered dumb buffers */
    for (int b = 0; b < 2; b++) {
        struct drm_mode_create_dumb cr = {
            .width  = d->mode.hdisplay,
            .height = d->mode.vdisplay,
            .bpp    = 32,
        };
        if (drmIoctl(d->fd, DRM_IOCTL_MODE_CREATE_DUMB, &cr))
            { perror("DRM create_dumb"); goto err; }
        d->gem_handle[b] = cr.handle;
        d->pitch         = cr.pitch;
        d->size          = cr.size;

        if (drmModeAddFB(d->fd, d->mode.hdisplay, d->mode.vdisplay,
                         24, 32, d->pitch, d->gem_handle[b], &d->fb_id[b]))
            { perror("DRM addFB"); goto err; }

        struct drm_mode_map_dumb mr = { .handle = d->gem_handle[b] };
        if (drmIoctl(d->fd, DRM_IOCTL_MODE_MAP_DUMB, &mr))
            { perror("DRM map_dumb"); goto err; }
        d->map[b] = mmap(NULL, d->size, PROT_READ | PROT_WRITE,
                         MAP_SHARED, d->fd, mr.offset);
        if (d->map[b] == MAP_FAILED) { perror("DRM mmap"); d->map[b] = NULL; goto err; }
        memset(d->map[b], 0, d->size);

        if (drmPrimeHandleToFD(d->fd, d->gem_handle[b],
                               DRM_CLOEXEC | DRM_RDWR, &d->dmabuf_fd[b]))
            { perror("DRM primeHandleToFD"); goto err; }
    }
    d->back = 0;

    if (drmModeSetCrtc(d->fd, d->crtc_id, d->fb_id[0], 0, 0,
                       &d->conn_id, 1, &d->mode))
        { perror("DRM setCrtc"); goto err; }

    printf("DRM: FB %u/%u (%ux%u pitch=%u) double-buffered\n",
           d->fb_id[0], d->fb_id[1],
           d->mode.hdisplay, d->mode.vdisplay, d->pitch);
    return 0;

err:
    for (int b = 0; b < 2; b++) {
        if (d->dmabuf_fd[b] >= 0) close(d->dmabuf_fd[b]);
        if (d->map[b])      munmap(d->map[b], d->size);
        if (d->fb_id[b])    drmModeRmFB(d->fd, d->fb_id[b]);
        if (d->gem_handle[b]) {
            struct drm_mode_destroy_dumb dd = { .handle = d->gem_handle[b] };
            drmIoctl(d->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dd);
        }
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
    for (int b = 0; b < 2; b++) {
        if (d->rga_handle[b]) releasebuffer_handle(d->rga_handle[b]);
        if (d->dmabuf_fd[b] >= 0) close(d->dmabuf_fd[b]);
        if (d->map[b])      munmap(d->map[b], d->size);
        if (d->fb_id[b])    drmModeRmFB(d->fd, d->fb_id[b]);
        if (d->gem_handle[b]) {
            struct drm_mode_destroy_dumb dd = { .handle = d->gem_handle[b] };
            drmIoctl(d->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dd);
        }
    }
    drmDropMaster(d->fd);
    close(d->fd);
    d->fd = -1;
}

/* ─────────────────── V4L2 ─────────────────── */

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

    int dv = ch_cfg[ch_idx].dv_idx;
    if (dv == DV_IDX_CVBS_NTSC || dv == DV_IDX_CVBS_PAL)
        c->src_height = c->height / 2;
    else
        c->src_height = c->height;

    printf("V4L2: %s -> %dx%d UYVY (src_h=%d)\n",
           c->path, c->width, c->height, c->src_height);

    struct v4l2_requestbuffers reqb = {
        .count  = V4L2_BUF_CNT,
        .type   = c->buf_type,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (xioctl(c->fd, VIDIOC_REQBUFS, &reqb) < 0) {
        fprintf(stderr, "V4L2: %s REQBUFS: %s\n", c->path, strerror(errno));
        goto err;
    }
    c->buf_count = reqb.count;

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

/* ─────────────────── RGA setup ─────────────────── */

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
    for (int b = 0; b < 2; b++) {
        d->rga_handle[b] = importbuffer_fd(d->dmabuf_fd[b], &dp);
        if (!d->rga_handle[b]) {
            fprintf(stderr, "RGA: import composite buf%d failed\n", b);
            return -1;
        }
        d->rga[b] = wrapbuffer_handle(d->rga_handle[b], w, h,
                                       RK_FORMAT_BGRX_8888, wstride, h);
    }
    printf("RGA: composite %dx%d stride=%d (double-buffered)\n", w, h, wstride);

    int qw = w / 2, qh = h / 2;
    int pos[4][2] = { {0,0}, {qw,0}, {0,qh}, {qw,qh} };

    for (int i = 0; i < ncams; i++) {
        struct cam *c = &cams[i];
        if (!c->active) continue;

        c->quad = (im_rect){ pos[i][0], pos[i][1], qw, qh };
        printf("RGA: cam%d %dx%d -> quad (%d,%d)+%dx%d\n",
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

/* ─────────────────── GPU (EGL/GLES2) ─────────────────── */

static const char *vert_src =
    "attribute vec2 a_pos;\n"
    "attribute vec2 a_uv;\n"
    "varying vec2 v_uv;\n"
    "void main() {\n"
    "    gl_Position = vec4(a_pos, 0.0, 1.0);\n"
    "    v_uv = a_uv;\n"
    "}\n";

static const char *frag_src =
    "precision highp float;\n"
    "varying vec2 v_uv;\n"
    "uniform sampler2D u_tex;\n"
    "uniform vec2 u_center;\n"
    "uniform vec2 u_res;\n"
    "uniform float u_max_r;\n"
    "uniform float u_k1, u_k2;\n"
    "uniform float u_fit_scale;\n"
    "uniform vec4 u_crop;\n"
    "void main() {\n"
    "    vec2 uv = u_crop.xy + v_uv * u_crop.zw;\n"
    "    vec2 uv2 = u_center + (uv - u_center) / u_fit_scale;\n"
    "    vec2 d = (uv2 - u_center) * u_res;\n"
    "    float r2 = dot(d, d) / (u_max_r * u_max_r);\n"
    "    float s = 1.0 + u_k1 * r2 + u_k2 * r2 * r2;\n"
    "    vec2 src = u_center + (uv2 - u_center) * s;\n"
    "    if (src.x < 0.0 || src.x > 1.0 || src.y < 0.0 || src.y > 1.0)\n"
    "        gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);\n"
    "    else\n"
    "        gl_FragColor = texture2D(u_tex, src);\n"
    "}\n";

static GLuint gpu_compile_shader(GLenum type, const char *src)
{
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, NULL);
    glCompileShader(s);
    GLint ok;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(s, sizeof(log), NULL, log);
        fprintf(stderr, "GPU: shader compile error: %s\n", log);
        glDeleteShader(s);
        return 0;
    }
    return s;
}

/* Fullscreen quad vertices + texcoords for FBO rendering.
 * FBO row 0 = DRM row 0 = top of screen, so no Y-flip needed. */
static const GLfloat quad_verts[] = {
    -1.0f, -1.0f,   1.0f, -1.0f,   -1.0f,  1.0f,   1.0f,  1.0f,
};
static const GLfloat quad_texcoords[] = {
     0.0f,  0.0f,   1.0f,  0.0f,    0.0f,  1.0f,   1.0f,  1.0f,
};

static int gpu_init(gpu_ctx *g, struct display *d)
{
    memset(g, 0, sizeof(*g));
    g->dst_w = d->mode.hdisplay;
    g->dst_h = d->mode.vdisplay;

    /* GBM + EGL */
    g->gbm = gbm_create_device(d->fd);
    if (!g->gbm) { fprintf(stderr, "GPU: gbm_create_device failed\n"); return -1; }

    PFNEGLGETPLATFORMDISPLAYEXTPROC GetPlatformDisplay =
        (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");
    if (GetPlatformDisplay)
        g->dpy = GetPlatformDisplay(EGL_PLATFORM_GBM_KHR, g->gbm, NULL);
    else
        g->dpy = eglGetDisplay((EGLNativeDisplayType)g->gbm);
    if (g->dpy == EGL_NO_DISPLAY) { fprintf(stderr, "GPU: eglGetDisplay failed\n"); return -1; }

    EGLint major, minor;
    if (!eglInitialize(g->dpy, &major, &minor)) {
        fprintf(stderr, "GPU: eglInitialize failed\n"); return -1;
    }
    printf("GPU: EGL %d.%d\n", major, minor);

    eglBindAPI(EGL_OPENGL_ES_API);

    EGLint cfg_attr[] = {
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
        EGL_NONE,
    };
    EGLConfig cfg;
    EGLint ncfg;
    eglChooseConfig(g->dpy, cfg_attr, &cfg, 1, &ncfg);

    EGLint ctx_attr[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE };
    g->ctx = eglCreateContext(g->dpy, cfg, EGL_NO_CONTEXT, ctx_attr);
    if (g->ctx == EGL_NO_CONTEXT) {
        fprintf(stderr, "GPU: eglCreateContext failed\n"); return -1;
    }

    if (!eglMakeCurrent(g->dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, g->ctx)) {
        fprintf(stderr, "GPU: eglMakeCurrent (surfaceless) failed\n"); return -1;
    }

    printf("GPU: GL_RENDERER = %s\n", glGetString(GL_RENDERER));

    /* Extension procs */
    g->CreateImage = (PFNEGLCREATEIMAGEKHRPROC)eglGetProcAddress("eglCreateImageKHR");
    g->DestroyImage = (PFNEGLDESTROYIMAGEKHRPROC)eglGetProcAddress("eglDestroyImageKHR");
    g->ImageTargetTex2D = (PFNGLEGLIMAGETARGETTEXTURE2DOESPROC)
        eglGetProcAddress("glEGLImageTargetTexture2DOES");
    if (!g->CreateImage || !g->DestroyImage || !g->ImageTargetTex2D) {
        fprintf(stderr, "GPU: missing EGL image extensions\n"); return -1;
    }

    /* Compile shaders */
    GLuint vs = gpu_compile_shader(GL_VERTEX_SHADER, vert_src);
    GLuint fs = gpu_compile_shader(GL_FRAGMENT_SHADER, frag_src);
    if (!vs || !fs) return -1;

    g->prog = glCreateProgram();
    glAttachShader(g->prog, vs);
    glAttachShader(g->prog, fs);
    glBindAttribLocation(g->prog, 0, "a_pos");
    glBindAttribLocation(g->prog, 1, "a_uv");
    glLinkProgram(g->prog);
    glDeleteShader(vs);
    glDeleteShader(fs);

    GLint linked;
    glGetProgramiv(g->prog, GL_LINK_STATUS, &linked);
    if (!linked) {
        char log[512];
        glGetProgramInfoLog(g->prog, sizeof(log), NULL, log);
        fprintf(stderr, "GPU: link error: %s\n", log);
        return -1;
    }

    g->loc_tex    = glGetUniformLocation(g->prog, "u_tex");
    g->loc_center = glGetUniformLocation(g->prog, "u_center");
    g->loc_res    = glGetUniformLocation(g->prog, "u_res");
    g->loc_max_r  = glGetUniformLocation(g->prog, "u_max_r");
    g->loc_k1     = glGetUniformLocation(g->prog, "u_k1");
    g->loc_k2     = glGetUniformLocation(g->prog, "u_k2");
    g->loc_fit_scale = glGetUniformLocation(g->prog, "u_fit_scale");
    g->loc_crop   = glGetUniformLocation(g->prog, "u_crop");

    /* Set up vertex arrays (shared for all draws) */
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, quad_verts);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, quad_texcoords);

    /* Import DRM dumb buffers as double-buffered FBO render targets */
    for (int b = 0; b < 2; b++) {
        EGLint attrs[] = {
            EGL_WIDTH, (EGLint)d->mode.hdisplay,
            EGL_HEIGHT, (EGLint)d->mode.vdisplay,
            EGL_LINUX_DRM_FOURCC_EXT, DRM_FORMAT_XRGB8888,
            EGL_DMA_BUF_PLANE0_FD_EXT, d->dmabuf_fd[b],
            EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0,
            EGL_DMA_BUF_PLANE0_PITCH_EXT, (EGLint)d->pitch,
            EGL_NONE,
        };
        g->dst_img[b] = g->CreateImage(g->dpy, EGL_NO_CONTEXT,
                                        EGL_LINUX_DMA_BUF_EXT, NULL, attrs);
        if (g->dst_img[b] == EGL_NO_IMAGE_KHR) {
            fprintf(stderr, "GPU: dst EGLImage[%d] failed\n", b); return -1;
        }

        glGenTextures(1, &g->dst_tex[b]);
        glBindTexture(GL_TEXTURE_2D, g->dst_tex[b]);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        g->ImageTargetTex2D(GL_TEXTURE_2D, (GLeglImageOES)g->dst_img[b]);

        glGenFramebuffers(1, &g->fbo[b]);
        glBindFramebuffer(GL_FRAMEBUFFER, g->fbo[b]);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, g->dst_tex[b], 0);
        GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        if (status != GL_FRAMEBUFFER_COMPLETE) {
            fprintf(stderr, "GPU: FBO[%d] incomplete: 0x%x\n", b, status);
            return -1;
        }
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    printf("GPU: initialized — double-buffered FBO (%dx%d)\n", g->dst_w, g->dst_h);
    return 0;
}

/* Create per-channel intermediate buffer + GL source texture */
static int gpu_init_cam_texture(gpu_ctx *g, struct cam *c, int drm_fd)
{
    c->inter_dmabuf_fd = -1;
    c->inter_gem = 0;
    c->src_img = EGL_NO_IMAGE_KHR;
    c->src_tex = 0;

    struct drm_mode_create_dumb cr = {
        .width = (uint32_t)c->width, .height = (uint32_t)c->height, .bpp = 32,
    };
    if (drmIoctl(drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &cr)) {
        perror("GPU: inter create_dumb"); return -1;
    }
    c->inter_gem = cr.handle;
    c->inter_pitch = cr.pitch;
    c->inter_size = cr.size;

    if (drmPrimeHandleToFD(drm_fd, cr.handle, DRM_CLOEXEC | DRM_RDWR,
                           &c->inter_dmabuf_fd)) {
        perror("GPU: inter primeHandleToFD"); return -1;
    }

    /* RGA handle for intermediate buffer */
    int wstride = (int)(cr.pitch / 4);
    im_handle_param_t ip = {
        .width = (uint32_t)wstride, .height = (uint32_t)c->height,
        .format = RK_FORMAT_BGRX_8888,
    };
    c->inter_rga_handle = importbuffer_fd(c->inter_dmabuf_fd, &ip);
    if (!c->inter_rga_handle) {
        fprintf(stderr, "GPU: cam inter RGA import failed\n"); return -1;
    }
    c->inter_rga = wrapbuffer_handle(c->inter_rga_handle,
                                      c->width, c->height,
                                      RK_FORMAT_BGRX_8888, wstride, c->height);

    /* Import as XRGB8888 GL texture */
    EGLint attrs[] = {
        EGL_WIDTH, c->width,
        EGL_HEIGHT, c->height,
        EGL_LINUX_DRM_FOURCC_EXT, DRM_FORMAT_XRGB8888,
        EGL_DMA_BUF_PLANE0_FD_EXT, c->inter_dmabuf_fd,
        EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0,
        EGL_DMA_BUF_PLANE0_PITCH_EXT, (EGLint)cr.pitch,
        EGL_NONE,
    };
    c->src_img = g->CreateImage(g->dpy, EGL_NO_CONTEXT,
                                 EGL_LINUX_DMA_BUF_EXT, NULL, attrs);
    if (c->src_img == EGL_NO_IMAGE_KHR) {
        fprintf(stderr, "GPU: cam src EGLImage failed\n"); return -1;
    }

    glGenTextures(1, &c->src_tex);
    glBindTexture(GL_TEXTURE_2D, c->src_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    g->ImageTargetTex2D(GL_TEXTURE_2D, (GLeglImageOES)c->src_img);

    printf("GPU: cam inter buf %dx%d pitch=%u (%uKB)\n",
           c->width, c->height, cr.pitch, (unsigned)(cr.size / 1024));
    return 0;
}

static void gpu_cleanup_cam_texture(gpu_ctx *g, struct cam *c, int drm_fd)
{
    if (c->src_tex) { glDeleteTextures(1, &c->src_tex); c->src_tex = 0; }
    if (c->src_img != EGL_NO_IMAGE_KHR) {
        g->DestroyImage(g->dpy, c->src_img);
        c->src_img = EGL_NO_IMAGE_KHR;
    }
    if (c->inter_rga_handle) { releasebuffer_handle(c->inter_rga_handle); c->inter_rga_handle = 0; }
    if (c->inter_dmabuf_fd >= 0) { close(c->inter_dmabuf_fd); c->inter_dmabuf_fd = -1; }
    if (c->inter_gem) {
        struct drm_mode_destroy_dumb dd = { .handle = c->inter_gem };
        drmIoctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dd);
        c->inter_gem = 0;
    }
}

static void gpu_cleanup(gpu_ctx *g)
{
    if (!g->dpy) return;

    eglMakeCurrent(g->dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, g->ctx);

    for (int b = 0; b < 2; b++) {
        if (g->fbo[b]) glDeleteFramebuffers(1, &g->fbo[b]);
        if (g->dst_tex[b]) glDeleteTextures(1, &g->dst_tex[b]);
        if (g->dst_img[b]) g->DestroyImage(g->dpy, g->dst_img[b]);
    }
    if (g->prog) glDeleteProgram(g->prog);

    eglMakeCurrent(g->dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    eglDestroyContext(g->dpy, g->ctx);
    eglTerminate(g->dpy);
    if (g->gbm) gbm_device_destroy(g->gbm);
}

/* ─────────────────── Drawing: grid, crosshair, calib points ─────────────────── */

static void draw_grid(uint32_t *fb, int pitch4, int w, int h, uint32_t color)
{
    int nx = 16, ny = 9;
    for (int i = 1; i < nx; i++) {
        int x = i * w / nx;
        for (int y = 0; y < h; y++)
            fb[y * pitch4 + x] = color;
    }
    for (int i = 1; i < ny; i++) {
        int y = i * h / ny;
        for (int x = 0; x < w; x++)
            fb[y * pitch4 + x] = color;
    }
    int cx = w / 2, cy = h / 2;
    uint32_t cc = 0x00FF0000;
    for (int y = 0; y < h; y++) {
        fb[y * pitch4 + cx - 1] = cc;
        fb[y * pitch4 + cx]     = cc;
        fb[y * pitch4 + cx + 1] = cc;
    }
    for (int x = 0; x < w; x++) {
        fb[(cy - 1) * pitch4 + x] = cc;
        fb[cy * pitch4 + x]       = cc;
        fb[(cy + 1) * pitch4 + x] = cc;
    }
}

static void draw_crosshair(uint32_t *fb, int pitch4, int w, int h)
{
    int cx = g_cursor_x, cy = g_cursor_y;
    uint32_t color = 0x0000FFFF;
    int arm = 40, gap = 4;
    for (int dx = gap; dx <= arm; dx++) {
        if (cx + dx < w) fb[cy * pitch4 + cx + dx] = color;
        if (cx - dx >= 0) fb[cy * pitch4 + cx - dx] = color;
    }
    for (int dy = gap; dy <= arm; dy++) {
        if (cy + dy < h) fb[(cy + dy) * pitch4 + cx] = color;
        if (cy - dy >= 0) fb[(cy - dy) * pitch4 + cx] = color;
    }
}

static const uint32_t calib_colors[4] = {
    0x00FF0000, 0x0000FF00, 0x000080FF, 0x0000FFFF,
};

static void draw_calib_points(uint32_t *fb, int pitch4, int w, int h,
                              int crop_x, int crop_y, int crop_w, int crop_h,
                              int src_w, int src_h)
{
    int npts = calib_npts;
    int prev_dx = 0, prev_dy = 0, prev_line = -1;
    for (int i = 0; i < npts; i++) {
        double sx = calib_pts[i].px * src_w;
        double sy = calib_pts[i].py * src_h;
        int px = (int)((sx - crop_x) * w / crop_w);
        int py = (int)((sy - crop_y) * h / crop_h);
        uint32_t color = calib_colors[calib_pts[i].line_idx % 4];
        int visible = (px >= -3 && px < w + 3 && py >= -3 && py < h + 3);
        if (visible) {
            for (int dy = -3; dy <= 2; dy++) {
                int ry = py + dy;
                if (ry < 0 || ry >= h) continue;
                for (int dx = -3; dx <= 2; dx++) {
                    int rx = px + dx;
                    if (rx < 0 || rx >= w) continue;
                    fb[ry * pitch4 + rx] = color;
                }
            }
            char ch = '0' + (calib_pts[i].line_idx % 10);
            int lx = px + 4, ly = py - 10;
            if (lx >= 0 && lx < w - 8 && ly >= 0 && ly < h - 8) {
                const uint8_t *g = font8x8[ch - 32];
                for (int row = 0; row < 8; row++)
                    for (int col = 0; col < 8; col++)
                        if (g[row] & (0x80 >> col)) {
                            int fxx = lx + col, fyy = ly + row;
                            if (fxx < w && fyy < h)
                                fb[fyy * pitch4 + fxx] = color;
                        }
            }
        }
        if (calib_pts[i].line_idx == prev_line) {
            int x0 = prev_dx, y0 = prev_dy, x1 = px, y1 = py;
            int adx = abs(x1 - x0), ady = abs(y1 - y0);
            int stepx = x0 < x1 ? 1 : -1, stepy = y0 < y1 ? 1 : -1;
            int err = adx - ady;
            for (;;) {
                if (x0 >= 0 && x0 < w && y0 >= 0 && y0 < h)
                    fb[y0 * pitch4 + x0] = color;
                if (x0 == x1 && y0 == y1) break;
                int e2 = 2 * err;
                if (e2 > -ady) { err -= ady; x0 += stepx; }
                if (e2 < adx) { err += adx; y0 += stepy; }
            }
        }
        prev_dx = px; prev_dy = py;
        prev_line = calib_pts[i].line_idx;
    }
}

/* ─────────────────── OSD rendering ─────────────────── */

#define OSD_SCALE  2
#define OSD_CW     (8 * OSD_SCALE)
#define OSD_LH     (10 * OSD_SCALE)

static void osd_rect(uint32_t *fb, int pitch4, int x, int y, int w, int h, uint32_t color)
{
    for (int r = y; r < y + h; r++)
        for (int c = x; c < x + w; c++)
            fb[r * pitch4 + c] = color;
}

static void osd_char(uint32_t *fb, int pitch4, int x, int y, char ch, uint32_t color)
{
    if (ch < 32 || ch > 126) return;
    const uint8_t *g = font8x8[ch - 32];
    for (int row = 0; row < 8; row++)
        for (int col = 0; col < 8; col++)
            if (g[row] & (0x80 >> col))
                for (int sy = 0; sy < OSD_SCALE; sy++)
                    for (int sx = 0; sx < OSD_SCALE; sx++)
                        fb[(y + row * OSD_SCALE + sy) * pitch4 +
                            x + col * OSD_SCALE + sx] = color;
}

static void osd_str(uint32_t *fb, int pitch4, int x, int y, const char *s, uint32_t color)
{
    while (*s) { osd_char(fb, pitch4, x, y, *s, color); x += OSD_CW; s++; }
}

static void draw_osd_single(void *map, int pitch, int disp_w, int disp_h,
                             double fps, int ch, double k1, double k2,
                             double cx, double cy, int grid_on, int bypass)
{
    (void)disp_w;
    uint32_t *fb = (uint32_t *)map;
    int p4 = pitch / 4;
    int ox = 16, oy = 12;
    int osd_w = 52 * OSD_CW + 16;
    int osd_h = 14 * OSD_LH + 12;
    osd_rect(fb, p4, ox - 8, oy - 6, osd_w, osd_h, 0x00181818);

    char line[80];
    snprintf(line, sizeof(line), "=== ch%d Calibration [%.0f fps] ===", ch, fps);
    osd_str(fb, p4, ox, oy, line, 0x0000CCCC);
    oy += OSD_LH * 2;

    snprintf(line, sizeof(line), "> k1 (barrel/pin):  %+.3f", k1);
    osd_str(fb, p4, ox, oy, line, k1 != 0.0 ? 0x0000FF00 : 0x00CCCCCC);
    oy += OSD_LH;

    snprintf(line, sizeof(line), "  k2 (4th order):   %+.4f", k2);
    osd_str(fb, p4, ox, oy, line, k2 != 0.0 ? 0x00FFFF00 : 0x00CCCCCC);
    oy += OSD_LH;

    snprintf(line, sizeof(line), "  cx (center X):    %.3f", cx);
    osd_str(fb, p4, ox, oy, line, cx != 0.5 ? 0x00FFFF00 : 0x00CCCCCC);
    oy += OSD_LH;

    snprintf(line, sizeof(line), "  cy (center Y):    %.3f", cy);
    osd_str(fb, p4, ox, oy, line, cy != 0.5 ? 0x00FFFF00 : 0x00CCCCCC);
    oy += OSD_LH;

    oy += OSD_LH / 2;
    snprintf(line, sizeof(line), "[GRID: %s]  [%s]  [ZOOM: %.1fx]",
             grid_on ? "ON" : "OFF", bypass ? "BYPASS" : "CORRECTED", g_zoom / 10.0);
    osd_str(fb, p4, ox, oy, line, bypass ? 0x00FF8844 : 0x0000FF00);
    oy += OSD_LH;

    oy += OSD_LH / 2;
    {
        int npts = calib_npts;
        int nlines_with_pts = 0;
        if (npts > 0) {
            int seen[MAX_CALIB_LINES], nseen = 0;
            for (int i = 0; i < npts; i++) {
                int lidx = calib_pts[i].line_idx;
                int found = 0;
                for (int j = 0; j < nseen; j++)
                    if (seen[j] == lidx) { found = 1; break; }
                if (!found && nseen < MAX_CALIB_LINES)
                    seen[nseen++] = lidx;
            }
            nlines_with_pts = nseen;
        }
        snprintf(line, sizeof(line), "[CALIB: %d lines, %d pts]  A=auto D=del Tab=line F=fit X=clr",
                 nlines_with_pts, npts);
        osd_str(fb, p4, ox, oy, line, npts > 0 ? 0x0000CCFF : 0x00666666);
    }
    oy += OSD_LH;

    oy += OSD_LH / 2;
    osd_str(fb, p4, ox, oy,
            "Arrows=k1/k2 C/V=cx B/N=cy G=grid SPC=bypass",
            0x00666666);
    oy += OSD_LH;
    osd_str(fb, p4, ox, oy,
            "S=save L=load R=reset +/-=zoom ESC=back Q=quit",
            0x00666666);
}

/* ─────────────────── Selection border (MODE_QUAD) ─────────────────── */

static void draw_selection_border(uint32_t *fb, int pitch4, int disp_w, int disp_h,
                                  int selected, int blink_on)
{
    if (!blink_on) return;

    int qw = disp_w / 2, qh = disp_h / 2;
    int pos[4][2] = { {0,0}, {qw,0}, {0,qh}, {qw,qh} };
    int bx = pos[selected][0], by = pos[selected][1];
    uint32_t color = 0x0000FF00;  /* bright green */
    int thick = 4;

    /* Top edge */
    for (int t = 0; t < thick; t++)
        for (int x = bx; x < bx + qw; x++)
            if (by + t < disp_h)
                fb[(by + t) * pitch4 + x] = color;
    /* Bottom edge */
    for (int t = 0; t < thick; t++)
        for (int x = bx; x < bx + qw; x++)
            if (by + qh - 1 - t >= 0 && by + qh - 1 - t < disp_h)
                fb[(by + qh - 1 - t) * pitch4 + x] = color;
    /* Left edge */
    for (int t = 0; t < thick; t++)
        for (int y = by; y < by + qh; y++)
            if (bx + t < disp_w)
                fb[y * pitch4 + bx + t] = color;
    /* Right edge */
    for (int t = 0; t < thick; t++)
        for (int y = by; y < by + qh; y++)
            if (bx + qw - 1 - t >= 0 && bx + qw - 1 - t < disp_w)
                fb[y * pitch4 + bx + qw - 1 - t] = color;
}

/* Small channel labels in quad mode */
static void draw_quad_labels(uint32_t *fb, int pitch4, int disp_w, int disp_h,
                             struct cam cams[], int ncams)
{
    int qw = disp_w / 2, qh = disp_h / 2;
    int pos[4][2] = { {0,0}, {qw,0}, {0,qh}, {qw,qh} };
    for (int i = 0; i < ncams; i++) {
        if (!cams[i].active) continue;
        char label[8];
        snprintf(label, sizeof(label), "ch%d", i);
        int lx = pos[i][0] + 8, ly = pos[i][1] + 6;
        /* Background behind label */
        osd_rect(fb, pitch4, lx - 2, ly - 2,
                 (int)strlen(label) * OSD_CW + 4, OSD_LH + 4, 0x00181818);
        osd_str(fb, pitch4, lx, ly, label,
                cams[i].cal.k1 != 0.0 || cams[i].cal.k2 != 0.0
                    ? 0x0000FF00 : 0x00CCCCCC);
    }
}

/* ─────────────────── System stats ─────────────────── */

static int read_gpu_util(void)
{
    /* Try devfreq load first (format: "NN@FREQHz"), then utilisation */
    static const char *load_paths[] = {
        "/sys/devices/platform/27800000.gpu/devfreq/27800000.gpu/load",
        NULL,
    };
    static const char *util_paths[] = {
        "/sys/devices/platform/27800000.gpu/utilisation",
        "/sys/devices/platform/fb000000.gpu/utilisation",
        NULL,
    };
    for (const char **p = load_paths; *p; p++) {
        int fd = open(*p, O_RDONLY);
        if (fd < 0) continue;
        char buf[64] = {};
        int n = read(fd, buf, sizeof(buf) - 1);
        close(fd);
        if (n > 0) { buf[n] = 0; return atoi(buf); } /* NN@freq -> atoi stops at @ */
    }
    for (const char **p = util_paths; *p; p++) {
        int fd = open(*p, O_RDONLY);
        if (fd < 0) continue;
        char buf[16] = {};
        int n = read(fd, buf, sizeof(buf) - 1);
        close(fd);
        if (n > 0) { buf[n] = 0; return atoi(buf); }
    }
    return -1;
}

/* Returns total CPU usage 0-100 between calls.
 * /proc/stat "cpu" line aggregates all cores: the idle ticks scale with
 * core count, so (busy / total) already gives the all-core average. */
static int read_cpu_util(void)
{
    static long prev_total, prev_idle;
    FILE *f = fopen("/proc/stat", "r");
    if (!f) return -1;
    long user, nice, sys, idle, iowait, irq, sirq, steal;
    if (fscanf(f, "cpu %ld %ld %ld %ld %ld %ld %ld %ld",
               &user, &nice, &sys, &idle, &iowait, &irq, &sirq, &steal) != 8) {
        fclose(f);
        return -1;
    }
    fclose(f);
    long total = user + nice + sys + idle + iowait + irq + sirq + steal;
    long idle_all = idle + iowait;
    long dt = total - prev_total;
    long di = idle_all - prev_idle;
    prev_total = total;
    prev_idle = idle_all;
    if (dt == 0) return 0;
    return (int)(100 * (dt - di) / dt);
}

static void draw_sys_stats(uint32_t *fb, int pitch4, int disp_w, int disp_h,
                            int gpu_pct, int cpu_pct, double fps)
{
    char line[48];
    snprintf(line, sizeof(line), "CPU:%2d%%  GPU:%2d%%  %.0ffps", cpu_pct, gpu_pct, fps);
    int len = (int)strlen(line);
    int lx = disp_w - len * OSD_CW - 10;
    int ly = disp_h - OSD_LH - 8;
    osd_rect(fb, pitch4, lx - 4, ly - 2, len * OSD_CW + 8, OSD_LH + 4, 0x00181818);
    osd_str(fb, pitch4, lx, ly, line, 0x00AAAAAA);
}

/* ─────────────────── Evdev keyboard + mouse ─────────────────── */

#define TEST_BIT(bit, arr) \
    ((arr)[(bit) / (8 * sizeof(unsigned long))] & \
     (1UL << ((bit) % (8 * sizeof(unsigned long)))))

static int find_keyboard(int exclude_fd)
{
    /* Get dev number of excluded fd (mouse) to avoid grabbing same device */
    dev_t excl_dev = 0;
    if (exclude_fd >= 0) {
        struct stat st;
        if (fstat(exclude_fd, &st) == 0) excl_dev = st.st_rdev;
    }

    char path[64];
    for (int i = 0; i < 20; i++) {
        snprintf(path, sizeof(path), "/dev/input/event%d", i);
        int fd = open(path, O_RDONLY | O_NONBLOCK);
        if (fd < 0) continue;

        /* Skip if same underlying device as mouse */
        if (excl_dev) {
            struct stat st;
            if (fstat(fd, &st) == 0 && st.st_rdev == excl_dev)
                { close(fd); continue; }
        }

        unsigned long evbits[1] = {0};
        if (ioctl(fd, EVIOCGBIT(0, sizeof(evbits)), evbits) < 0 ||
            !TEST_BIT(EV_KEY, evbits)) { close(fd); continue; }

        /* Reject devices with REL_X (mice/trackpads) */
        if (TEST_BIT(EV_REL, evbits)) {
            unsigned long relbits[1] = {0};
            ioctl(fd, EVIOCGBIT(EV_REL, sizeof(relbits)), relbits);
            if (TEST_BIT(REL_X, relbits)) { close(fd); continue; }
        }

        unsigned long keybits[128 / sizeof(unsigned long)];
        memset(keybits, 0, sizeof(keybits));
        ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(keybits)), keybits);
        if (TEST_BIT(KEY_Q, keybits) && TEST_BIT(KEY_1, keybits) &&
            TEST_BIT(KEY_A, keybits) && TEST_BIT(KEY_UP, keybits)) {
            char name[256] = "Unknown";
            ioctl(fd, EVIOCGNAME(sizeof(name)), name);
            printf("Keyboard: %s (%s)\n", name, path);
            if (ioctl(fd, EVIOCGRAB, (void *)1) < 0) {
                close(fd); continue;  /* already grabbed */
            }
            return fd;
        }
        close(fd);
    }
    return -1;
}

static int find_mouse(void)
{
    char path[64];
    for (int i = 0; i < 20; i++) {
        snprintf(path, sizeof(path), "/dev/input/event%d", i);
        int fd = open(path, O_RDONLY | O_NONBLOCK);
        if (fd < 0) continue;
        unsigned long evbits[1] = {0};
        if (ioctl(fd, EVIOCGBIT(0, sizeof(evbits)), evbits) < 0 ||
            !TEST_BIT(EV_REL, evbits)) { close(fd); continue; }
        unsigned long relbits[1] = {0};
        ioctl(fd, EVIOCGBIT(EV_REL, sizeof(relbits)), relbits);
        unsigned long keybits[128 / sizeof(unsigned long)];
        memset(keybits, 0, sizeof(keybits));
        ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(keybits)), keybits);
        if (TEST_BIT(REL_X, relbits) && TEST_BIT(REL_Y, relbits) &&
            TEST_BIT(BTN_LEFT, keybits)) {
            char name[256] = "Unknown";
            ioctl(fd, EVIOCGNAME(sizeof(name)), name);
            printf("Mouse: %s (%s)\n", name, path);
            ioctl(fd, EVIOCGRAB, (void *)1);
            return fd;
        }
        close(fd);
    }
    return -1;
}

/* ─────────────────── Terminal (SSH fallback) ─────────────────── */

static void term_restore(void)
{
    if (tio_saved)
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_tio);
}

static int stdin_usable = 0;

static void term_raw(void)
{
    if (!isatty(STDIN_FILENO)) return;
    struct termios raw;
    if (tcgetattr(STDIN_FILENO, &orig_tio) == 0) {
        tio_saved = 1;
        atexit(term_restore);
        raw = orig_tio;
        raw.c_lflag &= ~(tcflag_t)(ICANON | ECHO);
        raw.c_cc[VMIN]  = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
    }
    fcntl(STDIN_FILENO, F_SETFL,
          fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);
    stdin_usable = 1;
}

/* ─────────────────── Key action handler (mode-aware) ─────────────────── */

/* Active camera list for cycling (set in main before input thread starts) */
static int active_cams[MAX_CAMS];
static int num_active = 0;

static void handle_action(int action)
{
    pthread_mutex_lock(&param_lock);
    int mode = g_mode;

    if (mode == MODE_QUAD) {
        switch (action) {
        case 'q': quit = 1; break;
        case 'r': { /* right: next active cam */
            int cur = g_selected;
            for (int i = 0; i < num_active; i++) {
                if (active_cams[i] == cur) {
                    g_selected = active_cams[(i + 1) % num_active];
                    break;
                }
            }
            break;
        }
        case 'l': { /* left: prev active cam */
            int cur = g_selected;
            for (int i = 0; i < num_active; i++) {
                if (active_cams[i] == cur) {
                    g_selected = active_cams[(i - 1 + num_active) % num_active];
                    break;
                }
            }
            break;
        }
        case '\n': /* Enter: transition to single mode */
            g_enter_single = 1;
            break;
        }
    } else { /* MODE_SINGLE */
        switch (action) {
        case 'q': quit = 1; break;
        case 'E': /* ESC: back to quad */
            g_exit_single = 1;
            break;
        case 'r': g_k1 += 0.01; break;
        case 'l': g_k1 -= 0.01; break;
        case 'u': g_k2 += 0.005; break;
        case 'd': g_k2 -= 0.005; break;
        case 'C': g_cx -= 0.01; if (g_cx < 0.0) g_cx = 0.0; break;
        case 'V': g_cx += 0.01; if (g_cx > 1.0) g_cx = 1.0; break;
        case 'B': g_cy -= 0.01; if (g_cy < 0.0) g_cy = 0.0; break;
        case 'N': g_cy += 0.01; if (g_cy > 1.0) g_cy = 1.0; break;
        case 'G': g_grid = !g_grid; break;
        case ' ': g_bypass = !g_bypass; break;
        case 'S': g_save_req = 1; break;
        case 'L': g_load_req = 1; break;
        case 'R': g_reset_req = 1; break;
        case 'T': /* Tab: new calib line */
            if (calib_line < MAX_CALIB_LINES - 1) {
                calib_line++;
                printf("Calib: new line %d\n", calib_line);
            }
            break;
        case 'F': g_fit_req = 1; break;
        case 'X': calib_npts = 0; calib_line = 0; printf("Calib: cleared\n"); break;
        case 'Z': /* Backspace: undo */
            if (calib_npts > 0) { calib_npts--; printf("Calib: undo -> %d pts\n", calib_npts); }
            break;
        case 'A': g_autodetect_req = 1; break;
        case 'D': /* Delete last calib line */
            if (calib_line >= 0 && calib_npts > 0) {
                int del_line = calib_line;
                if (del_line > 0) del_line--;  /* delete highest used line */
                /* Find the highest line_idx actually used */
                int max_li = -1;
                for (int i = 0; i < calib_npts; i++)
                    if (calib_pts[i].line_idx > max_li) max_li = calib_pts[i].line_idx;
                if (max_li >= 0) {
                    /* Remove all points with that line_idx */
                    int dst = 0;
                    for (int i = 0; i < calib_npts; i++)
                        if (calib_pts[i].line_idx != max_li)
                            calib_pts[dst++] = calib_pts[i];
                    calib_npts = dst;
                    /* Update calib_line to next available */
                    int new_max = -1;
                    for (int i = 0; i < calib_npts; i++)
                        if (calib_pts[i].line_idx > new_max) new_max = calib_pts[i].line_idx;
                    calib_line = new_max + 1;
                    printf("Calib: deleted line %d, %d pts remain\n", max_li, calib_npts);
                }
            }
            break;
        case '+': if (g_zoom < MAX_ZOOM) { g_zoom++; printf("Zoom: %.1fx\n", g_zoom / 10.0); } break;
        case '-': if (g_zoom > MIN_ZOOM) { g_zoom--; printf("Zoom: %.1fx\n", g_zoom / 10.0); } break;
        }
    }

    pthread_mutex_unlock(&param_lock);
}

static int evdev_map_key(int code)
{
    int mode = g_mode;
    switch (code) {
    case KEY_Q:            return 'q';
    case KEY_ESC:          return mode == MODE_SINGLE ? 'E' : 'q';
    case KEY_UP:           return 'u';
    case KEY_DOWN:         return 'd';
    case KEY_LEFT:         return 'l';
    case KEY_RIGHT:        return 'r';
    case KEY_ENTER:
    case KEY_KPENTER:      return '\n';
    case KEY_G:            return 'G';
    case KEY_SPACE:        return ' ';
    case KEY_S:            return 'S';
    case KEY_L:            return 'L';
    case KEY_R:            return 'R';
    case KEY_C:            return 'C';
    case KEY_V:            return 'V';
    case KEY_B:            return 'B';
    case KEY_N:            return 'N';
    case KEY_TAB:          return 'T';
    case KEY_F:            return 'F';
    case KEY_A:            return 'A';
    case KEY_D:            return 'D';
    case KEY_X:            return 'X';
    case KEY_BACKSPACE:    return 'Z';
    case KEY_EQUAL:
    case KEY_KPPLUS:       return '+';
    case KEY_MINUS:
    case KEY_KPMINUS:      return '-';
    default:               return 0;
    }
}

/* ─────────────────── Fit scale (prevent zoom-in from undistort) ─────────── */

static float compute_fit_scale(float k1, float k2, float cx, float cy,
                               float w, float h, float max_r)
{
    /* Compute distortion scale at all 4 corners, take minimum.
     * For barrel correction (k1<0), s<1 at edges → image zooms in.
     * Dividing by min(s) in the shader keeps the full image visible. */
    float corners[4][2] = {
        {0.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 1.0f}, {1.0f, 1.0f}
    };
    float min_s = 1.0f;
    for (int i = 0; i < 4; i++) {
        float dx = (corners[i][0] - cx) * w;
        float dy = (corners[i][1] - cy) * h;
        float r2 = (dx*dx + dy*dy) / (max_r * max_r);
        float s = 1.0f + k1 * r2 + k2 * r2 * r2;
        if (s < min_s) min_s = s;
    }
    /* Also check edge midpoints */
    float edges[4][2] = {
        {0.5f, 0.0f}, {0.5f, 1.0f}, {0.0f, 0.5f}, {1.0f, 0.5f}
    };
    for (int i = 0; i < 4; i++) {
        float dx = (edges[i][0] - cx) * w;
        float dy = (edges[i][1] - cy) * h;
        float r2 = (dx*dx + dy*dy) / (max_r * max_r);
        float s = 1.0f + k1 * r2 + k2 * r2 * r2;
        if (s < min_s) min_s = s;
    }
    if (min_s < 0.1f) min_s = 0.1f;  /* clamp to avoid division issues */
    return min_s;
}

/* ─────────────────── Auto edge detection for calibration ─────────────────── */

static void auto_detect_edges(const uint8_t *uyvy, int w, int h)
{
    printf("Auto-detect: analyzing %dx%d frame...\n", w, h);

    /* 1. Extract Y channel from UYVY (Y at byte offsets 1,3,5,7...) */
    uint8_t *Y = malloc(w * h);
    if (!Y) { printf("Auto-detect: malloc failed\n"); return; }
    int bstride = w * 2;
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            Y[y * w + x] = uyvy[y * bstride + x * 2 + 1];

    /* 2. Light 3x3 box blur to reduce noise */
    uint8_t *Yb = malloc(w * h);
    if (!Yb) { free(Y); return; }
    memcpy(Yb, Y, w * h);
    for (int y = 1; y < h - 1; y++)
        for (int x = 1; x < w - 1; x++) {
            int sum = 0;
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++)
                    sum += Y[(y + dy) * w + (x + dx)];
            Yb[y * w + x] = sum / 9;
        }
    free(Y);

    /* 3. Compute Sobel Gy (horizontal edges) and Gx (vertical edges) */
    int16_t *gy = calloc(w * h, sizeof(int16_t));
    int16_t *gx = calloc(w * h, sizeof(int16_t));
    if (!gy || !gx) { free(Yb); free(gy); free(gx); return; }
    for (int y = 1; y < h - 1; y++)
        for (int x = 1; x < w - 1; x++) {
            gy[y*w+x] = (int16_t)(
                -Yb[(y-1)*w+x-1] - 2*Yb[(y-1)*w+x] - Yb[(y-1)*w+x+1]
                +Yb[(y+1)*w+x-1] + 2*Yb[(y+1)*w+x] + Yb[(y+1)*w+x+1]);
            gx[y*w+x] = (int16_t)(
                -Yb[(y-1)*w+x-1] + Yb[(y-1)*w+x+1]
                -2*Yb[y*w+x-1]   + 2*Yb[y*w+x+1]
                -Yb[(y+1)*w+x-1] + Yb[(y+1)*w+x+1]);
        }
    free(Yb);

    /* Clear existing calibration */
    calib_npts = 0;
    calib_line = 0;

    int pts_per_line = 8;
    int min_gradient = 30;
    int search_r = 5;

    /* ── 4. Find horizontal edge lines ── */
    {
        /* Accumulate |Gy| per row */
        int *row_str = calloc(h, sizeof(int));
        for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++)
                row_str[y] += abs(gy[y * w + x]);

        /* Find peak rows, spaced apart */
        int min_sp = h / 10;
        int max_hl = 4;
        int peaks[8], npeaks = 0;
        int thresh = w * 8;

        for (int pass = 0; pass < max_hl; pass++) {
            int best_r = -1, best_v = 0;
            for (int y = 20; y < h - 20; y++) {
                int skip = 0;
                for (int j = 0; j < npeaks; j++)
                    if (abs(y - peaks[j]) < min_sp) { skip = 1; break; }
                if (skip) continue;
                if (row_str[y] > best_v) { best_v = row_str[y]; best_r = y; }
            }
            if (best_r < 0 || best_v < thresh) break;
            peaks[npeaks++] = best_r;
        }
        free(row_str);

        /* Track edge along each peak row */
        for (int li = 0; li < npeaks; li++) {
            int seed_row = peaks[li];
            int *ty = malloc(w * sizeof(int));
            uint8_t *tv = calloc(w, 1);
            if (!ty || !tv) { free(ty); free(tv); continue; }

            /* Find seed at center column */
            int cx = w / 2;
            int by = seed_row, bg = 0;
            for (int d = -search_r; d <= search_r; d++) {
                int r = seed_row + d;
                if (r < 1 || r >= h - 1) continue;
                int g = abs(gy[r * w + cx]);
                if (g > bg) { bg = g; by = r; }
            }
            if (bg < min_gradient) { free(ty); free(tv); continue; }
            ty[cx] = by; tv[cx] = 1;

            /* Track right */
            int cy = by;
            for (int x = cx + 1; x < w - 1; x++) {
                int bb = cy, gg = 0;
                for (int d = -3; d <= 3; d++) {
                    int r = cy + d;
                    if (r < 1 || r >= h - 1) continue;
                    int g = abs(gy[r * w + x]);
                    if (g > gg) { gg = g; bb = r; }
                }
                if (gg < min_gradient) break;
                ty[x] = bb; tv[x] = 1; cy = bb;
            }

            /* Track left */
            cy = by;
            for (int x = cx - 1; x >= 1; x--) {
                int bb = cy, gg = 0;
                for (int d = -3; d <= 3; d++) {
                    int r = cy + d;
                    if (r < 1 || r >= h - 1) continue;
                    int g = abs(gy[r * w + x]);
                    if (g > gg) { gg = g; bb = r; }
                }
                if (gg < min_gradient) break;
                ty[x] = bb; tv[x] = 1; cy = bb;
            }

            /* Check span */
            int xmin = w, xmax = 0;
            for (int x = 0; x < w; x++)
                if (tv[x]) { if (x < xmin) xmin = x; if (x > xmax) xmax = x; }
            int span = xmax - xmin + 1;

            if (span * 100 / w >= 40) {
                int step = span / (pts_per_line + 1);
                int added = 0;
                for (int si = 0; si < pts_per_line && calib_npts < MAX_CALIB_PTS; si++) {
                    int sx = xmin + step * (si + 1);
                    if (sx >= w || !tv[sx]) continue;
                    calib_pts[calib_npts].line_idx = calib_line;
                    calib_pts[calib_npts].px = (double)sx / w;
                    calib_pts[calib_npts].py = (double)ty[sx] / h;
                    calib_pts[calib_npts].dx = sx;
                    calib_pts[calib_npts].dy = ty[sx];
                    calib_npts++; added++;
                }
                printf("  H-line %d: row~%d span=%d-%d (%d%%) %d pts\n",
                       calib_line, seed_row, xmin, xmax, span*100/w, added);
                calib_line++;
            }
            free(ty); free(tv);
        }
    }

    /* ── 5. Find vertical edge lines ── */
    {
        int *col_str = calloc(w, sizeof(int));
        for (int x = 0; x < w; x++)
            for (int y = 0; y < h; y++)
                col_str[x] += abs(gx[y * w + x]);

        int min_sp = w / 10;
        int max_vl = 4;
        int peaks[8], npeaks = 0;
        int thresh = h * 8;

        for (int pass = 0; pass < max_vl; pass++) {
            int best_c = -1, best_v = 0;
            for (int x = 20; x < w - 20; x++) {
                int skip = 0;
                for (int j = 0; j < npeaks; j++)
                    if (abs(x - peaks[j]) < min_sp) { skip = 1; break; }
                if (skip) continue;
                if (col_str[x] > best_v) { best_v = col_str[x]; best_c = x; }
            }
            if (best_c < 0 || best_v < thresh) break;
            peaks[npeaks++] = best_c;
        }
        free(col_str);

        for (int li = 0; li < npeaks; li++) {
            int seed_col = peaks[li];
            int *tx = malloc(h * sizeof(int));
            uint8_t *tv = calloc(h, 1);
            if (!tx || !tv) { free(tx); free(tv); continue; }

            int cy = h / 2;
            int bx = seed_col, bg = 0;
            for (int d = -search_r; d <= search_r; d++) {
                int c = seed_col + d;
                if (c < 1 || c >= w - 1) continue;
                int g = abs(gx[cy * w + c]);
                if (g > bg) { bg = g; bx = c; }
            }
            if (bg < min_gradient) { free(tx); free(tv); continue; }
            tx[cy] = bx; tv[cy] = 1;

            /* Track down */
            int cx = bx;
            for (int y = cy + 1; y < h - 1; y++) {
                int bb = cx, gg = 0;
                for (int d = -3; d <= 3; d++) {
                    int c = cx + d;
                    if (c < 1 || c >= w - 1) continue;
                    int g = abs(gx[y * w + c]);
                    if (g > gg) { gg = g; bb = c; }
                }
                if (gg < min_gradient) break;
                tx[y] = bb; tv[y] = 1; cx = bb;
            }

            /* Track up */
            cx = bx;
            for (int y = cy - 1; y >= 1; y--) {
                int bb = cx, gg = 0;
                for (int d = -3; d <= 3; d++) {
                    int c = cx + d;
                    if (c < 1 || c >= w - 1) continue;
                    int g = abs(gx[y * w + c]);
                    if (g > gg) { gg = g; bb = c; }
                }
                if (gg < min_gradient) break;
                tx[y] = bb; tv[y] = 1; cx = bb;
            }

            int ymin = h, ymax = 0;
            for (int y = 0; y < h; y++)
                if (tv[y]) { if (y < ymin) ymin = y; if (y > ymax) ymax = y; }
            int span = ymax - ymin + 1;

            if (span * 100 / h >= 40) {
                int step = span / (pts_per_line + 1);
                int added = 0;
                for (int si = 0; si < pts_per_line && calib_npts < MAX_CALIB_PTS; si++) {
                    int sy = ymin + step * (si + 1);
                    if (sy >= h || !tv[sy]) continue;
                    calib_pts[calib_npts].line_idx = calib_line;
                    calib_pts[calib_npts].px = (double)tx[sy] / w;
                    calib_pts[calib_npts].py = (double)sy / h;
                    calib_pts[calib_npts].dx = tx[sy];
                    calib_pts[calib_npts].dy = sy;
                    calib_npts++; added++;
                }
                printf("  V-line %d: col~%d span=%d-%d (%d%%) %d pts\n",
                       calib_line, seed_col, ymin, ymax, span*100/h, added);
                calib_line++;
            }
            free(tx); free(tv);
        }
    }

    free(gx);
    free(gy);
    printf("Auto-detect: %d lines, %d total points\n", calib_line, calib_npts);
}

/* ─────────────────── Auto-fit k1/k2 from calibration points ─────────────────── */

static void fit_undistort_point(double dx, double dy,
                                double cx, double cy, double max_r,
                                double k1, double k2,
                                double *ux, double *uy)
{
    double x = dx, y = dy;
    for (int iter = 0; iter < 20; iter++) {
        double rx = (x - cx) / max_r, ry = (y - cy) / max_r;
        double r2 = rx * rx + ry * ry, r4 = r2 * r2;
        double scale = 1.0 + k1 * r2 + k2 * r4;
        double fx = cx + (x - cx) * scale;
        double fy = cy + (y - cy) * scale;
        double ex = fx - dx, ey = fy - dy;
        double dscale_dr2 = k1 + 2.0 * k2 * r2;
        double j = scale + 2.0 * dscale_dr2 * r2;
        if (fabs(j) < 1e-10) break;
        x -= ex / j; y -= ey / j;
        if (ex * ex + ey * ey < 1e-6) break;
    }
    *ux = x; *uy = y;
}

static double fit_line_error(const double *xs, const double *ys, int n)
{
    if (n < 3) return 0.0;
    double mx = 0, my = 0;
    for (int i = 0; i < n; i++) { mx += xs[i]; my += ys[i]; }
    mx /= n; my /= n;
    double cxx = 0, cxy = 0, cyy = 0;
    for (int i = 0; i < n; i++) {
        double dx = xs[i] - mx, dy = ys[i] - my;
        cxx += dx * dx; cxy += dx * dy; cyy += dy * dy;
    }
    cxx /= n; cxy /= n; cyy /= n;
    double trace = cxx + cyy, det = cxx * cyy - cxy * cxy;
    double disc = trace * trace - 4.0 * det;
    if (disc < 0) disc = 0;
    return (trace - sqrt(disc)) * 0.5;
}

static double fit_eval_error(double k1, double k2,
                             double cx, double cy, double max_r,
                             int w, int h)
{
    double total = 0.0;
    int processed[MAX_CALIB_LINES], nproc = 0;
    int npts = calib_npts;
    for (int i = 0; i < npts; i++) {
        int lidx = calib_pts[i].line_idx;
        int found = 0;
        for (int j = 0; j < nproc; j++)
            if (processed[j] == lidx) { found = 1; break; }
        if (found) continue;
        if (nproc >= MAX_CALIB_LINES) break;
        processed[nproc++] = lidx;
        double uxs[MAX_CALIB_PTS], uys[MAX_CALIB_PTS];
        int count = 0;
        for (int j = 0; j < npts; j++) {
            if (calib_pts[j].line_idx != lidx) continue;
            fit_undistort_point(calib_pts[j].px * w, calib_pts[j].py * h,
                                cx, cy, max_r, k1, k2,
                                &uxs[count], &uys[count]);
            count++;
        }
        if (count >= 3)
            total += fit_line_error(uxs, uys, count);
    }
    return total;
}

static int auto_fit_k1k2(int src_w, int src_h, double cx_frac, double cy_frac)
{
    int npts = calib_npts;
    if (npts < 6) { printf("Fit: need >= 6 points (have %d)\n", npts); return -1; }
    int line_counts[MAX_CALIB_LINES], line_ids[MAX_CALIB_LINES], nlines = 0;
    for (int i = 0; i < npts; i++) {
        int lidx = calib_pts[i].line_idx, found = -1;
        for (int j = 0; j < nlines; j++)
            if (line_ids[j] == lidx) { found = j; break; }
        if (found >= 0) line_counts[found]++;
        else if (nlines < MAX_CALIB_LINES) {
            line_ids[nlines] = lidx; line_counts[nlines] = 1; nlines++;
        }
    }
    int valid = 0;
    for (int i = 0; i < nlines; i++) if (line_counts[i] >= 3) valid++;
    if (valid < 2) { printf("Fit: need >= 2 lines with >= 3 pts (have %d)\n", valid); return -1; }

    double cx = cx_frac * src_w, cy = cy_frac * src_h;
    double max_r = sqrt((src_w * 0.5) * (src_w * 0.5) + (src_h * 0.5) * (src_h * 0.5));

    double best_k1 = 0, best_k2 = 0, best_err = 1e30;
    for (double tk1 = -0.8; tk1 <= 0.801; tk1 += 0.02)
        for (double tk2 = -0.3; tk2 <= 0.301; tk2 += 0.01) {
            double err = fit_eval_error(tk1, tk2, cx, cy, max_r, src_w, src_h);
            if (err < best_err) { best_err = err; best_k1 = tk1; best_k2 = tk2; }
        }
    double fine_k1 = best_k1, fine_k2 = best_k2, fine_err = best_err;
    for (double tk1 = best_k1 - 0.02; tk1 <= best_k1 + 0.0201; tk1 += 0.002)
        for (double tk2 = best_k2 - 0.01; tk2 <= best_k2 + 0.0101; tk2 += 0.001) {
            double err = fit_eval_error(tk1, tk2, cx, cy, max_r, src_w, src_h);
            if (err < fine_err) { fine_err = err; fine_k1 = tk1; fine_k2 = tk2; }
        }
    printf("Fit: k1=%.4f k2=%.4f (error=%.6f)\n", fine_k1, fine_k2, fine_err);
    pthread_mutex_lock(&param_lock);
    g_k1 = fine_k1; g_k2 = fine_k2;
    pthread_mutex_unlock(&param_lock);
    return 0;
}

/* ─────────────────── Input thread ─────────────────── */

static void *input_thread(void *arg)
{
    (void)arg;
    int mouse_fd = find_mouse();
    if (mouse_fd < 0) printf("No USB mouse — plug one in (hotplug supported)\n");
    int kbd_fd = find_keyboard(mouse_fd);
    if (kbd_fd < 0) printf("No USB keyboard — plug one in (hotplug supported)\n");
    term_raw();
    unsigned long poll_count = 0;

    while (!quit) {
        struct pollfd pfds[3];
        int nfds = 0, kbd_idx = -1, mouse_idx = -1, stdin_idx = -1;
        if (kbd_fd >= 0) { pfds[nfds].fd = kbd_fd; pfds[nfds].events = POLLIN; kbd_idx = nfds++; }
        if (mouse_fd >= 0) { pfds[nfds].fd = mouse_fd; pfds[nfds].events = POLLIN; mouse_idx = nfds++; }
        if (stdin_usable) { pfds[nfds].fd = STDIN_FILENO; pfds[nfds].events = POLLIN; stdin_idx = nfds++; }

        int ret = poll(pfds, nfds, 200);
        if (ret < 0) { if (errno == EINTR) continue; break; }
        poll_count++;

        /* Evdev keyboard */
        if (kbd_idx >= 0 && (pfds[kbd_idx].revents & (POLLIN | POLLHUP | POLLERR))) {
            if (pfds[kbd_idx].revents & POLLIN) {
                struct input_event ev;
                ssize_t n;
                while ((n = read(kbd_fd, &ev, sizeof(ev))) == (ssize_t)sizeof(ev)) {
                    if (ev.type != EV_KEY) continue;
                    if (ev.value != 1 && ev.value != 2) continue;
                    int action = evdev_map_key(ev.code);
                    if (action) handle_action(action);
                }
                if (n < 0 && errno != EAGAIN) {
                    ioctl(kbd_fd, EVIOCGRAB, (void *)0); close(kbd_fd); kbd_fd = -1;
                }
            } else {
                ioctl(kbd_fd, EVIOCGRAB, (void *)0); close(kbd_fd); kbd_fd = -1;
            }
        }

        /* Evdev mouse (only active in MODE_SINGLE) */
        if (mouse_idx >= 0 && (pfds[mouse_idx].revents & (POLLIN | POLLHUP | POLLERR))) {
            if (pfds[mouse_idx].revents & POLLIN) {
                struct input_event ev;
                ssize_t n;
                while ((n = read(mouse_fd, &ev, sizeof(ev))) == (ssize_t)sizeof(ev)) {
                    if (g_mode != MODE_SINGLE) continue;
                    if (ev.type == EV_REL) {
                        if (ev.code == REL_X) {
                            int nx = g_cursor_x + ev.value;
                            if (nx < 0) nx = 0;
                            if (nx >= g_disp_w) nx = g_disp_w - 1;
                            g_cursor_x = nx; g_cursor_visible = 1;
                        } else if (ev.code == REL_Y) {
                            int ny = g_cursor_y + ev.value;
                            if (ny < 0) ny = 0;
                            if (ny >= g_disp_h) ny = g_disp_h - 1;
                            g_cursor_y = ny; g_cursor_visible = 1;
                        } else if (ev.code == REL_WHEEL) {
                            if (ev.value > 0) handle_action('+');
                            else if (ev.value < 0) handle_action('-');
                        }
                    } else if (ev.type == EV_KEY && ev.code == BTN_LEFT && ev.value == 1) {
                        int npts = calib_npts;
                        if (npts < MAX_CALIB_PTS) {
                            int cx = g_cursor_x, cy = g_cursor_y;
                            calib_pts[npts].line_idx = calib_line;
                            calib_pts[npts].dx = cx;
                            calib_pts[npts].dy = cy;
                            int cw = g_crop_w, ch = g_crop_h;
                            int cox = g_crop_x, coy = g_crop_y;
                            calib_pts[npts].px = (cox + (double)cx * cw / g_disp_w) / g_src_w;
                            calib_pts[npts].py = (coy + (double)cy * ch / g_disp_h) / g_src_h;
                            calib_npts = npts + 1;
                            printf("Calib: pt %d line %d @ (%d,%d) [%.3f,%.3f]\n",
                                   npts + 1, calib_line, cx, cy,
                                   calib_pts[npts].px, calib_pts[npts].py);
                        }
                    }
                }
                if (n < 0 && errno != EAGAIN) {
                    ioctl(mouse_fd, EVIOCGRAB, (void *)0); close(mouse_fd); mouse_fd = -1;
                }
            } else {
                ioctl(mouse_fd, EVIOCGRAB, (void *)0); close(mouse_fd); mouse_fd = -1;
            }
        }

        /* Stdin fallback (SSH) */
        if (stdin_idx >= 0 && (pfds[stdin_idx].revents & POLLIN)) {
            unsigned char ch;
            while (read(STDIN_FILENO, &ch, 1) == 1) {
                if (ch == 'q' || ch == 'Q') { handle_action('q'); break; }
                if (ch == 0x1B) {
                    unsigned char seq[2] = {0, 0};
                    if (read(STDIN_FILENO, &seq[0], 1) != 1) {
                        /* bare ESC */
                        if (g_mode == MODE_SINGLE) handle_action('E');
                        continue;
                    }
                    if (seq[0] != '[') continue;
                    if (read(STDIN_FILENO, &seq[1], 1) != 1) continue;
                    if (seq[1] == 'A') handle_action('u');
                    else if (seq[1] == 'B') handle_action('d');
                    else if (seq[1] == 'C') handle_action('r');
                    else if (seq[1] == 'D') handle_action('l');
                    continue;
                }
                if (ch == '\n' || ch == '\r') { handle_action('\n'); continue; }
                if (ch == 'g')  { handle_action('G'); continue; }
                if (ch == ' ')  { handle_action(' '); continue; }
                if (ch == 's')  { handle_action('S'); continue; }
                if (ch == 'L')  { handle_action('L'); continue; }
                if (ch == 'r')  { handle_action('R'); continue; }
                if (ch == 'c')  { handle_action('C'); continue; }
                if (ch == 'v')  { handle_action('V'); continue; }
                if (ch == 'b')  { handle_action('B'); continue; }
                if (ch == 'n')  { handle_action('N'); continue; }
                if (ch == '\t') { handle_action('T'); continue; }
                if (ch == 'f')  { handle_action('F'); continue; }
                if (ch == 'a')  { handle_action('A'); continue; }
                if (ch == 'D')  { handle_action('D'); continue; }
                if (ch == 'x')  { handle_action('X'); continue; }
                if (ch == 0x7F) { handle_action('Z'); continue; }
                if (ch == '=' || ch == '+') { handle_action('+'); continue; }
                if (ch == '-')  { handle_action('-'); continue; }
            }
        }

        /* HID hotplug: rescan every ~10s */
        if (mouse_fd < 0 && (poll_count % 50) == 0) mouse_fd = find_mouse();
        if (kbd_fd < 0 && (poll_count % 50) == 0) kbd_fd = find_keyboard(mouse_fd);
    }

    if (kbd_fd >= 0) { ioctl(kbd_fd, EVIOCGRAB, (void *)0); close(kbd_fd); }
    if (mouse_fd >= 0) { ioctl(mouse_fd, EVIOCGRAB, (void *)0); close(mouse_fd); }
    return NULL;
}

/* ─────────────────── Main ─────────────────── */

int main(int argc, char **argv)
{
    int video_base = 11;
    if (argc > 1) video_base = atoi(argv[1]);

    setvbuf(stdout, NULL, _IOLBF, 0);
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    /* ── NVP6324 + media pipeline ── */
    printf("=== Configuring NVP6324 ===\n");
    if (setup_nvp6324() < 0) return 1;

    printf("=== Configuring media pipeline ===\n");
    setup_media_pipeline();
    usleep(500000);

    /* ── DRM (double-buffered) ── */
    struct display disp;
    if (drm_init(&disp) < 0) return 1;

    /* ── Cameras ── */
    struct cam cams[MAX_CAMS];
    int active = 0;
    for (int i = 0; i < MAX_CAMS; i++) {
        memset(&cams[i], 0, sizeof(cams[i]));
        cams[i].fd = -1;
        cams[i].inter_dmabuf_fd = -1;
        snprintf(cams[i].path, sizeof(cams[i].path),
                 "/dev/video%d", video_base + i);
        if (ch_cfg[i].skip) {
            printf("Skip ch%d (%s) — CVBS interlace\n", i, cams[i].path);
            continue;
        }
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

    /* Build active camera list for input thread cycling */
    num_active = 0;
    for (int i = 0; i < MAX_CAMS; i++)
        if (cams[i].active) active_cams[num_active++] = i;
    /* Default selection: prefer cam2 (AHD camera) if active */
    g_selected = active_cams[0];
    for (int i = 0; i < num_active; i++)
        if (active_cams[i] == 2) { g_selected = 2; break; }

    /* ── RGA ── */
    if (rga_setup(cams, MAX_CAMS, &disp) < 0) {
        for (int i = 0; i < MAX_CAMS; i++) cam_cleanup(&cams[i]);
        drm_cleanup(&disp);
        return 1;
    }

    /* ── GPU ── */
    gpu_ctx gpu;
    if (gpu_init(&gpu, &disp) < 0) {
        fprintf(stderr, "GPU init failed\n");
        for (int i = 0; i < MAX_CAMS; i++) cam_cleanup(&cams[i]);
        drm_cleanup(&disp);
        return 1;
    }

    /* Load per-channel calibration + create GPU textures for active cams */
    for (int i = 0; i < MAX_CAMS; i++) {
        if (!cams[i].active) continue;
        cams[i].cal = (struct calib){ .k1 = 0, .k2 = 0, .cx = 0.5, .cy = 0.5, .zoom = MIN_ZOOM };
        char path[64];
        calib_path(path, sizeof(path), i);
        load_params(path, &cams[i].cal);

        if (cams[i].cal.k1 != 0.0 || cams[i].cal.k2 != 0.0) {
            if (gpu_init_cam_texture(&gpu, &cams[i], disp.fd) < 0) {
                fprintf(stderr, "GPU: cam%d texture init failed, using RGA bypass\n", i);
                cams[i].cal.k1 = 0;
                cams[i].cal.k2 = 0;
            } else {
                printf("GPU: cam%d undistort active (k1=%.4f k2=%.4f)\n",
                       i, cams[i].cal.k1, cams[i].cal.k2);
            }
        }
    }

    rga_buffer_t pat_empty;
    memset(&pat_empty, 0, sizeof(pat_empty));
    im_rect prect_empty = {0, 0, 0, 0};

    struct timespec t0;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    unsigned long frames = 0;
    unsigned long per_cam_cnt[MAX_CAMS] = {0};
    int last_bi[MAX_CAMS] = {0};  /* last dequeued buffer index per cam */
    double fps = 0;
    int gpu_pct = 0, cpu_pct = 0;
    struct timespec stats_t0;
    clock_gettime(CLOCK_MONOTONIC, &stats_t0);
    read_cpu_util();  /* prime the delta */

    int disp_w = disp.mode.hdisplay;
    int disp_h = disp.mode.vdisplay;
    int dst_stride4 = disp.pitch / 4;
    int qw = disp_w / 2;
    int qh = disp_h / 2;

    /* Initialize display globals for input thread */
    g_disp_w = disp_w;
    g_disp_h = disp_h;
    g_cursor_x = disp_w / 2;
    g_cursor_y = disp_h / 2;

    /*
     * GL viewport quadrant mapping for FBO rendering.
     * FBOs write row 0 to DRM row 0 (top of screen) — no Y-flip needed.
     * Same coordinate system as DRM/RGA (top-left origin).
     */
    int gl_vp[4][4] = {
        { 0,   0, qw, qh },   /* cam0: top-left */
        { qw,  0, qw, qh },   /* cam1: top-right */
        { 0,  qh, qw, qh },   /* cam2: bottom-left */
        { qw, qh, qw, qh },   /* cam3: bottom-right */
    };

    /* Start input thread */
    pthread_t input_tid;
    pthread_create(&input_tid, NULL, input_thread, NULL);

    /* ── Main loop ── */
    printf("Running — Left/Right=select Enter=calibrate Q=quit\n");
    while (!quit) {
        int mode = g_mode;
        int bb = disp.back;

        /* ── Handle mode transitions ── */
        if (mode == MODE_QUAD && g_enter_single) {
            g_enter_single = 0;
            int sel = g_selected;
            if (cams[sel].active) {
                /* Ensure GPU texture exists for selected channel */
                if (!cams[sel].src_tex) {
                    if (gpu_init_cam_texture(&gpu, &cams[sel], disp.fd) < 0) {
                        fprintf(stderr, "GPU: cam%d texture init failed\n", sel);
                        continue;
                    }
                    printf("GPU: cam%d texture lazy-init OK\n", sel);
                }
                /* Copy cam calib into globals */
                pthread_mutex_lock(&param_lock);
                g_k1 = cams[sel].cal.k1;
                g_k2 = cams[sel].cal.k2;
                g_cx = cams[sel].cal.cx;
                g_cy = cams[sel].cal.cy;
                g_grid = 0;
                g_bypass = 0;
                g_zoom = cams[sel].cal.zoom >= MIN_ZOOM ? cams[sel].cal.zoom : MIN_ZOOM;
                g_cursor_visible = 0;
                g_src_w = cams[sel].width;
                g_src_h = cams[sel].src_height;
                g_crop_x = 0; g_crop_y = 0;
                g_crop_w = cams[sel].width;
                g_crop_h = cams[sel].src_height;
                calib_npts = 0;
                calib_line = 0;
                g_mode = MODE_SINGLE;
                pthread_mutex_unlock(&param_lock);
                mode = MODE_SINGLE;
                printf("=== MODE_SINGLE: ch%d fullscreen calibration ===\n", sel);
            }
        }

        if (mode == MODE_SINGLE && g_exit_single) {
            g_exit_single = 0;
            int sel = g_selected;
            /* Write back calib params to cam struct */
            pthread_mutex_lock(&param_lock);
            cams[sel].cal.k1 = g_k1;
            cams[sel].cal.k2 = g_k2;
            cams[sel].cal.cx = g_cx;
            cams[sel].cal.cy = g_cy;
            cams[sel].cal.zoom = g_zoom;
            g_mode = MODE_QUAD;
            pthread_mutex_unlock(&param_lock);
            mode = MODE_QUAD;
            printf("=== MODE_QUAD: back to 4-up view ===\n");
        }

        /* ── MODE_QUAD: poll all cameras, draw 2x2 ── */
        if (mode == MODE_QUAD) {
            /* Double-buffered: dequeue new frames, then re-render ALL
             * quadrants + overlays to back buffer and flip.  Tracks
             * last buffer index per camera so unchanged cams are re-blitted. */
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
            if (ret < 0) { if (errno == EINTR) continue; break; }
            if (ret == 0) goto fps_check;

            /* Dequeue any ready frames, update last_bi */
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

                if (xioctl(c->fd, VIDIOC_DQBUF, &vb) < 0) continue;

                last_bi[ci] = vb.index;
                xioctl(c->fd, VIDIOC_QBUF, &vb);
                frames++;
                per_cam_cnt[ci]++;
            }

            /* Re-render ALL quadrants to back buffer */
            for (int i = 0; i < MAX_CAMS; i++) {
                struct cam *c = &cams[i];
                if (!c->active) continue;
                int bi = last_bi[i];
                im_rect src_rect = { 0, 0, c->width, c->src_height };

                /* Compute zoom crop for this cam */
                im_rect zr = src_rect;
                if (c->cal.zoom > MIN_ZOOM) {
                    double zf = c->cal.zoom / 10.0;
                    int zw = (int)(c->width / zf), zh = (int)(c->src_height / zf);
                    if (zw < 64) zw = 64; if (zh < 64) zh = 64;
                    int zx = (c->width - zw) / 2, zy = (c->src_height - zh) / 2;
                    zx &= ~1; zy &= ~1;
                    zr = (im_rect){ zx, zy, zw, zh };
                }

                if (c->cal.k1 != 0.0 || c->cal.k2 != 0.0) {
                    improcess(c->bufs[bi].rga, c->inter_rga, pat_empty,
                              src_rect, src_rect, prect_empty, IM_SYNC);
                    float crop[4] = {
                        zr.x / (float)c->width,
                        zr.y / (float)c->src_height,
                        zr.width / (float)c->width,
                        zr.height / (float)c->src_height,
                    };
                    glBindFramebuffer(GL_FRAMEBUFFER, gpu.fbo[bb]);
                    glViewport(gl_vp[i][0], gl_vp[i][1], gl_vp[i][2], gl_vp[i][3]);
                    glUseProgram(gpu.prog);
                    glActiveTexture(GL_TEXTURE0);
                    glBindTexture(GL_TEXTURE_2D, c->src_tex);
                    glUniform1i(gpu.loc_tex, 0);
                    glUniform2f(gpu.loc_center, (float)c->cal.cx, (float)c->cal.cy);
                    glUniform2f(gpu.loc_res, (float)c->width, (float)c->src_height);
                    float max_r = sqrtf((float)(c->width * c->width +
                                                c->src_height * c->src_height)) * 0.5f;
                    glUniform1f(gpu.loc_max_r, max_r);
                    glUniform1f(gpu.loc_k1, (float)c->cal.k1);
                    glUniform1f(gpu.loc_k2, (float)c->cal.k2);
                    float fs = compute_fit_scale((float)c->cal.k1, (float)c->cal.k2,
                                                 (float)c->cal.cx, (float)c->cal.cy,
                                                 (float)c->width, (float)c->src_height, max_r);
                    glUniform1f(gpu.loc_fit_scale, fs);
                    glUniform4fv(gpu.loc_crop, 1, crop);
                    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
                    glFinish();
                } else {
                    improcess(c->bufs[bi].rga, disp.rga[bb], pat_empty,
                              zr, c->quad, prect_empty, IM_SYNC);
                }
            }

            /* Draw overlays on back buffer */
            struct timespec blink_now;
            clock_gettime(CLOCK_MONOTONIC, &blink_now);
            int blink_on = ((blink_now.tv_nsec / 500000000) + blink_now.tv_sec) % 2;
            draw_selection_border((uint32_t *)disp.map[bb], dst_stride4,
                                 disp_w, disp_h, g_selected, blink_on);
            draw_quad_labels((uint32_t *)disp.map[bb], dst_stride4,
                             disp_w, disp_h, cams, MAX_CAMS);
            draw_sys_stats((uint32_t *)disp.map[bb], dst_stride4,
                           disp_w, disp_h, gpu_pct, cpu_pct, fps);

            /* Flip */
            drmModeSetCrtc(disp.fd, disp.crtc_id, disp.fb_id[bb],
                           0, 0, &disp.conn_id, 1, &disp.mode);
            disp.back = 1 - bb;
        }

        /* ── MODE_SINGLE: fullscreen calibration for selected channel ── */
        else {
            int sel = g_selected;
            struct cam *c = &cams[sel];

            struct pollfd pfd = { .fd = c->fd, .events = POLLIN };
            int ret = poll(&pfd, 1, 100);
            if (ret < 0) { if (errno == EINTR) continue; break; }
            if (!(pfd.revents & POLLIN)) goto fps_check;

            struct v4l2_buffer vb = {
                .type   = c->buf_type,
                .memory = V4L2_MEMORY_MMAP,
            };
            struct v4l2_plane planes[1] = {};
            if (c->is_mplane) { vb.m.planes = planes; vb.length = 1; }
            if (xioctl(c->fd, VIDIOC_DQBUF, &vb) < 0) goto fps_check;

            int bi = vb.index;

            /* Snapshot params */
            pthread_mutex_lock(&param_lock);
            double k1 = g_k1, k2 = g_k2, cx = g_cx, cy = g_cy;
            int grid_on = g_grid, bypass = g_bypass;
            int do_save = g_save_req, do_load = g_load_req;
            int do_reset = g_reset_req, do_fit = g_fit_req;
            int do_autodetect = g_autodetect_req;
            g_save_req = 0; g_load_req = 0;
            g_reset_req = 0; g_fit_req = 0;
            g_autodetect_req = 0;
            pthread_mutex_unlock(&param_lock);

            /* Handle save/load/reset/fit */
            if (do_save) {
                char path[64];
                calib_path(path, sizeof(path), sel);
                save_params(path, k1, k2, cx, cy, g_zoom);
            }
            if (do_load) {
                struct calib lc = { k1, k2, cx, cy, .zoom = MIN_ZOOM };
                char path[64];
                calib_path(path, sizeof(path), sel);
                if (load_params(path, &lc) == 0) {
                    pthread_mutex_lock(&param_lock);
                    g_k1 = lc.k1; g_k2 = lc.k2; g_cx = lc.cx; g_cy = lc.cy;
                    if (lc.zoom >= MIN_ZOOM && lc.zoom <= MAX_ZOOM) g_zoom = lc.zoom;
                    pthread_mutex_unlock(&param_lock);
                    k1 = lc.k1; k2 = lc.k2; cx = lc.cx; cy = lc.cy;
                }
            }
            if (do_reset) {
                pthread_mutex_lock(&param_lock);
                g_k1 = 0.0; g_k2 = 0.0; g_cx = 0.5; g_cy = 0.5;
                pthread_mutex_unlock(&param_lock);
                k1 = 0.0; k2 = 0.0; cx = 0.5; cy = 0.5;
                printf("Reset: k1=0 k2=0 cx=0.5 cy=0.5\n");
            }
            if (do_autodetect) {
                auto_detect_edges((const uint8_t *)c->bufs[bi].start,
                                  c->width, c->src_height);
                /* Auto-fit if enough points were detected */
                if (calib_npts >= 6) {
                    auto_fit_k1k2(c->width, c->src_height, cx, cy);
                    pthread_mutex_lock(&param_lock);
                    k1 = g_k1; k2 = g_k2;
                    pthread_mutex_unlock(&param_lock);
                    printf("Auto-detect + fit: k1=%.4f k2=%.4f\n", k1, k2);
                } else {
                    printf("Auto-detect: not enough points for fit (%d)\n",
                           calib_npts);
                }
            }
            if (do_fit) {
                auto_fit_k1k2(c->width, c->src_height, cx, cy);
                pthread_mutex_lock(&param_lock);
                k1 = g_k1; k2 = g_k2;
                pthread_mutex_unlock(&param_lock);
            }

            /* Compute zoom crop */
            int zoom = g_zoom;
            im_rect src_rect = { 0, 0, c->width, c->src_height };
            im_rect zoom_rect;
            if (zoom <= MIN_ZOOM) {
                zoom_rect = src_rect;
            } else {
                double zf = zoom / 10.0;
                int zw = (int)(c->width / zf), zh = (int)(c->src_height / zf);
                if (zw < 64) zw = 64;
                if (zh < 64) zh = 64;
                int zcx = g_cursor_visible
                         ? g_cursor_x * c->width / disp_w
                         : c->width / 2;
                int zcy = g_cursor_visible
                         ? g_cursor_y * c->src_height / disp_h
                         : c->src_height / 2;
                int zx = zcx - zw / 2, zy = zcy - zh / 2;
                if (zx < 0) zx = 0;
                if (zy < 0) zy = 0;
                if (zx + zw > c->width) zx = c->width - zw;
                if (zy + zh > c->src_height) zy = c->src_height - zh;
                zx &= ~1; zy &= ~1;
                zoom_rect = (im_rect){zx, zy, zw, zh};
            }
            g_crop_x = zoom_rect.x; g_crop_y = zoom_rect.y;
            g_crop_w = zoom_rect.width; g_crop_h = zoom_rect.height;

            im_rect dst_rect = { 0, 0, disp_w, disp_h };

            /* Render */
            if (bypass || (k1 == 0.0 && k2 == 0.0)) {
                improcess(c->bufs[bi].rga, disp.rga[bb], pat_empty,
                          zoom_rect, dst_rect, prect_empty, IM_SYNC);
            } else {
                improcess(c->bufs[bi].rga, c->inter_rga, pat_empty,
                          src_rect, src_rect, prect_empty, IM_SYNC);
                float crop[4] = {
                    zoom_rect.x / (float)c->width,
                    zoom_rect.y / (float)c->height,
                    zoom_rect.width / (float)c->width,
                    zoom_rect.height / (float)c->height,
                };
                glBindFramebuffer(GL_FRAMEBUFFER, gpu.fbo[bb]);
                glViewport(0, 0, disp_w, disp_h);
                glClear(GL_COLOR_BUFFER_BIT);
                glUseProgram(gpu.prog);
                glActiveTexture(GL_TEXTURE0);
                glBindTexture(GL_TEXTURE_2D, c->src_tex);
                glUniform1i(gpu.loc_tex, 0);
                glUniform2f(gpu.loc_center, (float)cx, (float)cy);
                glUniform2f(gpu.loc_res, (float)c->width, (float)c->src_height);
                float max_r = sqrtf((float)(c->width * c->width +
                                            c->src_height * c->src_height)) * 0.5f;
                glUniform1f(gpu.loc_max_r, max_r);
                glUniform1f(gpu.loc_k1, (float)k1);
                glUniform1f(gpu.loc_k2, (float)k2);
                float fs = compute_fit_scale((float)k1, (float)k2,
                                             (float)cx, (float)cy,
                                             (float)c->width, (float)c->src_height, max_r);
                glUniform1f(gpu.loc_fit_scale, fs);
                glUniform4fv(gpu.loc_crop, 1, crop);
                glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
                glFinish();
            }

            /* Grid */
            if (grid_on)
                draw_grid((uint32_t *)disp.map[bb], dst_stride4,
                          disp_w, disp_h, 0x0040FF40);

            /* OSD */
            draw_osd_single(disp.map[bb], disp.pitch, disp_w, disp_h,
                            fps, sel, k1, k2, cx, cy, grid_on, bypass);

            /* Calib points + crosshair */
            if (calib_npts > 0)
                draw_calib_points((uint32_t *)disp.map[bb], dst_stride4,
                                  disp_w, disp_h,
                                  zoom_rect.x, zoom_rect.y,
                                  zoom_rect.width, zoom_rect.height,
                                  c->width, c->src_height);
            if (g_cursor_visible)
                draw_crosshair((uint32_t *)disp.map[bb], dst_stride4,
                               disp_w, disp_h);

            /* Flip */
            drmModeSetCrtc(disp.fd, disp.crtc_id, disp.fb_id[bb],
                           0, 0, &disp.conn_id, 1, &disp.mode);
            disp.back = 1 - bb;

            xioctl(c->fd, VIDIOC_QBUF, &vb);
            frames++;
            per_cam_cnt[sel]++;

            /* Also drain other cameras to prevent queue stall */
            for (int i = 0; i < MAX_CAMS; i++) {
                if (i == sel || !cams[i].active) continue;
                struct v4l2_buffer dvb = {
                    .type   = cams[i].buf_type,
                    .memory = V4L2_MEMORY_MMAP,
                };
                struct v4l2_plane dp[1] = {};
                if (cams[i].is_mplane) { dvb.m.planes = dp; dvb.length = 1; }
                if (xioctl(cams[i].fd, VIDIOC_DQBUF, &dvb) == 0)
                    xioctl(cams[i].fd, VIDIOC_QBUF, &dvb);
            }
        }

fps_check:
        /* FPS + system stats */
        {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            double elapsed = (now.tv_sec - t0.tv_sec)
                           + (now.tv_nsec - t0.tv_nsec) / 1e9;
            if (elapsed >= 3.0) {
                fps = frames / elapsed;
                if (g_mode == MODE_QUAD) {
                    printf("%.1f fps [", fps);
                    for (int i = 0; i < MAX_CAMS; i++) {
                        if (i) printf(" ");
                        printf("cam%d:%.1f", i,
                               cams[i].active ? per_cam_cnt[i] / elapsed : 0.0);
                        per_cam_cnt[i] = 0;
                    }
                    printf("]\n");
                }
                t0 = now;
                frames = 0;
            }
            /* Update system stats once per second */
            double stats_elapsed = (now.tv_sec - stats_t0.tv_sec)
                                 + (now.tv_nsec - stats_t0.tv_nsec) / 1e9;
            if (stats_elapsed >= 1.0) {
                gpu_pct = read_gpu_util();
                cpu_pct = read_cpu_util();
                if (gpu_pct < 0) gpu_pct = 0;
                if (cpu_pct < 0) cpu_pct = 0;
                stats_t0 = now;
            }
        }
    }

    printf("\nShutting down...\n");
    pthread_join(input_tid, NULL);
    for (int i = 0; i < MAX_CAMS; i++) {
        if (cams[i].active)
            gpu_cleanup_cam_texture(&gpu, &cams[i], disp.fd);
        cam_cleanup(&cams[i]);
    }
    gpu_cleanup(&gpu);
    drm_cleanup(&disp);
    printf("Done.\n");
    return 0;
}
