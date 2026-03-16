/*
 * v4l2_rga_drm_undistort.c — Standalone lens distortion calibration tool
 *
 * Displays ch2 AHD 1080p25 fullscreen on HDMI with live barrel/pincushion
 * distortion correction. Adjust k1/k2 parameters with keyboard, toggle
 * grid overlay to judge straightness, save calibration to file.
 *
 * Build (on board):
 *   gcc -O2 -o v4l2_rga_drm_undistort v4l2_rga_drm_undistort.c \
 *       $(pkg-config --cflags --libs librga libdrm gbm egl glesv2) -lm -lpthread
 *
 * Usage:
 *   sudo systemctl stop lightdm
 *   sudo ./v4l2_rga_drm_undistort [/dev/videoN]   # default /dev/video13
 *   sudo systemctl start lightdm
 *
 * Keyboard (USB keyboard or SSH):
 *   Left/Right  = adjust k1 (barrel/pincushion), step +/-0.01
 *   Up/Down     = adjust k2 (4th-order term), step +/-0.005
 *   C/V         = adjust cx (optical center X), step +/-0.01
 *   B/N         = adjust cy (optical center Y), step +/-0.01
 *   G           = toggle grid overlay
 *   Space       = toggle bypass (compare original vs corrected)
 *   S           = save parameters to config file
 *   L           = load parameters from config file
 *   R           = reset all to defaults (k1=k2=0, cx=cy=0.5)
 *   Q / ESC     = quit
 *
 * Mouse calibration:
 *   Mouse move  = move crosshair cursor
 *   Left click  = place calibration point on current line
 *   Tab         = start new calibration line
 *   F           = auto-fit k1/k2 from placed points (needs 2+ lines, 3+ pts each)
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

#define DRM_CARD       "/dev/dri/card1"
#define V4L2_BUF_CNT   4

#define SYSFS_FMT      "/sys/devices/platform/2ac70000.i2c/i2c-4/4-0031/channel_fmt"

#define CH             2       /* NVP6324 channel (physical camera) */
#define MAX_CAMS       4

#define DV_CVBS_NTSC   0
#define DV_1080P25     6

static const struct { int dv_idx; int w; int h; } ch_cfg[MAX_CAMS] = {
    { DV_CVBS_NTSC,  720,  480 },
    { DV_CVBS_NTSC,  720,  480 },
    { DV_1080P25,   1920, 1080 },
    { DV_CVBS_NTSC,  720,  480 },
};

#define CALIB_PATH     "/root/lens_calib.conf"

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
static struct termios orig_tio;
static int tio_saved = 0;

static void on_signal(int s) { (void)s; quit = 1; }

/* Distortion parameters (accessed from input thread + render loop) */
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
static pthread_mutex_t param_lock = PTHREAD_MUTEX_INITIALIZER;

/* Mouse cursor state */
static volatile int g_cursor_x, g_cursor_y;
static volatile int g_cursor_visible = 0;
static volatile int g_zoom = 0;
#define MAX_ZOOM 3
static int g_disp_w, g_disp_h;  /* set before input thread starts */
static int g_src_w, g_src_h;    /* source image dimensions */
static volatile int g_crop_x, g_crop_y, g_crop_w, g_crop_h;

/* Calibration points */
#define MAX_CALIB_LINES  8
#define MAX_CALIB_PTS    64

static int calib_line = 0;
static volatile int calib_npts = 0;
static struct {
    int line_idx;
    int dx, dy;     /* display coords (for drawing) */
    double px, py;  /* source image fractions 0-1 (for fitting) */
} calib_pts[MAX_CALIB_PTS];

/* ─────────────────── Config file save/load ─────────────────── */

static int save_params(const char *path, double k1, double k2, double cx, double cy)
{
    FILE *fp = fopen(path, "w");
    if (!fp) {
        fprintf(stderr, "Cannot save to %s: %s\n", path, strerror(errno));
        return -1;
    }
    fprintf(fp, "k1=%.6f\nk2=%.6f\ncx=%.6f\ncy=%.6f\n", k1, k2, cx, cy);
    fclose(fp);
    printf("Saved: k1=%.4f k2=%.4f cx=%.3f cy=%.3f -> %s\n",
           k1, k2, cx, cy, path);
    return 0;
}

static int load_params(const char *path, double *k1, double *k2, double *cx, double *cy)
{
    FILE *fp = fopen(path, "r");
    if (!fp) {
        fprintf(stderr, "Cannot load %s: %s\n", path, strerror(errno));
        return -1;
    }
    char line[128];
    while (fgets(line, sizeof(line), fp)) {
        double val;
        if (sscanf(line, "k1=%lf", &val) == 1) *k1 = val;
        else if (sscanf(line, "k2=%lf", &val) == 1) *k2 = val;
        else if (sscanf(line, "cx=%lf", &val) == 1) *cx = val;
        else if (sscanf(line, "cy=%lf", &val) == 1) *cy = val;
    }
    fclose(fp);
    printf("Loaded: k1=%.4f k2=%.4f cx=%.3f cy=%.3f <- %s\n",
           *k1, *k2, *cx, *cy, path);
    return 0;
}

/* ─────────────────── Grid overlay ─────────────────── */

static void draw_grid(uint32_t *fb, int pitch4, int w, int h, uint32_t color)
{
    int nx = 16;  /* number of vertical lines */
    int ny = 9;   /* number of horizontal lines */

    /* Vertical lines */
    for (int i = 1; i < nx; i++) {
        int x = i * w / nx;
        for (int y = 0; y < h; y++)
            fb[y * pitch4 + x] = color;
    }

    /* Horizontal lines */
    for (int i = 1; i < ny; i++) {
        int y = i * h / ny;
        for (int x = 0; x < w; x++)
            fb[y * pitch4 + x] = color;
    }

    /* Center crosshair (thicker, different color) */
    int cx = w / 2, cy = h / 2;
    uint32_t cross_color = 0x00FF0000;  /* red */
    for (int y = 0; y < h; y++) {
        fb[y * pitch4 + cx - 1] = cross_color;
        fb[y * pitch4 + cx]     = cross_color;
        fb[y * pitch4 + cx + 1] = cross_color;
    }
    for (int x = 0; x < w; x++) {
        fb[(cy - 1) * pitch4 + x] = cross_color;
        fb[cy * pitch4 + x]       = cross_color;
        fb[(cy + 1) * pitch4 + x] = cross_color;
    }
}

/* ─────────────────── Crosshair + calibration point drawing ─────────────────── */

static void draw_crosshair(uint32_t *fb, int pitch4, int w, int h)
{
    int cx = g_cursor_x, cy = g_cursor_y;
    uint32_t color = 0x0000FFFF;  /* yellow in XRGB: 0x00RRGGBB */
    int arm = 40, gap = 4;

    /* Horizontal arms */
    for (int dx = gap; dx <= arm; dx++) {
        if (cx + dx < w) fb[cy * pitch4 + cx + dx] = color;
        if (cx - dx >= 0) fb[cy * pitch4 + cx - dx] = color;
    }
    /* Vertical arms */
    for (int dy = gap; dy <= arm; dy++) {
        if (cy + dy < h) fb[(cy + dy) * pitch4 + cx] = color;
        if (cy - dy >= 0) fb[(cy - dy) * pitch4 + cx] = color;
    }
}

static const uint32_t calib_colors[4] = {
    0x00FF0000,  /* red */
    0x0000FF00,  /* green */
    0x000080FF,  /* blue */
    0x0000FFFF,  /* cyan/yellow */
};

static void draw_calib_points(uint32_t *fb, int pitch4, int w, int h,
                              int crop_x, int crop_y, int crop_w, int crop_h,
                              int src_w, int src_h)
{
    int npts = calib_npts;
    int prev_dx = 0, prev_dy = 0, prev_line = -1;

    for (int i = 0; i < npts; i++) {
        /* Map source fraction → display position via current zoom crop */
        double sx = calib_pts[i].px * src_w;
        double sy = calib_pts[i].py * src_h;
        int px = (int)((sx - crop_x) * w / crop_w);
        int py = (int)((sy - crop_y) * h / crop_h);
        uint32_t color = calib_colors[calib_pts[i].line_idx % 4];

        int visible = (px >= -3 && px < w + 3 && py >= -3 && py < h + 3);

        if (visible) {
            /* 6x6 filled square */
            for (int dy = -3; dy <= 2; dy++) {
                int ry = py + dy;
                if (ry < 0 || ry >= h) continue;
                for (int dx = -3; dx <= 2; dx++) {
                    int rx = px + dx;
                    if (rx < 0 || rx >= w) continue;
                    fb[ry * pitch4 + rx] = color;
                }
            }

            /* Line number label */
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

        /* Connect to previous point on same line */
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

static void osd_rect(uint32_t *fb, int pitch4, int x, int y,
                     int w, int h, uint32_t color)
{
    for (int r = y; r < y + h; r++)
        for (int c = x; c < x + w; c++)
            fb[r * pitch4 + c] = color;
}

static void osd_char(uint32_t *fb, int pitch4, int x, int y,
                     char ch, uint32_t color)
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

static void osd_str(uint32_t *fb, int pitch4, int x, int y,
                    const char *s, uint32_t color)
{
    while (*s) {
        osd_char(fb, pitch4, x, y, *s, color);
        x += OSD_CW;
        s++;
    }
}

static void draw_osd(void *map, int pitch, int disp_w, int disp_h,
                     double fps, double k1, double k2, double cx, double cy,
                     int grid_on, int bypass)
{
    (void)disp_w;
    uint32_t *fb = (uint32_t *)map;
    int p4 = pitch / 4;
    int ox = 16, oy = 12;

    /* Background */
    int osd_w = 52 * OSD_CW + 16;
    int osd_h = 13 * OSD_LH + 12;
    osd_rect(fb, p4, ox - 8, oy - 6, osd_w, osd_h, 0x00181818);

    /* Title */
    char line[80];
    snprintf(line, sizeof(line), "=== Lens Distortion [%.0f fps] ===", fps);
    osd_str(fb, p4, ox, oy, line, 0x0000CCCC);
    oy += OSD_LH * 2;

    /* k1 */
    snprintf(line, sizeof(line), "> k1 (barrel/pin):  %+.3f", k1);
    osd_str(fb, p4, ox, oy, line,
            k1 != 0.0 ? 0x0000FF00 : 0x00CCCCCC);
    oy += OSD_LH;

    /* k2 */
    snprintf(line, sizeof(line), "  k2 (4th order):   %+.4f", k2);
    osd_str(fb, p4, ox, oy, line,
            k2 != 0.0 ? 0x00FFFF00 : 0x00CCCCCC);
    oy += OSD_LH;

    /* cx */
    snprintf(line, sizeof(line), "  cx (center X):    %.3f", cx);
    osd_str(fb, p4, ox, oy, line,
            cx != 0.5 ? 0x00FFFF00 : 0x00CCCCCC);
    oy += OSD_LH;

    /* cy */
    snprintf(line, sizeof(line), "  cy (center Y):    %.3f", cy);
    osd_str(fb, p4, ox, oy, line,
            cy != 0.5 ? 0x00FFFF00 : 0x00CCCCCC);
    oy += OSD_LH;

    /* Status */
    oy += OSD_LH / 2;
    snprintf(line, sizeof(line), "[GRID: %s]  [%s]  [ZOOM: %dx]",
             grid_on ? "ON" : "OFF",
             bypass ? "BYPASS" : "CORRECTED",
             1 << g_zoom);
    osd_str(fb, p4, ox, oy, line,
            bypass ? 0x00FF8844 : 0x0000FF00);
    oy += OSD_LH;

    /* Calibration status */
    oy += OSD_LH / 2;
    {
        /* Count points per line */
        int npts = calib_npts;
        int nlines_with_pts = 0;
        if (npts > 0) {
            int seen[MAX_CALIB_LINES];
            int nseen = 0;
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
        snprintf(line, sizeof(line), "[CALIB: %d lines, %d pts]  Tab=line F=fit X=clr",
                 nlines_with_pts, npts);
        osd_str(fb, p4, ox, oy, line,
                npts > 0 ? 0x0000CCFF : 0x00666666);
    }
    oy += OSD_LH;

    /* Help */
    oy += OSD_LH / 2;
    osd_str(fb, p4, ox, oy,
            "Arrows=k1/k2 C/V=cx B/N=cy G=grid SPC=bypass",
            0x00666666);
    oy += OSD_LH;
    osd_str(fb, p4, ox, oy,
            "S=save shift-L=load R=reset +/-=zoom Q=quit",
            0x00666666);
}

/* ─────────────────── Evdev keyboard ─────────────────── */

#define TEST_BIT(bit, arr) \
    ((arr)[(bit) / (8 * sizeof(unsigned long))] & \
     (1UL << ((bit) % (8 * sizeof(unsigned long)))))

static int find_keyboard(void)
{
    char path[64];
    for (int i = 0; i < 20; i++) {
        snprintf(path, sizeof(path), "/dev/input/event%d", i);
        int fd = open(path, O_RDONLY | O_NONBLOCK);
        if (fd < 0) continue;

        unsigned long evbits[1] = {0};
        if (ioctl(fd, EVIOCGBIT(0, sizeof(evbits)), evbits) < 0 ||
            !TEST_BIT(EV_KEY, evbits)) {
            close(fd);
            continue;
        }

        unsigned long keybits[128 / sizeof(unsigned long)];
        memset(keybits, 0, sizeof(keybits));
        ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(keybits)), keybits);

        if (TEST_BIT(KEY_Q, keybits) && TEST_BIT(KEY_1, keybits) &&
            TEST_BIT(KEY_UP, keybits)) {
            char name[256] = "Unknown";
            ioctl(fd, EVIOCGNAME(sizeof(name)), name);
            printf("Keyboard: %s (%s)\n", name, path);
            ioctl(fd, EVIOCGRAB, (void *)1);
            return fd;
        }
        close(fd);
    }
    return -1;
}

/* ─────────────────── Evdev mouse ─────────────────── */

static int find_mouse(void)
{
    char path[64];
    for (int i = 0; i < 20; i++) {
        snprintf(path, sizeof(path), "/dev/input/event%d", i);
        int fd = open(path, O_RDONLY | O_NONBLOCK);
        if (fd < 0) continue;

        unsigned long evbits[1] = {0};
        if (ioctl(fd, EVIOCGBIT(0, sizeof(evbits)), evbits) < 0 ||
            !TEST_BIT(EV_REL, evbits)) {
            close(fd);
            continue;
        }

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

static void term_raw(void)
{
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
}

/* ─────────────────── Key action handler ─────────────────── */

static void handle_action(int action)
{
    pthread_mutex_lock(&param_lock);

    switch (action) {
    case 'q':
        quit = 1;
        break;

    /* k1: Left/Right, step 0.01 */
    case 'r':  /* right = increase k1 */
        g_k1 += 0.01;

        break;
    case 'l':  /* left = decrease k1 */
        g_k1 -= 0.01;

        break;

    /* k2: Up/Down, step 0.005 */
    case 'u':  /* up = increase k2 */
        g_k2 += 0.005;

        break;
    case 'd':  /* down = decrease k2 */
        g_k2 -= 0.005;

        break;

    /* cx: C/V */
    case 'C':
        g_cx -= 0.01;
        if (g_cx < 0.0) g_cx = 0.0;

        break;
    case 'V':
        g_cx += 0.01;
        if (g_cx > 1.0) g_cx = 1.0;

        break;

    /* cy: B/N */
    case 'B':
        g_cy -= 0.01;
        if (g_cy < 0.0) g_cy = 0.0;

        break;
    case 'N':
        g_cy += 0.01;
        if (g_cy > 1.0) g_cy = 1.0;

        break;

    case 'G':
        g_grid = !g_grid;
        break;
    case ' ':
        g_bypass = !g_bypass;
        break;
    case 'S':
        g_save_req = 1;
        break;
    case 'L':
        g_load_req = 1;
        break;
    case 'R':
        g_reset_req = 1;
        break;

    /* Calibration: Tab = new line */
    case 'T':
        if (calib_line < MAX_CALIB_LINES - 1) {
            calib_line++;
            printf("Calib: new line %d\n", calib_line);
        }
        break;

    /* Calibration: F = auto-fit */
    case 'F':
        g_fit_req = 1;
        break;

    /* Calibration: X = clear all */
    case 'X':
        calib_npts = 0;
        calib_line = 0;
        printf("Calib: cleared all points\n");
        break;

    /* Calibration: Backspace = undo last */
    case 'Z':
        if (calib_npts > 0) {
            calib_npts--;
            printf("Calib: undo -> %d pts\n", calib_npts);
        }
        break;

    /* Zoom */
    case '+':
        if (g_zoom < MAX_ZOOM) {
            g_zoom++;
            printf("Zoom: %dx\n", 1 << g_zoom);
        }
        break;
    case '-':
        if (g_zoom > 0) {
            g_zoom--;
            printf("Zoom: %dx\n", 1 << g_zoom);
        }
        break;
    }

    pthread_mutex_unlock(&param_lock);
}

/* Map evdev keycode to action character */
static int evdev_map_key(int code)
{
    switch (code) {
    case KEY_ESC: case KEY_Q:  return 'q';
    case KEY_UP:               return 'u';
    case KEY_DOWN:             return 'd';
    case KEY_LEFT:             return 'l';
    case KEY_RIGHT:            return 'r';
    case KEY_G:                return 'G';
    case KEY_SPACE:            return ' ';
    case KEY_S:                return 'S';
    case KEY_L:                return 'L';
    case KEY_R:                return 'R';
    case KEY_C:                return 'C';
    case KEY_V:                return 'V';
    case KEY_B:                return 'B';
    case KEY_N:                return 'N';
    case KEY_TAB:              return 'T';
    case KEY_F:                return 'F';
    case KEY_X:                return 'X';
    case KEY_BACKSPACE:        return 'Z';
    case KEY_EQUAL:            return '+';
    case KEY_KPPLUS:           return '+';
    case KEY_MINUS:            return '-';
    case KEY_KPMINUS:          return '-';
    default:                   return 0;
    }
}

/* ─────────────────── Auto-fit k1/k2 from calibration points ─────────────────── */

/* Invert the forward distortion model via Newton iteration.
 * Forward: distorted = center + (undist - center) * (1 + k1*r² + k2*r⁴)
 * Given a distorted point, find the undistorted point. */
static void fit_undistort_point(double dx, double dy,
                                double cx, double cy, double max_r,
                                double k1, double k2,
                                double *ux, double *uy)
{
    double x = dx, y = dy;
    for (int iter = 0; iter < 20; iter++) {
        double rx = (x - cx) / max_r;
        double ry = (y - cy) / max_r;
        double r2 = rx * rx + ry * ry;
        double r4 = r2 * r2;
        double scale = 1.0 + k1 * r2 + k2 * r4;
        double fx = cx + (x - cx) * scale;
        double fy = cy + (y - cy) * scale;
        double ex = fx - dx;
        double ey = fy - dy;
        double dscale_dr2 = k1 + 2.0 * k2 * r2;
        double j = scale + 2.0 * dscale_dr2 * r2;
        if (fabs(j) < 1e-10) break;
        x -= ex / j;
        y -= ey / j;
        if (ex * ex + ey * ey < 1e-6) break;
    }
    *ux = x;
    *uy = y;
}

/* PCA line-straightness error: smallest eigenvalue of 2x2 covariance */
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
    double trace = cxx + cyy;
    double det = cxx * cyy - cxy * cxy;
    double disc = trace * trace - 4.0 * det;
    if (disc < 0) disc = 0;
    return (trace - sqrt(disc)) * 0.5;
}

/* Total straightness error over all calibration lines for given k1, k2 */
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
            double dx = calib_pts[j].px * w;
            double dy = calib_pts[j].py * h;
            fit_undistort_point(dx, dy, cx, cy, max_r, k1, k2,
                                &uxs[count], &uys[count]);
            count++;
        }
        if (count >= 3)
            total += fit_line_error(uxs, uys, count);
    }
    return total;
}

/* Grid-search fitter: coarse + fine search over k1, k2 */
static int auto_fit_k1k2(int src_w, int src_h, double cx_frac, double cy_frac)
{
    int npts = calib_npts;
    if (npts < 6) {
        printf("Fit: need >= 6 points total (have %d)\n", npts);
        return -1;
    }

    /* Count lines with >= 3 points */
    int line_counts[MAX_CALIB_LINES];
    int line_ids[MAX_CALIB_LINES];
    int nlines = 0;
    for (int i = 0; i < npts; i++) {
        int lidx = calib_pts[i].line_idx;
        int found = -1;
        for (int j = 0; j < nlines; j++)
            if (line_ids[j] == lidx) { found = j; break; }
        if (found >= 0) {
            line_counts[found]++;
        } else if (nlines < MAX_CALIB_LINES) {
            line_ids[nlines] = lidx;
            line_counts[nlines] = 1;
            nlines++;
        }
    }
    int valid = 0;
    for (int i = 0; i < nlines; i++)
        if (line_counts[i] >= 3) valid++;
    if (valid < 2) {
        printf("Fit: need >= 2 lines with >= 3 pts each (have %d valid)\n", valid);
        return -1;
    }

    double cx = cx_frac * src_w;
    double cy = cy_frac * src_h;
    double max_r = sqrt((src_w * 0.5) * (src_w * 0.5) +
                        (src_h * 0.5) * (src_h * 0.5));

    /* Coarse grid search */
    double best_k1 = 0, best_k2 = 0, best_err = 1e30;
    for (double tk1 = -0.8; tk1 <= 0.801; tk1 += 0.02) {
        for (double tk2 = -0.3; tk2 <= 0.301; tk2 += 0.01) {
            double err = fit_eval_error(tk1, tk2, cx, cy, max_r, src_w, src_h);
            if (err < best_err) {
                best_err = err;
                best_k1 = tk1;
                best_k2 = tk2;
            }
        }
    }

    /* Fine grid search around best coarse result */
    double fine_k1 = best_k1, fine_k2 = best_k2, fine_err = best_err;
    for (double tk1 = best_k1 - 0.02; tk1 <= best_k1 + 0.0201; tk1 += 0.002) {
        for (double tk2 = best_k2 - 0.01; tk2 <= best_k2 + 0.0101; tk2 += 0.001) {
            double err = fit_eval_error(tk1, tk2, cx, cy, max_r, src_w, src_h);
            if (err < fine_err) {
                fine_err = err;
                fine_k1 = tk1;
                fine_k2 = tk2;
            }
        }
    }

    printf("Fit: k1=%.4f k2=%.4f (error=%.6f)\n", fine_k1, fine_k2, fine_err);

    pthread_mutex_lock(&param_lock);
    g_k1 = fine_k1;
    g_k2 = fine_k2;
    pthread_mutex_unlock(&param_lock);

    return 0;
}

/* ─────────────────── Input thread ─────────────────── */

static void *input_thread(void *arg)
{
    (void)arg;
    int kbd_fd = find_keyboard();
    if (kbd_fd < 0)
        printf("No USB keyboard found — plug one in (hotplug supported)\n");

    int mouse_fd = find_mouse();
    if (mouse_fd < 0)
        printf("No USB mouse found — plug one in (hotplug supported)\n");

    term_raw();

    unsigned long poll_count = 0;

    while (!quit) {
        struct pollfd pfds[3];
        int nfds = 0;
        int kbd_idx = -1, mouse_idx = -1, stdin_idx;

        if (kbd_fd >= 0) {
            pfds[nfds].fd = kbd_fd;
            pfds[nfds].events = POLLIN;
            kbd_idx = nfds++;
        }
        if (mouse_fd >= 0) {
            pfds[nfds].fd = mouse_fd;
            pfds[nfds].events = POLLIN;
            mouse_idx = nfds++;
        }
        pfds[nfds].fd = STDIN_FILENO;
        pfds[nfds].events = POLLIN;
        stdin_idx = nfds++;

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
                    ioctl(kbd_fd, EVIOCGRAB, (void *)0);
                    close(kbd_fd);
                    kbd_fd = -1;
                }
            } else {
                ioctl(kbd_fd, EVIOCGRAB, (void *)0);
                close(kbd_fd);
                kbd_fd = -1;
            }
        }

        /* Evdev mouse */
        if (mouse_idx >= 0 && (pfds[mouse_idx].revents & (POLLIN | POLLHUP | POLLERR))) {
            if (pfds[mouse_idx].revents & POLLIN) {
                struct input_event ev;
                ssize_t n;
                while ((n = read(mouse_fd, &ev, sizeof(ev))) == (ssize_t)sizeof(ev)) {
                    if (ev.type == EV_REL) {
                        if (ev.code == REL_X) {
                            int nx = g_cursor_x + ev.value;
                            if (nx < 0) nx = 0;
                            if (nx >= g_disp_w) nx = g_disp_w - 1;
                            g_cursor_x = nx;
                            g_cursor_visible = 1;
                        } else if (ev.code == REL_Y) {
                            int ny = g_cursor_y + ev.value;
                            if (ny < 0) ny = 0;
                            if (ny >= g_disp_h) ny = g_disp_h - 1;
                            g_cursor_y = ny;
                            g_cursor_visible = 1;
                        } else if (ev.code == REL_WHEEL) {
                            if (ev.value > 0) handle_action('+');
                            else if (ev.value < 0) handle_action('-');
                        }
                    } else if (ev.type == EV_KEY && ev.code == BTN_LEFT && ev.value == 1) {
                        /* Left click: place calibration point */
                        int npts = calib_npts;
                        if (npts < MAX_CALIB_PTS) {
                            int cx = g_cursor_x, cy = g_cursor_y;
                            calib_pts[npts].line_idx = calib_line;
                            calib_pts[npts].dx = cx;
                            calib_pts[npts].dy = cy;
                            /* Map display → source fraction, accounting for zoom */
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
                    ioctl(mouse_fd, EVIOCGRAB, (void *)0);
                    close(mouse_fd);
                    mouse_fd = -1;
                }
            } else {
                ioctl(mouse_fd, EVIOCGRAB, (void *)0);
                close(mouse_fd);
                mouse_fd = -1;
            }
        }

        /* Stdin fallback (SSH) */
        if (pfds[stdin_idx].revents & POLLIN) {
            unsigned char ch;
            while (read(STDIN_FILENO, &ch, 1) == 1) {
                if (ch == 'q' || ch == 'Q') { handle_action('q'); break; }
                if (ch == 'g')              { handle_action('G'); continue; }
                if (ch == ' ')              { handle_action(' '); continue; }
                if (ch == 's')              { handle_action('S'); continue; }
                /* 'l' conflicts with left arrow, use 'L' for load via SSH */
                if (ch == 'L')              { handle_action('L'); continue; }
                if (ch == 'r')              { handle_action('R'); continue; }
                if (ch == 'c')              { handle_action('C'); continue; }
                if (ch == 'v')              { handle_action('V'); continue; }
                if (ch == 'b')              { handle_action('B'); continue; }
                if (ch == 'n')              { handle_action('N'); continue; }
                if (ch == '\t')             { handle_action('T'); continue; }
                if (ch == 'f')              { handle_action('F'); continue; }
                if (ch == 'x')              { handle_action('X'); continue; }
                if (ch == 0x7F)             { handle_action('Z'); continue; } /* backspace */
                if (ch == '=' || ch == '+') { handle_action('+'); continue; }
                if (ch == '-')              { handle_action('-'); continue; }
                if (ch == 0x1B) {
                    unsigned char seq[2] = {0, 0};
                    if (read(STDIN_FILENO, &seq[0], 1) != 1) continue;
                    if (seq[0] != '[') continue;
                    if (read(STDIN_FILENO, &seq[1], 1) != 1) continue;
                    if (seq[1] == 'A') handle_action('u');       /* Up */
                    else if (seq[1] == 'B') handle_action('d');  /* Down */
                    else if (seq[1] == 'C') handle_action('r');  /* Right */
                    else if (seq[1] == 'D') handle_action('l');  /* Left */
                    continue;
                }
            }
        }

        /* HID hotplug: rescan every ~10s if not connected */
        if (kbd_fd < 0 && (poll_count % 50) == 0)
            kbd_fd = find_keyboard();
        if (mouse_fd < 0 && (poll_count % 50) == 0)
            mouse_fd = find_mouse();
    }

    if (kbd_fd >= 0) {
        ioctl(kbd_fd, EVIOCGRAB, (void *)0);
        close(kbd_fd);
    }
    if (mouse_fd >= 0) {
        ioctl(mouse_fd, EVIOCGRAB, (void *)0);
        close(mouse_fd);
    }
    return NULL;
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
    char            path[64];
    uint32_t        buf_type;
    int             is_mplane;
    int             width, height;
    int             buf_count;
    struct cam_buf  bufs[V4L2_BUF_CNT];
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

static int cam_init(struct cam *c)
{
    for (int i = 0; i < V4L2_BUF_CNT; i++) {
        c->bufs[i].dmabuf_fd = -1;
        c->bufs[i].start = NULL;
        c->bufs[i].rga_handle = 0;
    }

    c->fd = open(c->path, O_RDWR | O_NONBLOCK);
    if (c->fd < 0) {
        fprintf(stderr, "V4L2: open %s: %s\n", c->path, strerror(errno));
        return -1;
    }

    struct v4l2_capability cap;
    if (xioctl(c->fd, VIDIOC_QUERYCAP, &cap) < 0) goto err;

    c->is_mplane = !!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE);
    if (!c->is_mplane && !(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
        goto err;
    c->buf_type = c->is_mplane
        ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
        : V4L2_BUF_TYPE_VIDEO_CAPTURE;

    struct v4l2_format fmt = { .type = c->buf_type };
    if (c->is_mplane) {
        fmt.fmt.pix_mp.width       = ch_cfg[CH].w;
        fmt.fmt.pix_mp.height      = ch_cfg[CH].h;
        fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_UYVY;
        fmt.fmt.pix_mp.num_planes  = 1;
        fmt.fmt.pix_mp.field       = V4L2_FIELD_NONE;
    } else {
        fmt.fmt.pix.width       = ch_cfg[CH].w;
        fmt.fmt.pix.height      = ch_cfg[CH].h;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
        fmt.fmt.pix.field       = V4L2_FIELD_NONE;
    }
    if (xioctl(c->fd, VIDIOC_S_FMT, &fmt) < 0) {
        fprintf(stderr, "V4L2: S_FMT: %s\n", strerror(errno));
        goto err;
    }
    c->width  = c->is_mplane ? fmt.fmt.pix_mp.width  : fmt.fmt.pix.width;
    c->height = c->is_mplane ? fmt.fmt.pix_mp.height : fmt.fmt.pix.height;
    printf("V4L2: %s %dx%d UYVY\n", c->path, c->width, c->height);

    struct v4l2_requestbuffers req = {
        .count  = V4L2_BUF_CNT,
        .type   = c->buf_type,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (xioctl(c->fd, VIDIOC_REQBUFS, &req) < 0) {
        fprintf(stderr, "V4L2: REQBUFS: %s\n", strerror(errno));
        goto err;
    }
    c->buf_count = req.count;

    for (int i = 0; i < c->buf_count; i++) {
        struct v4l2_buffer vb = {
            .type = c->buf_type, .memory = V4L2_MEMORY_MMAP, .index = i,
        };
        struct v4l2_plane planes[1] = {};
        if (c->is_mplane) { vb.m.planes = planes; vb.length = 1; }
        if (xioctl(c->fd, VIDIOC_QUERYBUF, &vb) < 0) goto err;

        uint32_t off = c->is_mplane ? planes[0].m.mem_offset : vb.m.offset;
        size_t   len = c->is_mplane ? planes[0].length       : vb.length;

        c->bufs[i].start = mmap(NULL, len, PROT_READ | PROT_WRITE,
                                MAP_SHARED, c->fd, off);
        if (c->bufs[i].start == MAP_FAILED) { c->bufs[i].start = NULL; goto err; }
        c->bufs[i].length = len;

        struct v4l2_exportbuffer eb = {
            .type = c->buf_type, .index = i, .plane = 0,
            .flags = O_RDWR | O_CLOEXEC,
        };
        if (xioctl(c->fd, VIDIOC_EXPBUF, &eb) < 0) {
            fprintf(stderr, "V4L2: EXPBUF %d: %s\n", i, strerror(errno));
            goto err;
        }
        c->bufs[i].dmabuf_fd = eb.fd;
    }

    for (int i = 0; i < c->buf_count; i++) {
        struct v4l2_buffer vb = {
            .type = c->buf_type, .memory = V4L2_MEMORY_MMAP, .index = i,
        };
        struct v4l2_plane planes[1] = {};
        if (c->is_mplane) { vb.m.planes = planes; vb.length = 1; }
        if (xioctl(c->fd, VIDIOC_QBUF, &vb) < 0) goto err;
    }

    int type_on = c->buf_type;
    if (xioctl(c->fd, VIDIOC_STREAMON, &type_on) < 0) {
        fprintf(stderr, "V4L2: STREAMON: %s\n", strerror(errno));
        goto err;
    }
    return 0;

err:
    for (int i = 0; i < V4L2_BUF_CNT; i++) {
        if (c->bufs[i].dmabuf_fd >= 0) { close(c->bufs[i].dmabuf_fd); c->bufs[i].dmabuf_fd = -1; }
        if (c->bufs[i].start) { munmap(c->bufs[i].start, c->bufs[i].length); c->bufs[i].start = NULL; }
    }
    if (c->fd >= 0) { close(c->fd); c->fd = -1; }
    return -1;
}

static void cam_cleanup(struct cam *c)
{
    if (c->fd < 0) return;
    int type = c->buf_type;
    xioctl(c->fd, VIDIOC_STREAMOFF, &type);
    for (int i = 0; i < c->buf_count; i++) {
        if (c->bufs[i].rga_handle) releasebuffer_handle(c->bufs[i].rga_handle);
        if (c->bufs[i].dmabuf_fd >= 0) close(c->bufs[i].dmabuf_fd);
        if (c->bufs[i].start) munmap(c->bufs[i].start, c->bufs[i].length);
    }
    close(c->fd);
    c->fd = -1;
}

/* ─────────────────── RGA ─────────────────── */

static int rga_setup(struct cam *c, struct display *d)
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
        if (!d->rga_handle[b]) { fprintf(stderr, "RGA: import dst%d failed\n", b); return -1; }
        d->rga[b] = wrapbuffer_handle(d->rga_handle[b], w, h,
                                       RK_FORMAT_BGRX_8888, wstride, h);
    }

    im_handle_param_t sp = {
        .width  = (uint32_t)c->width,
        .height = (uint32_t)c->height,
        .format = RK_FORMAT_UYVY_422,
    };
    for (int i = 0; i < c->buf_count; i++) {
        c->bufs[i].rga_handle = importbuffer_fd(c->bufs[i].dmabuf_fd, &sp);
        if (!c->bufs[i].rga_handle) {
            fprintf(stderr, "RGA: import buf%d failed\n", i);
            return -1;
        }
        c->bufs[i].rga = wrapbuffer_handle(c->bufs[i].rga_handle,
                                            c->width, c->height,
                                            RK_FORMAT_UYVY_422);
        c->bufs[i].rga.color_space_mode = IM_YUV_TO_RGB_BT601_LIMIT;
    }
    printf("RGA: %dx%d UYVY -> %dx%d XRGB fullscreen\n",
           c->width, c->height, w, h);
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
    "uniform vec4 u_crop;\n"
    "void main() {\n"
    "    vec2 uv = u_crop.xy + v_uv * u_crop.zw;\n"
    "    vec2 d = (uv - u_center) * u_res;\n"
    "    float r2 = dot(d, d) / (u_max_r * u_max_r);\n"
    "    float s = 1.0 + u_k1 * r2 + u_k2 * r2 * r2;\n"
    "    vec2 src = u_center + (uv - u_center) * s;\n"
    "    if (src.x < 0.0 || src.x > 1.0 || src.y < 0.0 || src.y > 1.0)\n"
    "        gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);\n"
    "    else\n"
    "        gl_FragColor = texture2D(u_tex, src);\n"
    "}\n";

typedef struct {
    struct gbm_device *gbm;
    EGLDisplay dpy;
    EGLContext ctx;
    GLuint prog;

    GLint loc_tex, loc_center, loc_res, loc_max_r, loc_k1, loc_k2, loc_crop;

    /* FBOs backed by DRM dumb buffers (double-buffered) */
    EGLImageKHR dst_img[2];
    GLuint dst_tex[2];
    GLuint fbo[2];

    /* Intermediate XRGB buffer (RGA converts UYVY→XRGB here, GPU reads it) */
    uint32_t inter_gem;
    uint32_t inter_pitch;
    uint64_t inter_size;
    int inter_dmabuf_fd;
    rga_buffer_handle_t inter_rga_handle;
    rga_buffer_t inter_rga;
    EGLImageKHR src_img;
    GLuint src_tex;

    PFNEGLCREATEIMAGEKHRPROC CreateImage;
    PFNEGLDESTROYIMAGEKHRPROC DestroyImage;
    PFNGLEGLIMAGETARGETTEXTURE2DOESPROC ImageTargetTex2D;

    int dst_w, dst_h;
    int src_w, src_h;
} gpu_ctx;

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

static int gpu_init(gpu_ctx *g, struct display *d, struct cam *c)
{
    memset(g, 0, sizeof(*g));
    g->inter_dmabuf_fd = -1;
    g->dst_w = d->mode.hdisplay;
    g->dst_h = d->mode.vdisplay;
    g->src_w = c->width;
    g->src_h = c->height;

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
    g->loc_crop   = glGetUniformLocation(g->prog, "u_crop");

    /* Fullscreen quad (GL has bottom-left origin, flip Y for DRM top-left) */
    static const GLfloat verts[] = {
        -1.0f, -1.0f,   1.0f, -1.0f,   -1.0f,  1.0f,   1.0f,  1.0f,
    };
    static const GLfloat texcoords[] = {
         0.0f,  1.0f,   1.0f,  1.0f,    0.0f,  0.0f,   1.0f,  0.0f,
    };
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, verts);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, texcoords);

    /* Import DRM dumb buffers as FBO render targets */
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

    /* Allocate intermediate DRM dumb buffer for RGA UYVY→XRGB conversion */
    {
        struct drm_mode_create_dumb cr = {
            .width = (uint32_t)c->width, .height = (uint32_t)c->height, .bpp = 32,
        };
        if (drmIoctl(d->fd, DRM_IOCTL_MODE_CREATE_DUMB, &cr)) {
            perror("GPU: inter create_dumb"); return -1;
        }
        g->inter_gem = cr.handle;
        g->inter_pitch = cr.pitch;
        g->inter_size = cr.size;

        if (drmPrimeHandleToFD(d->fd, cr.handle, DRM_CLOEXEC | DRM_RDWR,
                               &g->inter_dmabuf_fd)) {
            perror("GPU: inter primeHandleToFD"); return -1;
        }

        /* RGA handle for the intermediate buffer */
        int wstride = (int)(cr.pitch / 4);
        im_handle_param_t ip = {
            .width = (uint32_t)wstride, .height = (uint32_t)c->height,
            .format = RK_FORMAT_BGRX_8888,
        };
        g->inter_rga_handle = importbuffer_fd(g->inter_dmabuf_fd, &ip);
        if (!g->inter_rga_handle) {
            fprintf(stderr, "GPU: inter RGA import failed\n"); return -1;
        }
        g->inter_rga = wrapbuffer_handle(g->inter_rga_handle,
                                          c->width, c->height,
                                          RK_FORMAT_BGRX_8888, wstride, c->height);

        /* Import as XRGB8888 GL texture */
        EGLint attrs[] = {
            EGL_WIDTH, c->width,
            EGL_HEIGHT, c->height,
            EGL_LINUX_DRM_FOURCC_EXT, DRM_FORMAT_XRGB8888,
            EGL_DMA_BUF_PLANE0_FD_EXT, g->inter_dmabuf_fd,
            EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0,
            EGL_DMA_BUF_PLANE0_PITCH_EXT, (EGLint)cr.pitch,
            EGL_NONE,
        };
        g->src_img = g->CreateImage(g->dpy, EGL_NO_CONTEXT,
                                     EGL_LINUX_DMA_BUF_EXT, NULL, attrs);
        if (g->src_img == EGL_NO_IMAGE_KHR) {
            fprintf(stderr, "GPU: src EGLImage failed\n"); return -1;
        }

        glGenTextures(1, &g->src_tex);
        glBindTexture(GL_TEXTURE_2D, g->src_tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        g->ImageTargetTex2D(GL_TEXTURE_2D, (GLeglImageOES)g->src_img);

        printf("GPU: inter buf %dx%d pitch=%u (%uKB)\n",
               c->width, c->height, cr.pitch, (unsigned)(cr.size / 1024));
    }

    printf("GPU: initialized — XRGB intermediate + 2 FBOs (%dx%d)\n",
           g->dst_w, g->dst_h);
    return 0;
}

static void gpu_render(gpu_ctx *g, int back_buf,
                       float k1, float k2, float cx, float cy,
                       const float crop[4])
{
    glBindFramebuffer(GL_FRAMEBUFFER, g->fbo[back_buf]);
    glViewport(0, 0, g->dst_w, g->dst_h);
    glClear(GL_COLOR_BUFFER_BIT);

    glUseProgram(g->prog);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, g->src_tex);
    glUniform1i(g->loc_tex, 0);

    glUniform2f(g->loc_center, cx, cy);
    glUniform2f(g->loc_res, (float)g->src_w, (float)g->src_h);
    float max_r = sqrtf((float)(g->src_w * g->src_w + g->src_h * g->src_h)) * 0.5f;
    glUniform1f(g->loc_max_r, max_r);
    glUniform1f(g->loc_k1, k1);
    glUniform1f(g->loc_k2, k2);
    glUniform4fv(g->loc_crop, 1, crop);

    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glFinish();
}

static void gpu_cleanup(gpu_ctx *g, int drm_fd)
{
    if (!g->dpy) return;

    eglMakeCurrent(g->dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, g->ctx);

    if (g->src_tex) glDeleteTextures(1, &g->src_tex);
    if (g->src_img) g->DestroyImage(g->dpy, g->src_img);
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

    /* Free intermediate DRM buffer */
    if (g->inter_rga_handle) releasebuffer_handle(g->inter_rga_handle);
    if (g->inter_dmabuf_fd >= 0) close(g->inter_dmabuf_fd);
    if (g->inter_gem) {
        struct drm_mode_destroy_dumb dd = { .handle = g->inter_gem };
        drmIoctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dd);
    }
}

/* ─────────────────── NVP6324 + media ─────────────────── */

static int setup_nvp6324(void)
{
    int fd = open(SYSFS_FMT, O_WRONLY);
    if (fd < 0) {
        fprintf(stderr, "NVP6324: cannot open %s: %s\n", SYSFS_FMT, strerror(errno));
        return -1;
    }
    for (int ch = 0; ch < MAX_CAMS; ch++) {
        char cmd[16];
        int len = snprintf(cmd, sizeof(cmd), "%d %d", ch, ch_cfg[ch].dv_idx);
        if (write(fd, cmd, len) < 0)
            fprintf(stderr, "NVP6324: ch%d set failed: %s\n", ch, strerror(errno));
    }
    close(fd);

    fd = open(SYSFS_FMT, O_RDONLY);
    if (fd >= 0) {
        char buf[512];
        int n = read(fd, buf, sizeof(buf) - 1);
        if (n > 0) { buf[n] = 0; printf("NVP6324:\n%s", buf); }
        close(fd);
    }
    return 0;
}

static void setup_media(void)
{
    const char *cmds[] = {
        "media-ctl -d /dev/media1 --set-v4l2 "
            "'\"m00_b_jaguar2 4-0031\":0[fmt:UYVY8_2X8/1920x1080 field:none]'",
        "media-ctl -d /dev/media1 --set-v4l2 "
            "'\"rockchip-csi2-dphy0\":0[fmt:UYVY8_2X8/1920x1080 field:none]'",
        "media-ctl -d /dev/media1 --set-v4l2 "
            "'\"rockchip-csi2-dphy0\":1[fmt:UYVY8_2X8/1920x1080 field:none]'",
    };
    for (int i = 0; i < 3; i++) {
        int ret = system(cmds[i]);
        if (ret != 0)
            fprintf(stderr, "media-ctl cmd %d returned %d\n", i, ret);
    }
    printf("Media pipeline: 1920x1080 UYVY\n");
}

/* ─────────────────── Main ─────────────────── */

int main(int argc, char **argv)
{
    const char *dev = "/dev/video13";
    if (argc > 1) dev = argv[1];

    setvbuf(stdout, NULL, _IOLBF, 0);
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    printf("=== Configuring NVP6324 ===\n");
    if (setup_nvp6324() < 0) return 1;
    setup_media();
    usleep(500000);

    struct display disp;
    if (drm_init(&disp) < 0) return 1;

    struct cam cam;
    memset(&cam, 0, sizeof(cam));
    cam.fd = -1;
    snprintf(cam.path, sizeof(cam.path), "%s", dev);
    if (cam_init(&cam) < 0) { drm_cleanup(&disp); return 1; }

    if (rga_setup(&cam, &disp) < 0) {
        cam_cleanup(&cam);
        drm_cleanup(&disp);
        return 1;
    }

    /* GPU undistortion via EGL/GLES2 */
    gpu_ctx gpu;
    if (gpu_init(&gpu, &disp, &cam) < 0) {
        fprintf(stderr, "GPU init failed — cannot continue\n");
        cam_cleanup(&cam);
        drm_cleanup(&disp);
        return 1;
    }

    /* Try to load saved calibration */
    {
        double k1, k2, cx, cy;
        if (load_params(CALIB_PATH, &k1, &k2, &cx, &cy) == 0) {
            g_k1 = k1;
            g_k2 = k2;
            g_cx = cx;
            g_cy = cy;
        }
    }

    /* Start input thread */
    pthread_t input_tid;

    double fps = 0;
    struct timespec t0;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    unsigned long frames = 0;

    /* RGA bypass rendering setup */
    rga_buffer_t pat_empty;
    memset(&pat_empty, 0, sizeof(pat_empty));
    im_rect prect_empty = {0, 0, 0, 0};
    im_rect src_rect = {0, 0, cam.width, cam.height};
    im_rect dst_rect = {0, 0, (int)disp.mode.hdisplay, (int)disp.mode.vdisplay};

    int dst_w = disp.mode.hdisplay;
    int dst_h = disp.mode.vdisplay;
    int dst_stride4 = disp.pitch / 4;

    /* Initialize mouse/display globals for input thread */
    g_disp_w = dst_w;
    g_disp_h = dst_h;
    g_src_w = cam.width;
    g_src_h = cam.height;
    g_crop_x = 0; g_crop_y = 0;
    g_crop_w = cam.width; g_crop_h = cam.height;
    g_cursor_x = dst_w / 2;
    g_cursor_y = dst_h / 2;

    pthread_create(&input_tid, NULL, input_thread, NULL);

    printf("Running — video on HDMI, distortion correction active\n");
    printf("Keys: Arrows=k1/k2 C/V=cx B/N=cy G=grid SPACE=bypass S=save shift-L=load R=reset Q=quit\n");
    printf("Mouse: move=crosshair click=point scroll=zoom Tab=line F=fit X=clear BS=undo\n");

    while (!quit) {
        struct pollfd pfd = { .fd = cam.fd, .events = POLLIN };
        int ret = poll(&pfd, 1, 100);
        if (ret < 0) { if (errno == EINTR) continue; break; }
        if (!(pfd.revents & POLLIN)) continue;

        /* V4L2 frame */
        struct v4l2_buffer vb = {
            .type = cam.buf_type, .memory = V4L2_MEMORY_MMAP,
        };
        struct v4l2_plane planes[1] = {};
        if (cam.is_mplane) { vb.m.planes = planes; vb.length = 1; }

        if (xioctl(cam.fd, VIDIOC_DQBUF, &vb) < 0)
            continue;

        int bi = vb.index;
        int bb = disp.back;

        /* Snapshot current parameters under lock */
        pthread_mutex_lock(&param_lock);
        double k1 = g_k1;
        double k2 = g_k2;
        double cx = g_cx;
        double cy = g_cy;
        int grid_on = g_grid;
        int bypass = g_bypass;
        int do_save = g_save_req;
        int do_load = g_load_req;
        int do_reset = g_reset_req;
        int do_fit = g_fit_req;
        g_save_req = 0;
        g_load_req = 0;
        g_reset_req = 0;
        g_fit_req = 0;
        pthread_mutex_unlock(&param_lock);

        /* Handle save/load/reset */
        if (do_save)
            save_params(CALIB_PATH, k1, k2, cx, cy);

        if (do_load) {
            double lk1, lk2, lcx, lcy;
            lk1 = k1; lk2 = k2; lcx = cx; lcy = cy;
            if (load_params(CALIB_PATH, &lk1, &lk2, &lcx, &lcy) == 0) {
                pthread_mutex_lock(&param_lock);
                g_k1 = lk1; g_k2 = lk2; g_cx = lcx; g_cy = lcy;
                pthread_mutex_unlock(&param_lock);
                k1 = lk1; k2 = lk2; cx = lcx; cy = lcy;
            }
        }

        if (do_reset) {
            pthread_mutex_lock(&param_lock);
            g_k1 = 0.0; g_k2 = 0.0; g_cx = 0.5; g_cy = 0.5;
            pthread_mutex_unlock(&param_lock);
            k1 = 0.0; k2 = 0.0; cx = 0.5; cy = 0.5;
            printf("Reset: k1=0 k2=0 cx=0.5 cy=0.5\n");
        }

        if (do_fit) {
            auto_fit_k1k2(cam.width, cam.height, cx, cy);
            /* Re-read fitted values */
            pthread_mutex_lock(&param_lock);
            k1 = g_k1; k2 = g_k2;
            pthread_mutex_unlock(&param_lock);
        }

        /* Compute zoom crop rect (source coordinates) */
        int zoom = g_zoom;
        im_rect zoom_rect;
        if (zoom == 0) {
            zoom_rect = src_rect;
        } else {
            int zf = 1 << zoom;
            int zw = cam.width / zf;
            int zh = cam.height / zf;
            int zcx = g_cursor_visible
                     ? g_cursor_x * cam.width / dst_w
                     : cam.width / 2;
            int zcy = g_cursor_visible
                     ? g_cursor_y * cam.height / dst_h
                     : cam.height / 2;
            int zx = zcx - zw / 2;
            int zy = zcy - zh / 2;
            if (zx < 0) zx = 0;
            if (zy < 0) zy = 0;
            if (zx + zw > cam.width) zx = cam.width - zw;
            if (zy + zh > cam.height) zy = cam.height - zh;
            zx &= ~1; zy &= ~1;  /* RGA alignment */
            zoom_rect = (im_rect){zx, zy, zw, zh};
        }
        /* Update crop globals for zoom-aware click mapping */
        g_crop_x = zoom_rect.x; g_crop_y = zoom_rect.y;
        g_crop_w = zoom_rect.width; g_crop_h = zoom_rect.height;

        /* Render */
        if (bypass || (k1 == 0.0 && k2 == 0.0)) {
            /* Fast path: RGA crop+scale+colorconvert */
            improcess(cam.bufs[bi].rga, disp.rga[bb], pat_empty,
                      zoom_rect, dst_rect, prect_empty, IM_SYNC);
        } else {
            /* RGA: UYVY → XRGB into intermediate buffer (full frame) */
            improcess(cam.bufs[bi].rga, gpu.inter_rga, pat_empty,
                      src_rect, src_rect, prect_empty, IM_SYNC);
            /* GPU: undistort + zoom + scale to display FBO */
            float crop[4] = {
                zoom_rect.x / (float)cam.width,
                zoom_rect.y / (float)cam.height,
                zoom_rect.width / (float)cam.width,
                zoom_rect.height / (float)cam.height,
            };
            gpu_render(&gpu, bb,
                       (float)k1, (float)k2, (float)cx, (float)cy,
                       crop);
        }

        /* Grid overlay */
        if (grid_on)
            draw_grid((uint32_t *)disp.map[bb], dst_stride4, dst_w, dst_h,
                      0x0040FF40);

        /* OSD */
        draw_osd(disp.map[bb], disp.pitch, dst_w, dst_h, fps,
                 k1, k2, cx, cy, grid_on, bypass);

        /* Calibration points + mouse crosshair (drawn last = on top) */
        if (calib_npts > 0)
            draw_calib_points((uint32_t *)disp.map[bb], dst_stride4, dst_w, dst_h,
                              zoom_rect.x, zoom_rect.y, zoom_rect.width, zoom_rect.height,
                              cam.width, cam.height);
        if (g_cursor_visible)
            draw_crosshair((uint32_t *)disp.map[bb], dst_stride4, dst_w, dst_h);

        /* Flip */
        drmModeSetCrtc(disp.fd, disp.crtc_id, disp.fb_id[bb],
                       0, 0, &disp.conn_id, 1, &disp.mode);
        disp.back = 1 - bb;

        xioctl(cam.fd, VIDIOC_QBUF, &vb);
        frames++;

        /* FPS counter */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        double elapsed = (now.tv_sec - t0.tv_sec)
                       + (now.tv_nsec - t0.tv_nsec) / 1e9;
        if (elapsed >= 2.0) {
            fps = frames / elapsed;
            t0 = now;
            frames = 0;
        }
    }

    printf("Shutting down...\n");
    pthread_join(input_tid, NULL);
    gpu_cleanup(&gpu, disp.fd);
    cam_cleanup(&cam);
    drm_cleanup(&disp);
    printf("Done.\n");
    return 0;
}
