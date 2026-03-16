/*
 * app.h - Central types and constants for Vehicle Monitor
 */
#ifndef APP_H
#define APP_H

#include <gtk/gtk.h>
#include <sqlite3.h>
#include <pthread.h>
#include <stdint.h>
#include "core/homography.h"

/* --- Constants --- */
#define MAX_CAMERAS      8
#define MAX_SENSORS      16
#define MAX_GRID         64
#define MAX_ACTION_GROUPS 16
#define DEFAULT_GRID_COLS 48
#define DEFAULT_GRID_ROWS 27
#define ZONE_COOLDOWN_SEC 15.0

/* Action group action bitmask */
#define ACTION_FLASH_LED  (1 << 0)
#define ACTION_EMAIL      (1 << 1)
#define ACTION_BEEP       (1 << 2)
#define SPARKLINE_POINTS  60

/* Camera connection state */
#define CAM_IDLE        0
#define CAM_CONNECTING  1
#define CAM_CONNECTED   2
#define CAM_FAILED      3

/* Birdview positions */
#define BIRDVIEW_NONE  0
#define BIRDVIEW_FRONT 1
#define BIRDVIEW_REAR  2
#define BIRDVIEW_LEFT  3
#define BIRDVIEW_RIGHT 4

/* --- Forward declarations (avoid FFmpeg header pollution) --- */
typedef struct AVFormatContext AVFormatContext;
typedef struct AVCodecContext  AVCodecContext;
typedef struct AVFrame        AVFrame;
typedef struct SwsContext     SwsContext;
typedef struct AVBufferRef    AVBufferRef;
struct RingBuf;
struct ClipTracker;
struct ClipTranscoder;

/* --- Camera context (one per camera) --- */
typedef struct {
    int          id;             /* DB primary key */
    char         name[64];
    char         url[512];
    int          enabled;
    double       scale;
    int          rotation;       /* 0, 90, 180, 270 */
    int          aspect_w;       /* 16:9=16,9  4:3=4,3  Source=0,0 */
    int          aspect_h;
    int          grid_cols;
    int          grid_rows;
    int          birdview_pos;   /* BIRDVIEW_* */
    int          force_sw;
    int          threshold;
    float        alpha;
    float        trigger_pct;
    int          framestep;
    int          min_blob_cells;    /* minimum blob size to trigger (cells) */
    int          sort_order;

    /* FFmpeg (opaque pointers) */
    AVFormatContext *fmt_ctx;
    AVCodecContext  *dec_ctx;
    AVBufferRef     *hw_device_ctx;
    AVFrame         *sw_frame;
    SwsContext      *sws_color;
    int              video_idx;
    int              hw_decode;
    const char      *decoder_name;

    /* Source and display dimensions */
    int          src_w, src_h;
    int          disp_w, disp_h;
    int          gray_size;
    int          disp_size;      /* disp_w * disp_h * 3 */

    /* Motion buffers (worker-owned) */
    uint8_t     *curr_gray;
    uint8_t     *blurred;
    float       *bg_model;
    int          bg_initialized;
    uint8_t     *work_color;
    uint8_t     *work_motion;

    /* Shared buffers (mutex-protected) */
    pthread_mutex_t buf_mutex;
    uint8_t     *color_rgb;
    uint8_t     *motion_rgb;
    float        shared_motion_pct;
    int          shared_motion_pixels;
    int          shared_grid_cols;
    int          shared_grid_rows;
    float        shared_grid_pcts[MAX_GRID * MAX_GRID];
    float        shared_grid_max_pct;
    int          shared_grid_max_col;
    int          shared_grid_max_row;
    int          shared_blob_cells;
    int          shared_blob_col;
    int          shared_blob_row;
    float        shared_fps;
    int          buf_updated;

    /* Pre-rotated birdview buffer (worker writes, GTK reads, mutex-protected) */
    uint8_t     *bv_rotated;     /* rotated RGB for birdview display */
    int          bv_rot_w;       /* rotated width */
    int          bv_rot_h;       /* rotated height */

    /* Lens undistortion (worker applies, GTK thread builds LUT) */
    WarpLUT      undistort_lut;
    int volatile undistort_valid;
    uint8_t     *work_undistorted;

    /* Draw-local buffers (GTK thread only) */
    uint8_t     *draw_color;
    uint8_t     *draw_motion;
    float        draw_grid_pcts[MAX_GRID * MAX_GRID];
    float        draw_grid_max_pct;
    int          draw_blob_cells;
    float        draw_motion_pct;
    int          draw_motion_pixels;
    float        draw_fps;

    /* Thread control */
    pthread_t    thread;
    int          running;
    int          started;
    int          connect_state;   /* CAM_IDLE / CONNECTING / CONNECTED / FAILED */
    int          frame_count;
    float        fps;
    double       last_time;

    /* Zone event cooldown: last trigger time per action_group_id */
    double       last_trigger_time[MAX_ACTION_GROUPS];

    /* Recording (worker thread owns) */
    int              recording_enabled;
    int              rec_segment_sec;
    int              recording;
    void            *rec_file;         /* FILE* for raw HEVC output */
    int              rec_id;
    time_t           rec_start_time;
    int64_t          rec_first_pts;    /* first PTS in stream timebase */
    char             rec_dir[256];

    /* Event clip system (worker thread owns) */
    struct RingBuf      *ring_buf;
    struct ClipTracker  *clip_tracker;
} CameraCtx;

/* --- Sensor context (one per sensor) --- */
typedef struct {
    int          id;             /* DB primary key */
    char         name[64];
    char         type[16];       /* "RS485" or "CAN" */
    char         address[32];
    int          register_addr;
    double       scale;
    double       offset;
    char         units[16];
    int          poll_interval_ms;
    int          enabled;

    /* Live data */
    double       current_value;
    double       history[SPARKLINE_POINTS];
    int          history_count;
    int          history_idx;
    int          running;
    pthread_t    thread;
} SensorCtx;

/* --- Page plugin interface --- */
typedef struct AppCtx AppCtx;

typedef struct {
    const char  *id;
    const char  *title;
    GtkWidget * (*build)(AppCtx *app);
    void        (*on_show)(AppCtx *app);
    void        (*on_hide)(AppCtx *app);
    void        (*cleanup)(AppCtx *app);
} PageDef;

/* --- Application context --- */
struct AppCtx {
    /* GTK */
    GtkWidget   *window;
    GtkWidget   *stack;
    GtkWidget   *sidebar;

    /* Database */
    sqlite3     *db;
    char         db_path[256];

    /* Cameras */
    CameraCtx    cameras[MAX_CAMERAS];
    int          num_cameras;

    /* Sensors */
    SensorCtx    sensors[MAX_SENSORS];
    int          num_sensors;

    /* Pages */
    const PageDef **pages;
    int          num_pages;

    /* Page-specific state (opaque pointers, managed by each page) */
    void        *page_cameras_state;
    void        *page_camera_config_state;
    void        *page_motion_config_state;
    void        *page_dvr_state;
    void        *page_birdview_state;
    void        *page_sensors_state;
    void        *page_sensor_config_state;
    void        *page_birdview_config_state;
    void        *page_config_state;
    void        *page_events_state;
    void        *page_dvr_config_state;
    void        *page_lens_config_state;

    /* Event clip transcoder */
    struct ClipTranscoder *clip_transcoder;

    /* Global flags */
    int          running;
    int          birdview_visible;  /* set by birdview/config pages to gate pre-rotation */
    guint        stats_timer;
};

/* --- Page declarations --- */
extern const PageDef page_cameras;
extern const PageDef page_camera_config;
extern const PageDef page_motion_config;
extern const PageDef page_dvr;
extern const PageDef page_birdview;
extern const PageDef page_birdview_config;
extern const PageDef page_sensors;
extern const PageDef page_sensor_config;
extern const PageDef page_events;
extern const PageDef page_dvr_config;
extern const PageDef page_lens_config;
extern const PageDef page_config;

#endif /* APP_H */
