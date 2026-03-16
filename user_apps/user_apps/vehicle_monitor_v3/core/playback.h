/*
 * core/playback.h - DVR playback engine (SW decode from recording files)
 */
#ifndef PLAYBACK_H
#define PLAYBACK_H

#include "../app.h"

typedef enum {
    SPEED_HALF,
    SPEED_1X,
    SPEED_2X,
    SPEED_4X,
    SPEED_8X,
    SPEED_COUNT
} PlaybackSpeed;

typedef struct {
    AppCtx          *app;
    int              camera_id;
    int              aspect_w, aspect_h;
    double           scale;

    /* FFmpeg SW decode */
    AVFormatContext *fmt_ctx;
    AVCodecContext  *dec_ctx;
    SwsContext      *sws_ctx;
    int              video_idx;
    int              src_w, src_h;
    int              disp_w, disp_h;
    int              disp_size;

    /* Thread */
    pthread_t        thread;
    pthread_mutex_t  mutex;
    int              running;
    int              started;

    /* State (protected by mutex) */
    int              paused;
    PlaybackSpeed    speed;
    double           position_sec;    /* current epoch time */

    /* Segment info */
    char             current_file[512];
    double           seg_start_epoch;
    double           seg_end_epoch;

    /* Shared frame (protected by mutex) */
    uint8_t         *frame_rgb;
    int              frame_updated;
    double           frame_time;

    /* Seek request */
    int              seek_requested;
    double           seek_target;

    /* Pacing */
    double           wall_base;
    double           video_base;

    /* Raw HEVC mode (vs FFmpeg demux for legacy .mp4) */
    int              raw_mode;
    void            *raw_file;         /* FILE* */
    int              raw_rec_id;       /* DB recording ID for index */
    double           pts_to_sec;       /* time_base conversion factor */
} PlaybackCtx;

/* Create/destroy */
PlaybackCtx *playback_create(AppCtx *app, int camera_id);
void         playback_destroy(PlaybackCtx *pb);

/* Start playback at given epoch time, stop playback */
int  playback_start(PlaybackCtx *pb, double epoch_time);
void playback_stop(PlaybackCtx *pb);

/* Controls */
void   playback_toggle_pause(PlaybackCtx *pb);
void   playback_set_speed(PlaybackCtx *pb, PlaybackSpeed speed);
void   playback_seek(PlaybackCtx *pb, double epoch_time);
double playback_get_position(PlaybackCtx *pb);
int    playback_is_paused(PlaybackCtx *pb);
PlaybackSpeed playback_get_speed(PlaybackCtx *pb);

/* Speed helpers */
double      playback_speed_multiplier(PlaybackSpeed s);
const char *playback_speed_label(PlaybackSpeed s);

#endif /* PLAYBACK_H */
