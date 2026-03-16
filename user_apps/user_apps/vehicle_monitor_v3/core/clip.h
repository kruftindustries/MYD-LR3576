/*
 * core/clip.h - Event-triggered clip creation
 *
 * Per-camera ring buffer of HEVC packets, state machine for clip windows,
 * shared transcoder thread for MP4 output with HW encode + SW fallback.
 */
#ifndef CLIP_H
#define CLIP_H

#include <libavcodec/avcodec.h>
#include <libavutil/rational.h>
#include <pthread.h>
#include <stdint.h>
#include <sqlite3.h>

/* --- Ring Buffer --- */

#define RING_BUF_MAX_PKTS   1500   /* ~25s at 50fps with headroom */

typedef struct {
    uint8_t  *data;        /* deep-copied packet payload */
    int       size;
    int64_t   pts, dts;
    int       flags;       /* AV_PKT_FLAG_KEY etc. */
    double    wall_time;   /* CLOCK_MONOTONIC seconds */
} RingPacket;

typedef struct RingBuf {
    RingPacket  pkts[RING_BUF_MAX_PKTS];
    int         head;          /* next write position */
    int         count;         /* valid packet count */
    uint8_t    *extradata;     /* VPS/SPS/PPS for decoder init */
    int         extradata_size;
    int         src_w, src_h;
    AVRational  time_base;
} RingBuf;

/* --- Clip State Machine --- */

typedef enum {
    CLIP_IDLE,
    CLIP_RECORDING,
} ClipState;

#define CLIP_PRE_SEC   15.0
#define CLIP_POST_SEC  15.0
#define CLIP_MAX_EVENTS 64

typedef struct ClipTracker {
    ClipState   state;
    double      first_event_time;  /* wall_time of first event in window */
    double      last_event_time;   /* wall_time of most recent event */
    int         first_event_id;
    int         event_ids[CLIP_MAX_EVENTS];
    int         num_event_ids;
} ClipTracker;

/* --- Transcode Job --- */

typedef struct {
    RingPacket *pkts;          /* heap array of deep-copied packets */
    int         num_pkts;
    uint8_t    *extradata;
    int         extradata_size;
    int         src_w, src_h;
    AVRational  time_base;
    int         camera_id;
    char        camera_name[64];
    time_t      trigger_time;  /* calendar time for filename */
    int         snapshot_idx;  /* pkt index nearest first event, -1 if none */
    int         event_ids[CLIP_MAX_EVENTS];
    int         num_event_ids;
    sqlite3    *db;
} ClipJob;

/* --- Job Queue --- */

#define CLIP_QUEUE_MAX  16

typedef struct {
    ClipJob        *jobs[CLIP_QUEUE_MAX];
    int             head, tail, count;
    pthread_mutex_t mutex;
    pthread_cond_t  cond;
    int             shutdown;
} ClipQueue;

/* --- Transcoder (singleton) --- */

typedef struct ClipTranscoder {
    pthread_t   thread;
    int         started;
    ClipQueue   queue;
} ClipTranscoder;

/* --- API: Ring buffer (camera worker thread) --- */
void ring_buf_init(RingBuf *rb);
void ring_buf_free(RingBuf *rb);
void ring_buf_set_stream_params(RingBuf *rb, const AVCodecParameters *par,
                                AVRational time_base);
void ring_buf_push(RingBuf *rb, const AVPacket *pkt, double wall_time);

/* --- API: Clip tracker (camera worker thread) --- */
void clip_tracker_init(ClipTracker *ct);
void clip_tracker_event(ClipTracker *ct, RingBuf *rb, int event_id,
                        double wall_time);
/* Returns 1 and sets *out_job if a clip job is ready to submit */
int  clip_tracker_tick(ClipTracker *ct, RingBuf *rb, double wall_time,
                       ClipJob **out_job, int camera_id,
                       const char *camera_name, sqlite3 *db);

/* --- API: Transcoder (global) --- */
void clip_transcoder_init(ClipTranscoder *tc);
void clip_transcoder_start(ClipTranscoder *tc);
void clip_transcoder_stop(ClipTranscoder *tc);
void clip_transcoder_submit(ClipTranscoder *tc, ClipJob *job);

#endif /* CLIP_H */
