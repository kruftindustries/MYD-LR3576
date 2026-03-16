/*
 * core/camera.h - Camera decode threads and shared buffers
 */
#ifndef CAMERA_H
#define CAMERA_H

#include "../app.h"

/* Initialize FFmpeg for a camera (opens stream, allocates buffers) */
int camera_init(CameraCtx *cam);

/* Start the worker thread (decode + motion detection loop) */
int camera_start(CameraCtx *cam, AppCtx *app);

/* Stop the worker thread (sets running=0, joins thread) */
void camera_stop(CameraCtx *cam);

/* Free all FFmpeg and buffer resources */
void camera_free(CameraCtx *cam);

/* Find a camera by database ID, returns index or -1 */
int camera_find(AppCtx *app, int db_id);

/* Start all enabled cameras that aren't already running */
void camera_start_all(AppCtx *app);

/* Stop all cameras */
void camera_stop_all(AppCtx *app);

/* Recording control (safe to call from any thread) */
void camera_rec_start(CameraCtx *cam, AppCtx *app);
void camera_rec_stop(CameraCtx *cam, AppCtx *app);

#endif /* CAMERA_H */
