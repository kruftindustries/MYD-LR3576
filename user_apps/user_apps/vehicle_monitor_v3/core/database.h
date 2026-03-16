/*
 * core/database.h - SQLite schema and CRUD operations
 */
#ifndef DATABASE_H
#define DATABASE_H

#include <sqlite3.h>
#include <stdint.h>

/* Open database with WAL + FULLMUTEX, create all tables */
sqlite3 *db_open(const char *path);

/* --- Settings --- */
int  db_setting_set(sqlite3 *db, const char *key, const char *value);
char *db_setting_get(sqlite3 *db, const char *key); /* caller frees */
int  db_setting_get_int(sqlite3 *db, const char *key, int def);
double db_setting_get_double(sqlite3 *db, const char *key, double def);

/* --- Cameras --- */
typedef struct {
    int    id;
    char   name[64];
    char   url[512];
    int    enabled;
    double scale;
    int    rotation;
    int    aspect_w;
    int    aspect_h;
    int    grid_cols;
    int    grid_rows;
    int    birdview_pos;
    int    force_sw;
    int    threshold;
    double alpha;
    double trigger_pct;
    int    framestep;
    int    min_blob_cells;
    int    sort_order;
    int    recording_enabled;
    int    rec_segment_sec;
} DbCamera;

int db_camera_insert(sqlite3 *db, const DbCamera *cam); /* returns new id or -1 */
int db_camera_update(sqlite3 *db, const DbCamera *cam);
int db_camera_delete(sqlite3 *db, int id);
int db_camera_load_all(sqlite3 *db, DbCamera *out, int max); /* returns count */

/* --- Recordings --- */
typedef struct {
    int    id;
    int    camera_id;
    char   filepath[512];
    char   start_time[32];
    char   end_time[32];
    int    file_size;
    int    width;
    int    height;
} DbRecording;

int db_recording_insert(sqlite3 *db, int camera_id, const char *filepath,
                        const char *start_time, int width, int height);
int db_recording_finish(sqlite3 *db, int rec_id, const char *end_time,
                        int file_size);
int db_recording_load_range(sqlite3 *db, int camera_id,
                            const char *start, const char *end,
                            DbRecording *out, int max);
int db_recording_find_at_time(sqlite3 *db, int camera_id,
                              const char *timestamp, DbRecording *out);
int db_recording_delete(sqlite3 *db, int id);

/* --- Recording keyframe index --- */
int db_rec_index_insert(sqlite3 *db, int recording_id,
                        int64_t file_offset, int64_t pts_us);
int db_rec_index_find_keyframe(sqlite3 *db, int recording_id,
                               int64_t seek_pts_us,
                               int64_t *out_offset, int64_t *out_pts_us);

/* --- Action groups --- */
typedef struct {
    int  id;
    char name[64];
    char color[16];
    int  actions;       /* bitmask: ACTION_FLASH_LED | ACTION_EMAIL | ACTION_BEEP */
} DbActionGroup;

int db_action_group_insert(sqlite3 *db, const char *name, const char *color, int actions);
int db_action_group_update(sqlite3 *db, int id, const char *name, const char *color, int actions);
int db_action_group_delete(sqlite3 *db, int id);
int db_action_group_load_all(sqlite3 *db, DbActionGroup *out, int max);

/* --- Zones --- */
typedef struct {
    int id;
    int camera_id;
    int cell_col;
    int cell_row;
    int action_group_id;
} DbZone;

int db_zone_set(sqlite3 *db, int camera_id, int col, int row, int group_id);
int db_zone_delete(sqlite3 *db, int camera_id, int col, int row);
int db_zone_clear_camera(sqlite3 *db, int camera_id);
int db_zone_load_camera(sqlite3 *db, int camera_id, DbZone *out, int max);

/* --- Events --- */
typedef struct {
    int    id;
    char   timestamp[32];
    int    camera_id;
    char   action_group[64];
    double motion_pct;
    double max_cell_pct;
    int    max_cell_col;
    int    max_cell_row;
} DbEvent;

int db_event_insert(sqlite3 *db, int camera_id, const char *action_group,
                    double motion_pct, double max_cell_pct,
                    int max_col, int max_row);
int db_event_load_range(sqlite3 *db, const char *start, const char *end,
                        DbEvent *out, int max);
int db_event_count(sqlite3 *db);

/* --- Event clips --- */
typedef struct {
    int    id;
    int    camera_id;
    char   clip_path[512];
    char   thumb_path[512];
    char   start_time[32];
    char   end_time[32];
    double duration;
    int    file_size;
    int    width;
    int    height;
} DbEventClip;

int db_clip_insert(sqlite3 *db, int camera_id,
                   const char *clip_path, const char *thumb_path,
                   const char *start_time, const char *end_time,
                   double duration, int file_size, int width, int height,
                   const int *event_ids, int num_events);
int db_clip_load_range(sqlite3 *db, int camera_id,
                       const char *start, const char *end,
                       DbEventClip *out, int max);

/* --- Sensors --- */
typedef struct {
    int    id;
    char   name[64];
    char   type[16];
    char   address[32];
    int    register_addr;
    double scale;
    double offset;
    char   units[16];
    int    poll_interval_ms;
    int    enabled;
} DbSensor;

int db_sensor_insert(sqlite3 *db, const DbSensor *s);
int db_sensor_update(sqlite3 *db, const DbSensor *s);
int db_sensor_delete(sqlite3 *db, int id);
int db_sensor_load_all(sqlite3 *db, DbSensor *out, int max);

/* --- Ground markers (birdview calibration) --- */
typedef struct {
    int    id;
    double world_x, world_y;
} DbGroundMarker;

#define MAX_GROUND_MARKERS 32

int db_ground_marker_insert(sqlite3 *db, double world_x, double world_y);
int db_ground_marker_delete(sqlite3 *db, int id);
int db_ground_marker_clear_all(sqlite3 *db);
int db_ground_marker_load_all(sqlite3 *db, DbGroundMarker *out, int max);

/* --- Calibration points --- */
typedef struct {
    int    id;
    int    camera_id;
    double img_x, img_y;
    double world_x, world_y;
} DbCalibPoint;

int db_calib_insert(sqlite3 *db, const DbCalibPoint *pt);
int db_calib_delete(sqlite3 *db, int id);
int db_calib_clear_camera(sqlite3 *db, int camera_id);
int db_calib_load_camera(sqlite3 *db, int camera_id,
                         DbCalibPoint *out, int max);

/* --- Homographies --- */
int db_homography_save(sqlite3 *db, int camera_id, const double h[9]);
int db_homography_load(sqlite3 *db, int camera_id, double h[9]);
int db_homography_delete(sqlite3 *db, int camera_id);

/* --- Sensor readings --- */
int db_reading_insert(sqlite3 *db, int sensor_id, double raw, double scaled);
int db_reading_load_recent(sqlite3 *db, int sensor_id, int limit,
                           double *values, char timestamps[][32]);

/* --- Lens distortion calibration --- */
typedef struct {
    int    id;
    int    camera_id;
    int    line_idx;
    double px, py;  /* fraction 0.0-1.0 */
} DbLensLinePoint;

typedef struct {
    int    camera_id;
    double k1, k2;
    double cx_frac, cy_frac;
} DbLensParams;

int db_lens_line_insert(sqlite3 *db, int camera_id, int line_idx,
                        double px, double py);
int db_lens_line_clear_camera(sqlite3 *db, int camera_id);
int db_lens_line_delete_line(sqlite3 *db, int camera_id, int line_idx);
int db_lens_line_load_camera(sqlite3 *db, int camera_id,
                             DbLensLinePoint *out, int max);
int db_lens_params_save(sqlite3 *db, int camera_id,
                        double k1, double k2, double cx, double cy);
int db_lens_params_load(sqlite3 *db, int camera_id, DbLensParams *out);
int db_lens_params_delete(sqlite3 *db, int camera_id);

#endif /* DATABASE_H */
