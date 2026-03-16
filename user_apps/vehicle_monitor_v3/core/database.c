/*
 * core/database.c - SQLite schema and CRUD operations
 *
 * Thread-safe: WAL + FULLMUTEX mode.
 */
#include "database.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *SCHEMA_SQL =
    /* Settings key-value store */
    "CREATE TABLE IF NOT EXISTS settings ("
    "  key TEXT PRIMARY KEY, value TEXT);"

    /* Camera configuration */
    "CREATE TABLE IF NOT EXISTS cameras ("
    "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
    "  name TEXT DEFAULT 'Camera', url TEXT DEFAULT '',"
    "  enabled INTEGER DEFAULT 1, scale REAL DEFAULT 1.0,"
    "  rotation INTEGER DEFAULT 0,"
    "  grid_cols INTEGER DEFAULT 48, grid_rows INTEGER DEFAULT 27,"
    "  birdview_pos INTEGER DEFAULT 0, force_sw INTEGER DEFAULT 0,"
    "  threshold INTEGER DEFAULT 12, alpha REAL DEFAULT 0.05,"
    "  trigger_pct REAL DEFAULT 5.0, framestep INTEGER DEFAULT 5,"
    "  sort_order INTEGER DEFAULT 0);"

    /* Action groups for zone-based triggers */
    "CREATE TABLE IF NOT EXISTS action_groups ("
    "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
    "  name TEXT UNIQUE NOT NULL, color TEXT DEFAULT '#FF6600');"
    "INSERT OR IGNORE INTO action_groups (id, name, color)"
    "  VALUES (1, 'default', '#FF6600');"

    /* Zone cells assigned to action groups */
    "CREATE TABLE IF NOT EXISTS zones ("
    "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
    "  camera_id INTEGER, cell_col INTEGER, cell_row INTEGER,"
    "  action_group_id INTEGER DEFAULT 1,"
    "  UNIQUE(camera_id, cell_col, cell_row),"
    "  FOREIGN KEY (camera_id) REFERENCES cameras(id) ON DELETE CASCADE);"

    /* Motion events */
    "CREATE TABLE IF NOT EXISTS events ("
    "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
    "  timestamp TEXT DEFAULT (strftime('%Y-%m-%dT%H:%M:%f','now','localtime')),"
    "  camera_id INTEGER, action_group TEXT DEFAULT 'default',"
    "  motion_pct REAL, max_cell_pct REAL,"
    "  max_cell_col INTEGER, max_cell_row INTEGER);"
    "CREATE INDEX IF NOT EXISTS idx_events_time ON events(timestamp);"

    /* Sensor configuration */
    "CREATE TABLE IF NOT EXISTS sensors ("
    "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
    "  name TEXT DEFAULT 'Sensor', type TEXT DEFAULT 'RS485',"
    "  address TEXT DEFAULT '1', register_addr INTEGER DEFAULT 0,"
    "  scale REAL DEFAULT 1.0, offset REAL DEFAULT 0.0,"
    "  units TEXT DEFAULT '', poll_interval_ms INTEGER DEFAULT 1000,"
    "  enabled INTEGER DEFAULT 1);"

    /* Sensor readings log */
    "CREATE TABLE IF NOT EXISTS sensor_readings ("
    "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
    "  timestamp TEXT DEFAULT (strftime('%Y-%m-%dT%H:%M:%f','now','localtime')),"
    "  sensor_id INTEGER, raw_value REAL, scaled_value REAL);"
    "CREATE INDEX IF NOT EXISTS idx_readings_sensor"
    "  ON sensor_readings(sensor_id, timestamp);";

sqlite3 *db_open(const char *path)
{
    sqlite3 *db = NULL;
    int flags = SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE |
                SQLITE_OPEN_FULLMUTEX;

    int rc = sqlite3_open_v2(path, &db, flags, NULL);
    if (rc != SQLITE_OK) {
        fprintf(stderr, "DB open failed: %s\n", sqlite3_errmsg(db));
        if (db)
            sqlite3_close(db);
        return NULL;
    }

    /* WAL for concurrent read/write */
    char *err = NULL;
    sqlite3_exec(db, "PRAGMA journal_mode=WAL;", NULL, NULL, &err);
    if (err) {
        fprintf(stderr, "DB WAL warning: %s\n", err);
        sqlite3_free(err);
    }

    /* Foreign keys */
    sqlite3_exec(db, "PRAGMA foreign_keys=ON;", NULL, NULL, NULL);

    /* Create schema */
    err = NULL;
    rc = sqlite3_exec(db, SCHEMA_SQL, NULL, NULL, &err);
    if (rc != SQLITE_OK) {
        fprintf(stderr, "DB schema failed: %s\n", err);
        sqlite3_free(err);
        sqlite3_close(db);
        return NULL;
    }

    /* Migrations for new columns (silently ignore if already exist) */
    sqlite3_exec(db, "ALTER TABLE cameras ADD COLUMN aspect_w INTEGER DEFAULT 16;",
                 NULL, NULL, NULL);
    sqlite3_exec(db, "ALTER TABLE cameras ADD COLUMN aspect_h INTEGER DEFAULT 9;",
                 NULL, NULL, NULL);
    sqlite3_exec(db, "ALTER TABLE cameras ADD COLUMN recording_enabled INTEGER DEFAULT 1;",
                 NULL, NULL, NULL);
    sqlite3_exec(db, "ALTER TABLE cameras ADD COLUMN rec_segment_sec INTEGER DEFAULT 300;",
                 NULL, NULL, NULL);

    /* Action group actions column */
    sqlite3_exec(db, "ALTER TABLE action_groups ADD COLUMN actions INTEGER DEFAULT 0;",
                 NULL, NULL, NULL);

    /* Blob-based motion detection */
    sqlite3_exec(db, "ALTER TABLE cameras ADD COLUMN min_blob_cells INTEGER DEFAULT 3;",
                 NULL, NULL, NULL);

    /* Recordings table */
    sqlite3_exec(db,
        "CREATE TABLE IF NOT EXISTS recordings ("
        "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "  camera_id INTEGER NOT NULL,"
        "  filepath TEXT NOT NULL,"
        "  start_time TEXT NOT NULL,"
        "  end_time TEXT,"
        "  file_size INTEGER DEFAULT 0,"
        "  FOREIGN KEY (camera_id) REFERENCES cameras(id) ON DELETE CASCADE);"
        "CREATE INDEX IF NOT EXISTS idx_rec_cam_time"
        "  ON recordings(camera_id, start_time);",
        NULL, NULL, NULL);

    /* Recording width/height columns */
    sqlite3_exec(db, "ALTER TABLE recordings ADD COLUMN width INTEGER DEFAULT 0;",
                 NULL, NULL, NULL);
    sqlite3_exec(db, "ALTER TABLE recordings ADD COLUMN height INTEGER DEFAULT 0;",
                 NULL, NULL, NULL);

    /* Recording keyframe index */
    sqlite3_exec(db,
        "CREATE TABLE IF NOT EXISTS rec_index ("
        "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "  recording_id INTEGER NOT NULL,"
        "  file_offset INTEGER NOT NULL,"
        "  pts_us INTEGER NOT NULL,"
        "  FOREIGN KEY (recording_id) REFERENCES recordings(id) ON DELETE CASCADE);"
        "CREATE INDEX IF NOT EXISTS idx_rec_index_pts"
        "  ON rec_index(recording_id, pts_us);",
        NULL, NULL, NULL);

    /* Ground markers for birdview calibration */
    sqlite3_exec(db,
        "CREATE TABLE IF NOT EXISTS ground_markers ("
        "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "  world_x REAL NOT NULL,"
        "  world_y REAL NOT NULL);",
        NULL, NULL, NULL);

    /* Calibration and homography tables */
    sqlite3_exec(db,
        "CREATE TABLE IF NOT EXISTS calib_points ("
        "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "  camera_id INTEGER NOT NULL,"
        "  img_x REAL, img_y REAL,"
        "  world_x REAL, world_y REAL,"
        "  FOREIGN KEY (camera_id) REFERENCES cameras(id) ON DELETE CASCADE);",
        NULL, NULL, NULL);
    sqlite3_exec(db,
        "CREATE TABLE IF NOT EXISTS homographies ("
        "  camera_id INTEGER PRIMARY KEY,"
        "  h00 REAL, h01 REAL, h02 REAL,"
        "  h10 REAL, h11 REAL, h12 REAL,"
        "  h20 REAL, h21 REAL, h22 REAL,"
        "  FOREIGN KEY (camera_id) REFERENCES cameras(id) ON DELETE CASCADE);",
        NULL, NULL, NULL);

    /* Event clips */
    sqlite3_exec(db,
        "CREATE TABLE IF NOT EXISTS event_clips ("
        "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "  camera_id INTEGER NOT NULL,"
        "  clip_path TEXT NOT NULL,"
        "  thumb_path TEXT,"
        "  start_time TEXT NOT NULL,"
        "  end_time TEXT,"
        "  duration REAL DEFAULT 0,"
        "  file_size INTEGER DEFAULT 0,"
        "  width INTEGER DEFAULT 960,"
        "  height INTEGER DEFAULT 540,"
        "  FOREIGN KEY (camera_id) REFERENCES cameras(id) ON DELETE CASCADE);"
        "CREATE INDEX IF NOT EXISTS idx_clips_cam_time"
        "  ON event_clips(camera_id, start_time);",
        NULL, NULL, NULL);

    sqlite3_exec(db,
        "CREATE TABLE IF NOT EXISTS event_clip_events ("
        "  clip_id INTEGER NOT NULL,"
        "  event_id INTEGER NOT NULL,"
        "  PRIMARY KEY (clip_id, event_id),"
        "  FOREIGN KEY (clip_id) REFERENCES event_clips(id) ON DELETE CASCADE,"
        "  FOREIGN KEY (event_id) REFERENCES events(id) ON DELETE CASCADE);",
        NULL, NULL, NULL);

    /* Lens distortion calibration */
    sqlite3_exec(db,
        "CREATE TABLE IF NOT EXISTS lens_params ("
        "  camera_id INTEGER PRIMARY KEY,"
        "  k1 REAL, k2 REAL,"
        "  cx_frac REAL DEFAULT 0.5, cy_frac REAL DEFAULT 0.5,"
        "  FOREIGN KEY (camera_id) REFERENCES cameras(id) ON DELETE CASCADE);",
        NULL, NULL, NULL);
    sqlite3_exec(db,
        "CREATE TABLE IF NOT EXISTS lens_calib_lines ("
        "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "  camera_id INTEGER NOT NULL,"
        "  line_idx INTEGER NOT NULL,"
        "  px REAL, py REAL,"
        "  FOREIGN KEY (camera_id) REFERENCES cameras(id) ON DELETE CASCADE);",
        NULL, NULL, NULL);

    printf("Database opened: %s\n", path);
    return db;
}

/* ---- Settings ---- */

int db_setting_set(sqlite3 *db, const char *key, const char *value)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "INSERT OR REPLACE INTO settings (key, value) VALUES (?, ?);",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;

    sqlite3_bind_text(stmt, 1, key, -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, value, -1, SQLITE_TRANSIENT);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return (rc == SQLITE_DONE) ? 0 : -1;
}

char *db_setting_get(sqlite3 *db, const char *key)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "SELECT value FROM settings WHERE key = ?;",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return NULL;

    sqlite3_bind_text(stmt, 1, key, -1, SQLITE_TRANSIENT);
    char *result = NULL;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        const char *val = (const char *)sqlite3_column_text(stmt, 0);
        if (val)
            result = strdup(val);
    }
    sqlite3_finalize(stmt);
    return result;
}

int db_setting_get_int(sqlite3 *db, const char *key, int def)
{
    char *val = db_setting_get(db, key);
    if (!val) return def;
    int r = atoi(val);
    free(val);
    return r;
}

double db_setting_get_double(sqlite3 *db, const char *key, double def)
{
    char *val = db_setting_get(db, key);
    if (!val) return def;
    double r = atof(val);
    free(val);
    return r;
}

/* ---- Cameras ---- */

int db_camera_insert(sqlite3 *db, const DbCamera *cam)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "INSERT INTO cameras (name, url, enabled, scale, rotation,"
        " aspect_w, aspect_h,"
        " grid_cols, grid_rows, birdview_pos, force_sw,"
        " threshold, alpha, trigger_pct, framestep, min_blob_cells,"
        " sort_order, recording_enabled, rec_segment_sec)"
        " VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?);",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;

    sqlite3_bind_text(stmt, 1, cam->name, -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, cam->url, -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 3, cam->enabled);
    sqlite3_bind_double(stmt, 4, cam->scale);
    sqlite3_bind_int(stmt, 5, cam->rotation);
    sqlite3_bind_int(stmt, 6, cam->aspect_w);
    sqlite3_bind_int(stmt, 7, cam->aspect_h);
    sqlite3_bind_int(stmt, 8, cam->grid_cols);
    sqlite3_bind_int(stmt, 9, cam->grid_rows);
    sqlite3_bind_int(stmt, 10, cam->birdview_pos);
    sqlite3_bind_int(stmt, 11, cam->force_sw);
    sqlite3_bind_int(stmt, 12, cam->threshold);
    sqlite3_bind_double(stmt, 13, cam->alpha);
    sqlite3_bind_double(stmt, 14, cam->trigger_pct);
    sqlite3_bind_int(stmt, 15, cam->framestep);
    sqlite3_bind_int(stmt, 16, cam->min_blob_cells);
    sqlite3_bind_int(stmt, 17, cam->sort_order);
    sqlite3_bind_int(stmt, 18, cam->recording_enabled);
    sqlite3_bind_int(stmt, 19, cam->rec_segment_sec);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);

    if (rc != SQLITE_DONE) return -1;
    return (int)sqlite3_last_insert_rowid(db);
}

int db_camera_update(sqlite3 *db, const DbCamera *cam)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "UPDATE cameras SET name=?, url=?, enabled=?, scale=?, rotation=?,"
        " aspect_w=?, aspect_h=?,"
        " grid_cols=?, grid_rows=?, birdview_pos=?, force_sw=?,"
        " threshold=?, alpha=?, trigger_pct=?, framestep=?, min_blob_cells=?,"
        " sort_order=?, recording_enabled=?, rec_segment_sec=?"
        " WHERE id=?;",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;

    sqlite3_bind_text(stmt, 1, cam->name, -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, cam->url, -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 3, cam->enabled);
    sqlite3_bind_double(stmt, 4, cam->scale);
    sqlite3_bind_int(stmt, 5, cam->rotation);
    sqlite3_bind_int(stmt, 6, cam->aspect_w);
    sqlite3_bind_int(stmt, 7, cam->aspect_h);
    sqlite3_bind_int(stmt, 8, cam->grid_cols);
    sqlite3_bind_int(stmt, 9, cam->grid_rows);
    sqlite3_bind_int(stmt, 10, cam->birdview_pos);
    sqlite3_bind_int(stmt, 11, cam->force_sw);
    sqlite3_bind_int(stmt, 12, cam->threshold);
    sqlite3_bind_double(stmt, 13, cam->alpha);
    sqlite3_bind_double(stmt, 14, cam->trigger_pct);
    sqlite3_bind_int(stmt, 15, cam->framestep);
    sqlite3_bind_int(stmt, 16, cam->min_blob_cells);
    sqlite3_bind_int(stmt, 17, cam->sort_order);
    sqlite3_bind_int(stmt, 18, cam->recording_enabled);
    sqlite3_bind_int(stmt, 19, cam->rec_segment_sec);
    sqlite3_bind_int(stmt, 20, cam->id);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return (rc == SQLITE_DONE) ? 0 : -1;
}

int db_camera_delete(sqlite3 *db, int id)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "DELETE FROM cameras WHERE id=?;", -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_int(stmt, 1, id);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return (rc == SQLITE_DONE) ? 0 : -1;
}

int db_camera_load_all(sqlite3 *db, DbCamera *out, int max)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "SELECT id, name, url, enabled, scale, rotation,"
        " aspect_w, aspect_h,"
        " grid_cols, grid_rows, birdview_pos, force_sw,"
        " threshold, alpha, trigger_pct, framestep, min_blob_cells,"
        " sort_order, recording_enabled, rec_segment_sec"
        " FROM cameras ORDER BY sort_order, id;",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return 0;

    int count = 0;
    while (sqlite3_step(stmt) == SQLITE_ROW && count < max) {
        DbCamera *c = &out[count];
        c->id = sqlite3_column_int(stmt, 0);
        const char *s;
        s = (const char *)sqlite3_column_text(stmt, 1);
        snprintf(c->name, sizeof(c->name), "%s", s ? s : "Camera");
        s = (const char *)sqlite3_column_text(stmt, 2);
        snprintf(c->url, sizeof(c->url), "%s", s ? s : "");
        c->enabled     = sqlite3_column_int(stmt, 3);
        c->scale       = sqlite3_column_double(stmt, 4);
        c->rotation    = sqlite3_column_int(stmt, 5);
        c->aspect_w    = sqlite3_column_int(stmt, 6);
        c->aspect_h    = sqlite3_column_int(stmt, 7);
        c->grid_cols   = sqlite3_column_int(stmt, 8);
        c->grid_rows   = sqlite3_column_int(stmt, 9);
        c->birdview_pos = sqlite3_column_int(stmt, 10);
        c->force_sw    = sqlite3_column_int(stmt, 11);
        c->threshold   = sqlite3_column_int(stmt, 12);
        c->alpha       = sqlite3_column_double(stmt, 13);
        c->trigger_pct = sqlite3_column_double(stmt, 14);
        c->framestep   = sqlite3_column_int(stmt, 15);
        c->min_blob_cells = sqlite3_column_int(stmt, 16);
        if (c->min_blob_cells < 1) c->min_blob_cells = 3;
        c->sort_order  = sqlite3_column_int(stmt, 17);
        c->recording_enabled = sqlite3_column_int(stmt, 18);
        c->rec_segment_sec   = sqlite3_column_int(stmt, 19);
        if (c->rec_segment_sec <= 0) c->rec_segment_sec = 300;
        count++;
    }
    sqlite3_finalize(stmt);
    return count;
}

/* ---- Action groups ---- */

int db_action_group_insert(sqlite3 *db, const char *name, const char *color,
                           int actions)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "INSERT INTO action_groups (name, color, actions) VALUES (?, ?, ?);",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_text(stmt, 1, name, -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, color, -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 3, actions);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    if (rc != SQLITE_DONE) return -1;
    return (int)sqlite3_last_insert_rowid(db);
}

int db_action_group_update(sqlite3 *db, int id, const char *name,
                           const char *color, int actions)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "UPDATE action_groups SET name=?, color=?, actions=? WHERE id=?;",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_text(stmt, 1, name, -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, color, -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 3, actions);
    sqlite3_bind_int(stmt, 4, id);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return (rc == SQLITE_DONE) ? 0 : -1;
}

int db_action_group_delete(sqlite3 *db, int id)
{
    if (id == 1) return -1; /* protect default group */
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "DELETE FROM action_groups WHERE id=?;", -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_int(stmt, 1, id);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return (rc == SQLITE_DONE) ? 0 : -1;
}

int db_action_group_load_all(sqlite3 *db, DbActionGroup *out, int max)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "SELECT id, name, color, actions FROM action_groups ORDER BY id;",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return 0;

    int count = 0;
    while (sqlite3_step(stmt) == SQLITE_ROW && count < max) {
        out[count].id = sqlite3_column_int(stmt, 0);
        const char *s;
        s = (const char *)sqlite3_column_text(stmt, 1);
        snprintf(out[count].name, sizeof(out[count].name), "%s", s ? s : "");
        s = (const char *)sqlite3_column_text(stmt, 2);
        snprintf(out[count].color, sizeof(out[count].color), "%s",
                 s ? s : "#FF6600");
        out[count].actions = sqlite3_column_int(stmt, 3);
        count++;
    }
    sqlite3_finalize(stmt);
    return count;
}

/* ---- Zones ---- */

int db_zone_set(sqlite3 *db, int camera_id, int col, int row, int group_id)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "INSERT OR REPLACE INTO zones (camera_id, cell_col, cell_row,"
        " action_group_id) VALUES (?, ?, ?, ?);",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_int(stmt, 1, camera_id);
    sqlite3_bind_int(stmt, 2, col);
    sqlite3_bind_int(stmt, 3, row);
    sqlite3_bind_int(stmt, 4, group_id);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return (rc == SQLITE_DONE) ? 0 : -1;
}

int db_zone_delete(sqlite3 *db, int camera_id, int col, int row)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "DELETE FROM zones WHERE camera_id=? AND cell_col=? AND cell_row=?;",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_int(stmt, 1, camera_id);
    sqlite3_bind_int(stmt, 2, col);
    sqlite3_bind_int(stmt, 3, row);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return (rc == SQLITE_DONE) ? 0 : -1;
}

int db_zone_clear_camera(sqlite3 *db, int camera_id)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "DELETE FROM zones WHERE camera_id=?;", -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_int(stmt, 1, camera_id);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return (rc == SQLITE_DONE) ? 0 : -1;
}

int db_zone_load_camera(sqlite3 *db, int camera_id, DbZone *out, int max)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "SELECT id, camera_id, cell_col, cell_row, action_group_id"
        " FROM zones WHERE camera_id=? ORDER BY cell_row, cell_col;",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return 0;
    sqlite3_bind_int(stmt, 1, camera_id);

    int count = 0;
    while (sqlite3_step(stmt) == SQLITE_ROW && count < max) {
        out[count].id              = sqlite3_column_int(stmt, 0);
        out[count].camera_id       = sqlite3_column_int(stmt, 1);
        out[count].cell_col        = sqlite3_column_int(stmt, 2);
        out[count].cell_row        = sqlite3_column_int(stmt, 3);
        out[count].action_group_id = sqlite3_column_int(stmt, 4);
        count++;
    }
    sqlite3_finalize(stmt);
    return count;
}

/* ---- Events ---- */

int db_event_insert(sqlite3 *db, int camera_id, const char *action_group,
                    double motion_pct, double max_cell_pct,
                    int max_col, int max_row)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "INSERT INTO events (camera_id, action_group, motion_pct,"
        " max_cell_pct, max_cell_col, max_cell_row)"
        " VALUES (?, ?, ?, ?, ?, ?);",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_int(stmt, 1, camera_id);
    sqlite3_bind_text(stmt, 2, action_group, -1, SQLITE_TRANSIENT);
    sqlite3_bind_double(stmt, 3, motion_pct);
    sqlite3_bind_double(stmt, 4, max_cell_pct);
    sqlite3_bind_int(stmt, 5, max_col);
    sqlite3_bind_int(stmt, 6, max_row);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    if (rc != SQLITE_DONE) return -1;
    return (int)sqlite3_last_insert_rowid(db);
}

int db_event_load_range(sqlite3 *db, const char *start, const char *end,
                        DbEvent *out, int max)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "SELECT id, timestamp, camera_id, action_group, motion_pct,"
        " max_cell_pct, max_cell_col, max_cell_row"
        " FROM events WHERE timestamp BETWEEN ? AND ?"
        " ORDER BY timestamp DESC LIMIT ?;",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return 0;
    sqlite3_bind_text(stmt, 1, start, -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, end, -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 3, max);

    int count = 0;
    while (sqlite3_step(stmt) == SQLITE_ROW && count < max) {
        DbEvent *e = &out[count];
        e->id = sqlite3_column_int(stmt, 0);
        const char *s;
        s = (const char *)sqlite3_column_text(stmt, 1);
        snprintf(e->timestamp, sizeof(e->timestamp), "%s", s ? s : "");
        e->camera_id = sqlite3_column_int(stmt, 2);
        s = (const char *)sqlite3_column_text(stmt, 3);
        snprintf(e->action_group, sizeof(e->action_group), "%s",
                 s ? s : "default");
        e->motion_pct    = sqlite3_column_double(stmt, 4);
        e->max_cell_pct  = sqlite3_column_double(stmt, 5);
        e->max_cell_col  = sqlite3_column_int(stmt, 6);
        e->max_cell_row  = sqlite3_column_int(stmt, 7);
        count++;
    }
    sqlite3_finalize(stmt);
    return count;
}

int db_event_count(sqlite3 *db)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "SELECT COUNT(*) FROM events;", -1, &stmt, NULL);
    if (rc != SQLITE_OK) return 0;
    int count = 0;
    if (sqlite3_step(stmt) == SQLITE_ROW)
        count = sqlite3_column_int(stmt, 0);
    sqlite3_finalize(stmt);
    return count;
}

/* ---- Sensors ---- */

int db_sensor_insert(sqlite3 *db, const DbSensor *s)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "INSERT INTO sensors (name, type, address, register_addr,"
        " scale, offset, units, poll_interval_ms, enabled)"
        " VALUES (?,?,?,?,?,?,?,?,?);",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_text(stmt, 1, s->name, -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, s->type, -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 3, s->address, -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 4, s->register_addr);
    sqlite3_bind_double(stmt, 5, s->scale);
    sqlite3_bind_double(stmt, 6, s->offset);
    sqlite3_bind_text(stmt, 7, s->units, -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 8, s->poll_interval_ms);
    sqlite3_bind_int(stmt, 9, s->enabled);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    if (rc != SQLITE_DONE) return -1;
    return (int)sqlite3_last_insert_rowid(db);
}

int db_sensor_update(sqlite3 *db, const DbSensor *s)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "UPDATE sensors SET name=?, type=?, address=?, register_addr=?,"
        " scale=?, offset=?, units=?, poll_interval_ms=?, enabled=?"
        " WHERE id=?;",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_text(stmt, 1, s->name, -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, s->type, -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 3, s->address, -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 4, s->register_addr);
    sqlite3_bind_double(stmt, 5, s->scale);
    sqlite3_bind_double(stmt, 6, s->offset);
    sqlite3_bind_text(stmt, 7, s->units, -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 8, s->poll_interval_ms);
    sqlite3_bind_int(stmt, 9, s->enabled);
    sqlite3_bind_int(stmt, 10, s->id);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return (rc == SQLITE_DONE) ? 0 : -1;
}

int db_sensor_delete(sqlite3 *db, int id)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "DELETE FROM sensors WHERE id=?;", -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_int(stmt, 1, id);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return (rc == SQLITE_DONE) ? 0 : -1;
}

int db_sensor_load_all(sqlite3 *db, DbSensor *out, int max)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "SELECT id, name, type, address, register_addr,"
        " scale, offset, units, poll_interval_ms, enabled"
        " FROM sensors ORDER BY id;",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return 0;

    int count = 0;
    while (sqlite3_step(stmt) == SQLITE_ROW && count < max) {
        DbSensor *s = &out[count];
        s->id = sqlite3_column_int(stmt, 0);
        const char *t;
        t = (const char *)sqlite3_column_text(stmt, 1);
        snprintf(s->name, sizeof(s->name), "%s", t ? t : "Sensor");
        t = (const char *)sqlite3_column_text(stmt, 2);
        snprintf(s->type, sizeof(s->type), "%s", t ? t : "RS485");
        t = (const char *)sqlite3_column_text(stmt, 3);
        snprintf(s->address, sizeof(s->address), "%s", t ? t : "1");
        s->register_addr   = sqlite3_column_int(stmt, 4);
        s->scale           = sqlite3_column_double(stmt, 5);
        s->offset          = sqlite3_column_double(stmt, 6);
        t = (const char *)sqlite3_column_text(stmt, 7);
        snprintf(s->units, sizeof(s->units), "%s", t ? t : "");
        s->poll_interval_ms = sqlite3_column_int(stmt, 8);
        s->enabled          = sqlite3_column_int(stmt, 9);
        count++;
    }
    sqlite3_finalize(stmt);
    return count;
}

/* ---- Sensor readings ---- */

int db_reading_insert(sqlite3 *db, int sensor_id, double raw, double scaled)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "INSERT INTO sensor_readings (sensor_id, raw_value, scaled_value)"
        " VALUES (?, ?, ?);",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_int(stmt, 1, sensor_id);
    sqlite3_bind_double(stmt, 2, raw);
    sqlite3_bind_double(stmt, 3, scaled);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return (rc == SQLITE_DONE) ? 0 : -1;
}

int db_reading_load_recent(sqlite3 *db, int sensor_id, int limit,
                           double *values, char timestamps[][32])
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "SELECT scaled_value, timestamp FROM sensor_readings"
        " WHERE sensor_id=? ORDER BY id DESC LIMIT ?;",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return 0;
    sqlite3_bind_int(stmt, 1, sensor_id);
    sqlite3_bind_int(stmt, 2, limit);

    int count = 0;
    while (sqlite3_step(stmt) == SQLITE_ROW && count < limit) {
        values[count] = sqlite3_column_double(stmt, 0);
        if (timestamps) {
            const char *t = (const char *)sqlite3_column_text(stmt, 1);
            snprintf(timestamps[count], 32, "%s", t ? t : "");
        }
        count++;
    }
    sqlite3_finalize(stmt);
    return count;
}

/* ---- Recordings ---- */

int db_recording_insert(sqlite3 *db, int camera_id, const char *filepath,
                        const char *start_time, int width, int height)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "INSERT INTO recordings (camera_id, filepath, start_time, width, height)"
        " VALUES (?, ?, ?, ?, ?);",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_int(stmt, 1, camera_id);
    sqlite3_bind_text(stmt, 2, filepath, -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 3, start_time, -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 4, width);
    sqlite3_bind_int(stmt, 5, height);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    if (rc != SQLITE_DONE) return -1;
    return (int)sqlite3_last_insert_rowid(db);
}

int db_recording_finish(sqlite3 *db, int rec_id, const char *end_time,
                        int file_size)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "UPDATE recordings SET end_time=?, file_size=? WHERE id=?;",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_text(stmt, 1, end_time, -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 2, file_size);
    sqlite3_bind_int(stmt, 3, rec_id);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return (rc == SQLITE_DONE) ? 0 : -1;
}

int db_recording_load_range(sqlite3 *db, int camera_id,
                            const char *start, const char *end,
                            DbRecording *out, int max)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "SELECT id, camera_id, filepath, start_time, end_time, file_size,"
        " width, height"
        " FROM recordings"
        " WHERE camera_id=? AND start_time <= ? AND"
        " (end_time >= ? OR end_time IS NULL)"
        " ORDER BY start_time;",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return 0;
    sqlite3_bind_int(stmt, 1, camera_id);
    sqlite3_bind_text(stmt, 2, end, -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 3, start, -1, SQLITE_TRANSIENT);

    int count = 0;
    while (sqlite3_step(stmt) == SQLITE_ROW && count < max) {
        DbRecording *r = &out[count];
        r->id = sqlite3_column_int(stmt, 0);
        r->camera_id = sqlite3_column_int(stmt, 1);
        const char *s;
        s = (const char *)sqlite3_column_text(stmt, 2);
        snprintf(r->filepath, sizeof(r->filepath), "%s", s ? s : "");
        s = (const char *)sqlite3_column_text(stmt, 3);
        snprintf(r->start_time, sizeof(r->start_time), "%s", s ? s : "");
        s = (const char *)sqlite3_column_text(stmt, 4);
        snprintf(r->end_time, sizeof(r->end_time), "%s", s ? s : "");
        r->file_size = sqlite3_column_int(stmt, 5);
        r->width = sqlite3_column_int(stmt, 6);
        r->height = sqlite3_column_int(stmt, 7);
        count++;
    }
    sqlite3_finalize(stmt);
    return count;
}

int db_recording_find_at_time(sqlite3 *db, int camera_id,
                              const char *timestamp, DbRecording *out)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "SELECT id, camera_id, filepath, start_time, end_time, file_size,"
        " width, height"
        " FROM recordings"
        " WHERE camera_id=? AND start_time <= ? AND"
        " (end_time >= ? OR end_time IS NULL)"
        " ORDER BY start_time DESC LIMIT 1;",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return 0;
    sqlite3_bind_int(stmt, 1, camera_id);
    sqlite3_bind_text(stmt, 2, timestamp, -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 3, timestamp, -1, SQLITE_TRANSIENT);

    int found = 0;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        out->id = sqlite3_column_int(stmt, 0);
        out->camera_id = sqlite3_column_int(stmt, 1);
        const char *s;
        s = (const char *)sqlite3_column_text(stmt, 2);
        snprintf(out->filepath, sizeof(out->filepath), "%s", s ? s : "");
        s = (const char *)sqlite3_column_text(stmt, 3);
        snprintf(out->start_time, sizeof(out->start_time), "%s", s ? s : "");
        s = (const char *)sqlite3_column_text(stmt, 4);
        snprintf(out->end_time, sizeof(out->end_time), "%s", s ? s : "");
        out->file_size = sqlite3_column_int(stmt, 5);
        out->width = sqlite3_column_int(stmt, 6);
        out->height = sqlite3_column_int(stmt, 7);
        found = 1;
    }
    sqlite3_finalize(stmt);
    return found;
}

int db_recording_delete(sqlite3 *db, int id)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "DELETE FROM recordings WHERE id=?;", -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_int(stmt, 1, id);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return (rc == SQLITE_DONE) ? 0 : -1;
}

/* ---- Ground markers ---- */

int db_ground_marker_insert(sqlite3 *db, double world_x, double world_y)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "INSERT INTO ground_markers (world_x, world_y) VALUES (?, ?);",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_double(stmt, 1, world_x);
    sqlite3_bind_double(stmt, 2, world_y);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    if (rc != SQLITE_DONE) return -1;
    return (int)sqlite3_last_insert_rowid(db);
}

int db_ground_marker_delete(sqlite3 *db, int id)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "DELETE FROM ground_markers WHERE id=?;", -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_int(stmt, 1, id);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return (rc == SQLITE_DONE) ? 0 : -1;
}

int db_ground_marker_clear_all(sqlite3 *db)
{
    char *err = NULL;
    int rc = sqlite3_exec(db, "DELETE FROM ground_markers;", NULL, NULL, &err);
    if (err) sqlite3_free(err);
    return (rc == SQLITE_OK) ? 0 : -1;
}

int db_ground_marker_load_all(sqlite3 *db, DbGroundMarker *out, int max)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "SELECT id, world_x, world_y FROM ground_markers ORDER BY id;",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return 0;

    int count = 0;
    while (sqlite3_step(stmt) == SQLITE_ROW && count < max) {
        out[count].id      = sqlite3_column_int(stmt, 0);
        out[count].world_x = sqlite3_column_double(stmt, 1);
        out[count].world_y = sqlite3_column_double(stmt, 2);
        count++;
    }
    sqlite3_finalize(stmt);
    return count;
}

/* ---- Calibration points ---- */

int db_calib_insert(sqlite3 *db, const DbCalibPoint *pt)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "INSERT INTO calib_points (camera_id, img_x, img_y, world_x, world_y)"
        " VALUES (?,?,?,?,?);",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_int(stmt, 1, pt->camera_id);
    sqlite3_bind_double(stmt, 2, pt->img_x);
    sqlite3_bind_double(stmt, 3, pt->img_y);
    sqlite3_bind_double(stmt, 4, pt->world_x);
    sqlite3_bind_double(stmt, 5, pt->world_y);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    if (rc != SQLITE_DONE) return -1;
    return (int)sqlite3_last_insert_rowid(db);
}

int db_calib_delete(sqlite3 *db, int id)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "DELETE FROM calib_points WHERE id=?;", -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_int(stmt, 1, id);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return (rc == SQLITE_DONE) ? 0 : -1;
}

int db_calib_clear_camera(sqlite3 *db, int camera_id)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "DELETE FROM calib_points WHERE camera_id=?;", -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_int(stmt, 1, camera_id);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return (rc == SQLITE_DONE) ? 0 : -1;
}

int db_calib_load_camera(sqlite3 *db, int camera_id,
                         DbCalibPoint *out, int max)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "SELECT id, camera_id, img_x, img_y, world_x, world_y"
        " FROM calib_points WHERE camera_id=? ORDER BY id;",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return 0;
    sqlite3_bind_int(stmt, 1, camera_id);

    int count = 0;
    while (sqlite3_step(stmt) == SQLITE_ROW && count < max) {
        out[count].id        = sqlite3_column_int(stmt, 0);
        out[count].camera_id = sqlite3_column_int(stmt, 1);
        out[count].img_x     = sqlite3_column_double(stmt, 2);
        out[count].img_y     = sqlite3_column_double(stmt, 3);
        out[count].world_x   = sqlite3_column_double(stmt, 4);
        out[count].world_y   = sqlite3_column_double(stmt, 5);
        count++;
    }
    sqlite3_finalize(stmt);
    return count;
}

/* ---- Homographies ---- */

int db_homography_save(sqlite3 *db, int camera_id, const double h[9])
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "INSERT OR REPLACE INTO homographies"
        " (camera_id, h00, h01, h02, h10, h11, h12, h20, h21, h22)"
        " VALUES (?,?,?,?,?,?,?,?,?,?);",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_int(stmt, 1, camera_id);
    for (int i = 0; i < 9; i++)
        sqlite3_bind_double(stmt, 2 + i, h[i]);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return (rc == SQLITE_DONE) ? 0 : -1;
}

int db_homography_load(sqlite3 *db, int camera_id, double h[9])
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "SELECT h00, h01, h02, h10, h11, h12, h20, h21, h22"
        " FROM homographies WHERE camera_id=?;",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_int(stmt, 1, camera_id);
    if (sqlite3_step(stmt) != SQLITE_ROW) {
        sqlite3_finalize(stmt);
        return -1;
    }
    for (int i = 0; i < 9; i++)
        h[i] = sqlite3_column_double(stmt, i);
    sqlite3_finalize(stmt);
    return 0;
}

int db_homography_delete(sqlite3 *db, int camera_id)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "DELETE FROM homographies WHERE camera_id=?;", -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_int(stmt, 1, camera_id);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return (rc == SQLITE_DONE) ? 0 : -1;
}

/* ---- Recording keyframe index ---- */

int db_rec_index_insert(sqlite3 *db, int recording_id,
                        int64_t file_offset, int64_t pts_us)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "INSERT INTO rec_index (recording_id, file_offset, pts_us)"
        " VALUES (?, ?, ?);",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_int(stmt, 1, recording_id);
    sqlite3_bind_int64(stmt, 2, file_offset);
    sqlite3_bind_int64(stmt, 3, pts_us);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return (rc == SQLITE_DONE) ? 0 : -1;
}

int db_rec_index_find_keyframe(sqlite3 *db, int recording_id,
                               int64_t seek_pts_us,
                               int64_t *out_offset, int64_t *out_pts_us)
{
    sqlite3_stmt *stmt = NULL;
    /* Find nearest keyframe at or before seek point */
    int rc = sqlite3_prepare_v2(db,
        "SELECT file_offset, pts_us FROM rec_index"
        " WHERE recording_id=? AND pts_us <= ?"
        " ORDER BY pts_us DESC LIMIT 1;",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_int(stmt, 1, recording_id);
    sqlite3_bind_int64(stmt, 2, seek_pts_us);

    if (sqlite3_step(stmt) == SQLITE_ROW) {
        *out_offset = sqlite3_column_int64(stmt, 0);
        *out_pts_us = sqlite3_column_int64(stmt, 1);
        sqlite3_finalize(stmt);
        return 0;
    }
    sqlite3_finalize(stmt);

    /* No keyframe before target: try first keyframe in segment */
    rc = sqlite3_prepare_v2(db,
        "SELECT file_offset, pts_us FROM rec_index"
        " WHERE recording_id=?"
        " ORDER BY pts_us ASC LIMIT 1;",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_int(stmt, 1, recording_id);

    if (sqlite3_step(stmt) == SQLITE_ROW) {
        *out_offset = sqlite3_column_int64(stmt, 0);
        *out_pts_us = sqlite3_column_int64(stmt, 1);
        sqlite3_finalize(stmt);
        return 0;
    }
    sqlite3_finalize(stmt);
    return -1;
}

/* ---- Event Clips ---- */

int db_clip_insert(sqlite3 *db, int camera_id,
                   const char *clip_path, const char *thumb_path,
                   const char *start_time, const char *end_time,
                   double duration, int file_size, int width, int height,
                   const int *event_ids, int num_events)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "INSERT INTO event_clips (camera_id, clip_path, thumb_path,"
        " start_time, end_time, duration, file_size, width, height)"
        " VALUES (?,?,?,?,?,?,?,?,?);",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;

    sqlite3_bind_int(stmt, 1, camera_id);
    sqlite3_bind_text(stmt, 2, clip_path, -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 3, thumb_path ? thumb_path : "", -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 4, start_time, -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 5, end_time, -1, SQLITE_TRANSIENT);
    sqlite3_bind_double(stmt, 6, duration);
    sqlite3_bind_int(stmt, 7, file_size);
    sqlite3_bind_int(stmt, 8, width);
    sqlite3_bind_int(stmt, 9, height);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    if (rc != SQLITE_DONE) return -1;

    int clip_id = (int)sqlite3_last_insert_rowid(db);

    /* Link events to clip */
    for (int i = 0; i < num_events; i++) {
        rc = sqlite3_prepare_v2(db,
            "INSERT OR IGNORE INTO event_clip_events (clip_id, event_id)"
            " VALUES (?, ?);",
            -1, &stmt, NULL);
        if (rc == SQLITE_OK) {
            sqlite3_bind_int(stmt, 1, clip_id);
            sqlite3_bind_int(stmt, 2, event_ids[i]);
            sqlite3_step(stmt);
            sqlite3_finalize(stmt);
        }
    }

    return clip_id;
}

int db_clip_load_range(sqlite3 *db, int camera_id,
                       const char *start, const char *end,
                       DbEventClip *out, int max)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "SELECT id, camera_id, clip_path, thumb_path, start_time, end_time,"
        " duration, file_size, width, height"
        " FROM event_clips WHERE camera_id=?"
        " AND start_time >= ? AND start_time <= ?"
        " ORDER BY start_time;",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return 0;
    sqlite3_bind_int(stmt, 1, camera_id);
    sqlite3_bind_text(stmt, 2, start, -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 3, end, -1, SQLITE_TRANSIENT);

    int count = 0;
    while (sqlite3_step(stmt) == SQLITE_ROW && count < max) {
        DbEventClip *c = &out[count];
        c->id = sqlite3_column_int(stmt, 0);
        c->camera_id = sqlite3_column_int(stmt, 1);
        const char *s;
        s = (const char *)sqlite3_column_text(stmt, 2);
        snprintf(c->clip_path, sizeof(c->clip_path), "%s", s ? s : "");
        s = (const char *)sqlite3_column_text(stmt, 3);
        snprintf(c->thumb_path, sizeof(c->thumb_path), "%s", s ? s : "");
        s = (const char *)sqlite3_column_text(stmt, 4);
        snprintf(c->start_time, sizeof(c->start_time), "%s", s ? s : "");
        s = (const char *)sqlite3_column_text(stmt, 5);
        snprintf(c->end_time, sizeof(c->end_time), "%s", s ? s : "");
        c->duration = sqlite3_column_double(stmt, 6);
        c->file_size = sqlite3_column_int(stmt, 7);
        c->width = sqlite3_column_int(stmt, 8);
        c->height = sqlite3_column_int(stmt, 9);
        count++;
    }
    sqlite3_finalize(stmt);
    return count;
}

/* ---- Lens distortion calibration ---- */

int db_lens_line_insert(sqlite3 *db, int camera_id, int line_idx,
                        double px, double py)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "INSERT INTO lens_calib_lines (camera_id, line_idx, px, py)"
        " VALUES (?, ?, ?, ?);",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_int(stmt, 1, camera_id);
    sqlite3_bind_int(stmt, 2, line_idx);
    sqlite3_bind_double(stmt, 3, px);
    sqlite3_bind_double(stmt, 4, py);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    if (rc != SQLITE_DONE) return -1;
    return (int)sqlite3_last_insert_rowid(db);
}

int db_lens_line_clear_camera(sqlite3 *db, int camera_id)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "DELETE FROM lens_calib_lines WHERE camera_id=?;",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_int(stmt, 1, camera_id);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return (rc == SQLITE_DONE) ? 0 : -1;
}

int db_lens_line_delete_line(sqlite3 *db, int camera_id, int line_idx)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "DELETE FROM lens_calib_lines WHERE camera_id=? AND line_idx=?;",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_int(stmt, 1, camera_id);
    sqlite3_bind_int(stmt, 2, line_idx);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return (rc == SQLITE_DONE) ? 0 : -1;
}

int db_lens_line_load_camera(sqlite3 *db, int camera_id,
                             DbLensLinePoint *out, int max)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "SELECT id, camera_id, line_idx, px, py"
        " FROM lens_calib_lines WHERE camera_id=?"
        " ORDER BY line_idx, id;",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return 0;
    sqlite3_bind_int(stmt, 1, camera_id);

    int count = 0;
    while (sqlite3_step(stmt) == SQLITE_ROW && count < max) {
        out[count].id         = sqlite3_column_int(stmt, 0);
        out[count].camera_id  = sqlite3_column_int(stmt, 1);
        out[count].line_idx   = sqlite3_column_int(stmt, 2);
        out[count].px         = sqlite3_column_double(stmt, 3);
        out[count].py         = sqlite3_column_double(stmt, 4);
        count++;
    }
    sqlite3_finalize(stmt);
    return count;
}

int db_lens_params_save(sqlite3 *db, int camera_id,
                        double k1, double k2, double cx, double cy)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "INSERT OR REPLACE INTO lens_params"
        " (camera_id, k1, k2, cx_frac, cy_frac)"
        " VALUES (?, ?, ?, ?, ?);",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_int(stmt, 1, camera_id);
    sqlite3_bind_double(stmt, 2, k1);
    sqlite3_bind_double(stmt, 3, k2);
    sqlite3_bind_double(stmt, 4, cx);
    sqlite3_bind_double(stmt, 5, cy);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return (rc == SQLITE_DONE) ? 0 : -1;
}

int db_lens_params_load(sqlite3 *db, int camera_id, DbLensParams *out)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "SELECT camera_id, k1, k2, cx_frac, cy_frac"
        " FROM lens_params WHERE camera_id=?;",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_int(stmt, 1, camera_id);

    if (sqlite3_step(stmt) != SQLITE_ROW) {
        sqlite3_finalize(stmt);
        return -1;
    }
    out->camera_id = sqlite3_column_int(stmt, 0);
    out->k1        = sqlite3_column_double(stmt, 1);
    out->k2        = sqlite3_column_double(stmt, 2);
    out->cx_frac   = sqlite3_column_double(stmt, 3);
    out->cy_frac   = sqlite3_column_double(stmt, 4);
    sqlite3_finalize(stmt);
    return 0;
}

int db_lens_params_delete(sqlite3 *db, int camera_id)
{
    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db,
        "DELETE FROM lens_params WHERE camera_id=?;",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) return -1;
    sqlite3_bind_int(stmt, 1, camera_id);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return (rc == SQLITE_DONE) ? 0 : -1;
}
