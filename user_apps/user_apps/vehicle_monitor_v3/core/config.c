/*
 * core/config.c - Configuration load/save helpers
 */
#include "config.h"
#include "database.h"
#include <string.h>
#include <stdio.h>

static void db_cam_to_ctx(const DbCamera *db, CameraCtx *ctx)
{
    ctx->id          = db->id;
    snprintf(ctx->name, sizeof(ctx->name), "%s", db->name);
    snprintf(ctx->url, sizeof(ctx->url), "%s", db->url);
    ctx->enabled     = db->enabled;
    ctx->scale       = db->scale;
    ctx->rotation    = db->rotation;
    ctx->aspect_w    = db->aspect_w;
    ctx->aspect_h    = db->aspect_h;
    ctx->grid_cols   = db->grid_cols;
    ctx->grid_rows   = db->grid_rows;
    ctx->birdview_pos = db->birdview_pos;
    ctx->force_sw    = db->force_sw;
    ctx->threshold   = db->threshold;
    ctx->alpha       = (float)db->alpha;
    ctx->trigger_pct = (float)db->trigger_pct;
    ctx->framestep   = db->framestep;
    ctx->min_blob_cells = db->min_blob_cells;
    if (ctx->min_blob_cells < 1) ctx->min_blob_cells = 3;
    ctx->sort_order  = db->sort_order;
    ctx->recording_enabled = db->recording_enabled;
    ctx->rec_segment_sec   = db->rec_segment_sec;
    if (ctx->rec_segment_sec <= 0) ctx->rec_segment_sec = 300;
}

static void db_sensor_to_ctx(const DbSensor *db, SensorCtx *ctx)
{
    ctx->id               = db->id;
    snprintf(ctx->name, sizeof(ctx->name), "%s", db->name);
    snprintf(ctx->type, sizeof(ctx->type), "%s", db->type);
    snprintf(ctx->address, sizeof(ctx->address), "%s", db->address);
    ctx->register_addr    = db->register_addr;
    ctx->scale            = db->scale;
    ctx->offset           = db->offset;
    snprintf(ctx->units, sizeof(ctx->units), "%s", db->units);
    ctx->poll_interval_ms = db->poll_interval_ms;
    ctx->enabled          = db->enabled;
}

void config_load_all(AppCtx *app)
{
    /* Load cameras */
    DbCamera db_cams[MAX_CAMERAS];
    int n = db_camera_load_all(app->db, db_cams, MAX_CAMERAS);
    app->num_cameras = n;
    for (int i = 0; i < n; i++) {
        memset(&app->cameras[i], 0, sizeof(CameraCtx));
        pthread_mutex_init(&app->cameras[i].buf_mutex, NULL);
        db_cam_to_ctx(&db_cams[i], &app->cameras[i]);
    }

    /* Load sensors */
    DbSensor db_sensors[MAX_SENSORS];
    n = db_sensor_load_all(app->db, db_sensors, MAX_SENSORS);
    app->num_sensors = n;
    for (int i = 0; i < n; i++) {
        memset(&app->sensors[i], 0, sizeof(SensorCtx));
        db_sensor_to_ctx(&db_sensors[i], &app->sensors[i]);
    }

    printf("Config loaded: %d cameras, %d sensors\n",
           app->num_cameras, app->num_sensors);
}

void config_save_all(AppCtx *app)
{
    for (int i = 0; i < app->num_cameras; i++) {
        CameraCtx *c = &app->cameras[i];
        DbCamera db;
        db.id          = c->id;
        snprintf(db.name, sizeof(db.name), "%s", c->name);
        snprintf(db.url, sizeof(db.url), "%s", c->url);
        db.enabled     = c->enabled;
        db.scale       = c->scale;
        db.rotation    = c->rotation;
        db.aspect_w    = c->aspect_w;
        db.aspect_h    = c->aspect_h;
        db.grid_cols   = c->grid_cols;
        db.grid_rows   = c->grid_rows;
        db.birdview_pos = c->birdview_pos;
        db.force_sw    = c->force_sw;
        db.threshold   = c->threshold;
        db.alpha       = c->alpha;
        db.trigger_pct = c->trigger_pct;
        db.framestep   = c->framestep;
        db.min_blob_cells = c->min_blob_cells;
        db.sort_order  = c->sort_order;
        db.recording_enabled = c->recording_enabled;
        db.rec_segment_sec   = c->rec_segment_sec;
        db_camera_update(app->db, &db);
    }

    for (int i = 0; i < app->num_sensors; i++) {
        SensorCtx *s = &app->sensors[i];
        DbSensor db;
        db.id               = s->id;
        snprintf(db.name, sizeof(db.name), "%s", s->name);
        snprintf(db.type, sizeof(db.type), "%s", s->type);
        snprintf(db.address, sizeof(db.address), "%s", s->address);
        db.register_addr    = s->register_addr;
        db.scale            = s->scale;
        db.offset           = s->offset;
        snprintf(db.units, sizeof(db.units), "%s", s->units);
        db.poll_interval_ms = s->poll_interval_ms;
        db.enabled          = s->enabled;
        db_sensor_update(app->db, &db);
    }
}

void config_reload_cameras(AppCtx *app)
{
    DbCamera db_cams[MAX_CAMERAS];
    int n = db_camera_load_all(app->db, db_cams, MAX_CAMERAS);

    /* Update existing or add new, preserving runtime state */
    for (int i = 0; i < n; i++) {
        int found = -1;
        for (int j = 0; j < app->num_cameras; j++) {
            if (app->cameras[j].id == db_cams[i].id) {
                found = j;
                break;
            }
        }

        if (found >= 0) {
            /* Update config fields, keep runtime state */
            CameraCtx *c = &app->cameras[found];
            snprintf(c->name, sizeof(c->name), "%s", db_cams[i].name);
            snprintf(c->url, sizeof(c->url), "%s", db_cams[i].url);
            c->enabled     = db_cams[i].enabled;
            c->scale       = db_cams[i].scale;
            c->rotation    = db_cams[i].rotation;
            c->aspect_w    = db_cams[i].aspect_w;
            c->aspect_h    = db_cams[i].aspect_h;
            c->grid_cols   = db_cams[i].grid_cols;
            c->grid_rows   = db_cams[i].grid_rows;
            c->birdview_pos = db_cams[i].birdview_pos;
            c->force_sw    = db_cams[i].force_sw;
            c->threshold   = db_cams[i].threshold;
            c->alpha       = (float)db_cams[i].alpha;
            c->trigger_pct = (float)db_cams[i].trigger_pct;
            c->framestep   = db_cams[i].framestep;
            c->min_blob_cells = db_cams[i].min_blob_cells;
            if (c->min_blob_cells < 1) c->min_blob_cells = 3;
            c->sort_order  = db_cams[i].sort_order;
            c->recording_enabled = db_cams[i].recording_enabled;
            c->rec_segment_sec   = db_cams[i].rec_segment_sec;
            if (c->rec_segment_sec <= 0) c->rec_segment_sec = 300;
        } else if (app->num_cameras < MAX_CAMERAS) {
            /* New camera */
            int idx = app->num_cameras;
            memset(&app->cameras[idx], 0, sizeof(CameraCtx));
            pthread_mutex_init(&app->cameras[idx].buf_mutex, NULL);
            db_cam_to_ctx(&db_cams[i], &app->cameras[idx]);
            app->num_cameras++;
        }
    }
}

void config_reload_sensors(AppCtx *app)
{
    DbSensor db_sensors[MAX_SENSORS];
    int n = db_sensor_load_all(app->db, db_sensors, MAX_SENSORS);

    for (int i = 0; i < n; i++) {
        int found = -1;
        for (int j = 0; j < app->num_sensors; j++) {
            if (app->sensors[j].id == db_sensors[i].id) {
                found = j;
                break;
            }
        }

        if (found >= 0) {
            SensorCtx *s = &app->sensors[found];
            snprintf(s->name, sizeof(s->name), "%s", db_sensors[i].name);
            snprintf(s->type, sizeof(s->type), "%s", db_sensors[i].type);
            snprintf(s->address, sizeof(s->address), "%s", db_sensors[i].address);
            s->register_addr    = db_sensors[i].register_addr;
            s->scale            = db_sensors[i].scale;
            s->offset           = db_sensors[i].offset;
            snprintf(s->units, sizeof(s->units), "%s", db_sensors[i].units);
            s->poll_interval_ms = db_sensors[i].poll_interval_ms;
            s->enabled          = db_sensors[i].enabled;
        } else if (app->num_sensors < MAX_SENSORS) {
            int idx = app->num_sensors;
            memset(&app->sensors[idx], 0, sizeof(SensorCtx));
            db_sensor_to_ctx(&db_sensors[i], &app->sensors[idx]);
            app->num_sensors++;
        }
    }
}
