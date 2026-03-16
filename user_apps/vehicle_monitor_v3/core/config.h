/*
 * core/config.h - Configuration load/save helpers
 */
#ifndef CONFIG_H
#define CONFIG_H

#include "../app.h"

/* Load all cameras and sensors from DB into AppCtx */
void config_load_all(AppCtx *app);

/* Save all camera/sensor settings back to DB */
void config_save_all(AppCtx *app);

/* Reload cameras from DB (after add/edit/delete) */
void config_reload_cameras(AppCtx *app);

/* Reload sensors from DB */
void config_reload_sensors(AppCtx *app);

#endif /* CONFIG_H */
