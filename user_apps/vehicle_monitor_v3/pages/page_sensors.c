/*
 * pages/page_sensors.c - Live sensor values with sparklines
 *
 * Displays current value, units, and mini sparkline chart for each sensor.
 * Sensor threads simulate random-walk values (stub for real I/O).
 * Readings logged to sensor_readings table.
 */
#include "../app.h"
#include "../core/database.h"
#include "../core/config.h"
#include "../ui/util.h"
#include <cairo.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

typedef struct {
    AppCtx     *app;
    GtkWidget  *vbox_list;
    GtkWidget  *value_labels[MAX_SENSORS];
    GtkWidget  *sparkline_das[MAX_SENSORS];
    int         active;
    guint       update_timer;
} SensorsState;

/* Sensor thread argument: sensor + db for logging */
typedef struct {
    SensorCtx *sensor;
    sqlite3   *db;
} SensorThreadArg;

static void *sensor_thread_with_db(void *arg)
{
    SensorThreadArg *sta = (SensorThreadArg *)arg;
    SensorCtx *s = sta->sensor;
    sqlite3 *db = sta->db;
    free(sta);

    srand((unsigned)time(NULL) ^ (unsigned)(uintptr_t)s);
    s->current_value = 20.0 + (rand() % 100) / 10.0;

    int log_counter = 0;
    int log_interval = 10; /* log every 10 polls */

    while (s->running) {
        /* Random walk simulation */
        double delta = ((rand() % 1000) / 1000.0 - 0.5) * 2.0;
        double raw = s->current_value / s->scale - s->offset / s->scale;
        raw += delta;
        double scaled = raw * s->scale + s->offset;
        s->current_value = scaled;

        /* Store in ring buffer */
        s->history[s->history_idx] = scaled;
        s->history_idx = (s->history_idx + 1) % SPARKLINE_POINTS;
        if (s->history_count < SPARKLINE_POINTS)
            s->history_count++;

        /* Periodic DB logging */
        log_counter++;
        if (log_counter >= log_interval && db) {
            db_reading_insert(db, s->id, raw, scaled);
            log_counter = 0;
        }

        usleep(s->poll_interval_ms * 1000);
    }

    return NULL;
}

static void start_sensor_threads(SensorsState *st)
{
    for (int i = 0; i < st->app->num_sensors; i++) {
        SensorCtx *s = &st->app->sensors[i];
        if (!s->enabled || s->running) continue;

        s->running = 1;
        s->history_count = 0;
        s->history_idx = 0;

        SensorThreadArg *sta = malloc(sizeof(SensorThreadArg));
        sta->sensor = s;
        sta->db = st->app->db;
        pthread_create(&s->thread, NULL, sensor_thread_with_db, sta);
    }
}

static gboolean on_draw_sparkline(GtkWidget *widget, cairo_t *cr,
                                  gpointer data)
{
    SensorCtx *s = (SensorCtx *)data;
    int aw = gtk_widget_get_allocated_width(widget);
    int ah = gtk_widget_get_allocated_height(widget);

    /* Background */
    cairo_set_source_rgb(cr, 0.15, 0.15, 0.18);
    cairo_rectangle(cr, 0, 0, aw, ah);
    cairo_fill(cr);

    if (s->history_count < 2) return FALSE;

    /* Find min/max for scaling */
    double min_v = 1e30, max_v = -1e30;
    int count = s->history_count;
    int start = (s->history_idx - count + SPARKLINE_POINTS) % SPARKLINE_POINTS;

    for (int i = 0; i < count; i++) {
        int idx = (start + i) % SPARKLINE_POINTS;
        double v = s->history[idx];
        if (v < min_v) min_v = v;
        if (v > max_v) max_v = v;
    }

    double range = max_v - min_v;
    if (range < 0.001) range = 1.0;

    /* Draw sparkline */
    cairo_set_source_rgb(cr, 0.3, 0.8, 0.4);
    cairo_set_line_width(cr, 1.5);

    for (int i = 0; i < count; i++) {
        int idx = (start + i) % SPARKLINE_POINTS;
        double x = (double)i / (count - 1) * aw;
        double y = ah - 2 - (s->history[idx] - min_v) / range * (ah - 4);

        if (i == 0)
            cairo_move_to(cr, x, y);
        else
            cairo_line_to(cr, x, y);
    }
    cairo_stroke(cr);

    /* Current value line */
    double cur_y = ah - 2 - (s->current_value - min_v) / range * (ah - 4);
    cairo_set_source_rgba(cr, 1, 0.5, 0, 0.5);
    cairo_set_line_width(cr, 0.5);
    cairo_move_to(cr, 0, cur_y);
    cairo_line_to(cr, aw, cur_y);
    cairo_stroke(cr);

    return FALSE;
}

static gboolean update_timer_cb(gpointer data)
{
    SensorsState *st = (SensorsState *)data;
    if (!st->active)
        return G_SOURCE_REMOVE;

    for (int i = 0; i < st->app->num_sensors; i++) {
        SensorCtx *s = &st->app->sensors[i];
        if (!s->enabled) continue;

        if (st->value_labels[i]) {
            char buf[64];
            snprintf(buf, sizeof(buf), "%.2f %s", s->current_value, s->units);
            gtk_label_set_text(GTK_LABEL(st->value_labels[i]), buf);
        }
        if (st->sparkline_das[i])
            gtk_widget_queue_draw(st->sparkline_das[i]);
    }

    return G_SOURCE_CONTINUE;
}

static void rebuild_sensor_list(SensorsState *st)
{
    /* Clear existing */
    GList *children = gtk_container_get_children(
        GTK_CONTAINER(st->vbox_list));
    for (GList *l = children; l; l = l->next)
        gtk_container_remove(GTK_CONTAINER(st->vbox_list),
                             GTK_WIDGET(l->data));
    g_list_free(children);

    memset(st->value_labels, 0, sizeof(st->value_labels));
    memset(st->sparkline_das, 0, sizeof(st->sparkline_das));

    if (st->app->num_sensors == 0) {
        GtkWidget *ph = ui_placeholder("No Sensors",
            "Add sensors in Sensor Config");
        gtk_box_pack_start(GTK_BOX(st->vbox_list), ph, TRUE, TRUE, 0);
        gtk_widget_show_all(st->vbox_list);
        return;
    }

    for (int i = 0; i < st->app->num_sensors; i++) {
        SensorCtx *s = &st->app->sensors[i];

        GtkWidget *frame = gtk_frame_new(s->name);
        gtk_widget_set_margin_start(frame, 4);
        gtk_widget_set_margin_end(frame, 4);
        gtk_widget_set_margin_top(frame, 2);
        gtk_widget_set_margin_bottom(frame, 2);

        GtkWidget *hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
        gtk_widget_set_margin_start(hbox, 8);
        gtk_widget_set_margin_end(hbox, 8);
        gtk_widget_set_margin_top(hbox, 4);
        gtk_widget_set_margin_bottom(hbox, 4);
        gtk_container_add(GTK_CONTAINER(frame), hbox);

        /* Value label */
        GtkWidget *val_lbl = ui_label("--", 1);
        gtk_widget_set_size_request(val_lbl, 120, -1);
        st->value_labels[i] = val_lbl;
        gtk_box_pack_start(GTK_BOX(hbox), val_lbl, FALSE, FALSE, 0);

        /* Type/address info */
        char info[64];
        snprintf(info, sizeof(info), "%s @ %s", s->type, s->address);
        gtk_box_pack_start(GTK_BOX(hbox), ui_label(info, 0),
                           FALSE, FALSE, 0);

        /* Sparkline */
        GtkWidget *spark = gtk_drawing_area_new();
        gtk_widget_set_size_request(spark, 200, 40);
        gtk_widget_set_hexpand(spark, TRUE);
        g_signal_connect(spark, "draw",
                         G_CALLBACK(on_draw_sparkline), s);
        st->sparkline_das[i] = spark;
        gtk_box_pack_start(GTK_BOX(hbox), spark, TRUE, TRUE, 0);

        gtk_box_pack_start(GTK_BOX(st->vbox_list), frame,
                           FALSE, FALSE, 0);
    }

    gtk_widget_show_all(st->vbox_list);
}

static GtkWidget *sensors_build(AppCtx *app)
{
    SensorsState *st = (SensorsState *)calloc(1, sizeof(SensorsState));
    st->app = app;
    app->page_sensors_state = st;

    st->vbox_list = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
    gtk_widget_set_margin_start(st->vbox_list, 8);
    gtk_widget_set_margin_end(st->vbox_list, 8);
    gtk_widget_set_margin_top(st->vbox_list, 8);

    GtkWidget *scroll = ui_scrolled(st->vbox_list);
    rebuild_sensor_list(st);

    return scroll;
}

static void sensors_on_show(AppCtx *app)
{
    SensorsState *st = (SensorsState *)app->page_sensors_state;
    if (!st) return;

    config_reload_sensors(app);
    rebuild_sensor_list(st);
    start_sensor_threads(st);

    st->active = 1;
    st->update_timer = g_timeout_add(500, update_timer_cb, st);
}

static void sensors_on_hide(AppCtx *app)
{
    SensorsState *st = (SensorsState *)app->page_sensors_state;
    if (!st) return;
    st->active = 0;
}

static void sensors_cleanup(AppCtx *app)
{
    if (app->page_sensors_state) {
        free(app->page_sensors_state);
        app->page_sensors_state = NULL;
    }
}

const PageDef page_sensors = {
    .id      = "sensors",
    .title   = "Sensors",
    .build   = sensors_build,
    .on_show = sensors_on_show,
    .on_hide = sensors_on_hide,
    .cleanup = sensors_cleanup,
};
