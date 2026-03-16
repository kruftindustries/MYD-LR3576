/*
 * pages/page_cameras.c - Main camera preview grid with event timeline
 *
 * Shows live video from all enabled cameras in a grid layout.
 * Red tint overlay on cells exceeding trigger threshold.
 * FPS and motion % overlay per camera.
 * Embedded 24h event timeline bar at the bottom.
 */
#include "../app.h"
#include "../core/camera.h"
#include "../core/database.h"
#include "../ui/util.h"
#include <cairo.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <math.h>

#define TIMELINE_HEIGHT     50
#define MAX_TL_EVENTS      500

typedef struct {
    AppCtx      *app;
    GtkWidget   *vbox;
    GtkWidget   *grid;
    GtkWidget   *timeline_da;
    GtkWidget   *draw_areas[MAX_CAMERAS];
    guint        redraw_timer;
    int          active;

    /* Embedded timeline state */
    double       tl_zoom_hours;
    DbEvent      tl_events[MAX_TL_EVENTS];
    int          tl_num_events;
    DbActionGroup tl_groups[MAX_ACTION_GROUPS];
    int          tl_num_groups;
    guint        tl_refresh_timer;
} CamerasState;

/* --- Timeline helpers --- */

static void tl_load_events(CamerasState *st)
{
    time_t now = time(NULL);
    time_t start = now - (time_t)(st->tl_zoom_hours * 3600);

    struct tm tm_start, tm_end;
    localtime_r(&start, &tm_start);
    localtime_r(&now, &tm_end);

    char s_start[32], s_end[32];
    strftime(s_start, sizeof(s_start), "%Y-%m-%dT%H:%M:%S", &tm_start);
    strftime(s_end, sizeof(s_end), "%Y-%m-%dT%H:%M:%S", &tm_end);

    st->tl_num_events = db_event_load_range(st->app->db, s_start, s_end,
                                             st->tl_events, MAX_TL_EVENTS);
    st->tl_num_groups = db_action_group_load_all(st->app->db, st->tl_groups,
                                                  MAX_ACTION_GROUPS);
}

static void tl_get_group_color(CamerasState *st, const char *group_name,
                               double *r, double *g, double *b)
{
    for (int i = 0; i < st->tl_num_groups; i++) {
        if (strcmp(st->tl_groups[i].name, group_name) == 0) {
            ui_parse_color(st->tl_groups[i].color, r, g, b);
            return;
        }
    }
    *r = 1.0; *g = 0.4; *b = 0.0;
}

static double tl_ts_to_frac(const char *ts, double zoom_hours)
{
    struct tm tm_ev;
    memset(&tm_ev, 0, sizeof(tm_ev));
    sscanf(ts, "%d-%d-%dT%d:%d:%d",
           &tm_ev.tm_year, &tm_ev.tm_mon, &tm_ev.tm_mday,
           &tm_ev.tm_hour, &tm_ev.tm_min, &tm_ev.tm_sec);
    tm_ev.tm_year -= 1900;
    tm_ev.tm_mon -= 1;
    tm_ev.tm_isdst = -1;
    time_t ev_time = mktime(&tm_ev);

    time_t now = time(NULL);
    time_t start = now - (time_t)(zoom_hours * 3600);
    double range = (double)(now - start);
    if (range <= 0) return 0;
    return (double)(ev_time - start) / range;
}

static gboolean on_draw_timeline(GtkWidget *widget, cairo_t *cr,
                                 gpointer data)
{
    CamerasState *st = (CamerasState *)data;
    int aw = gtk_widget_get_allocated_width(widget);
    int ah = gtk_widget_get_allocated_height(widget);

    /* Background */
    cairo_set_source_rgb(cr, 0.12, 0.12, 0.15);
    cairo_paint(cr);

    /* Bar area */
    int bar_y = 16;
    int bar_h = ah - 28;
    if (bar_h < 8) bar_h = 8;

    cairo_set_source_rgb(cr, 0.18, 0.18, 0.22);
    cairo_rectangle(cr, 0, bar_y, aw, bar_h);
    cairo_fill(cr);

    /* Hour markers */
    cairo_set_source_rgba(cr, 1, 1, 1, 0.25);
    cairo_set_line_width(cr, 1.0);
    cairo_set_font_size(cr, 9);

    int num_marks = (int)st->tl_zoom_hours;
    if (num_marks < 1) num_marks = 1;
    if (num_marks > 48) num_marks = 48;

    time_t now = time(NULL);
    for (int i = 0; i <= num_marks; i++) {
        double frac = (double)i / st->tl_zoom_hours;
        double x = frac * aw;

        cairo_move_to(cr, x, bar_y);
        cairo_line_to(cr, x, bar_y + bar_h);
        cairo_stroke(cr);

        time_t t = now - (time_t)((st->tl_zoom_hours - i) * 3600);
        struct tm tm_t;
        localtime_r(&t, &tm_t);
        char buf[16];
        strftime(buf, sizeof(buf), "%H:%M", &tm_t);
        cairo_set_source_rgba(cr, 1, 1, 1, 0.5);
        cairo_move_to(cr, x + 2, bar_y + bar_h + 10);
        cairo_show_text(cr, buf);
        cairo_set_source_rgba(cr, 1, 1, 1, 0.25);
    }

    /* Event bars */
    double event_bar_w = st->tl_num_events > 100 ? 2.0 : 3.0;

    for (int i = 0; i < st->tl_num_events; i++) {
        double frac = tl_ts_to_frac(st->tl_events[i].timestamp,
                                     st->tl_zoom_hours);
        if (frac < 0 || frac > 1) continue;

        double x = frac * aw;
        double r, g, b;
        tl_get_group_color(st, st->tl_events[i].action_group, &r, &g, &b);

        double h_frac = 0.3 + 0.7 * (st->tl_events[i].max_cell_pct / 100.0);
        if (h_frac > 1.0) h_frac = 1.0;

        cairo_set_source_rgba(cr, r, g, b, 0.8);
        cairo_rectangle(cr, x - event_bar_w / 2,
                        bar_y + bar_h * (1 - h_frac),
                        event_bar_w, bar_h * h_frac);
        cairo_fill(cr);
    }

    /* Top label */
    cairo_set_source_rgba(cr, 1, 1, 1, 0.7);
    cairo_set_font_size(cr, 10);
    char title[64];
    snprintf(title, sizeof(title), "Events: %dh, %d total",
             (int)st->tl_zoom_hours, st->tl_num_events);
    cairo_move_to(cr, 4, 11);
    cairo_show_text(cr, title);

    return FALSE;
}

static gboolean tl_refresh_cb(gpointer data)
{
    CamerasState *st = (CamerasState *)data;
    if (!st->active)
        return G_SOURCE_REMOVE;
    tl_load_events(st);
    gtk_widget_queue_draw(st->timeline_da);
    return G_SOURCE_CONTINUE;
}

/* --- Camera drawing --- */

static gboolean on_draw_camera(GtkWidget *widget, cairo_t *cr, gpointer data)
{
    CameraCtx *cam = (CameraCtx *)data;
    int aw = gtk_widget_get_allocated_width(widget);
    int ah = gtk_widget_get_allocated_height(widget);

    /* Black background */
    cairo_set_source_rgb(cr, 0.1, 0.1, 0.1);
    cairo_paint(cr);

    /* Copy shared data under mutex - check buffer readiness inside lock
     * to avoid racing with worker thread freeing resources on reconnect */
    int gc = 0, gr = 0;
    float fps = 0;
    float motion_pct = 0;
    int blob_cells = 0;
    float grid_pcts[MAX_GRID * MAX_GRID];
    int has_video = 0;

    pthread_mutex_lock(&cam->buf_mutex);
    if (cam->disp_w > 0 && cam->color_rgb && cam->draw_color) {
        has_video = 1;
        memcpy(cam->draw_color, cam->color_rgb, cam->disp_size);
        gc = cam->shared_grid_cols;
        gr = cam->shared_grid_rows;
        blob_cells = cam->shared_blob_cells;
        fps = cam->shared_fps;
        motion_pct = cam->shared_motion_pct;
        memcpy(grid_pcts, cam->shared_grid_pcts, gc * gr * sizeof(float));
    }
    pthread_mutex_unlock(&cam->buf_mutex);

    if (!has_video) {
        /* No video: show camera name and connection status */
        cairo_set_source_rgb(cr, 0.5, 0.5, 0.5);
        cairo_set_font_size(cr, 14);
        cairo_move_to(cr, 10, ah / 2);
        cairo_show_text(cr, cam->name);
        cairo_move_to(cr, 10, ah / 2 + 20);
        cairo_set_font_size(cr, 11);
        if (!cam->url[0]) {
            cairo_show_text(cr, "No URL");
        } else if (cam->connect_state == CAM_FAILED) {
            cairo_set_source_rgb(cr, 0.8, 0.3, 0.3);
            cairo_show_text(cr, "Connection failed - retrying...");
        } else if (cam->connect_state == CAM_CONNECTING) {
            cairo_show_text(cr, "Connecting...");
        } else if (!cam->started) {
            cairo_show_text(cr, "Stopped");
        }
        return FALSE;
    }

    /* Render camera feed (fitted / letterboxed) */
    GdkPixbuf *pb = gdk_pixbuf_new_from_data(
        cam->draw_color, GDK_COLORSPACE_RGB, FALSE, 8,
        cam->disp_w, cam->disp_h, cam->disp_w * 3, NULL, NULL);

    double fx, fy, fw, fh;
    ui_draw_pixbuf_fitted(cr, pb, cam->disp_w, cam->disp_h, aw, ah,
                          &fx, &fy, &fw, &fh);
    g_object_unref(pb);

    /* Red tint on triggered cells (within fitted rect) */
    float trig = cam->trigger_pct;
    if (gc > 0 && gr > 0) {
        double cw = fw / gc;
        double ch = fh / gr;

        for (int row = 0; row < gr; row++) {
            for (int col = 0; col < gc; col++) {
                float pct = grid_pcts[row * gc + col];
                if (pct >= trig) {
                    double a = 0.1 + 0.3 * (pct / 100.0);
                    if (a > 0.4) a = 0.4;
                    cairo_set_source_rgba(cr, 1, 0, 0, a);
                    cairo_rectangle(cr, fx + col * cw, fy + row * ch,
                                    cw, ch);
                    cairo_fill(cr);
                }
            }
        }
    }

    /* Info overlay */
    cairo_set_source_rgba(cr, 0, 0, 0, 0.6);
    cairo_rectangle(cr, 0, 0, aw, 22);
    cairo_fill(cr);

    cairo_set_source_rgb(cr, 1, 1, 1);
    cairo_set_font_size(cr, 12);

    char info[128];
    snprintf(info, sizeof(info), "%s  %.1f fps  %.1f%%  B:%d",
             cam->name, fps, motion_pct, blob_cells);
    cairo_move_to(cr, 6, 15);
    cairo_show_text(cr, info);

    /* Motion indicator dot (blob-based) */
    if (blob_cells >= cam->min_blob_cells) {
        cairo_set_source_rgb(cr, 1, 0.2, 0.2);
        cairo_arc(cr, aw - 12, 11, 5, 0, 2 * G_PI);
        cairo_fill(cr);
    }

    /* Recording indicator (red circle with "R") */
    if (cam->recording && cam->rec_file) {
        cairo_set_source_rgb(cr, 0.9, 0.1, 0.1);
        cairo_arc(cr, aw - 30, 11, 5, 0, 2 * G_PI);
        cairo_fill(cr);
    }

    return FALSE;
}

static gboolean redraw_timer_cb(gpointer data)
{
    CamerasState *st = (CamerasState *)data;
    if (!st->active)
        return G_SOURCE_REMOVE;

    for (int i = 0; i < st->app->num_cameras; i++) {
        if (st->draw_areas[i] && st->app->cameras[i].buf_updated) {
            gtk_widget_queue_draw(st->draw_areas[i]);
            st->app->cameras[i].buf_updated = 0;
        }
    }
    return G_SOURCE_CONTINUE;
}

static void rebuild_grid(CamerasState *st)
{
    /* Remove existing children */
    GList *children = gtk_container_get_children(GTK_CONTAINER(st->grid));
    for (GList *l = children; l; l = l->next)
        gtk_container_remove(GTK_CONTAINER(st->grid), GTK_WIDGET(l->data));
    g_list_free(children);

    memset(st->draw_areas, 0, sizeof(st->draw_areas));

    int n = st->app->num_cameras;
    if (n == 0) {
        GtkWidget *ph = ui_placeholder("No Cameras",
            "Add cameras in Configuration tab");
        gtk_grid_attach(GTK_GRID(st->grid), ph, 0, 0, 1, 1);
        return;
    }

    /* Grid layout: 1 col for 1 cam, 2 cols for 2-4, 3 cols for 5-9 */
    int cols = 1;
    if (n >= 2) cols = 2;
    if (n >= 5) cols = 3;

    for (int i = 0; i < n; i++) {
        GtkWidget *da = gtk_drawing_area_new();
        gtk_widget_set_size_request(da, 320, 180);
        gtk_widget_set_hexpand(da, TRUE);
        gtk_widget_set_vexpand(da, TRUE);
        g_signal_connect(da, "draw",
                         G_CALLBACK(on_draw_camera), &st->app->cameras[i]);
        st->draw_areas[i] = da;

        int col = i % cols;
        int row = i / cols;
        gtk_grid_attach(GTK_GRID(st->grid), da, col, row, 1, 1);
    }
}

static GtkWidget *cameras_build(AppCtx *app)
{
    CamerasState *st = (CamerasState *)calloc(1, sizeof(CamerasState));
    st->app = app;
    st->tl_zoom_hours = 24.0;
    app->page_cameras_state = st;

    /* Vertical layout: camera grid on top, timeline at bottom */
    st->vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);

    st->grid = gtk_grid_new();
    gtk_grid_set_row_spacing(GTK_GRID(st->grid), 2);
    gtk_grid_set_column_spacing(GTK_GRID(st->grid), 2);
    gtk_widget_set_hexpand(st->grid, TRUE);
    gtk_widget_set_vexpand(st->grid, TRUE);

    rebuild_grid(st);
    gtk_box_pack_start(GTK_BOX(st->vbox), st->grid, TRUE, TRUE, 0);

    /* Timeline bar at bottom */
    st->timeline_da = gtk_drawing_area_new();
    gtk_widget_set_size_request(st->timeline_da, -1, TIMELINE_HEIGHT);
    g_signal_connect(st->timeline_da, "draw",
                     G_CALLBACK(on_draw_timeline), st);
    gtk_box_pack_start(GTK_BOX(st->vbox), st->timeline_da, FALSE, FALSE, 0);

    return st->vbox;
}

static void cameras_on_show(AppCtx *app)
{
    CamerasState *st = (CamerasState *)app->page_cameras_state;
    if (!st) return;

    /* Start all enabled cameras */
    camera_start_all(app);

    /* Auto-start recording for enabled cameras */
    for (int i = 0; i < app->num_cameras; i++) {
        CameraCtx *c = &app->cameras[i];
        if (c->enabled && c->recording_enabled && c->started && !c->recording)
            camera_rec_start(c, app);
    }

    /* Rebuild grid in case cameras changed */
    rebuild_grid(st);
    gtk_widget_show_all(st->vbox);

    /* Load and display timeline events */
    tl_load_events(st);
    gtk_widget_queue_draw(st->timeline_da);

    /* Start redraw timer */
    st->active = 1;
    st->redraw_timer = g_timeout_add(33, redraw_timer_cb, st); /* ~30fps */
    st->tl_refresh_timer = g_timeout_add(10000, tl_refresh_cb, st);
}

static void cameras_on_hide(AppCtx *app)
{
    CamerasState *st = (CamerasState *)app->page_cameras_state;
    if (!st) return;
    st->active = 0;
    /* Note: don't stop cameras here - other pages may need them */
}

static void cameras_cleanup(AppCtx *app)
{
    CamerasState *st = (CamerasState *)app->page_cameras_state;
    if (st) {
        st->active = 0;
        free(st);
        app->page_cameras_state = NULL;
    }
}

const PageDef page_cameras = {
    .id      = "cameras",
    .title   = "Cameras",
    .build   = cameras_build,
    .on_show = cameras_on_show,
    .on_hide = cameras_on_hide,
    .cleanup = cameras_cleanup,
};
