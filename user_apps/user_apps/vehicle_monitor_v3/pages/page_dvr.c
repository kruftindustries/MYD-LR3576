/*
 * pages/page_dvr.c - DVR Recording Playback & Interactive Timeline
 *
 * Layout (top to bottom):
 *   Transport bar: camera combo, playback controls, speed, time display
 *   Video DrawingArea: main focus, vexpand, letterboxed playback
 *   Timeline DrawingArea: 80px, recording bars, event ticks, draggable cursor
 *     scroll-wheel zoom centered on mouse
 */
#include "../app.h"
#include "../core/database.h"
#include "../core/playback.h"
#include "../ui/util.h"
#include <cairo.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <math.h>

#define MAX_TIMELINE_EVENTS 500
#define MAX_RECORDINGS      256
#define TIMELINE_HEIGHT     80

typedef struct {
    AppCtx        *app;
    PlaybackCtx   *playback;
    int            active;

    /* Widgets */
    GtkWidget     *vbox;
    GtkWidget     *cmb_camera;
    GtkWidget     *btn_play;
    GtkWidget     *btn_step_back;
    GtkWidget     *btn_step_fwd;
    GtkWidget     *cmb_speed;
    GtkWidget     *lbl_time;
    GtkWidget     *video_da;
    GtkWidget     *timeline_da;

    /* Timers */
    guint          redraw_timer;
    guint          refresh_timer;

    /* Timeline state */
    double         view_center;     /* epoch of view center */
    double         zoom_hours;      /* visible window in hours */
    double         cursor_epoch;    /* playback cursor position */
    int            dragging;        /* mouse drag on timeline */

    /* Cached data */
    DbRecording    recordings[MAX_RECORDINGS];
    int            num_recordings;
    DbEvent        events[MAX_TIMELINE_EVENTS];
    int            num_events;
    DbActionGroup  groups[MAX_ACTION_GROUPS];
    int            num_groups;
    int            selected_cam_id;
} DvrState;

/* --- Helpers --- */

static double ts_to_epoch(const char *ts)
{
    if (!ts || !ts[0]) return 0;
    struct tm tm;
    memset(&tm, 0, sizeof(tm));
    sscanf(ts, "%d-%d-%dT%d:%d:%d",
           &tm.tm_year, &tm.tm_mon, &tm.tm_mday,
           &tm.tm_hour, &tm.tm_min, &tm.tm_sec);
    tm.tm_year -= 1900;
    tm.tm_mon -= 1;
    tm.tm_isdst = -1;
    return (double)mktime(&tm);
}

static void epoch_to_hms(double epoch, char *buf, int buflen)
{
    time_t t = (time_t)epoch;
    struct tm tm;
    localtime_r(&t, &tm);
    snprintf(buf, buflen, "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);
}

static void epoch_to_ts(double epoch, char *buf, int buflen)
{
    time_t t = (time_t)epoch;
    struct tm tm;
    localtime_r(&t, &tm);
    strftime(buf, buflen, "%Y-%m-%dT%H:%M:%S", &tm);
}

static int get_selected_cam_id(DvrState *st)
{
    int idx = gtk_combo_box_get_active(GTK_COMBO_BOX(st->cmb_camera));
    if (idx < 0 || idx >= st->app->num_cameras) return -1;
    return st->app->cameras[idx].id;
}

static void get_group_color(DvrState *st, const char *group_name,
                            double *r, double *g, double *b)
{
    for (int i = 0; i < st->num_groups; i++) {
        if (strcmp(st->groups[i].name, group_name) == 0) {
            ui_parse_color(st->groups[i].color, r, g, b);
            return;
        }
    }
    *r = 1.0; *g = 0.4; *b = 0.0;
}

/* --- Data loading --- */

static void load_data(DvrState *st)
{
    double half = st->zoom_hours * 1800.0;
    double view_start = st->view_center - half;
    double view_end = st->view_center + half;

    char s_start[32], s_end[32];
    epoch_to_ts(view_start, s_start, sizeof(s_start));
    epoch_to_ts(view_end, s_end, sizeof(s_end));

    int cam_id = st->selected_cam_id;
    if (cam_id > 0) {
        st->num_recordings = db_recording_load_range(
            st->app->db, cam_id, s_start, s_end,
            st->recordings, MAX_RECORDINGS);
    } else {
        st->num_recordings = 0;
    }

    st->num_events = db_event_load_range(st->app->db, s_start, s_end,
                                         st->events, MAX_TIMELINE_EVENTS);
    st->num_groups = db_action_group_load_all(st->app->db, st->groups,
                                              MAX_ACTION_GROUPS);
}

/* --- Video drawing --- */

static gboolean on_draw_video(GtkWidget *widget, cairo_t *cr, gpointer data)
{
    DvrState *st = (DvrState *)data;
    int aw = gtk_widget_get_allocated_width(widget);
    int ah = gtk_widget_get_allocated_height(widget);

    /* Dark background */
    cairo_set_source_rgb(cr, 0.08, 0.08, 0.08);
    cairo_paint(cr);

    PlaybackCtx *pb = st->playback;
    if (!pb || !pb->started || !pb->frame_rgb || pb->disp_w == 0) {
        cairo_set_source_rgb(cr, 0.4, 0.4, 0.4);
        cairo_set_font_size(cr, 16);
        const char *msg = "Click timeline to start playback";
        cairo_text_extents_t ext;
        cairo_text_extents(cr, msg, &ext);
        cairo_move_to(cr, (aw - ext.width) / 2, (ah + ext.height) / 2);
        cairo_show_text(cr, msg);
        return FALSE;
    }

    /* Copy frame data under mutex */
    int fw = pb->disp_w;
    int fh = pb->disp_h;
    int fsize = pb->disp_size;
    uint8_t *buf = malloc(fsize);
    if (!buf) return FALSE;

    pthread_mutex_lock(&pb->mutex);
    memcpy(buf, pb->frame_rgb, fsize);
    pthread_mutex_unlock(&pb->mutex);

    /* Render fitted / letterboxed */
    GdkPixbuf *pixbuf = gdk_pixbuf_new_from_data(
        buf, GDK_COLORSPACE_RGB, FALSE, 8,
        fw, fh, fw * 3, NULL, NULL);

    double fx, fy, ffw, ffh;
    ui_draw_pixbuf_fitted(cr, pixbuf, fw, fh, aw, ah,
                          &fx, &fy, &ffw, &ffh);
    g_object_unref(pixbuf);
    free(buf);

    return FALSE;
}

/* --- Timeline drawing --- */

static void timeline_get_view(DvrState *st, double *view_start, double *view_end)
{
    double half = st->zoom_hours * 1800.0;
    *view_start = st->view_center - half;
    *view_end = st->view_center + half;
}

static double epoch_to_x(DvrState *st, double epoch, int width)
{
    double vs, ve;
    timeline_get_view(st, &vs, &ve);
    if (ve <= vs) return 0;
    return (epoch - vs) / (ve - vs) * width;
}

static double x_to_epoch(DvrState *st, double x, int width)
{
    double vs, ve;
    timeline_get_view(st, &vs, &ve);
    return vs + (x / width) * (ve - vs);
}

static gboolean on_draw_timeline(GtkWidget *widget, cairo_t *cr, gpointer data)
{
    DvrState *st = (DvrState *)data;
    int aw = gtk_widget_get_allocated_width(widget);
    int ah = gtk_widget_get_allocated_height(widget);

    double vs, ve;
    timeline_get_view(st, &vs, &ve);
    double range = ve - vs;
    if (range <= 0) return FALSE;

    /* Background */
    cairo_set_source_rgb(cr, 0.12, 0.12, 0.15);
    cairo_paint(cr);

    /* Bar area */
    int bar_y = 4;
    int bar_h = ah - 20;
    if (bar_h < 20) bar_h = 20;

    cairo_set_source_rgb(cr, 0.16, 0.16, 0.20);
    cairo_rectangle(cr, 0, bar_y, aw, bar_h);
    cairo_fill(cr);

    /* Recording bars (blue filled rectangles) */
    for (int i = 0; i < st->num_recordings; i++) {
        DbRecording *rec = &st->recordings[i];
        double rs = ts_to_epoch(rec->start_time);
        double re = rec->end_time[0] ? ts_to_epoch(rec->end_time) : (double)time(NULL);

        double x1 = epoch_to_x(st, rs, aw);
        double x2 = epoch_to_x(st, re, aw);
        if (x2 < 0 || x1 > aw) continue;
        if (x1 < 0) x1 = 0;
        if (x2 > aw) x2 = aw;

        cairo_set_source_rgba(cr, 0.2, 0.4, 0.8, 0.5);
        cairo_rectangle(cr, x1, bar_y, x2 - x1, bar_h);
        cairo_fill(cr);
    }

    /* Time markers */
    double step_sec;
    if (st->zoom_hours <= 1)        step_sec = 300;    /* 5 min */
    else if (st->zoom_hours <= 6)   step_sec = 1800;   /* 30 min */
    else if (st->zoom_hours <= 24)  step_sec = 3600;   /* 1 hour */
    else                            step_sec = 7200;   /* 2 hours */

    double first_mark = floor(vs / step_sec) * step_sec;
    cairo_set_font_size(cr, 9);

    for (double t = first_mark; t <= ve; t += step_sec) {
        double x = epoch_to_x(st, t, aw);
        if (x < 0 || x > aw) continue;

        cairo_set_source_rgba(cr, 1, 1, 1, 0.2);
        cairo_set_line_width(cr, 1.0);
        cairo_move_to(cr, x, bar_y);
        cairo_line_to(cr, x, bar_y + bar_h);
        cairo_stroke(cr);

        char buf[16];
        epoch_to_hms(t, buf, sizeof(buf));
        cairo_set_source_rgba(cr, 1, 1, 1, 0.5);
        cairo_move_to(cr, x + 2, ah - 2);
        cairo_show_text(cr, buf);
    }

    /* Event ticks */
    for (int i = 0; i < st->num_events; i++) {
        if (st->selected_cam_id > 0 &&
            st->events[i].camera_id != st->selected_cam_id)
            continue;

        double ev_epoch = ts_to_epoch(st->events[i].timestamp);
        double x = epoch_to_x(st, ev_epoch, aw);
        if (x < 0 || x > aw) continue;

        double r, g, b;
        get_group_color(st, st->events[i].action_group, &r, &g, &b);

        double h_frac = 0.3 + 0.7 * (st->events[i].max_cell_pct / 100.0);
        if (h_frac > 1.0) h_frac = 1.0;

        cairo_set_source_rgba(cr, r, g, b, 0.8);
        cairo_rectangle(cr, x - 1, bar_y + bar_h * (1 - h_frac),
                        2, bar_h * h_frac);
        cairo_fill(cr);
    }

    /* Playback cursor (red line) */
    double cx = epoch_to_x(st, st->cursor_epoch, aw);
    if (cx >= 0 && cx <= aw) {
        cairo_set_source_rgb(cr, 1.0, 0.2, 0.2);
        cairo_set_line_width(cr, 2.0);
        cairo_move_to(cr, cx, bar_y);
        cairo_line_to(cr, cx, bar_y + bar_h);
        cairo_stroke(cr);

        /* Cursor handle triangle */
        cairo_move_to(cr, cx - 5, bar_y);
        cairo_line_to(cr, cx + 5, bar_y);
        cairo_line_to(cr, cx, bar_y + 8);
        cairo_close_path(cr);
        cairo_fill(cr);
    }

    return FALSE;
}

/* --- Timeline mouse events --- */

static gboolean on_timeline_press(GtkWidget *widget, GdkEventButton *ev,
                                  gpointer data)
{
    DvrState *st = (DvrState *)data;
    if (ev->button != 1) return FALSE;

    int aw = gtk_widget_get_allocated_width(widget);
    double epoch = x_to_epoch(st, ev->x, aw);
    st->cursor_epoch = epoch;
    st->dragging = 1;

    /* Seek playback */
    if (st->playback && st->playback->started) {
        playback_seek(st->playback, epoch);
    } else if (st->selected_cam_id > 0) {
        if (!st->playback)
            st->playback = playback_create(st->app, st->selected_cam_id);
        playback_start(st->playback, epoch);
        gtk_button_set_label(GTK_BUTTON(st->btn_play), "||");
    }

    gtk_widget_queue_draw(st->timeline_da);
    return TRUE;
}

static gboolean on_timeline_motion(GtkWidget *widget, GdkEventMotion *ev,
                                   gpointer data)
{
    DvrState *st = (DvrState *)data;
    if (!st->dragging) return FALSE;

    int aw = gtk_widget_get_allocated_width(widget);
    double epoch = x_to_epoch(st, ev->x, aw);
    st->cursor_epoch = epoch;

    if (st->playback && st->playback->started)
        playback_seek(st->playback, epoch);

    gtk_widget_queue_draw(st->timeline_da);
    return TRUE;
}

static gboolean on_timeline_release(GtkWidget *widget, GdkEventButton *ev,
                                    gpointer data)
{
    (void)widget;
    (void)ev;
    DvrState *st = (DvrState *)data;
    st->dragging = 0;
    return TRUE;
}

static gboolean on_timeline_scroll(GtkWidget *widget, GdkEventScroll *ev,
                                   gpointer data)
{
    DvrState *st = (DvrState *)data;
    int aw = gtk_widget_get_allocated_width(widget);

    /* Zoom centered on mouse X */
    double mouse_epoch = x_to_epoch(st, ev->x, aw);

    if (ev->direction == GDK_SCROLL_UP) {
        st->zoom_hours /= 1.5;
    } else if (ev->direction == GDK_SCROLL_DOWN) {
        st->zoom_hours *= 1.5;
    } else if (ev->direction == GDK_SCROLL_SMOOTH) {
        if (ev->delta_y < 0)
            st->zoom_hours /= 1.5;
        else if (ev->delta_y > 0)
            st->zoom_hours *= 1.5;
    }

    if (st->zoom_hours < 0.1) st->zoom_hours = 0.1;
    if (st->zoom_hours > 168) st->zoom_hours = 168;

    /* Re-center so mouse position stays at same epoch */
    double new_half = st->zoom_hours * 1800.0;
    double mouse_frac = ev->x / aw;
    st->view_center = mouse_epoch + new_half - mouse_frac * 2.0 * new_half;

    load_data(st);
    gtk_widget_queue_draw(st->timeline_da);
    return TRUE;
}

/* --- Transport controls --- */

static void on_play_clicked(GtkButton *btn, gpointer data)
{
    DvrState *st = (DvrState *)data;

    if (!st->playback) {
        if (st->selected_cam_id <= 0) return;
        st->playback = playback_create(st->app, st->selected_cam_id);
        double start = st->cursor_epoch;
        if (start <= 0) start = (double)time(NULL) - 3600;
        playback_start(st->playback, start);
        gtk_button_set_label(btn, "||");
        return;
    }

    if (!st->playback->started) {
        double start = st->cursor_epoch;
        if (start <= 0) start = (double)time(NULL) - 3600;
        playback_start(st->playback, start);
        gtk_button_set_label(btn, "||");
        return;
    }

    playback_toggle_pause(st->playback);
    int paused = playback_is_paused(st->playback);
    gtk_button_set_label(btn, paused ? ">" : "||");
}

static void on_step_back(GtkButton *btn, gpointer data)
{
    (void)btn;
    DvrState *st = (DvrState *)data;
    if (!st->playback || !st->playback->started) return;
    double pos = playback_get_position(st->playback);
    playback_seek(st->playback, pos - 10.0);
    st->cursor_epoch = pos - 10.0;
}

static void on_step_fwd(GtkButton *btn, gpointer data)
{
    (void)btn;
    DvrState *st = (DvrState *)data;
    if (!st->playback || !st->playback->started) return;
    double pos = playback_get_position(st->playback);
    playback_seek(st->playback, pos + 10.0);
    st->cursor_epoch = pos + 10.0;
}

static void on_speed_changed(GtkComboBox *combo, gpointer data)
{
    DvrState *st = (DvrState *)data;
    if (!st->playback) return;
    int idx = gtk_combo_box_get_active(combo);
    if (idx >= 0 && idx < SPEED_COUNT)
        playback_set_speed(st->playback, (PlaybackSpeed)idx);
}

static void on_camera_changed(GtkComboBox *combo, gpointer data)
{
    (void)combo;
    DvrState *st = (DvrState *)data;
    int new_id = get_selected_cam_id(st);
    if (new_id == st->selected_cam_id) return;

    if (st->playback) {
        playback_destroy(st->playback);
        st->playback = NULL;
        gtk_button_set_label(GTK_BUTTON(st->btn_play), ">");
    }

    st->selected_cam_id = new_id;
    load_data(st);
    gtk_widget_queue_draw(st->timeline_da);
    gtk_widget_queue_draw(st->video_da);
}

/* --- Timers --- */

static gboolean redraw_timer_cb(gpointer data)
{
    DvrState *st = (DvrState *)data;
    if (!st->active)
        return G_SOURCE_REMOVE;

    /* Update playback cursor */
    if (st->playback && st->playback->started && !st->dragging) {
        double pos = playback_get_position(st->playback);
        if (pos > 0) {
            st->cursor_epoch = pos;

            /* Auto-scroll: if cursor nears edge, re-center */
            double vs, ve;
            timeline_get_view(st, &vs, &ve);
            double margin = (ve - vs) * 0.1;
            if (pos > ve - margin || pos < vs + margin) {
                st->view_center = pos;
            }
        }

        /* Update time label */
        char buf[16];
        epoch_to_hms(pos, buf, sizeof(buf));
        gtk_label_set_text(GTK_LABEL(st->lbl_time), buf);
    }

    gtk_widget_queue_draw(st->video_da);
    gtk_widget_queue_draw(st->timeline_da);

    return G_SOURCE_CONTINUE;
}

static gboolean refresh_timer_cb(gpointer data)
{
    DvrState *st = (DvrState *)data;
    if (!st->active)
        return G_SOURCE_REMOVE;
    load_data(st);
    return G_SOURCE_CONTINUE;
}

/* --- Build --- */

static GtkWidget *dvr_build(AppCtx *app)
{
    DvrState *st = (DvrState *)calloc(1, sizeof(DvrState));
    st->app = app;
    st->zoom_hours = 6.0;
    st->view_center = (double)time(NULL) - 3.0 * 3600;
    st->cursor_epoch = (double)time(NULL);
    app->page_dvr_state = st;

    st->vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 2);
    gtk_widget_set_margin_start(st->vbox, 4);
    gtk_widget_set_margin_end(st->vbox, 4);
    gtk_widget_set_margin_top(st->vbox, 4);
    gtk_widget_set_margin_bottom(st->vbox, 4);

    /* --- Transport bar --- */
    GtkWidget *transport = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    gtk_widget_set_margin_bottom(transport, 2);

    /* Camera combo */
    st->cmb_camera = gtk_combo_box_text_new();
    for (int i = 0; i < app->num_cameras; i++) {
        gtk_combo_box_text_append_text(
            GTK_COMBO_BOX_TEXT(st->cmb_camera), app->cameras[i].name);
    }
    if (app->num_cameras > 0) {
        gtk_combo_box_set_active(GTK_COMBO_BOX(st->cmb_camera), 0);
        st->selected_cam_id = app->cameras[0].id;
    }
    g_signal_connect(st->cmb_camera, "changed",
                     G_CALLBACK(on_camera_changed), st);
    gtk_box_pack_start(GTK_BOX(transport), st->cmb_camera, FALSE, FALSE, 0);

    st->btn_step_back = ui_button("<<", G_CALLBACK(on_step_back), st);
    gtk_box_pack_start(GTK_BOX(transport), st->btn_step_back, FALSE, FALSE, 0);

    st->btn_play = ui_button(">", G_CALLBACK(on_play_clicked), st);
    gtk_box_pack_start(GTK_BOX(transport), st->btn_play, FALSE, FALSE, 0);

    st->btn_step_fwd = ui_button(">>", G_CALLBACK(on_step_fwd), st);
    gtk_box_pack_start(GTK_BOX(transport), st->btn_step_fwd, FALSE, FALSE, 0);

    st->cmb_speed = gtk_combo_box_text_new();
    for (int i = 0; i < SPEED_COUNT; i++)
        gtk_combo_box_text_append_text(
            GTK_COMBO_BOX_TEXT(st->cmb_speed), playback_speed_label(i));
    gtk_combo_box_set_active(GTK_COMBO_BOX(st->cmb_speed), SPEED_1X);
    g_signal_connect(st->cmb_speed, "changed",
                     G_CALLBACK(on_speed_changed), st);
    gtk_box_pack_start(GTK_BOX(transport), st->cmb_speed, FALSE, FALSE, 0);

    st->lbl_time = gtk_label_new("--:--:--");
    gtk_box_pack_end(GTK_BOX(transport), st->lbl_time, FALSE, FALSE, 4);

    gtk_box_pack_start(GTK_BOX(st->vbox), transport, FALSE, FALSE, 0);

    /* --- Video area (main focus) --- */
    st->video_da = gtk_drawing_area_new();
    gtk_widget_set_vexpand(st->video_da, TRUE);
    gtk_widget_set_hexpand(st->video_da, TRUE);
    gtk_widget_set_size_request(st->video_da, 320, 180);
    g_signal_connect(st->video_da, "draw", G_CALLBACK(on_draw_video), st);
    gtk_box_pack_start(GTK_BOX(st->vbox), st->video_da, TRUE, TRUE, 0);

    /* --- Timeline (scroll-wheel zoom) --- */
    st->timeline_da = gtk_drawing_area_new();
    gtk_widget_set_size_request(st->timeline_da, -1, TIMELINE_HEIGHT);
    gtk_widget_add_events(st->timeline_da,
        GDK_BUTTON_PRESS_MASK | GDK_BUTTON_RELEASE_MASK |
        GDK_POINTER_MOTION_MASK | GDK_SCROLL_MASK |
        GDK_SMOOTH_SCROLL_MASK);
    g_signal_connect(st->timeline_da, "draw",
                     G_CALLBACK(on_draw_timeline), st);
    g_signal_connect(st->timeline_da, "button-press-event",
                     G_CALLBACK(on_timeline_press), st);
    g_signal_connect(st->timeline_da, "motion-notify-event",
                     G_CALLBACK(on_timeline_motion), st);
    g_signal_connect(st->timeline_da, "button-release-event",
                     G_CALLBACK(on_timeline_release), st);
    g_signal_connect(st->timeline_da, "scroll-event",
                     G_CALLBACK(on_timeline_scroll), st);
    gtk_box_pack_start(GTK_BOX(st->vbox), st->timeline_da, FALSE, FALSE, 0);

    return st->vbox;
}

/* --- Page callbacks --- */

static void dvr_on_show(AppCtx *app)
{
    DvrState *st = (DvrState *)app->page_dvr_state;
    if (!st) return;

    st->active = 1;
    st->view_center = (double)time(NULL) - st->zoom_hours * 1800.0;

    /* Refresh camera combo */
    gtk_combo_box_text_remove_all(GTK_COMBO_BOX_TEXT(st->cmb_camera));
    for (int i = 0; i < app->num_cameras; i++) {
        gtk_combo_box_text_append_text(
            GTK_COMBO_BOX_TEXT(st->cmb_camera), app->cameras[i].name);
    }
    if (app->num_cameras > 0) {
        gtk_combo_box_set_active(GTK_COMBO_BOX(st->cmb_camera), 0);
        st->selected_cam_id = app->cameras[0].id;
    }

    load_data(st);

    st->redraw_timer = g_timeout_add(33, redraw_timer_cb, st);
    st->refresh_timer = g_timeout_add(5000, refresh_timer_cb, st);
}

static void dvr_on_hide(AppCtx *app)
{
    DvrState *st = (DvrState *)app->page_dvr_state;
    if (!st) return;
    st->active = 0;
}

static void dvr_cleanup(AppCtx *app)
{
    DvrState *st = (DvrState *)app->page_dvr_state;
    if (!st) return;
    st->active = 0;
    if (st->playback)
        playback_destroy(st->playback);
    free(st);
    app->page_dvr_state = NULL;
}

const PageDef page_dvr = {
    .id      = "dvr",
    .title   = "DVR",
    .build   = dvr_build,
    .on_show = dvr_on_show,
    .on_hide = dvr_on_hide,
    .cleanup = dvr_cleanup,
};
