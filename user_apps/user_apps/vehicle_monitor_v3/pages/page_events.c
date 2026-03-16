/*
 * pages/page_events.c - Event log viewer
 *
 * Shows motion detection events in a filterable table with time range
 * and camera filter controls.
 */
#include "../app.h"
#include "../core/database.h"
#include "../ui/util.h"
#include <string.h>
#include <stdio.h>
#include <time.h>

#define MAX_EVENTS 1000

typedef struct {
    AppCtx       *app;
    GtkWidget    *tree_view;
    GtkListStore *list_store;
    GtkWidget    *cmb_camera;
    GtkWidget    *spn_hours;
    GtkWidget    *lbl_count;
    GtkWidget    *btn_refresh;
    int           active;
    guint         refresh_timer;
} EventsState;

static void refresh_events(EventsState *st)
{
    double hours = gtk_spin_button_get_value(GTK_SPIN_BUTTON(st->spn_hours));

    time_t now = time(NULL);
    time_t start = now - (time_t)(hours * 3600);

    struct tm tm_start, tm_end;
    localtime_r(&start, &tm_start);
    localtime_r(&now, &tm_end);

    char s_start[32], s_end[32];
    strftime(s_start, sizeof(s_start), "%Y-%m-%dT%H:%M:%S", &tm_start);
    strftime(s_end, sizeof(s_end), "%Y-%m-%dT%H:%M:%S", &tm_end);

    DbEvent events[MAX_EVENTS];
    int n = db_event_load_range(st->app->db, s_start, s_end,
                                events, MAX_EVENTS);

    /* Camera filter */
    int cam_idx = gtk_combo_box_get_active(GTK_COMBO_BOX(st->cmb_camera));
    int filter_cam_id = 0;
    if (cam_idx > 0 && cam_idx <= st->app->num_cameras)
        filter_cam_id = st->app->cameras[cam_idx - 1].id;

    gtk_list_store_clear(st->list_store);
    int shown = 0;
    for (int i = 0; i < n; i++) {
        if (filter_cam_id > 0 && events[i].camera_id != filter_cam_id)
            continue;
        GtkTreeIter iter;
        gtk_list_store_append(st->list_store, &iter);
        gtk_list_store_set(st->list_store, &iter,
            0, events[i].timestamp,
            1, events[i].camera_id,
            2, events[i].action_group,
            3, events[i].motion_pct,
            4, events[i].max_cell_pct,
            -1);
        shown++;
    }

    char buf[64];
    snprintf(buf, sizeof(buf), "%d events", shown);
    gtk_label_set_text(GTK_LABEL(st->lbl_count), buf);
}

static gboolean refresh_timer_cb(gpointer data)
{
    EventsState *st = (EventsState *)data;
    if (!st->active)
        return G_SOURCE_REMOVE;
    refresh_events(st);
    return G_SOURCE_CONTINUE;
}

static void on_refresh_clicked(GtkButton *btn, gpointer data)
{
    (void)btn;
    refresh_events((EventsState *)data);
}

static void on_hours_changed(GtkSpinButton *spin, gpointer data)
{
    (void)spin;
    refresh_events((EventsState *)data);
}

static void on_camera_changed(GtkComboBox *combo, gpointer data)
{
    (void)combo;
    refresh_events((EventsState *)data);
}

static GtkWidget *events_build(AppCtx *app)
{
    EventsState *st = (EventsState *)calloc(1, sizeof(EventsState));
    st->app = app;
    app->page_events_state = st;

    GtkWidget *vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
    gtk_widget_set_margin_start(vbox, 8);
    gtk_widget_set_margin_end(vbox, 8);
    gtk_widget_set_margin_top(vbox, 8);
    gtk_widget_set_margin_bottom(vbox, 8);

    /* Controls bar */
    GtkWidget *ctrl = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);

    /* Camera filter (All + each camera) */
    st->cmb_camera = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(st->cmb_camera), "All Cameras");
    for (int i = 0; i < app->num_cameras; i++)
        gtk_combo_box_text_append_text(
            GTK_COMBO_BOX_TEXT(st->cmb_camera), app->cameras[i].name);
    gtk_combo_box_set_active(GTK_COMBO_BOX(st->cmb_camera), 0);
    g_signal_connect(st->cmb_camera, "changed",
                     G_CALLBACK(on_camera_changed), st);
    gtk_box_pack_start(GTK_BOX(ctrl), st->cmb_camera, FALSE, FALSE, 0);

    gtk_box_pack_start(GTK_BOX(ctrl), gtk_label_new("Hours:"),
                       FALSE, FALSE, 0);
    st->spn_hours = ui_spin_float(24.0, 1.0, 168.0, 1.0, 0,
                                  G_CALLBACK(on_hours_changed), st);
    gtk_box_pack_start(GTK_BOX(ctrl), st->spn_hours, FALSE, FALSE, 0);

    st->btn_refresh = ui_button("Refresh",
                                G_CALLBACK(on_refresh_clicked), st);
    gtk_box_pack_start(GTK_BOX(ctrl), st->btn_refresh, FALSE, FALSE, 0);

    st->lbl_count = ui_label("0 events", 0);
    gtk_box_pack_end(GTK_BOX(ctrl), st->lbl_count, FALSE, FALSE, 0);

    gtk_box_pack_start(GTK_BOX(vbox), ctrl, FALSE, FALSE, 0);

    /* Event table */
    st->list_store = gtk_list_store_new(5,
        G_TYPE_STRING,  /* timestamp */
        G_TYPE_INT,     /* camera_id */
        G_TYPE_STRING,  /* action_group */
        G_TYPE_DOUBLE,  /* motion_pct */
        G_TYPE_DOUBLE   /* max_cell_pct */
    );

    st->tree_view = gtk_tree_view_new_with_model(
        GTK_TREE_MODEL(st->list_store));
    g_object_unref(st->list_store);

    GtkCellRenderer *ren = gtk_cell_renderer_text_new();
    GtkTreeViewColumn *col;

    col = gtk_tree_view_column_new_with_attributes("Time", ren, "text", 0, NULL);
    gtk_tree_view_column_set_resizable(col, TRUE);
    gtk_tree_view_column_set_min_width(col, 180);
    gtk_tree_view_append_column(GTK_TREE_VIEW(st->tree_view), col);

    col = gtk_tree_view_column_new_with_attributes("Cam", ren, "text", 1, NULL);
    gtk_tree_view_column_set_min_width(col, 40);
    gtk_tree_view_append_column(GTK_TREE_VIEW(st->tree_view), col);

    col = gtk_tree_view_column_new_with_attributes("Group", ren, "text", 2, NULL);
    gtk_tree_view_column_set_resizable(col, TRUE);
    gtk_tree_view_append_column(GTK_TREE_VIEW(st->tree_view), col);

    col = gtk_tree_view_column_new_with_attributes("Motion %", ren, "text", 3, NULL);
    gtk_tree_view_append_column(GTK_TREE_VIEW(st->tree_view), col);

    col = gtk_tree_view_column_new_with_attributes("Max Cell %", ren, "text", 4, NULL);
    gtk_tree_view_append_column(GTK_TREE_VIEW(st->tree_view), col);

    GtkWidget *scroll = ui_scrolled(st->tree_view);
    gtk_widget_set_vexpand(scroll, TRUE);
    gtk_box_pack_start(GTK_BOX(vbox), scroll, TRUE, TRUE, 0);

    return vbox;
}

static void events_on_show(AppCtx *app)
{
    EventsState *st = (EventsState *)app->page_events_state;
    if (!st) return;

    st->active = 1;

    /* Refresh camera combo */
    gtk_combo_box_text_remove_all(GTK_COMBO_BOX_TEXT(st->cmb_camera));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(st->cmb_camera), "All Cameras");
    for (int i = 0; i < app->num_cameras; i++)
        gtk_combo_box_text_append_text(
            GTK_COMBO_BOX_TEXT(st->cmb_camera), app->cameras[i].name);
    gtk_combo_box_set_active(GTK_COMBO_BOX(st->cmb_camera), 0);

    refresh_events(st);
    st->refresh_timer = g_timeout_add(5000, refresh_timer_cb, st);
}

static void events_on_hide(AppCtx *app)
{
    EventsState *st = (EventsState *)app->page_events_state;
    if (!st) return;
    st->active = 0;
}

static void events_cleanup(AppCtx *app)
{
    if (app->page_events_state) {
        free(app->page_events_state);
        app->page_events_state = NULL;
    }
}

const PageDef page_events = {
    .id      = "events",
    .title   = "Events",
    .build   = events_build,
    .on_show = events_on_show,
    .on_hide = events_on_hide,
    .cleanup = events_cleanup,
};
