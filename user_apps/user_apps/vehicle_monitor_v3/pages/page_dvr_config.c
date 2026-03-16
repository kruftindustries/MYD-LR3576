/*
 * pages/page_dvr_config.c - DVR recording configuration
 *
 * Per-camera recording enable/disable, segment duration,
 * and recording directory management.
 */
#include "../app.h"
#include "../core/database.h"
#include "../core/config.h"
#include "../core/camera.h"
#include "../ui/util.h"
#include <string.h>
#include <stdio.h>

typedef struct {
    AppCtx    *app;
    GtkWidget *grid;
    GtkWidget *chk_recording[MAX_CAMERAS];
    GtkWidget *spn_segment[MAX_CAMERAS];
    GtkWidget *lbl_status[MAX_CAMERAS];
    GtkWidget *btn_save;
    int        cam_ids[MAX_CAMERAS];
    int        num_cameras;
} DvrConfigState;

static void refresh_status(DvrConfigState *st)
{
    for (int i = 0; i < st->num_cameras; i++) {
        int cam_idx = -1;
        for (int j = 0; j < st->app->num_cameras; j++) {
            if (st->app->cameras[j].id == st->cam_ids[i]) {
                cam_idx = j;
                break;
            }
        }
        if (cam_idx < 0) {
            gtk_label_set_text(GTK_LABEL(st->lbl_status[i]), "Not loaded");
            continue;
        }
        CameraCtx *c = &st->app->cameras[cam_idx];
        const char *status = c->recording ? "Recording" :
                             (c->started ? "Idle" : "Stopped");
        gtk_label_set_text(GTK_LABEL(st->lbl_status[i]), status);
    }
}

static gboolean status_timer_cb(gpointer data)
{
    DvrConfigState *st = (DvrConfigState *)data;
    if (!st->app->page_dvr_config_state)
        return G_SOURCE_REMOVE;
    refresh_status(st);
    return G_SOURCE_CONTINUE;
}

static void on_save_clicked(GtkButton *btn, gpointer data)
{
    (void)btn;
    DvrConfigState *st = (DvrConfigState *)data;

    DbCamera cams[MAX_CAMERAS];
    int n = db_camera_load_all(st->app->db, cams, MAX_CAMERAS);

    for (int i = 0; i < st->num_cameras; i++) {
        int rec_en = gtk_toggle_button_get_active(
            GTK_TOGGLE_BUTTON(st->chk_recording[i])) ? 1 : 0;
        int seg_sec = gtk_spin_button_get_value_as_int(
            GTK_SPIN_BUTTON(st->spn_segment[i]));

        /* Find this camera in DB results and update */
        for (int j = 0; j < n; j++) {
            if (cams[j].id == st->cam_ids[i]) {
                cams[j].recording_enabled = rec_en;
                cams[j].rec_segment_sec = seg_sec;
                db_camera_update(st->app->db, &cams[j]);

                /* Update live CameraCtx */
                for (int k = 0; k < st->app->num_cameras; k++) {
                    if (st->app->cameras[k].id == st->cam_ids[i]) {
                        st->app->cameras[k].recording_enabled = rec_en;
                        st->app->cameras[k].rec_segment_sec = seg_sec;

                        /* Start/stop recording based on new setting */
                        if (rec_en && st->app->cameras[k].started &&
                            !st->app->cameras[k].recording)
                            camera_rec_start(&st->app->cameras[k], st->app);
                        else if (!rec_en && st->app->cameras[k].recording)
                            camera_rec_stop(&st->app->cameras[k], st->app);
                        break;
                    }
                }
                break;
            }
        }
    }

    printf("DVR config saved\n");
    refresh_status(st);
}

static GtkWidget *dvr_config_build(AppCtx *app)
{
    DvrConfigState *st = (DvrConfigState *)calloc(1, sizeof(DvrConfigState));
    st->app = app;
    app->page_dvr_config_state = st;

    GtkWidget *vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 8);
    gtk_widget_set_margin_start(vbox, 12);
    gtk_widget_set_margin_end(vbox, 12);
    gtk_widget_set_margin_top(vbox, 12);
    gtk_widget_set_margin_bottom(vbox, 12);

    gtk_box_pack_start(GTK_BOX(vbox), ui_label("DVR Recording Settings", 1),
                       FALSE, FALSE, 0);

    /* Camera recording table */
    st->grid = ui_grid();

    /* Header row */
    gtk_grid_attach(GTK_GRID(st->grid), ui_label("Camera", 1), 0, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(st->grid), ui_label("Record", 1), 1, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(st->grid), ui_label("Segment (sec)", 1), 2, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(st->grid), ui_label("Status", 1), 3, 0, 1, 1);

    /* Load cameras from DB */
    DbCamera cams[MAX_CAMERAS];
    int n = db_camera_load_all(app->db, cams, MAX_CAMERAS);
    st->num_cameras = n;

    for (int i = 0; i < n; i++) {
        int row = i + 1;
        st->cam_ids[i] = cams[i].id;

        /* Camera name label */
        GtkWidget *lbl_name = ui_label(cams[i].name, 0);
        gtk_label_set_xalign(GTK_LABEL(lbl_name), 0.0);
        gtk_grid_attach(GTK_GRID(st->grid), lbl_name, 0, row, 1, 1);

        /* Recording enabled checkbox */
        st->chk_recording[i] = gtk_check_button_new();
        gtk_toggle_button_set_active(
            GTK_TOGGLE_BUTTON(st->chk_recording[i]),
            cams[i].recording_enabled);
        gtk_grid_attach(GTK_GRID(st->grid), st->chk_recording[i],
                        1, row, 1, 1);

        /* Segment duration spinner */
        st->spn_segment[i] = ui_spin_int(
            cams[i].rec_segment_sec > 0 ? cams[i].rec_segment_sec : 300,
            30, 3600, 30, NULL, NULL);
        gtk_grid_attach(GTK_GRID(st->grid), st->spn_segment[i],
                        2, row, 1, 1);

        /* Status label */
        st->lbl_status[i] = ui_label("--", 0);
        gtk_grid_attach(GTK_GRID(st->grid), st->lbl_status[i],
                        3, row, 1, 1);
    }

    GtkWidget *scroll = ui_scrolled(st->grid);
    gtk_widget_set_vexpand(scroll, TRUE);
    gtk_box_pack_start(GTK_BOX(vbox), scroll, TRUE, TRUE, 0);

    /* Save button */
    st->btn_save = ui_button("Save", G_CALLBACK(on_save_clicked), st);
    gtk_box_pack_start(GTK_BOX(vbox), st->btn_save, FALSE, FALSE, 4);

    return vbox;
}

static void dvr_config_on_show(AppCtx *app)
{
    DvrConfigState *st = (DvrConfigState *)app->page_dvr_config_state;
    if (!st) return;

    /* Reload camera recording state into form */
    DbCamera cams[MAX_CAMERAS];
    int n = db_camera_load_all(app->db, cams, MAX_CAMERAS);

    for (int i = 0; i < st->num_cameras && i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (cams[j].id == st->cam_ids[i]) {
                gtk_toggle_button_set_active(
                    GTK_TOGGLE_BUTTON(st->chk_recording[i]),
                    cams[j].recording_enabled);
                gtk_spin_button_set_value(
                    GTK_SPIN_BUTTON(st->spn_segment[i]),
                    cams[j].rec_segment_sec > 0 ?
                    cams[j].rec_segment_sec : 300);
                break;
            }
        }
    }

    refresh_status(st);
    g_timeout_add(2000, status_timer_cb, st);
}

static void dvr_config_cleanup(AppCtx *app)
{
    if (app->page_dvr_config_state) {
        free(app->page_dvr_config_state);
        app->page_dvr_config_state = NULL;
    }
}

const PageDef page_dvr_config = {
    .id      = "dvr_config",
    .title   = "DVR Config",
    .build   = dvr_config_build,
    .on_show = dvr_config_on_show,
    .on_hide = NULL,
    .cleanup = dvr_config_cleanup,
};
