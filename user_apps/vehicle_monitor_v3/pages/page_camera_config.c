/*
 * pages/page_camera_config.c - Camera add/edit/delete configuration
 *
 * Form-based editor for camera settings (URL, scale, rotation, etc.)
 * Saves to SQLite, reloads into AppCtx.
 */
#include "../app.h"
#include "../core/database.h"
#include "../core/config.h"
#include "../core/camera.h"
#include "../ui/util.h"
#include <string.h>
#include <stdio.h>

typedef struct {
    AppCtx     *app;
    GtkWidget  *list_box;
    GtkWidget  *form_box;
    GtkWidget  *btn_add;
    GtkWidget  *btn_delete;
    GtkWidget  *btn_save;

    /* Form fields */
    GtkWidget  *ent_name;
    GtkWidget  *ent_url;
    GtkWidget  *chk_enabled;
    GtkWidget  *spn_scale;
    GtkWidget  *cmb_rotation;
    GtkWidget  *spn_grid_cols;
    GtkWidget  *spn_grid_rows;
    GtkWidget  *cmb_birdview;
    GtkWidget  *chk_force_sw;
    GtkWidget  *spn_threshold;
    GtkWidget  *spn_alpha;
    GtkWidget  *spn_trigger;
    GtkWidget  *spn_framestep;
    GtkWidget  *cmb_aspect;
    GtkWidget  *spn_aspect_w;
    GtkWidget  *spn_aspect_h;
    GtkWidget  *spn_sort;

    int         selected_id;  /* DB id of currently edited camera, 0=none */
} CamConfigState;

static void populate_list(CamConfigState *st);
static void load_form(CamConfigState *st, int cam_id);

static void on_list_row_selected(GtkListBox *box, GtkListBoxRow *row,
                                 gpointer data)
{
    (void)box;
    CamConfigState *st = (CamConfigState *)data;
    if (!row) {
        st->selected_id = 0;
        gtk_widget_set_sensitive(st->form_box, FALSE);
        gtk_widget_set_sensitive(st->btn_delete, FALSE);
        return;
    }
    int id = GPOINTER_TO_INT(g_object_get_data(G_OBJECT(row), "cam-id"));
    load_form(st, id);
    gtk_widget_set_sensitive(st->form_box, TRUE);
    gtk_widget_set_sensitive(st->btn_delete, TRUE);
}

static void on_add_clicked(GtkButton *btn, gpointer data)
{
    (void)btn;
    CamConfigState *st = (CamConfigState *)data;
    DbCamera cam;
    memset(&cam, 0, sizeof(cam));
    snprintf(cam.name, sizeof(cam.name), "Camera %d",
             st->app->num_cameras + 1);
    cam.enabled = 1;
    cam.scale = 1.0;
    cam.aspect_w = 16;
    cam.aspect_h = 9;
    cam.grid_cols = DEFAULT_GRID_COLS;
    cam.grid_rows = DEFAULT_GRID_ROWS;
    cam.threshold = 12;
    cam.alpha = 0.05;
    cam.trigger_pct = 5.0;
    cam.framestep = 5;
    cam.sort_order = st->app->num_cameras;
    cam.recording_enabled = 1;
    cam.rec_segment_sec = 300;

    int id = db_camera_insert(st->app->db, &cam);
    if (id > 0) {
        config_reload_cameras(st->app);
        populate_list(st);
        load_form(st, id);
    }
}

static void on_delete_clicked(GtkButton *btn, gpointer data)
{
    (void)btn;
    CamConfigState *st = (CamConfigState *)data;
    if (st->selected_id <= 0) return;

    /* Confirm */
    GtkWidget *dlg = gtk_message_dialog_new(
        GTK_WINDOW(st->app->window),
        GTK_DIALOG_MODAL, GTK_MESSAGE_QUESTION, GTK_BUTTONS_YES_NO,
        "Delete camera %d?", st->selected_id);
    int resp = gtk_dialog_run(GTK_DIALOG(dlg));
    gtk_widget_destroy(dlg);
    if (resp != GTK_RESPONSE_YES) return;

    int idx = camera_find(st->app, st->selected_id);

    db_camera_delete(st->app->db, st->selected_id);
    db_zone_clear_camera(st->app->db, st->selected_id);

    if (idx >= 0) {
        /* Stop cameras after idx - their worker threads hold pointers into
         * the array that become invalid after the shift */
        for (int i = idx + 1; i < st->app->num_cameras; i++)
            camera_stop(&st->app->cameras[i]);

        /* Fully free the deleted camera (stops thread + FFmpeg + buffers) */
        camera_free(&st->app->cameras[idx]);

        /* Destroy mutexes for cameras being shifted (can't memcpy a mutex) */
        for (int i = idx + 1; i < st->app->num_cameras; i++)
            pthread_mutex_destroy(&st->app->cameras[i].buf_mutex);

        /* Shift remaining cameras down */
        for (int i = idx; i < st->app->num_cameras - 1; i++)
            st->app->cameras[i] = st->app->cameras[i + 1];
        st->app->num_cameras--;
        memset(&st->app->cameras[st->app->num_cameras], 0,
               sizeof(CameraCtx));

        /* Reinit mutexes for shifted cameras */
        for (int i = idx; i < st->app->num_cameras; i++)
            pthread_mutex_init(&st->app->cameras[i].buf_mutex, NULL);

        /* Restart shifted cameras with correct thread pointers */
        for (int i = idx; i < st->app->num_cameras; i++) {
            if (st->app->cameras[i].enabled && st->app->cameras[i].fmt_ctx)
                camera_start(&st->app->cameras[i], st->app);
        }
    }

    st->selected_id = 0;
    populate_list(st);
    gtk_widget_set_sensitive(st->form_box, FALSE);
    gtk_widget_set_sensitive(st->btn_delete, FALSE);
}

static void on_save_clicked(GtkButton *btn, gpointer data)
{
    (void)btn;
    CamConfigState *st = (CamConfigState *)data;
    if (st->selected_id <= 0) return;

    DbCamera cam;
    cam.id = st->selected_id;
    snprintf(cam.name, sizeof(cam.name), "%s",
             gtk_entry_get_text(GTK_ENTRY(st->ent_name)));
    snprintf(cam.url, sizeof(cam.url), "%s",
             gtk_entry_get_text(GTK_ENTRY(st->ent_url)));
    cam.enabled = gtk_toggle_button_get_active(
        GTK_TOGGLE_BUTTON(st->chk_enabled)) ? 1 : 0;
    cam.scale = gtk_spin_button_get_value(GTK_SPIN_BUTTON(st->spn_scale));
    cam.rotation = gtk_combo_box_get_active(
        GTK_COMBO_BOX(st->cmb_rotation)) * 90;

    /* Aspect ratio from combo */
    {
        static const int presets[][2] = {
            {16,9}, {4,3}, {3,2}, {21,9}, {1,1}, {0,0}
        };
        int ai = gtk_combo_box_get_active(GTK_COMBO_BOX(st->cmb_aspect));
        if (ai >= 0 && ai <= 5) {
            cam.aspect_w = presets[ai][0];
            cam.aspect_h = presets[ai][1];
        } else {
            /* Custom */
            cam.aspect_w = gtk_spin_button_get_value_as_int(
                GTK_SPIN_BUTTON(st->spn_aspect_w));
            cam.aspect_h = gtk_spin_button_get_value_as_int(
                GTK_SPIN_BUTTON(st->spn_aspect_h));
        }
    }

    cam.grid_cols = gtk_spin_button_get_value_as_int(
        GTK_SPIN_BUTTON(st->spn_grid_cols));
    cam.grid_rows = gtk_spin_button_get_value_as_int(
        GTK_SPIN_BUTTON(st->spn_grid_rows));
    cam.birdview_pos = gtk_combo_box_get_active(
        GTK_COMBO_BOX(st->cmb_birdview));
    cam.force_sw = gtk_toggle_button_get_active(
        GTK_TOGGLE_BUTTON(st->chk_force_sw)) ? 1 : 0;
    cam.threshold = gtk_spin_button_get_value_as_int(
        GTK_SPIN_BUTTON(st->spn_threshold));
    cam.alpha = gtk_spin_button_get_value(GTK_SPIN_BUTTON(st->spn_alpha));
    cam.trigger_pct = gtk_spin_button_get_value(
        GTK_SPIN_BUTTON(st->spn_trigger));
    cam.framestep = gtk_spin_button_get_value_as_int(
        GTK_SPIN_BUTTON(st->spn_framestep));
    cam.sort_order = gtk_spin_button_get_value_as_int(
        GTK_SPIN_BUTTON(st->spn_sort));

    /* Preserve recording settings (managed by DVR Config page) */
    {
        DbCamera existing[MAX_CAMERAS];
        int en = db_camera_load_all(st->app->db, existing, MAX_CAMERAS);
        cam.recording_enabled = 1;
        cam.rec_segment_sec = 300;
        for (int i = 0; i < en; i++) {
            if (existing[i].id == cam.id) {
                cam.recording_enabled = existing[i].recording_enabled;
                cam.rec_segment_sec = existing[i].rec_segment_sec;
                break;
            }
        }
    }

    /* Snapshot current state for restart check */
    int cam_idx = camera_find(st->app, cam.id);
    int was_started = 0;
    char old_url[512] = {0};
    double old_scale = 0;
    int old_aw = 0, old_ah = 0, old_fsw = 0, old_rot = 0, old_en = 0;
    if (cam_idx >= 0) {
        CameraCtx *c = &st->app->cameras[cam_idx];
        was_started = c->started;
        strncpy(old_url, c->url, sizeof(old_url) - 1);
        old_scale = c->scale;
        old_aw = c->aspect_w;
        old_ah = c->aspect_h;
        old_fsw = c->force_sw;
        old_rot = c->rotation;
        old_en  = c->enabled;
    }

    db_camera_update(st->app->db, &cam);
    config_reload_cameras(st->app);
    populate_list(st);

    /* Restart camera if display-affecting config changed while running */
    if (cam_idx >= 0) {
        CameraCtx *c = &st->app->cameras[cam_idx];
        int need_restart = was_started && (
            strcmp(old_url, c->url) != 0 ||
            old_scale != c->scale ||
            old_aw != c->aspect_w ||
            old_ah != c->aspect_h ||
            old_fsw != c->force_sw ||
            old_rot != c->rotation);

        if (was_started && !c->enabled) {
            /* Disabled while running - stop */
            camera_free(c);
            pthread_mutex_init(&c->buf_mutex, NULL);
        } else if (need_restart) {
            /* Display config changed - full restart with new pipeline */
            camera_free(c);
            pthread_mutex_init(&c->buf_mutex, NULL);
            camera_start(c, st->app);
        } else if (!was_started && c->enabled && !old_en) {
            /* Re-enabled - start */
            camera_start(c, st->app);
        }
    }

    printf("Camera %d saved: %s\n", cam.id, cam.name);
}

static void populate_list(CamConfigState *st)
{
    /* Remove all rows */
    GList *children = gtk_container_get_children(
        GTK_CONTAINER(st->list_box));
    for (GList *l = children; l; l = l->next)
        gtk_container_remove(GTK_CONTAINER(st->list_box),
                             GTK_WIDGET(l->data));
    g_list_free(children);

    /* Load from DB */
    DbCamera cams[MAX_CAMERAS];
    int n = db_camera_load_all(st->app->db, cams, MAX_CAMERAS);

    for (int i = 0; i < n; i++) {
        char label[128];
        snprintf(label, sizeof(label), "%s%s",
                 cams[i].name,
                 cams[i].enabled ? "" : " (disabled)");
        GtkWidget *row = gtk_list_box_row_new();
        GtkWidget *lbl = gtk_label_new(label);
        gtk_label_set_xalign(GTK_LABEL(lbl), 0.0);
        gtk_widget_set_margin_start(lbl, 8);
        gtk_widget_set_margin_top(lbl, 4);
        gtk_widget_set_margin_bottom(lbl, 4);
        gtk_container_add(GTK_CONTAINER(row), lbl);
        g_object_set_data(G_OBJECT(row), "cam-id",
                          GINT_TO_POINTER(cams[i].id));
        gtk_list_box_insert(GTK_LIST_BOX(st->list_box), row, -1);
    }

    gtk_widget_show_all(st->list_box);
}

static void load_form(CamConfigState *st, int cam_id)
{
    st->selected_id = cam_id;

    DbCamera cams[MAX_CAMERAS];
    int n = db_camera_load_all(st->app->db, cams, MAX_CAMERAS);

    for (int i = 0; i < n; i++) {
        if (cams[i].id != cam_id) continue;
        DbCamera *c = &cams[i];

        gtk_entry_set_text(GTK_ENTRY(st->ent_name), c->name);
        gtk_entry_set_text(GTK_ENTRY(st->ent_url), c->url);
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(st->chk_enabled),
                                     c->enabled);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(st->spn_scale), c->scale);
        gtk_combo_box_set_active(GTK_COMBO_BOX(st->cmb_rotation),
                                 c->rotation / 90);

        /* Set aspect ratio combo */
        {
            static const int presets[][2] = {
                {16,9}, {4,3}, {3,2}, {21,9}, {1,1}, {0,0}
            };
            int found = 6; /* Custom */
            for (int p = 0; p < 6; p++) {
                if (c->aspect_w == presets[p][0] &&
                    c->aspect_h == presets[p][1]) {
                    found = p;
                    break;
                }
            }
            gtk_combo_box_set_active(GTK_COMBO_BOX(st->cmb_aspect), found);
            gtk_spin_button_set_value(
                GTK_SPIN_BUTTON(st->spn_aspect_w), c->aspect_w);
            gtk_spin_button_set_value(
                GTK_SPIN_BUTTON(st->spn_aspect_h), c->aspect_h);
            /* Show/hide custom spinners */
            gboolean custom = (found == 6);
            gtk_widget_set_visible(st->spn_aspect_w, custom);
            gtk_widget_set_visible(st->spn_aspect_h, custom);
        }

        gtk_spin_button_set_value(GTK_SPIN_BUTTON(st->spn_grid_cols),
                                  c->grid_cols);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(st->spn_grid_rows),
                                  c->grid_rows);
        gtk_combo_box_set_active(GTK_COMBO_BOX(st->cmb_birdview),
                                 c->birdview_pos);
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(st->chk_force_sw),
                                     c->force_sw);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(st->spn_threshold),
                                  c->threshold);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(st->spn_alpha), c->alpha);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(st->spn_trigger),
                                  c->trigger_pct);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(st->spn_framestep),
                                  c->framestep);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(st->spn_sort),
                                  c->sort_order);
        break;
    }
}

static GtkWidget *cam_config_build(AppCtx *app)
{
    CamConfigState *st = (CamConfigState *)calloc(1, sizeof(CamConfigState));
    st->app = app;
    app->page_camera_config_state = st;

    GtkWidget *hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
    gtk_widget_set_margin_start(hbox, 8);
    gtk_widget_set_margin_end(hbox, 8);
    gtk_widget_set_margin_top(hbox, 8);
    gtk_widget_set_margin_bottom(hbox, 8);

    /* Left panel: camera list + buttons */
    GtkWidget *left = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
    gtk_widget_set_size_request(left, 200, -1);

    st->list_box = gtk_list_box_new();
    gtk_list_box_set_selection_mode(GTK_LIST_BOX(st->list_box),
                                   GTK_SELECTION_SINGLE);
    g_signal_connect(st->list_box, "row-selected",
                     G_CALLBACK(on_list_row_selected), st);

    GtkWidget *scroll_list = ui_scrolled(st->list_box);
    gtk_widget_set_vexpand(scroll_list, TRUE);
    gtk_box_pack_start(GTK_BOX(left), scroll_list, TRUE, TRUE, 0);

    GtkWidget *btn_bar = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 4);
    st->btn_add = ui_button("Add", G_CALLBACK(on_add_clicked), st);
    st->btn_delete = ui_button("Delete", G_CALLBACK(on_delete_clicked), st);
    gtk_widget_set_sensitive(st->btn_delete, FALSE);
    gtk_box_pack_start(GTK_BOX(btn_bar), st->btn_add, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(btn_bar), st->btn_delete, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(left), btn_bar, FALSE, FALSE, 0);

    gtk_box_pack_start(GTK_BOX(hbox), left, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(hbox),
                       gtk_separator_new(GTK_ORIENTATION_VERTICAL),
                       FALSE, FALSE, 0);

    /* Right panel: form */
    st->form_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
    gtk_widget_set_sensitive(st->form_box, FALSE);
    gtk_widget_set_hexpand(st->form_box, TRUE);

    GtkWidget *grid = ui_grid();
    int row = 0;

    st->ent_name = ui_entry("", 63, NULL, NULL);
    row = ui_grid_row(grid, row, "Name", st->ent_name);

    st->ent_url = ui_entry("", 511, NULL, NULL);
    row = ui_grid_row(grid, row, "URL", st->ent_url);

    st->chk_enabled = gtk_check_button_new_with_label("Enabled");
    gtk_grid_attach(GTK_GRID(grid), st->chk_enabled, 0, row++, 2, 1);

    st->spn_scale = ui_spin_float(1.0, 0.1, 4.0, 0.1, 1, NULL, NULL);
    row = ui_grid_row(grid, row, "Scale", st->spn_scale);

    /* Aspect ratio combo */
    st->cmb_aspect = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(st->cmb_aspect), "16:9");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(st->cmb_aspect), "4:3");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(st->cmb_aspect), "3:2");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(st->cmb_aspect), "21:9");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(st->cmb_aspect), "1:1");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(st->cmb_aspect),
                                   "Source");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(st->cmb_aspect),
                                   "Custom");
    gtk_combo_box_set_active(GTK_COMBO_BOX(st->cmb_aspect), 0);
    row = ui_grid_row(grid, row, "Aspect Ratio", st->cmb_aspect);

    /* Custom aspect W:H spinners (hidden by default) */
    GtkWidget *aspect_hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 4);
    st->spn_aspect_w = ui_spin_int(16, 1, 100, 1, NULL, NULL);
    st->spn_aspect_h = ui_spin_int(9, 1, 100, 1, NULL, NULL);
    gtk_box_pack_start(GTK_BOX(aspect_hbox), st->spn_aspect_w, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(aspect_hbox), gtk_label_new(":"),
                       FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(aspect_hbox), st->spn_aspect_h, TRUE, TRUE, 0);
    gtk_widget_set_no_show_all(st->spn_aspect_w, TRUE);
    gtk_widget_set_no_show_all(st->spn_aspect_h, TRUE);
    row = ui_grid_row(grid, row, "Custom W:H", aspect_hbox);

    /* Show/hide custom spinners when aspect combo changes */
    g_signal_connect_swapped(st->cmb_aspect, "changed",
        G_CALLBACK(gtk_widget_queue_draw), aspect_hbox);

    /* Rotation combo */
    st->cmb_rotation = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(st->cmb_rotation), "0");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(st->cmb_rotation), "90");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(st->cmb_rotation), "180");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(st->cmb_rotation), "270");
    gtk_combo_box_set_active(GTK_COMBO_BOX(st->cmb_rotation), 0);
    row = ui_grid_row(grid, row, "Rotation", st->cmb_rotation);

    st->spn_grid_cols = ui_spin_int(DEFAULT_GRID_COLS, 4, MAX_GRID, 1,
                                    NULL, NULL);
    row = ui_grid_row(grid, row, "Grid Cols", st->spn_grid_cols);

    st->spn_grid_rows = ui_spin_int(DEFAULT_GRID_ROWS, 3, MAX_GRID, 1,
                                    NULL, NULL);
    row = ui_grid_row(grid, row, "Grid Rows", st->spn_grid_rows);

    /* Birdview position combo */
    st->cmb_birdview = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(st->cmb_birdview),
                                   "None");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(st->cmb_birdview),
                                   "Front");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(st->cmb_birdview),
                                   "Rear");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(st->cmb_birdview),
                                   "Left");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(st->cmb_birdview),
                                   "Right");
    gtk_combo_box_set_active(GTK_COMBO_BOX(st->cmb_birdview), 0);
    row = ui_grid_row(grid, row, "Bird View", st->cmb_birdview);

    st->chk_force_sw = gtk_check_button_new_with_label("Force SW decode");
    gtk_grid_attach(GTK_GRID(grid), st->chk_force_sw, 0, row++, 2, 1);

    /* Motion parameters */
    gtk_grid_attach(GTK_GRID(grid), ui_label("Motion Settings", 1),
                    0, row++, 2, 1);

    st->spn_threshold = ui_spin_int(12, 1, 100, 1, NULL, NULL);
    row = ui_grid_row(grid, row, "Threshold", st->spn_threshold);

    st->spn_alpha = ui_spin_float(0.05, 0.001, 0.5, 0.005, 3, NULL, NULL);
    row = ui_grid_row(grid, row, "BG Alpha", st->spn_alpha);

    st->spn_trigger = ui_spin_float(5.0, 0.1, 100.0, 0.5, 1, NULL, NULL);
    row = ui_grid_row(grid, row, "Trigger %", st->spn_trigger);

    st->spn_framestep = ui_spin_int(5, 1, 50, 1, NULL, NULL);
    row = ui_grid_row(grid, row, "Frame Step", st->spn_framestep);

    st->spn_sort = ui_spin_int(0, 0, 99, 1, NULL, NULL);
    row = ui_grid_row(grid, row, "Sort Order", st->spn_sort);

    (void)row;

    GtkWidget *scroll_form = ui_scrolled(grid);
    gtk_box_pack_start(GTK_BOX(st->form_box), scroll_form, TRUE, TRUE, 0);

    /* Save button */
    st->btn_save = ui_button("Save", G_CALLBACK(on_save_clicked), st);
    gtk_box_pack_start(GTK_BOX(st->form_box), st->btn_save, FALSE, FALSE, 4);

    gtk_box_pack_start(GTK_BOX(hbox), st->form_box, TRUE, TRUE, 0);

    /* Populate camera list */
    populate_list(st);

    return hbox;
}

static void cam_config_cleanup(AppCtx *app)
{
    if (app->page_camera_config_state) {
        free(app->page_camera_config_state);
        app->page_camera_config_state = NULL;
    }
}

const PageDef page_camera_config = {
    .id      = "camera_config",
    .title   = "Camera Config",
    .build   = cam_config_build,
    .on_show = NULL,
    .on_hide = NULL,
    .cleanup = cam_config_cleanup,
};
