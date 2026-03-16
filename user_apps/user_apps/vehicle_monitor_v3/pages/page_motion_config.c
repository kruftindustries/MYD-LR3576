/*
 * pages/page_motion_config.c - Zone-based motion configuration
 *
 * Camera selector, live feed with 48x27 grid overlay,
 * click/drag to select cells, assign to action groups with colors.
 * Per-cell % text, motion parameter spinners.
 */
#include "../app.h"
#include "../core/database.h"
#include "../core/camera.h"
#include "../core/config.h"
#include "../ui/util.h"
#include <cairo.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Preview modes */
#define PREVIEW_LIVE     0
#define PREVIEW_MOTION   1
#define PREVIEW_OVERLAY  2

typedef struct {
    AppCtx     *app;
    GtkWidget  *camera_combo;
    GtkWidget  *draw_area;
    GtkWidget  *group_combo;
    GtkWidget  *cmb_preview;
    GtkWidget  *lbl_info;
    GtkWidget  *spn_threshold;
    GtkWidget  *spn_alpha;
    GtkWidget  *spn_trigger;
    GtkWidget  *spn_min_blob;
    GtkWidget  *spn_framestep;
    GtkWidget  *btn_clear;

    /* Action group editing */
    GtkWidget  *ent_group_name;
    GtkWidget  *btn_group_color;
    GtkWidget  *chk_flash_led;
    GtkWidget  *chk_email;
    GtkWidget  *chk_beep;
    GtkWidget  *btn_add_group;
    GtkWidget  *btn_del_group;
    GtkWidget  *btn_save_group;

    int         sel_cam_idx;     /* index into app->cameras[] */
    int         preview_mode;    /* PREVIEW_* */
    int         active;
    guint       redraw_timer;

    /* Drag selection */
    int         dragging;
    int         drag_start_col;
    int         drag_start_row;
    int         drag_end_col;
    int         drag_end_row;

    /* Fitted rect (for click mapping) */
    double      fit_x, fit_y, fit_w, fit_h;

    /* Loaded zones for current camera */
    int         zone_map[MAX_GRID][MAX_GRID]; /* action_group_id per cell */

    /* Action groups */
    DbActionGroup groups[MAX_ACTION_GROUPS];
    int           num_groups;
} MotionConfigState;

static void load_zones(MotionConfigState *st)
{
    memset(st->zone_map, 0, sizeof(st->zone_map));
    if (st->sel_cam_idx < 0) return;

    CameraCtx *cam = &st->app->cameras[st->sel_cam_idx];
    DbZone zones[MAX_GRID * MAX_GRID];
    int nz = db_zone_load_camera(st->app->db, cam->id, zones,
                                 MAX_GRID * MAX_GRID);
    for (int i = 0; i < nz; i++) {
        int c = zones[i].cell_col;
        int r = zones[i].cell_row;
        if (c >= 0 && c < MAX_GRID && r >= 0 && r < MAX_GRID)
            st->zone_map[r][c] = zones[i].action_group_id;
    }
}

static void load_groups(MotionConfigState *st)
{
    st->num_groups = db_action_group_load_all(st->app->db, st->groups,
                                              MAX_ACTION_GROUPS);
}

static void save_zones(MotionConfigState *st)
{
    if (st->sel_cam_idx < 0) return;
    CameraCtx *cam = &st->app->cameras[st->sel_cam_idx];
    int gcols = cam->grid_cols;
    int grows = cam->grid_rows;

    db_zone_clear_camera(st->app->db, cam->id);

    for (int r = 0; r < grows; r++) {
        for (int c = 0; c < gcols; c++) {
            if (st->zone_map[r][c] > 0) {
                db_zone_set(st->app->db, cam->id, c, r,
                            st->zone_map[r][c]);
            }
        }
    }
}

static int get_selected_group_id(MotionConfigState *st)
{
    int idx = gtk_combo_box_get_active(GTK_COMBO_BOX(st->group_combo));
    if (idx >= 0 && idx < st->num_groups)
        return st->groups[idx].id;
    return 1; /* default */
}

static void get_group_color(MotionConfigState *st, int group_id,
                            double *r, double *g, double *b)
{
    for (int i = 0; i < st->num_groups; i++) {
        if (st->groups[i].id == group_id) {
            ui_parse_color(st->groups[i].color, r, g, b);
            return;
        }
    }
    *r = 1.0; *g = 0.4; *b = 0.0; /* fallback orange */
}

static gboolean on_draw_motion_config(GtkWidget *widget, cairo_t *cr,
                                      gpointer data)
{
    MotionConfigState *st = (MotionConfigState *)data;
    int aw = gtk_widget_get_allocated_width(widget);
    int ah = gtk_widget_get_allocated_height(widget);

    cairo_set_source_rgb(cr, 0.1, 0.1, 0.1);
    cairo_paint(cr);

    if (st->sel_cam_idx < 0) {
        cairo_set_source_rgb(cr, 0.5, 0.5, 0.5);
        cairo_set_font_size(cr, 14);
        cairo_move_to(cr, 10, ah / 2);
        cairo_show_text(cr, "Select a camera");
        return FALSE;
    }

    CameraCtx *cam = &st->app->cameras[st->sel_cam_idx];

    if (cam->disp_w == 0 || !cam->started) {
        cairo_set_source_rgb(cr, 0.5, 0.5, 0.5);
        cairo_set_font_size(cr, 14);
        cairo_move_to(cr, 10, ah / 2);
        cairo_show_text(cr, "Camera not running");
        return FALSE;
    }

    /* Copy shared data */
    int gc, gr;
    float fps;
    float grid_pcts[MAX_GRID * MAX_GRID];

    pthread_mutex_lock(&cam->buf_mutex);
    memcpy(cam->draw_color, cam->color_rgb, cam->disp_size);
    memcpy(cam->draw_motion, cam->motion_rgb, cam->disp_size);
    gc = cam->shared_grid_cols;
    gr = cam->shared_grid_rows;
    fps = cam->shared_fps;
    memcpy(grid_pcts, cam->shared_grid_pcts, gc * gr * sizeof(float));
    pthread_mutex_unlock(&cam->buf_mutex);

    /* Select source buffer based on preview mode */
    uint8_t *render_buf = cam->draw_color;
    int mode = st->preview_mode;

    if (mode == PREVIEW_MOTION) {
        render_buf = cam->draw_motion;
    } else if (mode == PREVIEW_OVERLAY) {
        /* Composite: overlay motion onto live where motion is non-black */
        int npix = cam->disp_w * cam->disp_h;
        for (int i = 0; i < npix; i++) {
            int off = i * 3;
            uint8_t mr = cam->draw_motion[off];
            uint8_t mg = cam->draw_motion[off + 1];
            uint8_t mb = cam->draw_motion[off + 2];
            if (mr | mg | mb) {
                /* Blend motion over live (50/50 mix) */
                cam->draw_color[off]     = (cam->draw_color[off]     + mr) >> 1;
                cam->draw_color[off + 1] = (cam->draw_color[off + 1] + mg) >> 1;
                cam->draw_color[off + 2] = (cam->draw_color[off + 2] + mb) >> 1;
            }
        }
        render_buf = cam->draw_color;
    }

    /* Render feed (fitted / letterboxed) */
    GdkPixbuf *pb = gdk_pixbuf_new_from_data(
        render_buf, GDK_COLORSPACE_RGB, FALSE, 8,
        cam->disp_w, cam->disp_h, cam->disp_w * 3, NULL, NULL);

    double fx, fy, fw, fh;
    ui_draw_pixbuf_fitted(cr, pb, cam->disp_w, cam->disp_h, aw, ah,
                          &fx, &fy, &fw, &fh);
    g_object_unref(pb);

    /* Store for click mapping */
    st->fit_x = fx; st->fit_y = fy;
    st->fit_w = fw; st->fit_h = fh;

    if (gc <= 0 || gr <= 0)
        return FALSE;

    double cw = fw / gc;
    double ch = fh / gr;

    /* Zone colors (semi-transparent fill for assigned cells) */
    for (int row = 0; row < gr; row++) {
        for (int col = 0; col < gc; col++) {
            int gid = st->zone_map[row][col];
            if (gid > 0) {
                double r, g, b;
                get_group_color(st, gid, &r, &g, &b);
                cairo_set_source_rgba(cr, r, g, b, 0.25);
                cairo_rectangle(cr, fx + col * cw, fy + row * ch, cw, ch);
                cairo_fill(cr);
            }
        }
    }

    /* Drag selection highlight */
    if (st->dragging) {
        int c0 = st->drag_start_col < st->drag_end_col ?
                 st->drag_start_col : st->drag_end_col;
        int c1 = st->drag_start_col > st->drag_end_col ?
                 st->drag_start_col : st->drag_end_col;
        int r0 = st->drag_start_row < st->drag_end_row ?
                 st->drag_start_row : st->drag_end_row;
        int r1 = st->drag_start_row > st->drag_end_row ?
                 st->drag_start_row : st->drag_end_row;

        cairo_set_source_rgba(cr, 1, 1, 0, 0.3);
        cairo_rectangle(cr, fx + c0 * cw, fy + r0 * ch,
                        (c1 - c0 + 1) * cw, (r1 - r0 + 1) * ch);
        cairo_fill(cr);

        cairo_set_source_rgba(cr, 1, 1, 0, 0.8);
        cairo_set_line_width(cr, 2.0);
        cairo_rectangle(cr, fx + c0 * cw, fy + r0 * ch,
                        (c1 - c0 + 1) * cw, (r1 - r0 + 1) * ch);
        cairo_stroke(cr);
    }

    /* Grid lines */
    cairo_set_source_rgba(cr, 1, 1, 1, 0.2);
    cairo_set_line_width(cr, 0.5);
    for (int i = 1; i < gc; i++) {
        cairo_move_to(cr, fx + i * cw, fy);
        cairo_line_to(cr, fx + i * cw, fy + fh);
    }
    for (int i = 1; i < gr; i++) {
        cairo_move_to(cr, fx, fy + i * ch);
        cairo_line_to(cr, fx + fw, fy + i * ch);
    }
    cairo_stroke(cr);

    /* Per-cell percentage text */
    double font_sz = cw / 4.5;
    if (font_sz < 7) font_sz = 7;
    if (font_sz > 13) font_sz = 13;
    cairo_set_font_size(cr, font_sz);

    float trig = cam->trigger_pct;
    for (int row = 0; row < gr; row++) {
        for (int col = 0; col < gc; col++) {
            float pct = grid_pcts[row * gc + col];
            if (pct < 0.5f) continue; /* skip near-zero */

            double cx = fx + col * cw + cw / 2;
            double cy = fy + row * ch + ch / 2;

            char buf[8];
            snprintf(buf, sizeof(buf), "%.0f", pct);
            cairo_text_extents_t ext;
            cairo_text_extents(cr, buf, &ext);

            /* Text background */
            cairo_set_source_rgba(cr, 0, 0, 0, 0.5);
            cairo_rectangle(cr, cx - ext.width / 2 - 1,
                            cy - ext.height / 2 - 1,
                            ext.width + 2, ext.height + 2);
            cairo_fill(cr);

            if (pct >= trig)
                cairo_set_source_rgb(cr, 1, 1, 0);
            else
                cairo_set_source_rgba(cr, 1, 1, 1, 0.7);

            cairo_move_to(cr, cx - ext.width / 2, cy + ext.height / 2);
            cairo_show_text(cr, buf);
        }
    }

    /* Info bar */
    cairo_set_source_rgba(cr, 0, 0, 0, 0.6);
    cairo_rectangle(cr, 0, ah - 22, aw, 22);
    cairo_fill(cr);

    cairo_set_source_rgb(cr, 1, 1, 1);
    cairo_set_font_size(cr, 12);
    char info[128];
    snprintf(info, sizeof(info), "%s  Grid: %dx%d  %.1f fps",
             cam->name, gc, gr, fps);
    cairo_move_to(cr, 6, ah - 6);
    cairo_show_text(cr, info);

    return FALSE;
}

static gboolean on_button_press(GtkWidget *widget, GdkEventButton *ev,
                                gpointer data)
{
    MotionConfigState *st = (MotionConfigState *)data;
    if (st->sel_cam_idx < 0) return FALSE;

    CameraCtx *cam = &st->app->cameras[st->sel_cam_idx];
    (void)widget;
    int gc = cam->grid_cols;
    int gr = cam->grid_rows;
    if (gc <= 0 || gr <= 0 || st->fit_w < 1 || st->fit_h < 1) return FALSE;

    int col = (int)((ev->x - st->fit_x) * gc / st->fit_w);
    int row = (int)((ev->y - st->fit_y) * gr / st->fit_h);
    if (col < 0) col = 0;
    if (col >= gc) col = gc - 1;
    if (row < 0) row = 0;
    if (row >= gr) row = gr - 1;

    if (ev->button == 1) {
        st->dragging = 1;
        st->drag_start_col = col;
        st->drag_start_row = row;
        st->drag_end_col = col;
        st->drag_end_row = row;
    } else if (ev->button == 3) {
        /* Right-click: clear cell */
        st->zone_map[row][col] = 0;
        save_zones(st);
        gtk_widget_queue_draw(widget);
    }

    return TRUE;
}

static gboolean on_motion_notify(GtkWidget *widget, GdkEventMotion *ev,
                                 gpointer data)
{
    MotionConfigState *st = (MotionConfigState *)data;
    if (!st->dragging || st->sel_cam_idx < 0) return FALSE;

    CameraCtx *cam = &st->app->cameras[st->sel_cam_idx];
    (void)widget;
    int gc = cam->grid_cols;
    int gr = cam->grid_rows;
    if (st->fit_w < 1 || st->fit_h < 1) return FALSE;

    int col = (int)((ev->x - st->fit_x) * gc / st->fit_w);
    int row = (int)((ev->y - st->fit_y) * gr / st->fit_h);
    if (col < 0) col = 0;
    if (col >= gc) col = gc - 1;
    if (row < 0) row = 0;
    if (row >= gr) row = gr - 1;

    st->drag_end_col = col;
    st->drag_end_row = row;
    gtk_widget_queue_draw(widget);

    return TRUE;
}

static gboolean on_button_release(GtkWidget *widget, GdkEventButton *ev,
                                  gpointer data)
{
    (void)ev;
    MotionConfigState *st = (MotionConfigState *)data;
    if (!st->dragging || st->sel_cam_idx < 0) return FALSE;

    st->dragging = 0;

    int c0 = st->drag_start_col < st->drag_end_col ?
             st->drag_start_col : st->drag_end_col;
    int c1 = st->drag_start_col > st->drag_end_col ?
             st->drag_start_col : st->drag_end_col;
    int r0 = st->drag_start_row < st->drag_end_row ?
             st->drag_start_row : st->drag_end_row;
    int r1 = st->drag_start_row > st->drag_end_row ?
             st->drag_start_row : st->drag_end_row;

    int gid = get_selected_group_id(st);

    for (int r = r0; r <= r1; r++)
        for (int c = c0; c <= c1; c++)
            st->zone_map[r][c] = gid;

    save_zones(st);
    gtk_widget_queue_draw(widget);

    return TRUE;
}

static gboolean redraw_timer_cb(gpointer data)
{
    MotionConfigState *st = (MotionConfigState *)data;
    if (!st->active)
        return G_SOURCE_REMOVE;
    if (st->sel_cam_idx >= 0 &&
        st->app->cameras[st->sel_cam_idx].buf_updated) {
        st->app->cameras[st->sel_cam_idx].buf_updated = 0;
        gtk_widget_queue_draw(st->draw_area);
    }
    return G_SOURCE_CONTINUE;
}

static void on_camera_changed(GtkComboBox *combo, gpointer data)
{
    MotionConfigState *st = (MotionConfigState *)data;
    int idx = gtk_combo_box_get_active(combo);
    if (idx < 0 || idx >= st->app->num_cameras) {
        st->sel_cam_idx = -1;
        return;
    }
    st->sel_cam_idx = idx;

    CameraCtx *cam = &st->app->cameras[idx];

    /* Update spinners to match camera settings */
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(st->spn_threshold),
                              cam->threshold);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(st->spn_alpha), cam->alpha);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(st->spn_trigger),
                              cam->trigger_pct);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(st->spn_min_blob),
                              cam->min_blob_cells);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(st->spn_framestep),
                              cam->framestep);

    /* Start camera if needed */
    if (!cam->started && cam->enabled)
        camera_start(cam, st->app);

    load_zones(st);
    gtk_widget_queue_draw(st->draw_area);
}

static void on_threshold_changed(GtkSpinButton *spin, gpointer data)
{
    MotionConfigState *st = (MotionConfigState *)data;
    if (st->sel_cam_idx >= 0)
        st->app->cameras[st->sel_cam_idx].threshold =
            gtk_spin_button_get_value_as_int(spin);
}

static void on_alpha_changed(GtkSpinButton *spin, gpointer data)
{
    MotionConfigState *st = (MotionConfigState *)data;
    if (st->sel_cam_idx >= 0)
        st->app->cameras[st->sel_cam_idx].alpha =
            (float)gtk_spin_button_get_value(spin);
}

static void on_trigger_changed(GtkSpinButton *spin, gpointer data)
{
    MotionConfigState *st = (MotionConfigState *)data;
    if (st->sel_cam_idx >= 0)
        st->app->cameras[st->sel_cam_idx].trigger_pct =
            (float)gtk_spin_button_get_value(spin);
}

static void on_min_blob_changed(GtkSpinButton *spin, gpointer data)
{
    MotionConfigState *st = (MotionConfigState *)data;
    if (st->sel_cam_idx >= 0)
        st->app->cameras[st->sel_cam_idx].min_blob_cells =
            gtk_spin_button_get_value_as_int(spin);
}

static void on_framestep_changed(GtkSpinButton *spin, gpointer data)
{
    MotionConfigState *st = (MotionConfigState *)data;
    if (st->sel_cam_idx >= 0)
        st->app->cameras[st->sel_cam_idx].framestep =
            gtk_spin_button_get_value_as_int(spin);
}

static void on_preview_changed(GtkComboBox *combo, gpointer data)
{
    MotionConfigState *st = (MotionConfigState *)data;
    st->preview_mode = gtk_combo_box_get_active(combo);
}

static void on_clear_zones(GtkButton *btn, gpointer data)
{
    (void)btn;
    MotionConfigState *st = (MotionConfigState *)data;
    if (st->sel_cam_idx < 0) return;

    CameraCtx *cam = &st->app->cameras[st->sel_cam_idx];
    db_zone_clear_camera(st->app->db, cam->id);
    memset(st->zone_map, 0, sizeof(st->zone_map));
    gtk_widget_queue_draw(st->draw_area);
}

static void populate_camera_combo(MotionConfigState *st)
{
    gtk_combo_box_text_remove_all(GTK_COMBO_BOX_TEXT(st->camera_combo));
    for (int i = 0; i < st->app->num_cameras; i++) {
        gtk_combo_box_text_append_text(
            GTK_COMBO_BOX_TEXT(st->camera_combo),
            st->app->cameras[i].name);
    }
}

static void populate_group_combo(MotionConfigState *st)
{
    gtk_combo_box_text_remove_all(GTK_COMBO_BOX_TEXT(st->group_combo));
    load_groups(st);
    for (int i = 0; i < st->num_groups; i++) {
        char label[80];
        snprintf(label, sizeof(label), "%s (%s)",
                 st->groups[i].name, st->groups[i].color);
        gtk_combo_box_text_append_text(
            GTK_COMBO_BOX_TEXT(st->group_combo), label);
    }
    if (st->num_groups > 0)
        gtk_combo_box_set_active(GTK_COMBO_BOX(st->group_combo), 0);
}

static void load_group_fields(MotionConfigState *st)
{
    int idx = gtk_combo_box_get_active(GTK_COMBO_BOX(st->group_combo));
    if (idx < 0 || idx >= st->num_groups) {
        gtk_entry_set_text(GTK_ENTRY(st->ent_group_name), "");
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(st->chk_flash_led), FALSE);
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(st->chk_email), FALSE);
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(st->chk_beep), FALSE);
        gtk_widget_set_sensitive(st->btn_del_group, FALSE);
        gtk_widget_set_sensitive(st->btn_save_group, FALSE);
        return;
    }
    DbActionGroup *g = &st->groups[idx];
    gtk_entry_set_text(GTK_ENTRY(st->ent_group_name), g->name);

    GdkRGBA rgba;
    if (gdk_rgba_parse(&rgba, g->color))
        gtk_color_chooser_set_rgba(GTK_COLOR_CHOOSER(st->btn_group_color), &rgba);

    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(st->chk_flash_led),
                                 (g->actions & ACTION_FLASH_LED) != 0);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(st->chk_email),
                                 (g->actions & ACTION_EMAIL) != 0);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(st->chk_beep),
                                 (g->actions & ACTION_BEEP) != 0);

    /* Protect default group from deletion */
    gtk_widget_set_sensitive(st->btn_del_group, g->id != 1);
    gtk_widget_set_sensitive(st->btn_save_group, TRUE);
}

static void on_group_combo_changed(GtkComboBox *combo, gpointer data)
{
    (void)combo;
    load_group_fields((MotionConfigState *)data);
}

static void on_add_group(GtkButton *btn, gpointer data)
{
    (void)btn;
    MotionConfigState *st = (MotionConfigState *)data;
    char name[64];
    snprintf(name, sizeof(name), "Group %d", st->num_groups + 1);
    int id = db_action_group_insert(st->app->db, name, "#00CC00", 0);
    if (id > 0) {
        populate_group_combo(st);
        /* Select the new group */
        gtk_combo_box_set_active(GTK_COMBO_BOX(st->group_combo),
                                 st->num_groups - 1);
    }
}

static void on_delete_group(GtkButton *btn, gpointer data)
{
    (void)btn;
    MotionConfigState *st = (MotionConfigState *)data;
    int gid = get_selected_group_id(st);
    if (gid <= 1) return; /* protect default */

    db_action_group_delete(st->app->db, gid);
    populate_group_combo(st);
}

static void on_save_group(GtkButton *btn, gpointer data)
{
    (void)btn;
    MotionConfigState *st = (MotionConfigState *)data;
    int idx = gtk_combo_box_get_active(GTK_COMBO_BOX(st->group_combo));
    if (idx < 0 || idx >= st->num_groups) return;

    int gid = st->groups[idx].id;
    const char *name = gtk_entry_get_text(GTK_ENTRY(st->ent_group_name));

    GdkRGBA rgba;
    gtk_color_chooser_get_rgba(GTK_COLOR_CHOOSER(st->btn_group_color), &rgba);
    char color[16];
    snprintf(color, sizeof(color), "#%02X%02X%02X",
             (int)(rgba.red * 255), (int)(rgba.green * 255),
             (int)(rgba.blue * 255));

    int actions = 0;
    if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(st->chk_flash_led)))
        actions |= ACTION_FLASH_LED;
    if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(st->chk_email)))
        actions |= ACTION_EMAIL;
    if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(st->chk_beep)))
        actions |= ACTION_BEEP;

    db_action_group_update(st->app->db, gid, name, color, actions);

    int prev = idx;
    populate_group_combo(st);
    if (prev < st->num_groups)
        gtk_combo_box_set_active(GTK_COMBO_BOX(st->group_combo), prev);
}

static GtkWidget *motion_config_build(AppCtx *app)
{
    MotionConfigState *st = (MotionConfigState *)calloc(1,
                                sizeof(MotionConfigState));
    st->app = app;
    st->sel_cam_idx = -1;
    app->page_motion_config_state = st;

    GtkWidget *hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 4);
    gtk_widget_set_margin_start(hbox, 4);
    gtk_widget_set_margin_end(hbox, 4);
    gtk_widget_set_margin_top(hbox, 4);
    gtk_widget_set_margin_bottom(hbox, 4);

    /* Left: video with grid overlay */
    GtkWidget *left = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
    gtk_widget_set_hexpand(left, TRUE);
    gtk_widget_set_vexpand(left, TRUE);

    st->draw_area = gtk_drawing_area_new();
    gtk_widget_set_size_request(st->draw_area, 640, 360);
    gtk_widget_set_hexpand(st->draw_area, TRUE);
    gtk_widget_set_vexpand(st->draw_area, TRUE);
    gtk_widget_add_events(st->draw_area,
                          GDK_BUTTON_PRESS_MASK |
                          GDK_BUTTON_RELEASE_MASK |
                          GDK_POINTER_MOTION_MASK);
    g_signal_connect(st->draw_area, "draw",
                     G_CALLBACK(on_draw_motion_config), st);
    g_signal_connect(st->draw_area, "button-press-event",
                     G_CALLBACK(on_button_press), st);
    g_signal_connect(st->draw_area, "button-release-event",
                     G_CALLBACK(on_button_release), st);
    g_signal_connect(st->draw_area, "motion-notify-event",
                     G_CALLBACK(on_motion_notify), st);

    gtk_box_pack_start(GTK_BOX(left), st->draw_area, TRUE, TRUE, 0);

    st->lbl_info = ui_label("Select a camera to configure zones", 0);
    gtk_box_pack_start(GTK_BOX(left), st->lbl_info, FALSE, FALSE, 0);

    gtk_box_pack_start(GTK_BOX(hbox), left, TRUE, TRUE, 0);

    /* Right: controls panel */
    GtkWidget *right = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
    gtk_widget_set_margin_start(right, 8);
    gtk_widget_set_margin_end(right, 4);

    /* Camera selector */
    gtk_box_pack_start(GTK_BOX(right), ui_label("Camera", 1),
                       FALSE, FALSE, 0);
    st->camera_combo = gtk_combo_box_text_new();
    g_signal_connect(st->camera_combo, "changed",
                     G_CALLBACK(on_camera_changed), st);
    gtk_box_pack_start(GTK_BOX(right), st->camera_combo, FALSE, FALSE, 0);

    /* Preview mode */
    gtk_box_pack_start(GTK_BOX(right), ui_label("Preview", 1),
                       FALSE, FALSE, 4);
    st->cmb_preview = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(st->cmb_preview),
                                   "Live");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(st->cmb_preview),
                                   "Motion");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(st->cmb_preview),
                                   "Live + Motion");
    gtk_combo_box_set_active(GTK_COMBO_BOX(st->cmb_preview), 0);
    g_signal_connect(st->cmb_preview, "changed",
        G_CALLBACK(on_preview_changed), st);
    gtk_box_pack_start(GTK_BOX(right), st->cmb_preview, FALSE, FALSE, 0);

    /* Action group selector */
    gtk_box_pack_start(GTK_BOX(right), ui_label("Action Group", 1),
                       FALSE, FALSE, 4);
    st->group_combo = gtk_combo_box_text_new();
    g_signal_connect(st->group_combo, "changed",
                     G_CALLBACK(on_group_combo_changed), st);
    gtk_box_pack_start(GTK_BOX(right), st->group_combo, FALSE, FALSE, 0);

    /* Add/Delete group buttons */
    GtkWidget *grp_btns = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 4);
    st->btn_add_group = ui_button("Add", G_CALLBACK(on_add_group), st);
    st->btn_del_group = ui_button("Delete", G_CALLBACK(on_delete_group), st);
    gtk_widget_set_sensitive(st->btn_del_group, FALSE);
    gtk_box_pack_start(GTK_BOX(grp_btns), st->btn_add_group, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(grp_btns), st->btn_del_group, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(right), grp_btns, FALSE, FALSE, 0);

    /* Group name */
    st->ent_group_name = ui_entry("", 63, NULL, NULL);
    gtk_box_pack_start(GTK_BOX(right), st->ent_group_name, FALSE, FALSE, 0);

    /* Group color */
    GdkRGBA default_rgba = {1.0, 0.4, 0.0, 1.0};
    st->btn_group_color = gtk_color_button_new_with_rgba(&default_rgba);
    gtk_box_pack_start(GTK_BOX(right), st->btn_group_color, FALSE, FALSE, 0);

    /* Action checkboxes */
    st->chk_flash_led = gtk_check_button_new_with_label("Flash LED");
    st->chk_email = gtk_check_button_new_with_label("Email");
    st->chk_beep = gtk_check_button_new_with_label("Beep");
    gtk_box_pack_start(GTK_BOX(right), st->chk_flash_led, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(right), st->chk_email, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(right), st->chk_beep, FALSE, FALSE, 0);

    /* Save group button */
    st->btn_save_group = ui_button("Save Group",
                                   G_CALLBACK(on_save_group), st);
    gtk_widget_set_sensitive(st->btn_save_group, FALSE);
    gtk_box_pack_start(GTK_BOX(right), st->btn_save_group, FALSE, FALSE, 0);

    gtk_box_pack_start(GTK_BOX(right), ui_separator(), FALSE, FALSE, 4);

    /* Motion parameters */
    gtk_box_pack_start(GTK_BOX(right), ui_label("Motion Params", 1),
                       FALSE, FALSE, 0);

    GtkWidget *grid = ui_grid();
    int row = 0;

    st->spn_threshold = ui_spin_int(12, 1, 100, 1,
        G_CALLBACK(on_threshold_changed), st);
    row = ui_grid_row(grid, row, "Threshold", st->spn_threshold);

    st->spn_alpha = ui_spin_float(0.05, 0.001, 0.5, 0.005, 3,
        G_CALLBACK(on_alpha_changed), st);
    row = ui_grid_row(grid, row, "BG Alpha", st->spn_alpha);

    st->spn_trigger = ui_spin_float(5.0, 0.1, 100.0, 0.5, 1,
        G_CALLBACK(on_trigger_changed), st);
    row = ui_grid_row(grid, row, "Trigger %", st->spn_trigger);

    st->spn_min_blob = ui_spin_int(3, 1, 100, 1,
        G_CALLBACK(on_min_blob_changed), st);
    row = ui_grid_row(grid, row, "Min Blob", st->spn_min_blob);

    st->spn_framestep = ui_spin_int(5, 1, 50, 1,
        G_CALLBACK(on_framestep_changed), st);
    row = ui_grid_row(grid, row, "Frame Step", st->spn_framestep);
    (void)row;

    gtk_box_pack_start(GTK_BOX(right), grid, FALSE, FALSE, 0);

    gtk_box_pack_start(GTK_BOX(right), ui_separator(), FALSE, FALSE, 4);

    /* Zone controls */
    gtk_box_pack_start(GTK_BOX(right),
                       ui_label("Left-click drag: assign zone", 0),
                       FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(right),
                       ui_label("Right-click: clear cell", 0),
                       FALSE, FALSE, 0);

    st->btn_clear = ui_button("Clear All Zones",
                              G_CALLBACK(on_clear_zones), st);
    gtk_box_pack_start(GTK_BOX(right), st->btn_clear, FALSE, FALSE, 4);

    GtkWidget *right_scroll = ui_scrolled(right);
    gtk_widget_set_size_request(right_scroll, 220, -1);
    gtk_box_pack_start(GTK_BOX(hbox), right_scroll, FALSE, FALSE, 0);

    return hbox;
}

static void motion_config_on_show(AppCtx *app)
{
    MotionConfigState *st = (MotionConfigState *)app->page_motion_config_state;
    if (!st) return;

    populate_camera_combo(st);
    populate_group_combo(st);

    /* Start cameras */
    camera_start_all(app);

    st->active = 1;
    st->redraw_timer = g_timeout_add(33, redraw_timer_cb, st);
}

static void motion_config_on_hide(AppCtx *app)
{
    MotionConfigState *st = (MotionConfigState *)app->page_motion_config_state;
    if (!st) return;
    st->active = 0;

    /* Save current camera motion params to DB (preserve all other fields) */
    if (st->sel_cam_idx >= 0) {
        CameraCtx *cam = &st->app->cameras[st->sel_cam_idx];
        DbCamera all[MAX_CAMERAS];
        int n = db_camera_load_all(app->db, all, MAX_CAMERAS);
        for (int i = 0; i < n; i++) {
            if (all[i].id == cam->id) {
                all[i].threshold = cam->threshold;
                all[i].alpha = cam->alpha;
                all[i].trigger_pct = cam->trigger_pct;
                all[i].min_blob_cells = cam->min_blob_cells;
                all[i].framestep = cam->framestep;
                db_camera_update(app->db, &all[i]);
                break;
            }
        }
    }
}

static void motion_config_cleanup(AppCtx *app)
{
    if (app->page_motion_config_state) {
        free(app->page_motion_config_state);
        app->page_motion_config_state = NULL;
    }
}

const PageDef page_motion_config = {
    .id      = "motion_config",
    .title   = "Motion Config",
    .build   = motion_config_build,
    .on_show = motion_config_on_show,
    .on_hide = motion_config_on_hide,
    .cleanup = motion_config_cleanup,
};
