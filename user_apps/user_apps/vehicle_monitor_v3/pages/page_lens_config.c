/*
 * pages/page_lens_config.c - Lens distortion calibration page
 *
 * Users draw lines along real-world straight edges in the camera feed.
 * A grid-search fitter computes radial distortion coefficients (k1, k2)
 * that minimize line curvature. The result is applied as a WarpLUT in
 * the camera worker thread for per-frame undistortion.
 */
#include "../app.h"
#include "../core/database.h"
#include "../core/camera.h"
#include "../core/undistort.h"
#include "../ui/util.h"
#include <cairo.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Line colors (up to MAX_LENS_LINES) */
static const double LINE_COLORS[][3] = {
    {1.0, 0.2, 0.2},  /* red */
    {0.2, 0.8, 0.2},  /* green */
    {0.3, 0.3, 1.0},  /* blue */
    {1.0, 0.8, 0.0},  /* yellow */
    {1.0, 0.4, 0.8},  /* pink */
    {0.0, 0.8, 0.8},  /* cyan */
    {1.0, 0.5, 0.0},  /* orange */
    {0.6, 0.3, 0.8},  /* purple */
    {0.5, 1.0, 0.5},  /* lime */
    {0.8, 0.8, 0.8},  /* gray */
    {0.4, 0.2, 0.0},  /* brown */
    {0.0, 0.5, 0.0},  /* dark green */
    {0.5, 0.0, 0.5},  /* magenta */
    {0.0, 0.4, 0.8},  /* steel blue */
    {0.8, 0.4, 0.4},  /* salmon */
    {0.4, 0.8, 0.4},  /* light green */
};

typedef struct {
    AppCtx       *app;
    GtkWidget    *draw_area;
    GtkWidget    *camera_combo;
    GtkWidget    *line_list;       /* GtkListBox for line summary */
    GtkWidget    *btn_add_line;
    GtkWidget    *btn_finish_line;
    GtkWidget    *btn_calibrate;
    GtkWidget    *btn_preview;
    GtkWidget    *btn_clear;
    GtkWidget    *lbl_k1;
    GtkWidget    *lbl_k2;
    GtkWidget    *lbl_status;

    int           sel_cam_idx;
    int           active;
    guint         redraw_timer;

    /* Calibration lines loaded from DB */
    DbLensLinePoint db_pts[MAX_LENS_TOTAL_PTS];
    int             num_db_pts;

    /* Current line being drawn */
    int           drawing;         /* 1 = drawing mode active */
    int           cur_line_idx;    /* line index for current line */
    double        cur_pts_x[MAX_LINE_POINTS];
    double        cur_pts_y[MAX_LINE_POINTS];
    int           cur_npts;

    /* Fitted rect for coordinate mapping */
    double        fit_x, fit_y, fit_w, fit_h;

    /* Preview mode */
    int           show_preview;
    uint8_t      *preview_buf;
    WarpLUT       preview_lut;
    int           preview_valid;

    /* Current params */
    double        k1, k2;
    int           has_params;
} LensConfigState;

static void load_lines(LensConfigState *st);
static void populate_line_list(LensConfigState *st);

static int get_camera_id(LensConfigState *st)
{
    if (st->sel_cam_idx < 0 || st->sel_cam_idx >= st->app->num_cameras)
        return -1;
    return st->app->cameras[st->sel_cam_idx].id;
}

static CameraCtx *get_camera(LensConfigState *st)
{
    if (st->sel_cam_idx < 0 || st->sel_cam_idx >= st->app->num_cameras)
        return NULL;
    return &st->app->cameras[st->sel_cam_idx];
}

/* Count distinct lines and points per line */
static int count_lines(LensConfigState *st, int *line_ids, int *line_counts,
                       int max_lines)
{
    int nlines = 0;
    for (int i = 0; i < st->num_db_pts; i++) {
        int lidx = st->db_pts[i].line_idx;
        int found = -1;
        for (int j = 0; j < nlines; j++) {
            if (line_ids[j] == lidx) { found = j; break; }
        }
        if (found >= 0) {
            line_counts[found]++;
        } else if (nlines < max_lines) {
            line_ids[nlines] = lidx;
            line_counts[nlines] = 1;
            nlines++;
        }
    }
    return nlines;
}

/* Find the next available line index */
static int next_line_idx(LensConfigState *st)
{
    int max_idx = -1;
    for (int i = 0; i < st->num_db_pts; i++) {
        if (st->db_pts[i].line_idx > max_idx)
            max_idx = st->db_pts[i].line_idx;
    }
    return max_idx + 1;
}

/* --- Camera combo --- */

static void on_camera_changed(GtkComboBoxText *combo, gpointer data)
{
    (void)combo;
    LensConfigState *st = (LensConfigState *)data;
    int idx = gtk_combo_box_get_active(GTK_COMBO_BOX(st->camera_combo));
    if (idx < 0) return;
    st->sel_cam_idx = idx;

    /* Cancel any in-progress drawing */
    st->drawing = 0;
    st->cur_npts = 0;

    /* Load calibration data for this camera */
    load_lines(st);
    populate_line_list(st);

    /* Load existing params */
    DbLensParams lp;
    int cam_id = get_camera_id(st);
    if (cam_id > 0 && db_lens_params_load(st->app->db, cam_id, &lp) == 0) {
        st->k1 = lp.k1;
        st->k2 = lp.k2;
        st->has_params = 1;
        char buf[64];
        snprintf(buf, sizeof(buf), "k1: %.4f", lp.k1);
        gtk_label_set_text(GTK_LABEL(st->lbl_k1), buf);
        snprintf(buf, sizeof(buf), "k2: %.4f", lp.k2);
        gtk_label_set_text(GTK_LABEL(st->lbl_k2), buf);
        gtk_label_set_text(GTK_LABEL(st->lbl_status), "Calibrated");
    } else {
        st->k1 = 0;
        st->k2 = 0;
        st->has_params = 0;
        gtk_label_set_text(GTK_LABEL(st->lbl_k1), "k1: --");
        gtk_label_set_text(GTK_LABEL(st->lbl_k2), "k2: --");
        gtk_label_set_text(GTK_LABEL(st->lbl_status), "Not calibrated");
    }

    /* Reset preview */
    st->show_preview = 0;
    if (st->preview_lut.entries) {
        warp_lut_free(&st->preview_lut);
        st->preview_valid = 0;
    }

    gtk_widget_queue_draw(st->draw_area);
}

/* --- Add Line / Finish Line --- */

static void on_add_line(GtkButton *btn, gpointer data)
{
    (void)btn;
    LensConfigState *st = (LensConfigState *)data;
    if (get_camera_id(st) <= 0) return;

    st->drawing = 1;
    st->cur_line_idx = next_line_idx(st);
    st->cur_npts = 0;
    gtk_label_set_text(GTK_LABEL(st->lbl_status),
                       "Click along a straight edge...");
    gtk_widget_set_sensitive(st->btn_add_line, FALSE);
    gtk_widget_set_sensitive(st->btn_finish_line, TRUE);
    gtk_widget_queue_draw(st->draw_area);
}

static void on_finish_line(GtkButton *btn, gpointer data)
{
    (void)btn;
    LensConfigState *st = (LensConfigState *)data;
    int cam_id = get_camera_id(st);

    if (st->cur_npts >= 3 && cam_id > 0) {
        /* Save points to DB */
        for (int i = 0; i < st->cur_npts; i++) {
            db_lens_line_insert(st->app->db, cam_id, st->cur_line_idx,
                                st->cur_pts_x[i], st->cur_pts_y[i]);
        }
        load_lines(st);
        populate_line_list(st);
        gtk_label_set_text(GTK_LABEL(st->lbl_status), "Line saved");
    } else {
        gtk_label_set_text(GTK_LABEL(st->lbl_status),
                           "Need >= 3 points per line");
    }

    st->drawing = 0;
    st->cur_npts = 0;
    gtk_widget_set_sensitive(st->btn_add_line, TRUE);
    gtk_widget_set_sensitive(st->btn_finish_line, FALSE);
    gtk_widget_queue_draw(st->draw_area);
}

/* --- Click on drawing area --- */

static gboolean on_draw_click(GtkWidget *widget, GdkEventButton *ev,
                               gpointer data)
{
    (void)widget;
    LensConfigState *st = (LensConfigState *)data;
    if (!st->drawing || ev->button != 1)
        return FALSE;
    if (st->fit_w < 1 || st->fit_h < 1)
        return FALSE;

    /* Map click to fraction of image */
    double px_frac = (ev->x - st->fit_x) / st->fit_w;
    double py_frac = (ev->y - st->fit_y) / st->fit_h;

    if (px_frac < 0 || px_frac > 1 || py_frac < 0 || py_frac > 1)
        return FALSE;

    if (st->cur_npts < MAX_LINE_POINTS) {
        st->cur_pts_x[st->cur_npts] = px_frac;
        st->cur_pts_y[st->cur_npts] = py_frac;
        st->cur_npts++;

        char buf[64];
        snprintf(buf, sizeof(buf), "Line %d: %d points",
                 st->cur_line_idx, st->cur_npts);
        gtk_label_set_text(GTK_LABEL(st->lbl_status), buf);
        gtk_widget_queue_draw(st->draw_area);
    }

    return TRUE;
}

/* --- Calibrate --- */

static void on_calibrate(GtkButton *btn, gpointer data)
{
    (void)btn;
    LensConfigState *st = (LensConfigState *)data;
    CameraCtx *cam = get_camera(st);
    int cam_id = get_camera_id(st);
    if (!cam || cam_id <= 0) return;

    /* Convert DB points to LensLinePoint array */
    LensLinePoint pts[MAX_LENS_TOTAL_PTS];
    for (int i = 0; i < st->num_db_pts; i++) {
        pts[i].line_idx = st->db_pts[i].line_idx;
        pts[i].px = st->db_pts[i].px;
        pts[i].py = st->db_pts[i].py;
    }

    int img_w = cam->disp_w > 0 ? cam->disp_w : 960;
    int img_h = cam->disp_h > 0 ? cam->disp_h : 540;

    double k1, k2;
    int ret = undistort_fit(pts, st->num_db_pts, 0.5, 0.5,
                            img_w, img_h, &k1, &k2);
    if (ret != 0) {
        gtk_label_set_text(GTK_LABEL(st->lbl_status),
                           "Fit failed: need 2+ lines, 3+ pts each");
        return;
    }

    /* Save to DB */
    db_lens_params_save(st->app->db, cam_id, k1, k2, 0.5, 0.5);

    st->k1 = k1;
    st->k2 = k2;
    st->has_params = 1;

    char buf[64];
    snprintf(buf, sizeof(buf), "k1: %.4f", k1);
    gtk_label_set_text(GTK_LABEL(st->lbl_k1), buf);
    snprintf(buf, sizeof(buf), "k2: %.4f", k2);
    gtk_label_set_text(GTK_LABEL(st->lbl_k2), buf);

    /* Install LUT on live camera (thread-safe swap) */
    if (cam->disp_w > 0 && cam->work_undistorted) {
        LensParams lp = { .k1 = k1, .k2 = k2,
                          .cx_frac = 0.5, .cy_frac = 0.5 };
        cam->undistort_valid = 0;
        if (cam->undistort_lut.entries)
            warp_lut_free(&cam->undistort_lut);
        undistort_lut_build(&lp, cam->disp_w, cam->disp_h,
                            &cam->undistort_lut);
        cam->undistort_valid = 1;
    }

    /* Build preview LUT */
    if (st->preview_lut.entries)
        warp_lut_free(&st->preview_lut);
    LensParams lpp = { .k1 = k1, .k2 = k2,
                       .cx_frac = 0.5, .cy_frac = 0.5 };
    undistort_lut_build(&lpp, img_w, img_h, &st->preview_lut);
    st->preview_valid = 1;

    gtk_label_set_text(GTK_LABEL(st->lbl_status), "Calibration applied");
    gtk_widget_queue_draw(st->draw_area);
}

/* --- Preview toggle --- */

static void on_preview(GtkButton *btn, gpointer data)
{
    (void)btn;
    LensConfigState *st = (LensConfigState *)data;
    st->show_preview = !st->show_preview;

    if (st->show_preview && !st->preview_valid && st->has_params) {
        CameraCtx *cam = get_camera(st);
        int img_w = cam && cam->disp_w > 0 ? cam->disp_w : 960;
        int img_h = cam && cam->disp_h > 0 ? cam->disp_h : 540;
        LensParams lp = { .k1 = st->k1, .k2 = st->k2,
                          .cx_frac = 0.5, .cy_frac = 0.5 };
        if (st->preview_lut.entries)
            warp_lut_free(&st->preview_lut);
        undistort_lut_build(&lp, img_w, img_h, &st->preview_lut);
        st->preview_valid = 1;
    }

    gtk_button_set_label(GTK_BUTTON(st->btn_preview),
                         st->show_preview ? "Hide Preview" : "Preview");
    gtk_widget_queue_draw(st->draw_area);
}

/* --- Clear All --- */

static void on_clear_all(GtkButton *btn, gpointer data)
{
    (void)btn;
    LensConfigState *st = (LensConfigState *)data;
    int cam_id = get_camera_id(st);
    if (cam_id <= 0) return;

    db_lens_line_clear_camera(st->app->db, cam_id);
    db_lens_params_delete(st->app->db, cam_id);

    /* Remove live LUT */
    CameraCtx *cam = get_camera(st);
    if (cam) {
        cam->undistort_valid = 0;
        if (cam->undistort_lut.entries)
            warp_lut_free(&cam->undistort_lut);
    }

    st->num_db_pts = 0;
    st->drawing = 0;
    st->cur_npts = 0;
    st->has_params = 0;
    st->k1 = 0;
    st->k2 = 0;
    st->show_preview = 0;
    if (st->preview_lut.entries) {
        warp_lut_free(&st->preview_lut);
        st->preview_valid = 0;
    }

    gtk_label_set_text(GTK_LABEL(st->lbl_k1), "k1: --");
    gtk_label_set_text(GTK_LABEL(st->lbl_k2), "k2: --");
    gtk_label_set_text(GTK_LABEL(st->lbl_status), "Cleared");
    gtk_widget_set_sensitive(st->btn_add_line, TRUE);
    gtk_widget_set_sensitive(st->btn_finish_line, FALSE);
    gtk_button_set_label(GTK_BUTTON(st->btn_preview), "Preview");

    load_lines(st);
    populate_line_list(st);
    gtk_widget_queue_draw(st->draw_area);
}

/* --- Delete individual line --- */

static void on_delete_line(GtkButton *btn, gpointer data)
{
    (void)data;
    LensConfigState *st = (LensConfigState *)
        g_object_get_data(G_OBJECT(btn), "lens-state");
    int line_idx = GPOINTER_TO_INT(
        g_object_get_data(G_OBJECT(btn), "line-idx"));
    int cam_id = get_camera_id(st);
    if (cam_id <= 0) return;

    db_lens_line_delete_line(st->app->db, cam_id, line_idx);
    load_lines(st);
    populate_line_list(st);
    gtk_widget_queue_draw(st->draw_area);
}

/* --- Line list --- */

static void load_lines(LensConfigState *st)
{
    int cam_id = get_camera_id(st);
    if (cam_id <= 0) {
        st->num_db_pts = 0;
        return;
    }
    st->num_db_pts = db_lens_line_load_camera(st->app->db, cam_id,
                                               st->db_pts,
                                               MAX_LENS_TOTAL_PTS);
}

static void populate_line_list(LensConfigState *st)
{
    /* Clear existing rows */
    GList *children = gtk_container_get_children(
        GTK_CONTAINER(st->line_list));
    for (GList *l = children; l; l = l->next)
        gtk_widget_destroy(GTK_WIDGET(l->data));
    g_list_free(children);

    int line_ids[MAX_LENS_LINES];
    int line_counts[MAX_LENS_LINES];
    int nlines = count_lines(st, line_ids, line_counts, MAX_LENS_LINES);

    for (int i = 0; i < nlines; i++) {
        GtkWidget *hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 4);

        /* Color swatch */
        int ci = line_ids[i] % 16;
        char markup[128];
        snprintf(markup, sizeof(markup),
                 "<span foreground=\"#%02X%02X%02X\">Line %d</span>"
                 " (%d pts)",
                 (int)(LINE_COLORS[ci][0] * 255),
                 (int)(LINE_COLORS[ci][1] * 255),
                 (int)(LINE_COLORS[ci][2] * 255),
                 line_ids[i], line_counts[i]);

        GtkWidget *lbl = gtk_label_new(NULL);
        gtk_label_set_markup(GTK_LABEL(lbl), markup);
        gtk_label_set_xalign(GTK_LABEL(lbl), 0.0);
        gtk_widget_set_hexpand(lbl, TRUE);
        gtk_box_pack_start(GTK_BOX(hbox), lbl, TRUE, TRUE, 0);

        GtkWidget *del_btn = gtk_button_new_with_label("X");
        gtk_widget_set_size_request(del_btn, 30, -1);
        g_object_set_data(G_OBJECT(del_btn), "lens-state", st);
        g_object_set_data(G_OBJECT(del_btn), "line-idx",
                          GINT_TO_POINTER(line_ids[i]));
        g_signal_connect(del_btn, "clicked",
                         G_CALLBACK(on_delete_line), NULL);
        gtk_box_pack_end(GTK_BOX(hbox), del_btn, FALSE, FALSE, 0);

        gtk_list_box_insert(GTK_LIST_BOX(st->line_list), hbox, -1);
    }

    gtk_widget_show_all(st->line_list);
}

/* --- Drawing --- */

static void draw_line_points(cairo_t *cr, LensConfigState *st,
                             const double *pxs, const double *pys,
                             int npts, int line_idx)
{
    if (npts < 1) return;
    int ci = line_idx % 16;
    cairo_set_source_rgba(cr, LINE_COLORS[ci][0], LINE_COLORS[ci][1],
                          LINE_COLORS[ci][2], 0.9);
    cairo_set_line_width(cr, 2.0);

    /* Polyline */
    double x0 = st->fit_x + pxs[0] * st->fit_w;
    double y0 = st->fit_y + pys[0] * st->fit_h;
    cairo_move_to(cr, x0, y0);
    for (int i = 1; i < npts; i++) {
        double x = st->fit_x + pxs[i] * st->fit_w;
        double y = st->fit_y + pys[i] * st->fit_h;
        cairo_line_to(cr, x, y);
    }
    cairo_stroke(cr);

    /* Dots */
    for (int i = 0; i < npts; i++) {
        double x = st->fit_x + pxs[i] * st->fit_w;
        double y = st->fit_y + pys[i] * st->fit_h;
        cairo_arc(cr, x, y, 4.0, 0, 2 * M_PI);
        cairo_fill(cr);
    }
}

static void draw_grid_overlay(cairo_t *cr, LensConfigState *st)
{
    cairo_set_source_rgba(cr, 0.0, 1.0, 0.0, 0.3);
    cairo_set_line_width(cr, 0.5);

    int num_h = 10;
    int num_v = 10;
    for (int i = 1; i < num_h; i++) {
        double y = st->fit_y + (st->fit_h * i) / num_h;
        cairo_move_to(cr, st->fit_x, y);
        cairo_line_to(cr, st->fit_x + st->fit_w, y);
    }
    for (int i = 1; i < num_v; i++) {
        double x = st->fit_x + (st->fit_w * i) / num_v;
        cairo_move_to(cr, x, st->fit_y);
        cairo_line_to(cr, x, st->fit_y + st->fit_h);
    }
    cairo_stroke(cr);
}

static gboolean on_draw(GtkWidget *widget, cairo_t *cr, gpointer data)
{
    LensConfigState *st = (LensConfigState *)data;
    CameraCtx *cam = get_camera(st);

    int area_w = gtk_widget_get_allocated_width(widget);
    int area_h = gtk_widget_get_allocated_height(widget);

    /* Background */
    cairo_set_source_rgb(cr, 0.1, 0.1, 0.1);
    cairo_paint(cr);

    if (!cam || cam->disp_w <= 0 || cam->disp_h <= 0) {
        cairo_set_source_rgb(cr, 0.8, 0.8, 0.8);
        cairo_select_font_face(cr, "monospace", CAIRO_FONT_SLANT_NORMAL,
                               CAIRO_FONT_WEIGHT_NORMAL);
        cairo_set_font_size(cr, 14);
        cairo_move_to(cr, 20, area_h / 2);
        cairo_show_text(cr, "No camera feed");
        return TRUE;
    }

    int img_w = cam->disp_w;
    int img_h = cam->disp_h;

    /* Get frame data */
    pthread_mutex_lock(&cam->buf_mutex);
    if (cam->buf_updated && cam->color_rgb) {
        if (!cam->draw_color)
            cam->draw_color = (uint8_t *)malloc(cam->disp_size);
        if (cam->draw_color)
            memcpy(cam->draw_color, cam->color_rgb, cam->disp_size);
    }
    pthread_mutex_unlock(&cam->buf_mutex);

    uint8_t *frame_data = cam->draw_color;

    /* If preview mode: apply preview LUT */
    if (st->show_preview && st->preview_valid && frame_data) {
        int bufsz = img_w * img_h * 3;
        if (!st->preview_buf || st->preview_lut.out_w != img_w ||
            st->preview_lut.out_h != img_h) {
            free(st->preview_buf);
            st->preview_buf = (uint8_t *)malloc(bufsz);
        }
        if (st->preview_buf) {
            warp_lut_apply(&st->preview_lut, frame_data, st->preview_buf);
            frame_data = st->preview_buf;
        }
    }

    if (frame_data) {
        GdkPixbuf *pb = gdk_pixbuf_new_from_data(
            frame_data, GDK_COLORSPACE_RGB, FALSE, 8,
            img_w, img_h, img_w * 3, NULL, NULL);
        ui_draw_pixbuf_fitted(cr, pb, img_w, img_h, area_w, area_h,
                              &st->fit_x, &st->fit_y,
                              &st->fit_w, &st->fit_h);
        g_object_unref(pb);
    }

    /* Draw saved calibration lines */
    int line_ids[MAX_LENS_LINES];
    int line_counts[MAX_LENS_LINES];
    int nlines = count_lines(st, line_ids, line_counts, MAX_LENS_LINES);

    for (int li = 0; li < nlines; li++) {
        double pxs[MAX_LINE_POINTS], pys[MAX_LINE_POINTS];
        int cnt = 0;
        for (int j = 0; j < st->num_db_pts && cnt < MAX_LINE_POINTS; j++) {
            if (st->db_pts[j].line_idx == line_ids[li]) {
                pxs[cnt] = st->db_pts[j].px;
                pys[cnt] = st->db_pts[j].py;
                cnt++;
            }
        }
        draw_line_points(cr, st, pxs, pys, cnt, line_ids[li]);
    }

    /* Draw current line being drawn */
    if (st->drawing && st->cur_npts > 0) {
        draw_line_points(cr, st, st->cur_pts_x, st->cur_pts_y,
                         st->cur_npts, st->cur_line_idx);
    }

    /* Grid overlay in preview mode */
    if (st->show_preview && st->preview_valid)
        draw_grid_overlay(cr, st);

    return TRUE;
}

/* --- Redraw timer --- */

static gboolean redraw_tick(gpointer data)
{
    LensConfigState *st = (LensConfigState *)data;
    if (!st->active)
        return G_SOURCE_REMOVE;
    if (st->draw_area)
        gtk_widget_queue_draw(st->draw_area);
    return G_SOURCE_CONTINUE;
}

/* --- Build / Show / Hide / Cleanup --- */

static GtkWidget *lens_config_build(AppCtx *app)
{
    LensConfigState *st = (LensConfigState *)calloc(1, sizeof(*st));
    st->app = app;
    st->sel_cam_idx = -1;
    app->page_lens_config_state = st;

    GtkWidget *hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
    gtk_widget_set_margin_start(hbox, 4);
    gtk_widget_set_margin_end(hbox, 4);
    gtk_widget_set_margin_top(hbox, 4);
    gtk_widget_set_margin_bottom(hbox, 4);

    /* Left: drawing area */
    st->draw_area = gtk_drawing_area_new();
    gtk_widget_set_hexpand(st->draw_area, TRUE);
    gtk_widget_set_vexpand(st->draw_area, TRUE);
    gtk_widget_add_events(st->draw_area, GDK_BUTTON_PRESS_MASK);
    g_signal_connect(st->draw_area, "draw", G_CALLBACK(on_draw), st);
    g_signal_connect(st->draw_area, "button-press-event",
                     G_CALLBACK(on_draw_click), st);
    gtk_box_pack_start(GTK_BOX(hbox), st->draw_area, TRUE, TRUE, 0);

    /* Right: controls panel */
    GtkWidget *panel = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
    gtk_widget_set_size_request(panel, 200, -1);

    /* Camera selector */
    GtkWidget *cam_lbl = ui_label("Camera:", 1);
    gtk_box_pack_start(GTK_BOX(panel), cam_lbl, FALSE, FALSE, 0);

    st->camera_combo = gtk_combo_box_text_new();
    for (int i = 0; i < app->num_cameras; i++) {
        gtk_combo_box_text_append_text(
            GTK_COMBO_BOX_TEXT(st->camera_combo),
            app->cameras[i].name);
    }
    g_signal_connect(st->camera_combo, "changed",
                     G_CALLBACK(on_camera_changed), st);
    gtk_box_pack_start(GTK_BOX(panel), st->camera_combo, FALSE, FALSE, 0);

    gtk_box_pack_start(GTK_BOX(panel), gtk_separator_new(
        GTK_ORIENTATION_HORIZONTAL), FALSE, FALSE, 4);

    /* Line drawing buttons */
    st->btn_add_line = ui_button("Add Line", G_CALLBACK(on_add_line), st);
    gtk_box_pack_start(GTK_BOX(panel), st->btn_add_line, FALSE, FALSE, 0);

    st->btn_finish_line = ui_button("Finish Line",
                                     G_CALLBACK(on_finish_line), st);
    gtk_widget_set_sensitive(st->btn_finish_line, FALSE);
    gtk_box_pack_start(GTK_BOX(panel), st->btn_finish_line,
                       FALSE, FALSE, 0);

    /* Lines list label */
    GtkWidget *lines_lbl = ui_label("Lines:", 1);
    gtk_box_pack_start(GTK_BOX(panel), lines_lbl, FALSE, FALSE, 0);

    /* Scrolled line list */
    GtkWidget *scroll = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scroll),
                                   GTK_POLICY_NEVER, GTK_POLICY_AUTOMATIC);
    gtk_widget_set_size_request(scroll, -1, 120);
    st->line_list = gtk_list_box_new();
    gtk_list_box_set_selection_mode(GTK_LIST_BOX(st->line_list),
                                    GTK_SELECTION_NONE);
    gtk_container_add(GTK_CONTAINER(scroll), st->line_list);
    gtk_box_pack_start(GTK_BOX(panel), scroll, FALSE, FALSE, 0);

    gtk_box_pack_start(GTK_BOX(panel), gtk_separator_new(
        GTK_ORIENTATION_HORIZONTAL), FALSE, FALSE, 4);

    /* Action buttons */
    st->btn_calibrate = ui_button("Calibrate",
                                   G_CALLBACK(on_calibrate), st);
    gtk_box_pack_start(GTK_BOX(panel), st->btn_calibrate,
                       FALSE, FALSE, 0);

    st->btn_preview = ui_button("Preview", G_CALLBACK(on_preview), st);
    gtk_box_pack_start(GTK_BOX(panel), st->btn_preview, FALSE, FALSE, 0);

    st->btn_clear = ui_button("Clear All", G_CALLBACK(on_clear_all), st);
    gtk_box_pack_start(GTK_BOX(panel), st->btn_clear, FALSE, FALSE, 0);

    gtk_box_pack_start(GTK_BOX(panel), gtk_separator_new(
        GTK_ORIENTATION_HORIZONTAL), FALSE, FALSE, 4);

    /* Parameter display */
    st->lbl_k1 = gtk_label_new("k1: --");
    gtk_label_set_xalign(GTK_LABEL(st->lbl_k1), 0.0);
    gtk_box_pack_start(GTK_BOX(panel), st->lbl_k1, FALSE, FALSE, 0);

    st->lbl_k2 = gtk_label_new("k2: --");
    gtk_label_set_xalign(GTK_LABEL(st->lbl_k2), 0.0);
    gtk_box_pack_start(GTK_BOX(panel), st->lbl_k2, FALSE, FALSE, 0);

    st->lbl_status = gtk_label_new("Select a camera");
    gtk_label_set_xalign(GTK_LABEL(st->lbl_status), 0.0);
    gtk_label_set_line_wrap(GTK_LABEL(st->lbl_status), TRUE);
    gtk_box_pack_start(GTK_BOX(panel), st->lbl_status, FALSE, FALSE, 0);

    gtk_box_pack_end(GTK_BOX(hbox), panel, FALSE, FALSE, 0);

    return hbox;
}

static void lens_config_on_show(AppCtx *app)
{
    LensConfigState *st = (LensConfigState *)app->page_lens_config_state;
    if (!st) return;

    st->active = 1;
    if (!st->redraw_timer)
        st->redraw_timer = g_timeout_add(100, redraw_tick, st);

    /* Auto-select first camera if none selected */
    if (st->sel_cam_idx < 0 && app->num_cameras > 0) {
        gtk_combo_box_set_active(GTK_COMBO_BOX(st->camera_combo), 0);
    }
}

static void lens_config_on_hide(AppCtx *app)
{
    LensConfigState *st = (LensConfigState *)app->page_lens_config_state;
    if (!st) return;
    st->active = 0;
    if (st->redraw_timer) {
        g_source_remove(st->redraw_timer);
        st->redraw_timer = 0;
    }
}

static void lens_config_cleanup(AppCtx *app)
{
    LensConfigState *st = (LensConfigState *)app->page_lens_config_state;
    if (!st) return;

    if (st->redraw_timer)
        g_source_remove(st->redraw_timer);
    if (st->preview_lut.entries)
        warp_lut_free(&st->preview_lut);
    free(st->preview_buf);
    free(st);
    app->page_lens_config_state = NULL;
}

const PageDef page_lens_config = {
    .id      = "lens_config",
    .title   = "Lens Config",
    .build   = lens_config_build,
    .on_show = lens_config_on_show,
    .on_hide = lens_config_on_hide,
    .cleanup = lens_config_cleanup,
};
