/*
 * pages/page_birdview_config.c - Birdview calibration page
 *
 * Dual-view calibration: ground view (place markers on 800x600 canvas)
 * and camera view (link markers to camera feed). Ground markers define
 * world coordinates, then are linked to camera image coordinates.
 */
#include "../app.h"
#include "../core/database.h"
#include "../core/camera.h"
#include "../core/config.h"
#include "../core/homography.h"
#include "../ui/util.h"
#include <cairo.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#define MAX_CALIB_POINTS 32
#define FUSED_W 800
#define FUSED_H 600

typedef struct {
    AppCtx       *app;
    GtkWidget    *draw_area;
    GtkWidget    *camera_combo;
    GtkWidget    *marker_list;      /* GtkListBox */
    GtkWidget    *btn_place;
    GtkWidget    *btn_mode;
    GtkWidget    *btn_compute;
    GtkWidget    *btn_preview;
    GtkWidget    *btn_clear;
    GtkWidget    *lbl_status;

    int           ground_mode;      /* 1=ground view, 0=camera view */
    int           sel_cam_idx;
    int           active;
    guint         redraw_timer;

    /* Ground marker placement */
    int           placing;

    /* Camera linking */
    int           linking_marker_id;  /* ground marker being linked, or -1 */

    /* Ground markers (shared) */
    DbGroundMarker markers[MAX_GROUND_MARKERS];
    int            num_markers;

    /* Calib points for current camera */
    DbCalibPoint   points[MAX_CALIB_POINTS];
    int            num_points;

    /* Fitted rect for coordinate mapping */
    double         fit_x, fit_y, fit_w, fit_h;

    /* Warp preview */
    int            show_preview;
    uint8_t       *warp_buf;
    WarpLUT        warp_lut;
    int            warp_valid;

    /* Draw-local copies of pre-rotated birdview buffers for ground view */
    uint8_t       *bv_draw[4];
    int            bv_draw_w[4];
    int            bv_draw_h[4];
} BirdviewConfigState;

static void populate_marker_list(BirdviewConfigState *st);
static void load_markers(BirdviewConfigState *st);
static void load_points(BirdviewConfigState *st);

static CameraCtx *find_birdview_cam(AppCtx *app, int pos)
{
    for (int i = 0; i < app->num_cameras; i++) {
        if (app->cameras[i].birdview_pos == pos &&
            app->cameras[i].enabled)
            return &app->cameras[i];
    }
    return NULL;
}

/* --- Marker list callbacks --- */

static void on_delete_marker_clicked(GtkButton *btn, gpointer data)
{
    (void)data;
    BirdviewConfigState *st = (BirdviewConfigState *)
        g_object_get_data(G_OBJECT(btn), "bv-state");
    int mid = GPOINTER_TO_INT(
        g_object_get_data(G_OBJECT(btn), "marker-id"));

    db_ground_marker_delete(st->app->db, mid);

    /* Also delete any calib_points linked to this marker's world coords */
    for (int i = 0; i < st->num_markers; i++) {
        if (st->markers[i].id == mid) {
            /* Find and remove calib points with matching world coords */
            sqlite3_stmt *stmt = NULL;
            sqlite3_prepare_v2(st->app->db,
                "DELETE FROM calib_points WHERE world_x=? AND world_y=?;",
                -1, &stmt, NULL);
            if (stmt) {
                sqlite3_bind_double(stmt, 1, st->markers[i].world_x);
                sqlite3_bind_double(stmt, 2, st->markers[i].world_y);
                sqlite3_step(stmt);
                sqlite3_finalize(stmt);
            }
            break;
        }
    }

    load_markers(st);
    load_points(st);
    populate_marker_list(st);
    gtk_widget_queue_draw(st->draw_area);
}

static void on_marker_row_selected(GtkListBox *box, GtkListBoxRow *row,
                                   gpointer data)
{
    (void)box;
    BirdviewConfigState *st = (BirdviewConfigState *)data;

    if (!row) {
        st->linking_marker_id = -1;
        return;
    }

    /* In camera mode, selecting a marker starts linking */
    if (!st->ground_mode && st->sel_cam_idx >= 0) {
        int idx = gtk_list_box_row_get_index(row);
        if (idx >= 0 && idx < st->num_markers) {
            st->linking_marker_id = st->markers[idx].id;
            char msg[64];
            snprintf(msg, sizeof(msg),
                     "Click on camera feed to link marker #%d", idx + 1);
            gtk_label_set_text(GTK_LABEL(st->lbl_status), msg);
            gtk_widget_queue_draw(st->draw_area);
        }
    }
}

static void load_markers(BirdviewConfigState *st)
{
    st->num_markers = db_ground_marker_load_all(
        st->app->db, st->markers, MAX_GROUND_MARKERS);
}

static void load_points(BirdviewConfigState *st)
{
    st->num_points = 0;
    if (st->sel_cam_idx < 0) return;
    CameraCtx *cam = &st->app->cameras[st->sel_cam_idx];
    st->num_points = db_calib_load_camera(st->app->db, cam->id,
                                           st->points, MAX_CALIB_POINTS);
}

static int marker_has_link(BirdviewConfigState *st, int marker_idx)
{
    if (st->sel_cam_idx < 0) return 0;
    DbGroundMarker *m = &st->markers[marker_idx];
    for (int i = 0; i < st->num_points; i++) {
        if (fabs(st->points[i].world_x - m->world_x) < 0.01 &&
            fabs(st->points[i].world_y - m->world_y) < 0.01)
            return 1;
    }
    return 0;
}

static void populate_marker_list(BirdviewConfigState *st)
{
    GList *children = gtk_container_get_children(
        GTK_CONTAINER(st->marker_list));
    for (GList *l = children; l; l = l->next)
        gtk_container_remove(GTK_CONTAINER(st->marker_list),
                             GTK_WIDGET(l->data));
    g_list_free(children);

    for (int i = 0; i < st->num_markers; i++) {
        char label[128];
        int linked = marker_has_link(st, i);
        snprintf(label, sizeof(label), "#%d  (%.0f, %.0f)%s",
                 i + 1, st->markers[i].world_x, st->markers[i].world_y,
                 linked ? " [linked]" : "");

        GtkWidget *row = gtk_list_box_row_new();
        GtkWidget *hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 4);
        GtkWidget *lbl = gtk_label_new(label);
        gtk_label_set_xalign(GTK_LABEL(lbl), 0.0);
        gtk_widget_set_hexpand(lbl, TRUE);
        gtk_box_pack_start(GTK_BOX(hbox), lbl, TRUE, TRUE, 4);

        GtkWidget *btn_del = gtk_button_new_with_label("X");
        gtk_widget_set_size_request(btn_del, 28, -1);
        g_object_set_data(G_OBJECT(btn_del), "marker-id",
                          GINT_TO_POINTER(st->markers[i].id));
        g_object_set_data(G_OBJECT(btn_del), "bv-state", st);
        gtk_box_pack_end(GTK_BOX(hbox), btn_del, FALSE, FALSE, 0);

        gtk_container_add(GTK_CONTAINER(row), hbox);
        gtk_list_box_insert(GTK_LIST_BOX(st->marker_list), row, -1);

        g_signal_connect(btn_del, "clicked",
                         G_CALLBACK(on_delete_marker_clicked), NULL);
    }

    gtk_widget_show_all(st->marker_list);
    gtk_widget_set_sensitive(st->btn_compute, st->num_points >= 4);
}

/* --- Draw callbacks --- */

static void draw_ground_view(BirdviewConfigState *st, cairo_t *cr,
                             int aw, int ah)
{
    AppCtx *app = st->app;

    /* Fit FUSED_W x FUSED_H into draw area */
    double scale = fmin((double)aw / FUSED_W, (double)ah / FUSED_H);
    double fw = FUSED_W * scale;
    double fh = FUSED_H * scale;
    double fx = (aw - fw) / 2;
    double fy = (ah - fh) / 2;

    st->fit_x = fx; st->fit_y = fy;
    st->fit_w = fw; st->fit_h = fh;

    /* Dark background */
    cairo_set_source_rgb(cr, 0.12, 0.12, 0.15);
    cairo_rectangle(cr, fx, fy, fw, fh);
    cairo_fill(cr);

    /* --- Live camera feeds (same layout as bird view page) --- */
    CameraCtx *left_cam  = find_birdview_cam(app, BIRDVIEW_LEFT);
    CameraCtx *right_cam = find_birdview_cam(app, BIRDVIEW_RIGHT);
    CameraCtx *front_cam = find_birdview_cam(app, BIRDVIEW_FRONT);
    CameraCtx *rear_cam  = find_birdview_cam(app, BIRDVIEW_REAR);

    /* Side strip width based on rotated camera aspect ratio */
    CameraCtx *ref = left_cam ? left_cam : right_cam;
    double strip_w;
    if (ref && ref->started && ref->disp_w > 0)
        strip_w = (double)FUSED_H * ref->disp_h / ref->disp_w;
    else
        strip_w = (double)FUSED_H * 9.0 / 16.0;
    if (strip_w > FUSED_W * 0.425)
        strip_w = FUSED_W * 0.425;

    int sw = (int)strip_w;
    int veh_x = sw;
    int veh_w = FUSED_W - 2 * sw;
    int veh_h = (int)(FUSED_H * 0.35);
    int veh_y = (FUSED_H - veh_h) / 2;

    /* Camera slots (same layout as bird view page) */
    struct { CameraCtx *cam; int dx, dy, dw, dh; } slots[] = {
        { left_cam,  0,              0,            sw,              FUSED_H },
        { right_cam, FUSED_W - sw,   0,            sw,              FUSED_H },
        { front_cam, veh_x,          0,            veh_w,           veh_y },
        { rear_cam,  veh_x,          veh_y+veh_h,  veh_w,           FUSED_H-veh_y-veh_h },
    };

    for (int i = 0; i < 4; i++) {
        CameraCtx *cam = slots[i].cam;
        if (!cam || !cam->started || cam->disp_w == 0) continue;

        /* Copy pre-rotated buffer under mutex */
        int rw = 0, rh = 0;
        pthread_mutex_lock(&cam->buf_mutex);
        if (cam->bv_rotated && cam->bv_rot_w > 0) {
            rw = cam->bv_rot_w;
            rh = cam->bv_rot_h;
            int sz = rw * rh * 3;
            if (!st->bv_draw[i] || st->bv_draw_w[i] != rw ||
                st->bv_draw_h[i] != rh) {
                free(st->bv_draw[i]);
                st->bv_draw[i] = (uint8_t *)malloc(sz);
                st->bv_draw_w[i] = rw;
                st->bv_draw_h[i] = rh;
            }
            memcpy(st->bv_draw[i], cam->bv_rotated, sz);
        }
        pthread_mutex_unlock(&cam->buf_mutex);

        if (rw == 0 || !st->bv_draw[i]) continue;

        GdkPixbuf *pb = gdk_pixbuf_new_from_data(
            st->bv_draw[i], GDK_COLORSPACE_RGB, FALSE, 8,
            rw, rh, rw * 3, NULL, NULL);

        /* Map slot rect through the fit transform */
        double sx = fx + slots[i].dx * scale;
        double sy = fy + slots[i].dy * scale;
        double sdw = slots[i].dw * scale;
        double sdh = slots[i].dh * scale;

        cairo_save(cr);
        cairo_rectangle(cr, sx, sy, sdw, sdh);
        cairo_clip(cr);

        double img_sc = fmin(sdw / rw, sdh / rh);
        double ow = rw * img_sc;
        double oh = rh * img_sc;
        double ox = sx + (sdw - ow) / 2;
        double oy = sy + (sdh - oh) / 2;

        cairo_translate(cr, ox, oy);
        cairo_scale(cr, img_sc, img_sc);
        gdk_cairo_set_source_pixbuf(cr, pb, 0, 0);
        cairo_pattern_set_filter(cairo_get_source(cr), CAIRO_FILTER_BILINEAR);
        cairo_paint(cr);
        cairo_restore(cr);
        g_object_unref(pb);
    }

    /* --- Vehicle rectangle --- */
    double vx_s = fx + veh_x * scale;
    double vy_s = fy + veh_y * scale;
    double vw_s = veh_w * scale;
    double vh_s = veh_h * scale;

    cairo_set_source_rgba(cr, 0.3, 0.5, 0.8, 0.7);
    cairo_rectangle(cr, vx_s, vy_s, vw_s, vh_s);
    cairo_fill(cr);
    cairo_set_source_rgb(cr, 0.5, 0.7, 1.0);
    cairo_set_line_width(cr, 1.5);
    cairo_rectangle(cr, vx_s, vy_s, vw_s, vh_s);
    cairo_stroke(cr);

    /* Vehicle label */
    cairo_set_source_rgb(cr, 1, 1, 1);
    cairo_set_font_size(cr, 11);
    cairo_text_extents_t ext;
    cairo_text_extents(cr, "Vehicle", &ext);
    cairo_move_to(cr, vx_s + (vw_s - ext.width) / 2,
                  vy_s + (vh_s + ext.height) / 2);
    cairo_show_text(cr, "Vehicle");

    /* --- Grid overlay (semi-transparent) --- */
    cairo_set_source_rgba(cr, 0.5, 0.5, 0.5, 0.3);
    cairo_set_line_width(cr, 0.5);
    for (int gx = 0; gx <= FUSED_W; gx += 100) {
        double lx = fx + gx * scale;
        cairo_move_to(cr, lx, fy);
        cairo_line_to(cr, lx, fy + fh);
        cairo_stroke(cr);
    }
    for (int gy = 0; gy <= FUSED_H; gy += 100) {
        double ly = fy + gy * scale;
        cairo_move_to(cr, fx, ly);
        cairo_line_to(cr, fx + fw, ly);
        cairo_stroke(cr);
    }

    /* --- Draw ground markers --- */
    for (int i = 0; i < st->num_markers; i++) {
        double mx = fx + st->markers[i].world_x * scale;
        double my = fy + st->markers[i].world_y * scale;

        int linked = marker_has_link(st, i);
        if (linked)
            cairo_set_source_rgb(cr, 0, 0.8, 0);
        else
            cairo_set_source_rgb(cr, 0, 1, 0);

        cairo_arc(cr, mx, my, 6, 0, 2 * G_PI);
        cairo_fill_preserve(cr);
        cairo_set_source_rgb(cr, 1, 1, 1);
        cairo_set_line_width(cr, 1.5);
        cairo_stroke(cr);

        char num[8];
        snprintf(num, sizeof(num), "%d", i + 1);
        cairo_set_source_rgb(cr, 1, 1, 1);
        cairo_set_font_size(cr, 10);
        cairo_move_to(cr, mx + 9, my - 2);
        cairo_show_text(cr, num);
    }

    /* Placing indicator */
    if (st->placing) {
        cairo_set_source_rgba(cr, 1, 1, 0, 0.8);
        cairo_set_font_size(cr, 12);
        cairo_move_to(cr, fx + 4, fy + fh - 6);
        cairo_show_text(cr, "Click to place marker on ground");
    }

    /* Info bar */
    cairo_set_source_rgba(cr, 0, 0, 0, 0.6);
    cairo_rectangle(cr, 0, 0, aw, 22);
    cairo_fill(cr);
    cairo_set_source_rgb(cr, 1, 1, 1);
    cairo_set_font_size(cr, 12);
    cairo_move_to(cr, 6, 15);
    char info[128];
    snprintf(info, sizeof(info), "Ground View  %d markers  %dx%d",
             st->num_markers, FUSED_W, FUSED_H);
    cairo_show_text(cr, info);
}

static void draw_camera_view(BirdviewConfigState *st, cairo_t *cr,
                             int aw, int ah)
{
    if (st->sel_cam_idx < 0) {
        cairo_set_source_rgb(cr, 0.5, 0.5, 0.5);
        cairo_set_font_size(cr, 14);
        cairo_move_to(cr, 10, ah / 2);
        cairo_show_text(cr, "Select a camera");
        return;
    }

    CameraCtx *cam = &st->app->cameras[st->sel_cam_idx];

    /* Show warp preview if active */
    if (st->show_preview && st->warp_valid && st->warp_buf) {
        GdkPixbuf *pb = gdk_pixbuf_new_from_data(
            st->warp_buf, GDK_COLORSPACE_RGB, FALSE, 8,
            FUSED_W, FUSED_H, FUSED_W * 3, NULL, NULL);
        ui_draw_pixbuf_fitted(cr, pb, FUSED_W, FUSED_H, aw, ah,
                              NULL, NULL, NULL, NULL);
        g_object_unref(pb);

        cairo_set_source_rgba(cr, 0, 0, 0, 0.6);
        cairo_rectangle(cr, 0, 0, aw, 22);
        cairo_fill(cr);
        cairo_set_source_rgb(cr, 1, 1, 0);
        cairo_set_font_size(cr, 12);
        cairo_move_to(cr, 6, 15);
        cairo_show_text(cr, "Warp Preview (click Preview again to exit)");
        return;
    }

    if (cam->disp_w == 0 || !cam->started) {
        cairo_set_source_rgb(cr, 0.5, 0.5, 0.5);
        cairo_set_font_size(cr, 14);
        cairo_move_to(cr, 10, ah / 2);
        cairo_show_text(cr, "Camera not running");
        return;
    }

    /* Copy frame */
    pthread_mutex_lock(&cam->buf_mutex);
    memcpy(cam->draw_color, cam->color_rgb, cam->disp_size);
    pthread_mutex_unlock(&cam->buf_mutex);

    GdkPixbuf *pb = gdk_pixbuf_new_from_data(
        cam->draw_color, GDK_COLORSPACE_RGB, FALSE, 8,
        cam->disp_w, cam->disp_h, cam->disp_w * 3, NULL, NULL);

    double fx, fy, fw, fh;
    ui_draw_pixbuf_fitted(cr, pb, cam->disp_w, cam->disp_h, aw, ah,
                          &fx, &fy, &fw, &fh);
    g_object_unref(pb);

    st->fit_x = fx; st->fit_y = fy;
    st->fit_w = fw; st->fit_h = fh;

    /* Draw linked calibration points (green) */
    double sx = fw / cam->disp_w;
    double sy = fh / cam->disp_h;

    for (int i = 0; i < st->num_points; i++) {
        double px = fx + st->points[i].img_x * sx;
        double py = fy + st->points[i].img_y * sy;

        /* Green circle */
        cairo_set_source_rgb(cr, 0, 1, 0);
        cairo_arc(cr, px, py, 6, 0, 2 * G_PI);
        cairo_stroke(cr);

        /* Find which marker number this corresponds to */
        int mnum = 0;
        for (int j = 0; j < st->num_markers; j++) {
            if (fabs(st->points[i].world_x - st->markers[j].world_x) < 0.01 &&
                fabs(st->points[i].world_y - st->markers[j].world_y) < 0.01) {
                mnum = j + 1;
                break;
            }
        }

        char num[8];
        snprintf(num, sizeof(num), "%d", mnum ? mnum : (i + 1));
        cairo_set_font_size(cr, 10);
        cairo_move_to(cr, px + 8, py - 2);
        cairo_show_text(cr, num);
    }

    /* Linking indicator */
    if (st->linking_marker_id >= 0) {
        /* Find marker index for display */
        int midx = -1;
        for (int i = 0; i < st->num_markers; i++) {
            if (st->markers[i].id == st->linking_marker_id) {
                midx = i;
                break;
            }
        }
        if (midx >= 0) {
            cairo_set_source_rgba(cr, 1, 1, 0, 0.8);
            cairo_set_font_size(cr, 12);
            cairo_move_to(cr, fx + 4, fy + fh - 6);
            char msg[80];
            snprintf(msg, sizeof(msg),
                     "Click to link marker #%d (%.0f, %.0f)",
                     midx + 1, st->markers[midx].world_x,
                     st->markers[midx].world_y);
            cairo_show_text(cr, msg);
        }
    }

    /* Info bar */
    cairo_set_source_rgba(cr, 0, 0, 0, 0.6);
    cairo_rectangle(cr, 0, 0, aw, 22);
    cairo_fill(cr);
    cairo_set_source_rgb(cr, 1, 1, 1);
    cairo_set_font_size(cr, 12);
    cairo_move_to(cr, 6, 15);
    char info[128];
    snprintf(info, sizeof(info), "Camera View: %s  %d linked points  %dx%d",
             cam->name, st->num_points, cam->disp_w, cam->disp_h);
    cairo_show_text(cr, info);
}

static gboolean on_draw_bv_config(GtkWidget *widget, cairo_t *cr,
                                   gpointer data)
{
    BirdviewConfigState *st = (BirdviewConfigState *)data;
    int aw = gtk_widget_get_allocated_width(widget);
    int ah = gtk_widget_get_allocated_height(widget);

    cairo_set_source_rgb(cr, 0.1, 0.1, 0.1);
    cairo_paint(cr);

    if (st->ground_mode)
        draw_ground_view(st, cr, aw, ah);
    else
        draw_camera_view(st, cr, aw, ah);

    return FALSE;
}

/* --- Click handling --- */

static gboolean on_click_bv_config(GtkWidget *widget, GdkEventButton *ev,
                                    gpointer data)
{
    (void)widget;
    BirdviewConfigState *st = (BirdviewConfigState *)data;

    if (st->show_preview && !st->ground_mode) {
        st->show_preview = 0;
        gtk_widget_queue_draw(st->draw_area);
        return TRUE;
    }

    if (st->ground_mode) {
        /* Ground view: place marker */
        if (!st->placing) return FALSE;
        if (st->fit_w < 1 || st->fit_h < 1) return FALSE;

        /* Map click to world coordinates */
        double world_x = (ev->x - st->fit_x) * FUSED_W / st->fit_w;
        double world_y = (ev->y - st->fit_y) * FUSED_H / st->fit_h;

        if (world_x < 0 || world_y < 0 ||
            world_x >= FUSED_W || world_y >= FUSED_H)
            return TRUE;

        if (st->num_markers >= MAX_GROUND_MARKERS) {
            gtk_label_set_text(GTK_LABEL(st->lbl_status),
                               "Max markers reached");
            return TRUE;
        }

        db_ground_marker_insert(st->app->db, world_x, world_y);
        st->placing = 0;

        load_markers(st);
        populate_marker_list(st);
        gtk_widget_queue_draw(st->draw_area);

        char msg[64];
        snprintf(msg, sizeof(msg), "Marker placed (%d total)",
                 st->num_markers);
        gtk_label_set_text(GTK_LABEL(st->lbl_status), msg);
        return TRUE;
    } else {
        /* Camera view: link marker to feed */
        if (st->linking_marker_id < 0 || st->sel_cam_idx < 0)
            return FALSE;
        if (st->fit_w < 1 || st->fit_h < 1) return FALSE;

        CameraCtx *cam = &st->app->cameras[st->sel_cam_idx];

        double img_x = (ev->x - st->fit_x) * cam->disp_w / st->fit_w;
        double img_y = (ev->y - st->fit_y) * cam->disp_h / st->fit_h;

        if (img_x < 0 || img_y < 0 ||
            img_x >= cam->disp_w || img_y >= cam->disp_h)
            return TRUE;

        /* Find world coords from the selected ground marker */
        double wx = 0, wy = 0;
        int found = 0;
        for (int i = 0; i < st->num_markers; i++) {
            if (st->markers[i].id == st->linking_marker_id) {
                wx = st->markers[i].world_x;
                wy = st->markers[i].world_y;
                found = 1;
                break;
            }
        }
        if (!found) return TRUE;

        /* Remove any existing calib point for this camera with same world coords */
        for (int i = 0; i < st->num_points; i++) {
            if (fabs(st->points[i].world_x - wx) < 0.01 &&
                fabs(st->points[i].world_y - wy) < 0.01) {
                db_calib_delete(st->app->db, st->points[i].id);
                break;
            }
        }

        DbCalibPoint pt;
        pt.camera_id = cam->id;
        pt.img_x = img_x;
        pt.img_y = img_y;
        pt.world_x = wx;
        pt.world_y = wy;

        db_calib_insert(st->app->db, &pt);
        st->linking_marker_id = -1;

        load_points(st);
        populate_marker_list(st);
        gtk_widget_queue_draw(st->draw_area);

        char msg[64];
        snprintf(msg, sizeof(msg), "Point linked (%d total)", st->num_points);
        gtk_label_set_text(GTK_LABEL(st->lbl_status), msg);
        return TRUE;
    }
}

/* --- Timer --- */

static gboolean redraw_timer_cb(gpointer data)
{
    BirdviewConfigState *st = (BirdviewConfigState *)data;
    if (!st->active) return G_SOURCE_REMOVE;

    if (st->ground_mode) {
        /* Redraw ground view when any birdview camera updates */
        int needs = 0;
        for (int pos = BIRDVIEW_FRONT; pos <= BIRDVIEW_RIGHT; pos++) {
            CameraCtx *cam = find_birdview_cam(st->app, pos);
            if (cam && cam->buf_updated) {
                cam->buf_updated = 0;
                needs = 1;
            }
        }
        if (needs)
            gtk_widget_queue_draw(st->draw_area);
    } else if (st->sel_cam_idx >= 0 &&
               st->app->cameras[st->sel_cam_idx].buf_updated) {
        st->app->cameras[st->sel_cam_idx].buf_updated = 0;
        if (!st->show_preview)
            gtk_widget_queue_draw(st->draw_area);
    }
    return G_SOURCE_CONTINUE;
}

/* --- Button callbacks --- */

static void on_camera_changed(GtkComboBox *combo, gpointer data)
{
    BirdviewConfigState *st = (BirdviewConfigState *)data;
    int idx = gtk_combo_box_get_active(combo);
    if (idx < 0 || idx >= st->app->num_cameras) {
        st->sel_cam_idx = -1;
        return;
    }
    st->sel_cam_idx = idx;
    st->linking_marker_id = -1;
    st->show_preview = 0;
    st->warp_valid = 0;

    CameraCtx *cam = &st->app->cameras[idx];
    if (!cam->started && cam->enabled)
        camera_start(cam, st->app);

    load_points(st);
    populate_marker_list(st);
    gtk_widget_queue_draw(st->draw_area);
}

static void on_mode_toggle(GtkButton *btn, gpointer data)
{
    (void)btn;
    BirdviewConfigState *st = (BirdviewConfigState *)data;
    st->ground_mode = !st->ground_mode;
    st->placing = 0;
    st->linking_marker_id = -1;
    st->show_preview = 0;

    gtk_button_set_label(GTK_BUTTON(st->btn_mode),
                         st->ground_mode ? "Camera View" : "Ground View");

    if (st->ground_mode)
        gtk_label_set_text(GTK_LABEL(st->lbl_status),
                           "Ground view: place markers on canvas");
    else
        gtk_label_set_text(GTK_LABEL(st->lbl_status),
                           "Camera view: select marker then click feed to link");

    load_points(st);
    populate_marker_list(st);
    gtk_widget_queue_draw(st->draw_area);
}

static void on_place_marker(GtkButton *btn, gpointer data)
{
    (void)btn;
    BirdviewConfigState *st = (BirdviewConfigState *)data;
    if (!st->ground_mode) {
        gtk_label_set_text(GTK_LABEL(st->lbl_status),
                           "Switch to Ground View first");
        return;
    }
    if (st->num_markers >= MAX_GROUND_MARKERS) {
        gtk_label_set_text(GTK_LABEL(st->lbl_status),
                           "Max markers reached");
        return;
    }

    st->placing = 1;
    gtk_label_set_text(GTK_LABEL(st->lbl_status),
                       "Click on the ground canvas to place marker");
    gtk_widget_queue_draw(st->draw_area);
}

static void on_compute_clicked(GtkButton *btn, gpointer data)
{
    (void)btn;
    BirdviewConfigState *st = (BirdviewConfigState *)data;
    if (st->sel_cam_idx < 0 || st->num_points < 4) {
        gtk_label_set_text(GTK_LABEL(st->lbl_status),
                           "Need camera selected and >= 4 linked points");
        return;
    }

    CameraCtx *cam = &st->app->cameras[st->sel_cam_idx];

    CalibPoint pts[MAX_CALIB_POINTS];
    for (int i = 0; i < st->num_points; i++) {
        pts[i].img_x   = st->points[i].img_x;
        pts[i].img_y   = st->points[i].img_y;
        pts[i].world_x = st->points[i].world_x;
        pts[i].world_y = st->points[i].world_y;
    }

    Homography H;
    if (homography_compute(pts, st->num_points, &H) < 0) {
        gtk_label_set_text(GTK_LABEL(st->lbl_status),
                           "Homography failed (singular matrix)");
        return;
    }

    db_homography_save(st->app->db, cam->id, H.h);

    char msg[128];
    snprintf(msg, sizeof(msg), "Homography computed from %d points and saved",
             st->num_points);
    gtk_label_set_text(GTK_LABEL(st->lbl_status), msg);

    printf("[BirdviewConfig] H for camera %d:\n", cam->id);
    printf("  [%.6f %.6f %.6f]\n", H.h[0], H.h[1], H.h[2]);
    printf("  [%.6f %.6f %.6f]\n", H.h[3], H.h[4], H.h[5]);
    printf("  [%.6f %.6f %.6f]\n", H.h[6], H.h[7], H.h[8]);
    fflush(stdout);
}

static void on_preview_clicked(GtkButton *btn, gpointer data)
{
    (void)btn;
    BirdviewConfigState *st = (BirdviewConfigState *)data;
    if (st->sel_cam_idx < 0) return;

    if (st->show_preview) {
        st->show_preview = 0;
        gtk_widget_queue_draw(st->draw_area);
        return;
    }

    /* Switch to camera view for preview */
    if (st->ground_mode) {
        st->ground_mode = 0;
        gtk_button_set_label(GTK_BUTTON(st->btn_mode), "Ground View");
    }

    CameraCtx *cam = &st->app->cameras[st->sel_cam_idx];
    if (!cam->started || cam->disp_w == 0) return;

    double h[9];
    if (db_homography_load(st->app->db, cam->id, h) < 0) {
        gtk_label_set_text(GTK_LABEL(st->lbl_status),
                           "No homography - compute first");
        return;
    }

    Homography H, Hinv;
    memcpy(H.h, h, sizeof(h));
    if (homography_invert(&H, &Hinv) < 0) {
        gtk_label_set_text(GTK_LABEL(st->lbl_status),
                           "Homography inversion failed");
        return;
    }

    if (st->warp_lut.entries) {
        warp_lut_free(&st->warp_lut);
        memset(&st->warp_lut, 0, sizeof(st->warp_lut));
    }

    if (warp_lut_build(&Hinv, FUSED_W, FUSED_H,
                       cam->disp_w, cam->disp_h, &st->warp_lut) < 0) {
        gtk_label_set_text(GTK_LABEL(st->lbl_status),
                           "Warp LUT build failed");
        return;
    }

    if (!st->warp_buf)
        st->warp_buf = (uint8_t *)malloc(FUSED_W * FUSED_H * 3);

    pthread_mutex_lock(&cam->buf_mutex);
    memcpy(cam->draw_color, cam->color_rgb, cam->disp_size);
    pthread_mutex_unlock(&cam->buf_mutex);

    warp_lut_apply(&st->warp_lut, cam->draw_color, st->warp_buf);

    st->warp_valid = 1;
    st->show_preview = 1;
    gtk_widget_queue_draw(st->draw_area);
    gtk_label_set_text(GTK_LABEL(st->lbl_status), "Warp preview shown");
}

static void on_clear_clicked(GtkButton *btn, gpointer data)
{
    (void)btn;
    BirdviewConfigState *st = (BirdviewConfigState *)data;

    /* Clear all ground markers and all calib points for all cameras */
    db_ground_marker_clear_all(st->app->db);

    /* Clear calib points and homographies for birdview cameras */
    for (int i = 0; i < st->app->num_cameras; i++) {
        CameraCtx *cam = &st->app->cameras[i];
        if (cam->birdview_pos >= BIRDVIEW_FRONT &&
            cam->birdview_pos <= BIRDVIEW_RIGHT) {
            db_calib_clear_camera(st->app->db, cam->id);
            db_homography_delete(st->app->db, cam->id);
        }
    }

    load_markers(st);
    load_points(st);
    populate_marker_list(st);
    st->warp_valid = 0;
    st->show_preview = 0;
    gtk_widget_queue_draw(st->draw_area);
    gtk_label_set_text(GTK_LABEL(st->lbl_status), "All markers and calibration cleared");
}

static void populate_camera_combo(BirdviewConfigState *st)
{
    gtk_combo_box_text_remove_all(GTK_COMBO_BOX_TEXT(st->camera_combo));
    for (int i = 0; i < st->app->num_cameras; i++) {
        CameraCtx *cam = &st->app->cameras[i];
        if (cam->birdview_pos >= BIRDVIEW_FRONT &&
            cam->birdview_pos <= BIRDVIEW_RIGHT) {
            const char *pos_names[] = { "", "Front", "Rear", "Left", "Right" };
            char label[128];
            snprintf(label, sizeof(label), "%s (%s)", cam->name,
                     pos_names[cam->birdview_pos]);
            gtk_combo_box_text_append_text(
                GTK_COMBO_BOX_TEXT(st->camera_combo), label);
        } else {
            gtk_combo_box_text_append_text(
                GTK_COMBO_BOX_TEXT(st->camera_combo), cam->name);
        }
    }
}

/* --- Build / lifecycle --- */

static GtkWidget *birdview_config_build(AppCtx *app)
{
    BirdviewConfigState *st = (BirdviewConfigState *)calloc(1,
                                   sizeof(BirdviewConfigState));
    st->app = app;
    st->sel_cam_idx = -1;
    st->ground_mode = 1;
    st->linking_marker_id = -1;
    app->page_birdview_config_state = st;

    GtkWidget *hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 4);
    gtk_widget_set_margin_start(hbox, 4);
    gtk_widget_set_margin_end(hbox, 4);
    gtk_widget_set_margin_top(hbox, 4);
    gtk_widget_set_margin_bottom(hbox, 4);

    /* Left: main canvas */
    GtkWidget *left = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
    gtk_widget_set_hexpand(left, TRUE);
    gtk_widget_set_vexpand(left, TRUE);

    st->draw_area = gtk_drawing_area_new();
    gtk_widget_set_size_request(st->draw_area, 480, 360);
    gtk_widget_set_hexpand(st->draw_area, TRUE);
    gtk_widget_set_vexpand(st->draw_area, TRUE);
    gtk_widget_add_events(st->draw_area, GDK_BUTTON_PRESS_MASK);
    g_signal_connect(st->draw_area, "draw",
                     G_CALLBACK(on_draw_bv_config), st);
    g_signal_connect(st->draw_area, "button-press-event",
                     G_CALLBACK(on_click_bv_config), st);

    gtk_box_pack_start(GTK_BOX(left), st->draw_area, TRUE, TRUE, 0);

    st->lbl_status = ui_label("Ground view: place markers on canvas", 0);
    gtk_box_pack_start(GTK_BOX(left), st->lbl_status, FALSE, FALSE, 0);

    gtk_box_pack_start(GTK_BOX(hbox), left, TRUE, TRUE, 0);

    /* Right: controls */
    GtkWidget *right = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
    gtk_widget_set_size_request(right, 260, -1);
    gtk_widget_set_margin_start(right, 8);

    /* Camera selector */
    gtk_box_pack_start(GTK_BOX(right), ui_label("Camera", 1),
                       FALSE, FALSE, 0);
    st->camera_combo = gtk_combo_box_text_new();
    g_signal_connect(st->camera_combo, "changed",
                     G_CALLBACK(on_camera_changed), st);
    gtk_box_pack_start(GTK_BOX(right), st->camera_combo, FALSE, FALSE, 0);

    /* Mode toggle */
    st->btn_mode = ui_button("Camera View",
                              G_CALLBACK(on_mode_toggle), st);
    gtk_box_pack_start(GTK_BOX(right), st->btn_mode, FALSE, FALSE, 0);

    gtk_box_pack_start(GTK_BOX(right), ui_separator(), FALSE, FALSE, 4);

    /* Place marker button */
    st->btn_place = ui_button("Place Marker",
                               G_CALLBACK(on_place_marker), st);
    gtk_box_pack_start(GTK_BOX(right), st->btn_place, FALSE, FALSE, 0);

    /* Markers list */
    gtk_box_pack_start(GTK_BOX(right), ui_label("Markers", 1),
                       FALSE, FALSE, 0);

    st->marker_list = gtk_list_box_new();
    g_signal_connect(st->marker_list, "row-selected",
                     G_CALLBACK(on_marker_row_selected), st);
    GtkWidget *scroll = ui_scrolled(st->marker_list);
    gtk_widget_set_vexpand(scroll, TRUE);
    gtk_box_pack_start(GTK_BOX(right), scroll, TRUE, TRUE, 0);

    gtk_box_pack_start(GTK_BOX(right), ui_separator(), FALSE, FALSE, 4);

    /* Action buttons */
    st->btn_compute = ui_button("Compute Homography",
                                 G_CALLBACK(on_compute_clicked), st);
    gtk_widget_set_sensitive(st->btn_compute, FALSE);
    gtk_box_pack_start(GTK_BOX(right), st->btn_compute, FALSE, FALSE, 0);

    st->btn_preview = ui_button("Preview Warp",
                                 G_CALLBACK(on_preview_clicked), st);
    gtk_box_pack_start(GTK_BOX(right), st->btn_preview, FALSE, FALSE, 0);

    st->btn_clear = ui_button("Clear All",
                               G_CALLBACK(on_clear_clicked), st);
    gtk_box_pack_start(GTK_BOX(right), st->btn_clear, FALSE, FALSE, 4);

    gtk_box_pack_start(GTK_BOX(hbox), right, FALSE, FALSE, 0);

    return hbox;
}

static void birdview_config_on_show(AppCtx *app)
{
    BirdviewConfigState *st =
        (BirdviewConfigState *)app->page_birdview_config_state;
    if (!st) return;

    camera_start_all(app);
    app->birdview_visible = 1;
    populate_camera_combo(st);
    load_markers(st);
    load_points(st);
    populate_marker_list(st);

    st->active = 1;
    st->redraw_timer = g_timeout_add(33, redraw_timer_cb, st);
}

static void birdview_config_on_hide(AppCtx *app)
{
    BirdviewConfigState *st =
        (BirdviewConfigState *)app->page_birdview_config_state;
    if (!st) return;
    st->active = 0;
    app->birdview_visible = 0;
}

static void birdview_config_cleanup(AppCtx *app)
{
    BirdviewConfigState *st =
        (BirdviewConfigState *)app->page_birdview_config_state;
    if (st) {
        if (st->warp_lut.entries)
            warp_lut_free(&st->warp_lut);
        free(st->warp_buf);
        for (int i = 0; i < 4; i++)
            free(st->bv_draw[i]);
        free(st);
        app->page_birdview_config_state = NULL;
    }
}

const PageDef page_birdview_config = {
    .id      = "birdview_config",
    .title   = "BirdView Config",
    .build   = birdview_config_build,
    .on_show = birdview_config_on_show,
    .on_hide = birdview_config_on_hide,
    .cleanup = birdview_config_cleanup,
};
