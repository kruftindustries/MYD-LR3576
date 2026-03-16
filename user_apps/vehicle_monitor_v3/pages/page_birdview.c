/*
 * pages/page_birdview.c - Overhead 4-camera bird view composition
 *
 * Pre-composes the entire bird view into an off-screen buffer using
 * software 90° rotation (NEON on aarch64) and nearest-neighbor scaling.
 * The draw callback does a single cairo_paint of the pre-composed surface.
 */
#include "../app.h"
#include "../core/camera.h"
#include "../core/database.h"
#include "../core/homography.h"
#include "../ui/util.h"
#include <cairo.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#ifdef __aarch64__
#include <arm_neon.h>
#endif

#define FUSED_W 800
#define FUSED_H 600

/* Fixed render resolution for the pre-composed buffer */
#define RENDER_W 960
#define RENDER_H 540

typedef struct {
    AppCtx     *app;
    GtkWidget  *draw_area;
    GtkWidget  *btn_mode;
    GtkWidget  *spn_veh_w;
    GtkWidget  *spn_veh_l;
    int         active;
    guint       redraw_timer;
    int         fused_mode;       /* 0=simple, 1=fused */
    WarpLUT     fused_luts[4];    /* one per birdview position */
    int         fused_valid[4];
    uint8_t    *fused_buf;        /* FUSED_W * FUSED_H * 3 */

    /* Vehicle real-world dimensions (feet) */
    double      veh_width_ft;
    double      veh_length_ft;

    /* Vehicle image (optional, loaded from vehicle.png) */
    GdkPixbuf  *veh_pixbuf;

    /* Hover tracking */
    double      hover_x, hover_y;
    int         hover_active;

    /* Pre-composed frame buffer (RENDER_W x RENDER_H x 4 BGRA) */
    cairo_surface_t *render_surface;
} BirdViewState;

static CameraCtx *find_birdview_cam(AppCtx *app, int pos)
{
    for (int i = 0; i < app->num_cameras; i++) {
        if (app->cameras[i].birdview_pos == pos &&
            app->cameras[i].enabled)
            return &app->cameras[i];
    }
    return NULL;
}

/*
 * Nearest-neighbor blit of RGB src into BGRA dst at given rect.
 * Scales src (sw x sh) to fit (dw x dh) using fmin (no crop).
 * Centers within the rect.
 */
static void blit_rgb_to_bgra(const uint8_t *src, int sw, int sh,
                              uint8_t *dst, int dst_stride, int dst_h,
                              int dx, int dy, int dw, int dh)
{
    if (sw <= 0 || sh <= 0 || dw <= 0 || dh <= 0) return;

    double sc = fmin((double)dw / sw, (double)dh / sh);
    int rw = (int)(sw * sc);
    int rh = (int)(sh * sc);
    int ox = dx + (dw - rw) / 2;
    int oy = dy + (dh - rh) / 2;

    for (int y = 0; y < rh; y++) {
        int sy = y * sh / rh;
        if (sy >= sh) sy = sh - 1;
        int out_y = oy + y;
        if (out_y < 0 || out_y >= dst_h) continue;
        uint8_t *row = dst + out_y * dst_stride;
        const uint8_t *srow = src + sy * sw * 3;

        for (int x = 0; x < rw; x++) {
            int sx = x * sw / rw;
            if (sx >= sw) sx = sw - 1;
            int out_x = ox + x;
            if (out_x < 0 || out_x >= dst_stride / 4) continue;

            const uint8_t *sp = srow + sx * 3;
            uint8_t *dp = row + out_x * 4;
            dp[0] = sp[2];  /* B */
            dp[1] = sp[1];  /* G */
            dp[2] = sp[0];  /* R */
            dp[3] = 255;    /* A */
        }
    }
}

/* Fill a rect in BGRA buffer with a solid color */
static void fill_bgra_rect(uint8_t *dst, int stride, int dst_h,
                            int x, int y, int w, int h,
                            uint8_t r, uint8_t g, uint8_t b)
{
    for (int row = y; row < y + h && row < dst_h; row++) {
        if (row < 0) continue;
        uint8_t *p = dst + row * stride;
        for (int col = x; col < x + w && col < stride / 4; col++) {
            if (col < 0) continue;
            uint8_t *dp = p + col * 4;
            dp[0] = b; dp[1] = g; dp[2] = r; dp[3] = 255;
        }
    }
}

static void build_fused_luts(BirdViewState *st)
{
    for (int i = 0; i < 4; i++) {
        if (st->fused_luts[i].entries) {
            warp_lut_free(&st->fused_luts[i]);
            memset(&st->fused_luts[i], 0, sizeof(WarpLUT));
        }
        st->fused_valid[i] = 0;
    }

    for (int pos = BIRDVIEW_FRONT; pos <= BIRDVIEW_RIGHT; pos++) {
        CameraCtx *cam = find_birdview_cam(st->app, pos);
        if (!cam || !cam->started || cam->disp_w == 0) continue;

        double h[9];
        if (db_homography_load(st->app->db, cam->id, h) < 0) continue;

        Homography H, Hinv;
        memcpy(H.h, h, sizeof(h));
        if (homography_invert(&H, &Hinv) < 0) continue;

        int idx = pos - BIRDVIEW_FRONT;
        if (warp_lut_build(&Hinv, FUSED_W, FUSED_H,
                           cam->disp_w, cam->disp_h,
                           &st->fused_luts[idx]) == 0)
            st->fused_valid[idx] = 1;
    }
}

static void render_fused(BirdViewState *st)
{
    if (!st->fused_buf)
        st->fused_buf = (uint8_t *)calloc(FUSED_W * FUSED_H * 3, 1);
    else
        memset(st->fused_buf, 0, FUSED_W * FUSED_H * 3);

    for (int pos = BIRDVIEW_FRONT; pos <= BIRDVIEW_RIGHT; pos++) {
        int idx = pos - BIRDVIEW_FRONT;
        if (!st->fused_valid[idx]) continue;

        CameraCtx *cam = find_birdview_cam(st->app, pos);
        if (!cam || !cam->started || cam->disp_w == 0) continue;

        pthread_mutex_lock(&cam->buf_mutex);
        memcpy(cam->draw_color, cam->color_rgb, cam->disp_size);
        pthread_mutex_unlock(&cam->buf_mutex);

        uint8_t *tmp = (uint8_t *)malloc(FUSED_W * FUSED_H * 3);
        warp_lut_apply(&st->fused_luts[idx], cam->draw_color, tmp);

        int total = FUSED_W * FUSED_H;
#ifdef __aarch64__
        int neon_count = total & ~15;
        for (int i = 0; i < neon_count; i += 16) {
            int off = i * 3;
            uint8x16x3_t src = vld3q_u8(tmp + off);
            uint8x16x3_t dst = vld3q_u8(st->fused_buf + off);
            uint8x16_t mask = vorrq_u8(vorrq_u8(src.val[0], src.val[1]),
                                       src.val[2]);
            uint8x16_t nz = vcgtq_u8(mask, vdupq_n_u8(0));
            dst.val[0] = vbslq_u8(nz, src.val[0], dst.val[0]);
            dst.val[1] = vbslq_u8(nz, src.val[1], dst.val[1]);
            dst.val[2] = vbslq_u8(nz, src.val[2], dst.val[2]);
            vst3q_u8(st->fused_buf + off, dst);
        }
        for (int i = neon_count; i < total; i++) {
            int off = i * 3;
            if (tmp[off] || tmp[off+1] || tmp[off+2]) {
                st->fused_buf[off]   = tmp[off];
                st->fused_buf[off+1] = tmp[off+1];
                st->fused_buf[off+2] = tmp[off+2];
            }
        }
#else
        for (int i = 0; i < total; i++) {
            int off = i * 3;
            if (tmp[off] || tmp[off+1] || tmp[off+2]) {
                st->fused_buf[off]   = tmp[off];
                st->fused_buf[off+1] = tmp[off+1];
                st->fused_buf[off+2] = tmp[off+2];
            }
        }
#endif
        free(tmp);
    }
}

static void on_mode_toggle(GtkButton *btn, gpointer data)
{
    BirdViewState *st = (BirdViewState *)data;
    st->fused_mode = !st->fused_mode;
    gtk_button_set_label(btn, st->fused_mode ? "Simple" : "Fused");

    if (st->fused_mode)
        build_fused_luts(st);

    gtk_widget_queue_draw(st->draw_area);
}

static void on_veh_size_changed(GtkSpinButton *spin, gpointer data)
{
    (void)spin;
    BirdViewState *st = (BirdViewState *)data;

    st->veh_width_ft = gtk_spin_button_get_value(
        GTK_SPIN_BUTTON(st->spn_veh_w));
    st->veh_length_ft = gtk_spin_button_get_value(
        GTK_SPIN_BUTTON(st->spn_veh_l));

    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", st->veh_width_ft);
    db_setting_set(st->app->db, "bv_veh_width_ft", buf);
    snprintf(buf, sizeof(buf), "%.1f", st->veh_length_ft);
    db_setting_set(st->app->db, "bv_veh_length_ft", buf);

    gtk_widget_queue_draw(st->draw_area);
}

static gboolean on_motion_notify(GtkWidget *widget, GdkEventMotion *ev,
                                 gpointer data)
{
    (void)widget;
    BirdViewState *st = (BirdViewState *)data;
    st->hover_x = ev->x;
    st->hover_y = ev->y;
    st->hover_active = 1;
    gtk_widget_queue_draw(st->draw_area);
    return FALSE;
}

static gboolean on_leave_notify(GtkWidget *widget, GdkEventCrossing *ev,
                                gpointer data)
{
    (void)widget;
    (void)ev;
    BirdViewState *st = (BirdViewState *)data;
    st->hover_active = 0;
    gtk_widget_queue_draw(st->draw_area);
    return FALSE;
}

/*
 * Pre-compose the bird view into the render_surface.
 * All camera rotation/scaling done in software with nearest-neighbor.
 */
static void compose_birdview(BirdViewState *st)
{
    if (!st->render_surface) {
        st->render_surface = cairo_image_surface_create(
            CAIRO_FORMAT_ARGB32, RENDER_W, RENDER_H);
    }

    uint8_t *buf = cairo_image_surface_get_data(st->render_surface);
    int stride = cairo_image_surface_get_stride(st->render_surface);

    cairo_surface_flush(st->render_surface);

    /* Clear to dark background */
    for (int y = 0; y < RENDER_H; y++) {
        uint8_t *row = buf + y * stride;
        for (int x = 0; x < RENDER_W; x++) {
            uint8_t *p = row + x * 4;
            p[0] = 31; p[1] = 31; p[2] = 31; p[3] = 255;
        }
    }

    /* Compute layout at render resolution */
    CameraCtx *left_cam  = find_birdview_cam(st->app, BIRDVIEW_LEFT);
    CameraCtx *right_cam = find_birdview_cam(st->app, BIRDVIEW_RIGHT);
    CameraCtx *front_cam = find_birdview_cam(st->app, BIRDVIEW_FRONT);
    CameraCtx *rear_cam  = find_birdview_cam(st->app, BIRDVIEW_REAR);

    CameraCtx *ref = left_cam ? left_cam : right_cam;
    double strip_w;
    if (ref && ref->started && ref->disp_w > 0)
        strip_w = (double)RENDER_H * ref->disp_h / ref->disp_w;
    else
        strip_w = (double)RENDER_H * 9.0 / 16.0;
    if (strip_w > RENDER_W * 0.425)
        strip_w = RENDER_W * 0.425;

    int sw = (int)strip_w;
    int veh_x = sw;
    int veh_w = RENDER_W - 2 * sw;
    int veh_h = (int)(RENDER_H * 0.35);
    int veh_y = (RENDER_H - veh_h) / 2;

    /* Blit pre-rotated camera feeds (rotation done in worker threads) */
    struct { CameraCtx *cam; int dx, dy, dw, dh; } slots[] = {
        { left_cam,  0,           0,     sw,               RENDER_H },
        { right_cam, RENDER_W-sw, 0,     sw,               RENDER_H },
        { front_cam, veh_x,       0,     veh_w,            veh_y },
        { rear_cam,  veh_x,       veh_y+veh_h, veh_w,     RENDER_H-veh_y-veh_h },
    };
    for (int i = 0; i < 4; i++) {
        CameraCtx *cam = slots[i].cam;
        if (!cam || !cam->started || cam->disp_w == 0) continue;

        pthread_mutex_lock(&cam->buf_mutex);
        if (cam->bv_rotated && cam->bv_rot_w > 0) {
            blit_rgb_to_bgra(cam->bv_rotated, cam->bv_rot_w, cam->bv_rot_h,
                              buf, stride, RENDER_H,
                              slots[i].dx, slots[i].dy,
                              slots[i].dw, slots[i].dh);
        }
        pthread_mutex_unlock(&cam->buf_mutex);
    }

    /* Vehicle rectangle */
    fill_bgra_rect(buf, stride, RENDER_H,
                   veh_x, veh_y, veh_w, veh_h, 77, 128, 204);

    /* Vehicle outline (2px border) */
    fill_bgra_rect(buf, stride, RENDER_H, veh_x, veh_y, veh_w, 2, 128, 179, 255);
    fill_bgra_rect(buf, stride, RENDER_H, veh_x, veh_y + veh_h - 2, veh_w, 2, 128, 179, 255);
    fill_bgra_rect(buf, stride, RENDER_H, veh_x, veh_y, 2, veh_h, 128, 179, 255);
    fill_bgra_rect(buf, stride, RENDER_H, veh_x + veh_w - 2, veh_y, 2, veh_h, 128, 179, 255);

    cairo_surface_mark_dirty(st->render_surface);
}

static gboolean on_draw_birdview(GtkWidget *widget, cairo_t *cr,
                                 gpointer data)
{
    BirdViewState *st = (BirdViewState *)data;
    int aw = gtk_widget_get_allocated_width(widget);
    int ah = gtk_widget_get_allocated_height(widget);

    /* Fused mode */
    if (st->fused_mode) {
        render_fused(st);
        if (st->fused_buf) {
            GdkPixbuf *pb = gdk_pixbuf_new_from_data(
                st->fused_buf, GDK_COLORSPACE_RGB, FALSE, 8,
                FUSED_W, FUSED_H, FUSED_W * 3, NULL, NULL);
            ui_draw_pixbuf_fitted(cr, pb, FUSED_W, FUSED_H, aw, ah,
                                  NULL, NULL, NULL, NULL);
            g_object_unref(pb);
        }
        cairo_set_source_rgba(cr, 0, 0, 0, 0.5);
        cairo_rectangle(cr, 0, 0, aw, 22);
        cairo_fill(cr);
        cairo_set_source_rgb(cr, 1, 1, 0);
        cairo_set_font_size(cr, 12);
        cairo_move_to(cr, 6, 15);
        cairo_show_text(cr, "Fused Birdview");
        return FALSE;
    }

    /* Pre-compose entire frame in software */
    compose_birdview(st);

    /* Single scaled blit of the pre-composed surface */
    double sx = (double)aw / RENDER_W;
    double sy = (double)ah / RENDER_H;
    double sc = fmin(sx, sy);
    double ox = (aw - RENDER_W * sc) / 2;
    double oy = (ah - RENDER_H * sc) / 2;

    cairo_save(cr);
    cairo_translate(cr, ox, oy);
    cairo_scale(cr, sc, sc);
    cairo_set_source_surface(cr, st->render_surface, 0, 0);
    cairo_pattern_set_filter(cairo_get_source(cr), CAIRO_FILTER_NEAREST);
    cairo_paint(cr);
    cairo_restore(cr);
    /* Vehicle label overlay (drawn on widget surface for crisp text) */
    double strip_w_sc;
    {
        CameraCtx *ref = find_birdview_cam(st->app, BIRDVIEW_LEFT);
        if (!ref) ref = find_birdview_cam(st->app, BIRDVIEW_RIGHT);
        if (ref && ref->started && ref->disp_w > 0)
            strip_w_sc = (double)RENDER_H * ref->disp_h / ref->disp_w;
        else
            strip_w_sc = (double)RENDER_H * 9.0 / 16.0;
        if (strip_w_sc > RENDER_W * 0.425) strip_w_sc = RENDER_W * 0.425;
    }
    double vw_sc = RENDER_W - 2 * strip_w_sc;
    double vh_sc = RENDER_H * 0.35;
    double vx_widget = ox + strip_w_sc * sc;
    double vy_widget = oy + ((RENDER_H - vh_sc) / 2) * sc;
    double vw_widget = vw_sc * sc;
    double vh_widget = vh_sc * sc;

    /* Direction arrow */
    double arrow_x = vx_widget + vw_widget / 2;
    double arrow_y = vy_widget + 8;
    cairo_set_source_rgba(cr, 1, 1, 1, 0.7);
    cairo_move_to(cr, arrow_x, arrow_y - 5);
    cairo_line_to(cr, arrow_x - 6, arrow_y + 5);
    cairo_line_to(cr, arrow_x + 6, arrow_y + 5);
    cairo_close_path(cr);
    cairo_fill(cr);

    /* Vehicle size label */
    cairo_set_source_rgb(cr, 1, 1, 1);
    cairo_set_font_size(cr, 11);
    char vlbl[32];
    snprintf(vlbl, sizeof(vlbl), "%.1f x %.1f ft",
             st->veh_width_ft, st->veh_length_ft);
    cairo_text_extents_t ext;
    cairo_text_extents(cr, vlbl, &ext);
    cairo_move_to(cr, vx_widget + (vw_widget - ext.width) / 2,
                  vy_widget + (vh_widget + ext.height) / 2);
    cairo_show_text(cr, vlbl);

    /* Hover camera labels */
    if (st->hover_active) {
        /* Map hover coords back to render space to determine which camera */
        double rx = (st->hover_x - ox) / sc;
        double ry = (st->hover_y - oy) / sc;
        int sw = (int)strip_w_sc;
        int veh_y_r = (RENDER_H - (int)(RENDER_H * 0.35)) / 2;

        CameraCtx *hover_cam = NULL;
        double lx = 0, ly = 0, lw = 0;
        if (rx >= 0 && rx < sw) {
            hover_cam = find_birdview_cam(st->app, BIRDVIEW_LEFT);
            lx = ox; ly = oy; lw = sw * sc;
        } else if (rx >= RENDER_W - sw && rx < RENDER_W) {
            hover_cam = find_birdview_cam(st->app, BIRDVIEW_RIGHT);
            lx = ox + (RENDER_W - sw) * sc; ly = oy; lw = sw * sc;
        } else if (ry >= 0 && ry < veh_y_r && rx >= sw && rx < RENDER_W - sw) {
            hover_cam = find_birdview_cam(st->app, BIRDVIEW_FRONT);
            lx = ox + sw * sc; ly = oy; lw = (RENDER_W - 2 * sw) * sc;
        } else if (ry >= veh_y_r + (int)(RENDER_H * 0.35) && rx >= sw && rx < RENDER_W - sw) {
            hover_cam = find_birdview_cam(st->app, BIRDVIEW_REAR);
            lx = ox + sw * sc; ly = oy + (veh_y_r + (int)(RENDER_H * 0.35)) * sc;
            lw = (RENDER_W - 2 * sw) * sc;
        }

        if (hover_cam && hover_cam->name[0]) {
            cairo_set_source_rgba(cr, 0, 0, 0, 0.5);
            cairo_rectangle(cr, lx, ly, lw, 18);
            cairo_fill(cr);
            cairo_set_source_rgb(cr, 1, 1, 1);
            cairo_set_font_size(cr, 10);
            cairo_move_to(cr, lx + 4, ly + 13);
            cairo_show_text(cr, hover_cam->name);
        }
    }

    return FALSE;
}

static gboolean redraw_timer_cb(gpointer data)
{
    BirdViewState *st = (BirdViewState *)data;
    if (!st->active)
        return G_SOURCE_REMOVE;

    int needs_redraw = 0;
    for (int pos = BIRDVIEW_FRONT; pos <= BIRDVIEW_RIGHT; pos++) {
        CameraCtx *cam = find_birdview_cam(st->app, pos);
        if (cam && cam->buf_updated) {
            cam->buf_updated = 0;
            needs_redraw = 1;
        }
    }
    if (needs_redraw)
        gtk_widget_queue_draw(st->draw_area);

    return G_SOURCE_CONTINUE;
}

static GtkWidget *birdview_build(AppCtx *app)
{
    BirdViewState *st = (BirdViewState *)calloc(1, sizeof(BirdViewState));
    st->app = app;
    app->page_birdview_state = st;

    /* Load vehicle dimensions from settings */
    st->veh_width_ft  = db_setting_get_double(app->db, "bv_veh_width_ft", 6.5);
    st->veh_length_ft = db_setting_get_double(app->db, "bv_veh_length_ft", 18.0);
    if (st->veh_width_ft < 3.0) st->veh_width_ft = 3.0;
    if (st->veh_width_ft > 15.0) st->veh_width_ft = 15.0;
    if (st->veh_length_ft < 5.0) st->veh_length_ft = 5.0;
    if (st->veh_length_ft > 60.0) st->veh_length_ft = 60.0;

    /* Try to load vehicle image */
    st->veh_pixbuf = gdk_pixbuf_new_from_file("vehicle.png", NULL);
    if (!st->veh_pixbuf)
        st->veh_pixbuf = gdk_pixbuf_new_from_file(
            "/root/vehicle_monitor_v2/vehicle.png", NULL);

    GtkWidget *vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
    gtk_widget_set_margin_start(vbox, 4);
    gtk_widget_set_margin_end(vbox, 4);
    gtk_widget_set_margin_top(vbox, 4);
    gtk_widget_set_margin_bottom(vbox, 4);

    GtkWidget *top_bar = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
    GtkWidget *lbl = ui_label(
        "Assign cameras to birdview positions in Camera Config", 0);
    gtk_box_pack_start(GTK_BOX(top_bar), lbl, TRUE, TRUE, 0);

    st->btn_mode = gtk_button_new_with_label("Fused");
    g_signal_connect(st->btn_mode, "clicked",
                     G_CALLBACK(on_mode_toggle), st);
    gtk_box_pack_end(GTK_BOX(top_bar), st->btn_mode, FALSE, FALSE, 0);

    /* Vehicle dimension spin buttons (feet) */
    GtkWidget *lbl_l = ui_label("L ft", 0);
    st->spn_veh_l = ui_spin_float(st->veh_length_ft, 5.0, 60.0, 0.5, 1,
                                  G_CALLBACK(on_veh_size_changed), st);
    gtk_widget_set_size_request(st->spn_veh_l, 70, -1);
    gtk_box_pack_end(GTK_BOX(top_bar), st->spn_veh_l, FALSE, FALSE, 0);
    gtk_box_pack_end(GTK_BOX(top_bar), lbl_l, FALSE, FALSE, 0);

    GtkWidget *lbl_w = ui_label("W ft", 0);
    st->spn_veh_w = ui_spin_float(st->veh_width_ft, 3.0, 15.0, 0.5, 1,
                                  G_CALLBACK(on_veh_size_changed), st);
    gtk_widget_set_size_request(st->spn_veh_w, 70, -1);
    gtk_box_pack_end(GTK_BOX(top_bar), st->spn_veh_w, FALSE, FALSE, 0);
    gtk_box_pack_end(GTK_BOX(top_bar), lbl_w, FALSE, FALSE, 0);

    gtk_box_pack_start(GTK_BOX(vbox), top_bar, FALSE, FALSE, 0);

    st->draw_area = gtk_drawing_area_new();
    gtk_widget_set_hexpand(st->draw_area, TRUE);
    gtk_widget_set_vexpand(st->draw_area, TRUE);
    gtk_widget_set_size_request(st->draw_area, 640, 480);
    gtk_widget_add_events(st->draw_area,
        GDK_POINTER_MOTION_MASK | GDK_LEAVE_NOTIFY_MASK);
    g_signal_connect(st->draw_area, "draw",
                     G_CALLBACK(on_draw_birdview), st);
    g_signal_connect(st->draw_area, "motion-notify-event",
                     G_CALLBACK(on_motion_notify), st);
    g_signal_connect(st->draw_area, "leave-notify-event",
                     G_CALLBACK(on_leave_notify), st);

    gtk_box_pack_start(GTK_BOX(vbox), st->draw_area, TRUE, TRUE, 0);

    return vbox;
}

static void birdview_on_show(AppCtx *app)
{
    BirdViewState *st = (BirdViewState *)app->page_birdview_state;
    if (!st) return;

    camera_start_all(app);
    app->birdview_visible = 1;

    st->active = 1;
    st->redraw_timer = g_timeout_add(33, redraw_timer_cb, st);
}

static void birdview_on_hide(AppCtx *app)
{
    BirdViewState *st = (BirdViewState *)app->page_birdview_state;
    if (!st) return;
    st->active = 0;
    app->birdview_visible = 0;
}

static void birdview_cleanup(AppCtx *app)
{
    BirdViewState *st = (BirdViewState *)app->page_birdview_state;
    if (st) {
        for (int i = 0; i < 4; i++) {
            if (st->fused_luts[i].entries)
                warp_lut_free(&st->fused_luts[i]);
        }
        free(st->fused_buf);
        if (st->veh_pixbuf)
            g_object_unref(st->veh_pixbuf);
        if (st->render_surface)
            cairo_surface_destroy(st->render_surface);
        free(st);
        app->page_birdview_state = NULL;
    }
}

const PageDef page_birdview = {
    .id      = "birdview",
    .title   = "Bird View",
    .build   = birdview_build,
    .on_show = birdview_on_show,
    .on_hide = birdview_on_hide,
    .cleanup = birdview_cleanup,
};
