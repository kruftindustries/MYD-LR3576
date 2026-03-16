/*
 * ui/util.c - Reusable widget factories
 */
#include "util.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

GtkWidget *ui_label(const char *text, int bold)
{
    GtkWidget *lbl = gtk_label_new(NULL);
    if (bold) {
        char markup[256];
        snprintf(markup, sizeof(markup), "<b>%s</b>", text);
        gtk_label_set_markup(GTK_LABEL(lbl), markup);
    } else {
        gtk_label_set_text(GTK_LABEL(lbl), text);
    }
    gtk_label_set_xalign(GTK_LABEL(lbl), 0.0);
    return lbl;
}

GtkWidget *ui_label_expand(const char *text, int bold)
{
    GtkWidget *lbl = ui_label(text, bold);
    gtk_widget_set_hexpand(lbl, TRUE);
    return lbl;
}

GtkWidget *ui_spin_int(int val, int lo, int hi, int step,
                       GCallback cb, gpointer data)
{
    GtkAdjustment *adj = gtk_adjustment_new(val, lo, hi, step, step, 0);
    GtkWidget *spin = gtk_spin_button_new(adj, 1.0, 0);
    if (cb)
        g_signal_connect(spin, "value-changed", cb, data);
    return spin;
}

GtkWidget *ui_spin_float(double val, double lo, double hi,
                         double step, int digits,
                         GCallback cb, gpointer data)
{
    GtkAdjustment *adj = gtk_adjustment_new(val, lo, hi, step, step, 0);
    GtkWidget *spin = gtk_spin_button_new(adj, 1.0, digits);
    if (cb)
        g_signal_connect(spin, "value-changed", cb, data);
    return spin;
}

GtkWidget *ui_entry(const char *text, int max_len,
                    GCallback changed_cb, gpointer data)
{
    GtkWidget *entry = gtk_entry_new();
    if (text)
        gtk_entry_set_text(GTK_ENTRY(entry), text);
    if (max_len > 0)
        gtk_entry_set_max_length(GTK_ENTRY(entry), max_len);
    gtk_widget_set_hexpand(entry, TRUE);
    if (changed_cb)
        g_signal_connect(entry, "changed", changed_cb, data);
    return entry;
}

GtkWidget *ui_frame(const char *title, GtkWidget *child)
{
    GtkWidget *frame = gtk_frame_new(title);
    gtk_container_add(GTK_CONTAINER(frame), child);
    return frame;
}

GtkWidget *ui_grid(void)
{
    GtkWidget *grid = gtk_grid_new();
    gtk_grid_set_row_spacing(GTK_GRID(grid), 6);
    gtk_grid_set_column_spacing(GTK_GRID(grid), 8);
    gtk_widget_set_margin_start(grid, 8);
    gtk_widget_set_margin_end(grid, 8);
    gtk_widget_set_margin_top(grid, 4);
    gtk_widget_set_margin_bottom(grid, 4);
    return grid;
}

int ui_grid_row(GtkWidget *grid, int row,
                const char *label, GtkWidget *widget)
{
    gtk_grid_attach(GTK_GRID(grid), ui_label(label, 0), 0, row, 1, 1);
    gtk_widget_set_hexpand(widget, TRUE);
    gtk_grid_attach(GTK_GRID(grid), widget, 1, row, 1, 1);
    return row + 1;
}

GtkWidget *ui_separator(void)
{
    return gtk_separator_new(GTK_ORIENTATION_HORIZONTAL);
}

GtkWidget *ui_placeholder(const char *title, const char *subtitle)
{
    GtkWidget *box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 8);
    gtk_widget_set_valign(box, GTK_ALIGN_CENTER);
    gtk_widget_set_halign(box, GTK_ALIGN_CENTER);

    GtkWidget *lbl_title = ui_label(title, 1);
    gtk_label_set_xalign(GTK_LABEL(lbl_title), 0.5);
    gtk_box_pack_start(GTK_BOX(box), lbl_title, FALSE, FALSE, 0);

    if (subtitle) {
        GtkWidget *lbl_sub = ui_label(subtitle, 0);
        gtk_label_set_xalign(GTK_LABEL(lbl_sub), 0.5);
        gtk_box_pack_start(GTK_BOX(box), lbl_sub, FALSE, FALSE, 0);
    }
    return box;
}

GtkWidget *ui_scrolled(GtkWidget *child)
{
    GtkWidget *sw = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(sw),
                                  GTK_POLICY_AUTOMATIC,
                                  GTK_POLICY_AUTOMATIC);
    gtk_container_add(GTK_CONTAINER(sw), child);
    return sw;
}

GtkWidget *ui_button(const char *label, GCallback cb, gpointer data)
{
    GtkWidget *btn = gtk_button_new_with_label(label);
    if (cb)
        g_signal_connect(btn, "clicked", cb, data);
    return btn;
}

GtkWidget *ui_color_button(const char *hex)
{
    GdkRGBA rgba = {0.0, 0.0, 0.0, 1.0};
    if (hex)
        gdk_rgba_parse(&rgba, hex);
    GtkWidget *btn = gtk_color_button_new_with_rgba(&rgba);
    return btn;
}

void ui_parse_color(const char *hex, double *r, double *g, double *b)
{
    unsigned int ri = 0, gi = 0, bi = 0;
    if (hex && hex[0] == '#' && strlen(hex) >= 7)
        sscanf(hex + 1, "%02x%02x%02x", &ri, &gi, &bi);
    *r = ri / 255.0;
    *g = gi / 255.0;
    *b = bi / 255.0;
}

void ui_draw_pixbuf_fitted(cairo_t *cr, GdkPixbuf *pb,
                           int img_w, int img_h, int area_w, int area_h,
                           double *out_x, double *out_y,
                           double *out_w, double *out_h)
{
    double sc = fmin((double)area_w / img_w, (double)area_h / img_h);
    double fw = img_w * sc;
    double fh = img_h * sc;
    double fx = (area_w - fw) / 2.0;
    double fy = (area_h - fh) / 2.0;

    cairo_save(cr);
    cairo_translate(cr, fx, fy);
    cairo_scale(cr, sc, sc);
    gdk_cairo_set_source_pixbuf(cr, pb, 0, 0);
    cairo_pattern_set_filter(cairo_get_source(cr), CAIRO_FILTER_BILINEAR);
    cairo_paint(cr);
    cairo_restore(cr);

    if (out_x) *out_x = fx;
    if (out_y) *out_y = fy;
    if (out_w) *out_w = fw;
    if (out_h) *out_h = fh;
}
