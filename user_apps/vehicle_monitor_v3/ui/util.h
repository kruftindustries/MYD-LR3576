/*
 * ui/util.h - Reusable widget factories
 */
#ifndef UI_UTIL_H
#define UI_UTIL_H

#include <gtk/gtk.h>

/* Label with optional bold markup */
GtkWidget *ui_label(const char *text, int bold);

/* Left-aligned label that expands horizontally */
GtkWidget *ui_label_expand(const char *text, int bold);

/* Integer spin button with callback */
GtkWidget *ui_spin_int(int val, int lo, int hi, int step,
                       GCallback cb, gpointer data);

/* Float spin button with callback */
GtkWidget *ui_spin_float(double val, double lo, double hi,
                         double step, int digits,
                         GCallback cb, gpointer data);

/* Text entry with initial value and optional callback */
GtkWidget *ui_entry(const char *text, int max_len,
                    GCallback changed_cb, gpointer data);

/* Labeled frame containing a child widget */
GtkWidget *ui_frame(const char *title, GtkWidget *child);

/* GtkGrid with standard spacing and margins */
GtkWidget *ui_grid(void);

/* Convenience: attach label + widget pair to grid row, returns next row */
int ui_grid_row(GtkWidget *grid, int row,
                const char *label, GtkWidget *widget);

/* Horizontal separator */
GtkWidget *ui_separator(void);

/* Placeholder page with centered message */
GtkWidget *ui_placeholder(const char *title, const char *subtitle);

/* Scrolled window wrapping a child */
GtkWidget *ui_scrolled(GtkWidget *child);

/* Button with label and callback */
GtkWidget *ui_button(const char *label, GCallback cb, gpointer data);

/* Color button initialized from hex string "#RRGGBB" */
GtkWidget *ui_color_button(const char *hex);

/* Parse hex color string to GdkRGBA */
void ui_parse_color(const char *hex, double *r, double *g, double *b);

/* Draw pixbuf fitted (uniform scale, centered/letterboxed) into area.
 * Returns the fitted rect via out_x/out_y/out_w/out_h (may be NULL). */
void ui_draw_pixbuf_fitted(cairo_t *cr, GdkPixbuf *pb,
                           int img_w, int img_h, int area_w, int area_h,
                           double *out_x, double *out_y,
                           double *out_w, double *out_h);

#endif /* UI_UTIL_H */
