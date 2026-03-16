/*
 * core/motion.h - Motion detection algorithms
 *
 * Extracted from motion_detect_v4: blur, background model, grid diff.
 */
#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>

/* 3x3 box blur for noise suppression (Y-plane) */
void motion_blur_3x3(const uint8_t *src, uint8_t *dst, int w, int h);

/* Initialize background model from first blurred frame */
void motion_bg_init(float *bg, const uint8_t *blurred, int size);

/* Update background model with exponential moving average */
void motion_bg_update(float *bg, const uint8_t *blurred, int size, float alpha);

/*
 * Grid-based motion detection.
 * Returns max cell percentage. Fills cell_pcts array [grid_rows * grid_cols].
 * total_count = total pixels above threshold across all cells.
 * max_col/max_row = cell with highest motion %.
 */
float motion_bg_diff_grid(const uint8_t *blurred, const float *bg,
                          int w, int h, int threshold,
                          int grid_cols, int grid_rows,
                          float *cell_pcts, int *total_count,
                          int *max_col, int *max_row);

/*
 * Find largest connected component (blob) of active cells.
 * A cell is "active" when cell_pcts[r*cols+c] >= active_pct.
 * Uses 8-connectivity (includes diagonal neighbors).
 * Returns largest blob size (in cells). Sets blob_col/blob_row to centroid.
 */
int motion_find_blobs(const float *cell_pcts, int cols, int rows,
                      float active_pct, int *blob_col, int *blob_row);

/*
 * Visualize motion: green for above-threshold, gray for noise.
 * Output is RGB24 at disp_w x disp_h (nearest-neighbor from src).
 */
void motion_visualize(const uint8_t *blurred, const float *bg,
                      int src_w, int src_h, int threshold,
                      int disp_w, int disp_h, uint8_t *vis_rgb);

#endif /* MOTION_H */
