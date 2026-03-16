/*
 * core/undistort.h - Radial lens distortion correction
 *
 * 2-parameter Brown-Conrady model (k1, k2) with straight-line
 * calibration fitter. Builds a WarpLUT for per-frame correction
 * using the same NEON-optimized warp path as birdview.
 */
#ifndef UNDISTORT_H
#define UNDISTORT_H

#include "homography.h"  /* WarpLUT, WarpEntry, warp_lut_apply, warp_lut_free */

#define MAX_LENS_LINES      16
#define MAX_LINE_POINTS      32
#define MAX_LENS_TOTAL_PTS  (MAX_LENS_LINES * MAX_LINE_POINTS)

typedef struct {
    double k1, k2;
    double cx_frac, cy_frac;  /* optical center as fraction of image (0.5 = center) */
} LensParams;

typedef struct {
    int    line_idx;
    double px, py;  /* point as fraction of image (0.0-1.0) */
} LensLinePoint;

/* Build undistortion WarpLUT. Same WarpLUT struct as birdview warp.
 * For each output pixel, computes the distorted source coordinate.
 * Returns 0 on success. Caller frees via warp_lut_free(). */
int undistort_lut_build(const LensParams *p, int w, int h, WarpLUT *lut);

/* Fit k1, k2 from calibration lines (straight-line constraint).
 * Needs >= 2 lines with >= 3 points each. Returns 0 on success.
 * cx_frac/cy_frac are the assumed optical center (typically 0.5, 0.5).
 * w, h are the image dimensions for normalizing coordinates. */
int undistort_fit(const LensLinePoint *pts, int npts,
                  double cx_frac, double cy_frac, int w, int h,
                  double *out_k1, double *out_k2);

#endif /* UNDISTORT_H */
