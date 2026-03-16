/*
 * core/undistort.c - Radial lens distortion correction
 *
 * Brown-Conrady radial model: for each undistorted output pixel (x,y),
 * compute the distorted source pixel via:
 *   r² = ((x-cx)/max_r)² + ((y-cy)/max_r)²
 *   scale = 1 + k1*r² + k2*r⁴
 *   src_x = cx + (x - cx) * scale
 *   src_y = cy + (y - cy) * scale
 *
 * Fitting uses grid search over (k1, k2) minimizing line-straightness
 * error measured via PCA on undistorted calibration points.
 */
#include "undistort.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

/* Undistort a single point: given a distorted pixel coordinate,
 * compute the undistorted coordinate via iterative Newton's method.
 * The forward model maps undistorted → distorted, so we invert it. */
static void undistort_point(double dx, double dy,
                            double cx, double cy, double max_r,
                            double k1, double k2,
                            double *ux, double *uy)
{
    /* Initial guess: the distorted point itself */
    double x = dx;
    double y = dy;

    for (int iter = 0; iter < 20; iter++) {
        double rx = (x - cx) / max_r;
        double ry = (y - cy) / max_r;
        double r2 = rx * rx + ry * ry;
        double r4 = r2 * r2;
        double scale = 1.0 + k1 * r2 + k2 * r4;

        /* Forward: distorted = cx + (x-cx)*scale */
        double fx = cx + (x - cx) * scale;
        double fy = cy + (y - cy) * scale;

        /* Error */
        double ex = fx - dx;
        double ey = fy - dy;

        /* Jacobian approximation: d(distorted)/d(undistorted) ≈ scale
         * (ignoring the r-dependent derivative for stability) */
        double dscale_dr2 = k1 + 2.0 * k2 * r2;
        double j = scale + 2.0 * dscale_dr2 * r2;
        if (fabs(j) < 1e-10)
            break;

        x -= ex / j;
        y -= ey / j;

        if (ex * ex + ey * ey < 1e-6)
            break;
    }

    *ux = x;
    *uy = y;
}

/* Compute line-straightness error for a set of 2D points using PCA.
 * Returns the smallest eigenvalue of the 2x2 covariance matrix,
 * which equals the variance perpendicular to the best-fit line. */
static double line_error(const double *xs, const double *ys, int n)
{
    if (n < 3)
        return 0.0;

    /* Compute centroid */
    double mx = 0, my = 0;
    for (int i = 0; i < n; i++) {
        mx += xs[i];
        my += ys[i];
    }
    mx /= n;
    my /= n;

    /* 2x2 covariance matrix */
    double cxx = 0, cxy = 0, cyy = 0;
    for (int i = 0; i < n; i++) {
        double dx = xs[i] - mx;
        double dy = ys[i] - my;
        cxx += dx * dx;
        cxy += dx * dy;
        cyy += dy * dy;
    }
    cxx /= n;
    cxy /= n;
    cyy /= n;

    /* Smallest eigenvalue of [[cxx, cxy], [cxy, cyy]] */
    double trace = cxx + cyy;
    double det = cxx * cyy - cxy * cxy;
    double disc = trace * trace - 4.0 * det;
    if (disc < 0) disc = 0;
    double lambda_min = (trace - sqrt(disc)) * 0.5;

    return lambda_min;
}

/* Evaluate total straightness error for a given (k1, k2) */
static double eval_error(const LensLinePoint *pts, int npts,
                         double cx, double cy, double max_r,
                         double k1, double k2,
                         int w, int h)
{
    double total_err = 0.0;

    /* Find distinct line indices and process each */
    int processed[MAX_LENS_LINES];
    int nprocessed = 0;

    for (int i = 0; i < npts; i++) {
        int lidx = pts[i].line_idx;

        /* Check if already processed */
        int found = 0;
        for (int j = 0; j < nprocessed; j++) {
            if (processed[j] == lidx) { found = 1; break; }
        }
        if (found) continue;
        if (nprocessed >= MAX_LENS_LINES) break;
        processed[nprocessed++] = lidx;

        /* Collect points for this line */
        double uxs[MAX_LINE_POINTS], uys[MAX_LINE_POINTS];
        int count = 0;
        for (int j = 0; j < npts && count < MAX_LINE_POINTS; j++) {
            if (pts[j].line_idx != lidx)
                continue;
            /* Convert fraction to pixel coordinates */
            double dx = pts[j].px * w;
            double dy = pts[j].py * h;
            undistort_point(dx, dy, cx, cy, max_r, k1, k2,
                            &uxs[count], &uys[count]);
            count++;
        }

        if (count >= 3)
            total_err += line_error(uxs, uys, count);
    }

    return total_err;
}

int undistort_fit(const LensLinePoint *pts, int npts,
                  double cx_frac, double cy_frac, int w, int h,
                  double *out_k1, double *out_k2)
{
    if (npts < 6)  /* need at least 2 lines × 3 points */
        return -1;

    /* Validate: at least 2 lines with >= 3 points each */
    int line_counts[MAX_LENS_LINES];
    int line_ids[MAX_LENS_LINES];
    int nlines = 0;

    for (int i = 0; i < npts; i++) {
        int lidx = pts[i].line_idx;
        int found = -1;
        for (int j = 0; j < nlines; j++) {
            if (line_ids[j] == lidx) { found = j; break; }
        }
        if (found >= 0) {
            line_counts[found]++;
        } else if (nlines < MAX_LENS_LINES) {
            line_ids[nlines] = lidx;
            line_counts[nlines] = 1;
            nlines++;
        }
    }

    int valid_lines = 0;
    for (int i = 0; i < nlines; i++) {
        if (line_counts[i] >= 3)
            valid_lines++;
    }
    if (valid_lines < 2)
        return -1;

    double cx = cx_frac * w;
    double cy = cy_frac * h;
    double max_r = sqrt((w * 0.5) * (w * 0.5) + (h * 0.5) * (h * 0.5));

    /* Coarse grid search: k1 in [-0.8, 0.8], k2 in [-0.3, 0.3] */
    double best_k1 = 0, best_k2 = 0;
    double best_err = 1e30;

    for (double tk1 = -0.8; tk1 <= 0.801; tk1 += 0.02) {
        for (double tk2 = -0.3; tk2 <= 0.301; tk2 += 0.01) {
            double err = eval_error(pts, npts, cx, cy, max_r,
                                    tk1, tk2, w, h);
            if (err < best_err) {
                best_err = err;
                best_k1 = tk1;
                best_k2 = tk2;
            }
        }
    }

    /* Fine grid search around best coarse result */
    double fine_k1 = best_k1, fine_k2 = best_k2;
    double fine_err = best_err;

    for (double tk1 = best_k1 - 0.02; tk1 <= best_k1 + 0.0201; tk1 += 0.002) {
        for (double tk2 = best_k2 - 0.01; tk2 <= best_k2 + 0.0101; tk2 += 0.001) {
            double err = eval_error(pts, npts, cx, cy, max_r,
                                    tk1, tk2, w, h);
            if (err < fine_err) {
                fine_err = err;
                fine_k1 = tk1;
                fine_k2 = tk2;
            }
        }
    }

    *out_k1 = fine_k1;
    *out_k2 = fine_k2;

    printf("[undistort] Fit: k1=%.4f k2=%.4f (error=%.6f)\n",
           fine_k1, fine_k2, fine_err);

    return 0;
}

int undistort_lut_build(const LensParams *p, int w, int h, WarpLUT *lut)
{
    lut->out_w = w;
    lut->out_h = h;
    lut->src_w = w;
    lut->src_h = h;
    lut->entries = (WarpEntry *)malloc(w * h * sizeof(WarpEntry));
    if (!lut->entries)
        return -1;

    double cx = p->cx_frac * w;
    double cy = p->cy_frac * h;
    double max_r = sqrt((w * 0.5) * (w * 0.5) + (h * 0.5) * (h * 0.5));
    double inv_max_r = 1.0 / max_r;

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            /* Normalized distance from optical center */
            double rx = (x - cx) * inv_max_r;
            double ry = (y - cy) * inv_max_r;
            double r2 = rx * rx + ry * ry;
            double r4 = r2 * r2;
            double scale = 1.0 + p->k1 * r2 + p->k2 * r4;

            /* Source (distorted) pixel */
            double sx = cx + (x - cx) * scale;
            double sy = cy + (y - cy) * scale;

            WarpEntry *e = &lut->entries[y * w + x];
            if (sx < 0 || sy < 0 || sx >= w - 1 || sy >= h - 1) {
                e->sx = -1;
                e->sy = -1;
            } else {
                e->sx = (int32_t)(sx * 256.0 + 0.5);
                e->sy = (int32_t)(sy * 256.0 + 0.5);
            }
        }
    }

    printf("[undistort] LUT built: %dx%d k1=%.4f k2=%.4f cx=%.2f cy=%.2f\n",
           w, h, p->k1, p->k2, p->cx_frac, p->cy_frac);

    return 0;
}
