/*
 * core/homography.c - DLT homography solver and warp LUT
 *
 * Solves Ah=0 via SVD-free DLT: normalize h22=1, solve 8x8 linear
 * system with Gaussian elimination + partial pivoting.
 */
#include "homography.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#ifdef __aarch64__
#include <arm_neon.h>
#endif

/* Solve 8x8 linear system A*x = b via Gaussian elimination
 * with partial pivoting. A is 8x8 (row-major), b is 8-vector.
 * Solution written to x. Returns 0 on success. */
static int solve_8x8(double A[8][8], double b[8], double x[8])
{
    /* Augmented matrix */
    double M[8][9];
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++)
            M[i][j] = A[i][j];
        M[i][8] = b[i];
    }

    /* Forward elimination with partial pivoting */
    for (int col = 0; col < 8; col++) {
        /* Find pivot */
        int pivot = col;
        double max_val = fabs(M[col][col]);
        for (int row = col + 1; row < 8; row++) {
            if (fabs(M[row][col]) > max_val) {
                max_val = fabs(M[row][col]);
                pivot = row;
            }
        }
        if (max_val < 1e-12)
            return -1; /* Singular */

        /* Swap rows */
        if (pivot != col) {
            for (int j = 0; j < 9; j++) {
                double tmp = M[col][j];
                M[col][j] = M[pivot][j];
                M[pivot][j] = tmp;
            }
        }

        /* Eliminate below */
        for (int row = col + 1; row < 8; row++) {
            double factor = M[row][col] / M[col][col];
            for (int j = col; j < 9; j++)
                M[row][j] -= factor * M[col][j];
        }
    }

    /* Back substitution */
    for (int i = 7; i >= 0; i--) {
        x[i] = M[i][8];
        for (int j = i + 1; j < 8; j++)
            x[i] -= M[i][j] * x[j];
        x[i] /= M[i][i];
    }

    return 0;
}

int homography_compute(const CalibPoint *pts, int n, Homography *H)
{
    if (n < 4)
        return -1;

    /* Build overdetermined system for DLT with h22=1.
     * For each point pair (x,y) -> (X,Y):
     *   x*h00 + y*h01 + h02 - x*X*h20 - y*X*h21 = X
     *   x*h10 + y*h11 + h12 - x*Y*h20 - y*Y*h21 = Y
     * 8 unknowns: h00..h21 (h22=1).
     *
     * For n>4, use least-squares normal equations: A^T A x = A^T b.
     */
    double AtA[8][8];
    double Atb[8];
    memset(AtA, 0, sizeof(AtA));
    memset(Atb, 0, sizeof(Atb));

    for (int k = 0; k < n; k++) {
        double x = pts[k].img_x;
        double y = pts[k].img_y;
        double X = pts[k].world_x;
        double Y = pts[k].world_y;

        /* Row 1: [x, y, 1, 0, 0, 0, -x*X, -y*X] */
        double r1[8] = { x, y, 1, 0, 0, 0, -x*X, -y*X };
        /* Row 2: [0, 0, 0, x, y, 1, -x*Y, -y*Y] */
        double r2[8] = { 0, 0, 0, x, y, 1, -x*Y, -y*Y };

        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 8; j++) {
                AtA[i][j] += r1[i] * r1[j] + r2[i] * r2[j];
            }
            Atb[i] += r1[i] * X + r2[i] * Y;
        }
    }

    double sol[8];
    if (solve_8x8(AtA, Atb, sol) < 0)
        return -1;

    H->h[0] = sol[0]; H->h[1] = sol[1]; H->h[2] = sol[2];
    H->h[3] = sol[3]; H->h[4] = sol[4]; H->h[5] = sol[5];
    H->h[6] = sol[6]; H->h[7] = sol[7]; H->h[8] = 1.0;

    return 0;
}

int homography_invert(const Homography *H, Homography *Hinv)
{
    const double *m = H->h;

    double det = m[0] * (m[4]*m[8] - m[5]*m[7])
               - m[1] * (m[3]*m[8] - m[5]*m[6])
               + m[2] * (m[3]*m[7] - m[4]*m[6]);

    if (fabs(det) < 1e-12)
        return -1;

    double inv_det = 1.0 / det;
    double *o = Hinv->h;

    o[0] =  (m[4]*m[8] - m[5]*m[7]) * inv_det;
    o[1] = -(m[1]*m[8] - m[2]*m[7]) * inv_det;
    o[2] =  (m[1]*m[5] - m[2]*m[4]) * inv_det;
    o[3] = -(m[3]*m[8] - m[5]*m[6]) * inv_det;
    o[4] =  (m[0]*m[8] - m[2]*m[6]) * inv_det;
    o[5] = -(m[0]*m[5] - m[2]*m[3]) * inv_det;
    o[6] =  (m[3]*m[7] - m[4]*m[6]) * inv_det;
    o[7] = -(m[0]*m[7] - m[1]*m[6]) * inv_det;
    o[8] =  (m[0]*m[4] - m[1]*m[3]) * inv_det;

    return 0;
}

int warp_lut_build(const Homography *Hinv, int out_w, int out_h,
                   int src_w, int src_h, WarpLUT *lut)
{
    lut->out_w = out_w;
    lut->out_h = out_h;
    lut->src_w = src_w;
    lut->src_h = src_h;
    lut->entries = (WarpEntry *)malloc(out_w * out_h * sizeof(WarpEntry));
    if (!lut->entries)
        return -1;

    const double *h = Hinv->h;

    for (int y = 0; y < out_h; y++) {
        for (int x = 0; x < out_w; x++) {
            /* Apply inverse homography: output -> source */
            double w = h[6] * x + h[7] * y + h[8];
            double sx, sy;
            if (fabs(w) < 1e-10) {
                sx = -1;
                sy = -1;
            } else {
                sx = (h[0] * x + h[1] * y + h[2]) / w;
                sy = (h[3] * x + h[4] * y + h[5]) / w;
            }

            WarpEntry *e = &lut->entries[y * out_w + x];
            if (sx < 0 || sy < 0 || sx >= src_w - 1 || sy >= src_h - 1) {
                e->sx = -1;
                e->sy = -1;
            } else {
                /* 16.8 fixed-point */
                e->sx = (int32_t)(sx * 256.0 + 0.5);
                e->sy = (int32_t)(sy * 256.0 + 0.5);
            }
        }
    }

    return 0;
}

void warp_lut_apply(const WarpLUT *lut, const uint8_t *src, uint8_t *out)
{
    int ow = lut->out_w;
    int oh = lut->out_h;
    int sw = lut->src_w;
    int total = ow * oh;

    for (int i = 0; i < total; i++) {
        const WarpEntry *e = &lut->entries[i];
        uint8_t *dst = out + i * 3;

        if (e->sx < 0) {
            dst[0] = dst[1] = dst[2] = 0;
            continue;
        }

        int ix = e->sx >> 8;
        int iy = e->sy >> 8;
        int fx = e->sx & 0xFF;
        int fy = e->sy & 0xFF;

        const uint8_t *p00 = src + (iy * sw + ix) * 3;
        const uint8_t *p10 = p00 + 3;
        const uint8_t *p01 = p00 + sw * 3;
        const uint8_t *p11 = p01 + 3;

#ifdef __aarch64__
        /* NEON: process 3 RGB channels in parallel using uint16 pipeline.
         * Horizontal blend uses uint16 weights to avoid overflow when
         * fx=0 (256-fx=256 doesn't fit uint8). Max per-channel result
         * after horizontal blend: 255*256 = 65280, fits uint16. */
        uint16x8_t w00 = vmovl_u8(vld1_u8(p00));
        uint16x8_t w10 = vmovl_u8(vld1_u8(p10));
        uint16x8_t w01 = vmovl_u8(vld1_u8(p01));
        uint16x8_t w11 = vmovl_u8(vld1_u8(p11));

        uint16x8_t vfx  = vdupq_n_u16((uint16_t)fx);
        uint16x8_t vnfx = vdupq_n_u16((uint16_t)(256 - fx));
        uint16x8_t top = vmulq_u16(w00, vnfx);
        top = vmlaq_u16(top, w10, vfx);
        uint16x8_t bot = vmulq_u16(w01, vnfx);
        bot = vmlaq_u16(bot, w11, vfx);

        /* Vertical blend: (top*(256-fy) + bot*fy) >> 16 with rounding */
        uint16x8_t vfy  = vdupq_n_u16((uint16_t)fy);
        uint16x8_t vnfy = vdupq_n_u16((uint16_t)(256 - fy));
        uint32x4_t lo32 = vmull_u16(vget_low_u16(top), vget_low_u16(vnfy));
        lo32 = vmlal_u16(lo32, vget_low_u16(bot), vget_low_u16(vfy));
        uint16x4_t lo16 = vrshrn_n_u32(lo32, 16);

        /* Extract 3 channels from lo16 */
        dst[0] = (uint8_t)vget_lane_u16(lo16, 0);
        dst[1] = (uint8_t)vget_lane_u16(lo16, 1);
        dst[2] = (uint8_t)vget_lane_u16(lo16, 2);
#else
        for (int c = 0; c < 3; c++) {
            int v = ((p00[c] * (256 - fx) + p10[c] * fx) * (256 - fy) +
                     (p01[c] * (256 - fx) + p11[c] * fx) * fy + 32768)
                    >> 16;
            dst[c] = (uint8_t)(v > 255 ? 255 : v);
        }
#endif
    }
}

void warp_lut_free(WarpLUT *lut)
{
    free(lut->entries);
    lut->entries = NULL;
}
