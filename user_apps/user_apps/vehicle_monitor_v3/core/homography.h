/*
 * core/homography.h - Homography computation and warp LUT
 *
 * DLT (Direct Linear Transform) for computing 3x3 homography
 * from 4+ point correspondences. Warp lookup table with
 * fixed-point bilinear interpolation.
 */
#ifndef HOMOGRAPHY_H
#define HOMOGRAPHY_H

#include <stdint.h>

/* Calibration point pair */
typedef struct {
    double img_x, img_y;
    double world_x, world_y;
} CalibPoint;

/* 3x3 homography matrix (row-major) */
typedef struct {
    double h[9]; /* h[0..2] = row0, h[3..5] = row1, h[6..8] = row2 */
} Homography;

/* Warp lookup table entry (16.8 fixed-point source coordinates) */
typedef struct {
    int32_t sx, sy; /* source x,y in 8-bit fractional (val * 256) */
} WarpEntry;

/* Warp lookup table for one camera */
typedef struct {
    WarpEntry *entries;  /* out_w * out_h entries */
    int        out_w, out_h;
    int        src_w, src_h;
} WarpLUT;

/* Compute homography from n>=4 point pairs using DLT.
 * Returns 0 on success, -1 on failure (singular matrix). */
int homography_compute(const CalibPoint *pts, int n, Homography *H);

/* Invert a 3x3 homography. Returns 0 on success. */
int homography_invert(const Homography *H, Homography *Hinv);

/* Build warp LUT: for each output pixel, compute source pixel via Hinv.
 * Caller must free lut->entries when done. */
int warp_lut_build(const Homography *Hinv, int out_w, int out_h,
                   int src_w, int src_h, WarpLUT *lut);

/* Apply warp LUT to produce output image from source RGB24 buffer.
 * out must be out_w * out_h * 3 bytes. */
void warp_lut_apply(const WarpLUT *lut, const uint8_t *src,
                    uint8_t *out);

/* Free warp LUT entries */
void warp_lut_free(WarpLUT *lut);

#endif /* HOMOGRAPHY_H */
