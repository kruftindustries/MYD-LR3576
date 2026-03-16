/*
 * core/motion.c - Motion detection algorithms (NEON optimized)
 *
 * ARM NEON SIMD intrinsics for Cortex-A72 (RK3576).
 * All hot loops process 16 uint8 pixels or 4 float32 values per iteration.
 */
#include "motion.h"
#include <stdlib.h>
#include <string.h>
#include <arm_neon.h>

/* ---- Helper: convert 16 bg floats to uint8 with rounding ---- */
static inline uint8x16_t bg_floats_to_u8(const float *bg)
{
    uint32x4_t u0 = vcvtnq_u32_f32(vld1q_f32(bg));
    uint32x4_t u1 = vcvtnq_u32_f32(vld1q_f32(bg + 4));
    uint32x4_t u2 = vcvtnq_u32_f32(vld1q_f32(bg + 8));
    uint32x4_t u3 = vcvtnq_u32_f32(vld1q_f32(bg + 12));
    uint8x8_t lo = vmovn_u16(vcombine_u16(vmovn_u32(u0), vmovn_u32(u1)));
    uint8x8_t hi = vmovn_u16(vcombine_u16(vmovn_u32(u2), vmovn_u32(u3)));
    return vcombine_u8(lo, hi);
}

/* ---- 3x3 Box Blur (NEON) ---- */

void motion_blur_3x3(const uint8_t *src, uint8_t *dst, int w, int h)
{
    /* Copy edge rows */
    memcpy(dst, src, w);
    memcpy(dst + (h - 1) * w, src + (h - 1) * w, w);

    /* Copy edge columns */
    for (int y = 1; y < h - 1; y++) {
        dst[y * w] = src[y * w];
        dst[y * w + w - 1] = src[y * w + w - 1];
    }

    /* Interior: NEON 3x3 average, 16 pixels per iteration.
     * Divide by 9 via SQRDMULH: (2 * sum * 3641 + 32768) >> 16 ≈ sum / 9 */
    const int16x8_t div9 = vdupq_n_s16(3641);

    for (int y = 1; y < h - 1; y++) {
        const uint8_t *r0 = src + (y - 1) * w;
        const uint8_t *r1 = src + y * w;
        const uint8_t *r2 = src + (y + 1) * w;
        uint8_t *out = dst + y * w;

        int x = 1;
        for (; x <= w - 17; x += 16) {
            /* Load 3 shifted vectors per row (9 loads) */
            uint8x16_t v00 = vld1q_u8(r0 + x - 1);
            uint8x16_t v01 = vld1q_u8(r0 + x);
            uint8x16_t v02 = vld1q_u8(r0 + x + 1);
            uint8x16_t v10 = vld1q_u8(r1 + x - 1);
            uint8x16_t v11 = vld1q_u8(r1 + x);
            uint8x16_t v12 = vld1q_u8(r1 + x + 1);
            uint8x16_t v20 = vld1q_u8(r2 + x - 1);
            uint8x16_t v21 = vld1q_u8(r2 + x);
            uint8x16_t v22 = vld1q_u8(r2 + x + 1);

            /* Widen to u16 and sum — low 8 pixels */
            uint16x8_t s_lo = vaddl_u8(vget_low_u8(v00), vget_low_u8(v01));
            s_lo = vaddw_u8(s_lo, vget_low_u8(v02));
            s_lo = vaddw_u8(s_lo, vget_low_u8(v10));
            s_lo = vaddw_u8(s_lo, vget_low_u8(v11));
            s_lo = vaddw_u8(s_lo, vget_low_u8(v12));
            s_lo = vaddw_u8(s_lo, vget_low_u8(v20));
            s_lo = vaddw_u8(s_lo, vget_low_u8(v21));
            s_lo = vaddw_u8(s_lo, vget_low_u8(v22));

            /* High 8 pixels */
            uint16x8_t s_hi = vaddl_u8(vget_high_u8(v00), vget_high_u8(v01));
            s_hi = vaddw_u8(s_hi, vget_high_u8(v02));
            s_hi = vaddw_u8(s_hi, vget_high_u8(v10));
            s_hi = vaddw_u8(s_hi, vget_high_u8(v11));
            s_hi = vaddw_u8(s_hi, vget_high_u8(v12));
            s_hi = vaddw_u8(s_hi, vget_high_u8(v20));
            s_hi = vaddw_u8(s_hi, vget_high_u8(v21));
            s_hi = vaddw_u8(s_hi, vget_high_u8(v22));

            /* Divide by 9 */
            int16x8_t d_lo = vqrdmulhq_s16(vreinterpretq_s16_u16(s_lo), div9);
            int16x8_t d_hi = vqrdmulhq_s16(vreinterpretq_s16_u16(s_hi), div9);

            /* Narrow to u8 */
            uint8x16_t result = vcombine_u8(vqmovun_s16(d_lo),
                                            vqmovun_s16(d_hi));
            vst1q_u8(out + x, result);
        }

        /* Scalar tail */
        for (; x < w - 1; x++) {
            int sum = 0;
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++)
                    sum += src[(y + dy) * w + (x + dx)];
            out[x] = (uint8_t)(sum / 9);
        }
    }
}

/* ---- Background Init (NEON) ---- */

void motion_bg_init(float *bg, const uint8_t *blurred, int size)
{
    int i = 0;
    for (; i <= size - 16; i += 16) {
        uint8x16_t px = vld1q_u8(blurred + i);
        uint16x8_t lo16 = vmovl_u8(vget_low_u8(px));
        uint16x8_t hi16 = vmovl_u8(vget_high_u8(px));

        vst1q_f32(bg + i,      vcvtq_f32_u32(vmovl_u16(vget_low_u16(lo16))));
        vst1q_f32(bg + i + 4,  vcvtq_f32_u32(vmovl_u16(vget_high_u16(lo16))));
        vst1q_f32(bg + i + 8,  vcvtq_f32_u32(vmovl_u16(vget_low_u16(hi16))));
        vst1q_f32(bg + i + 12, vcvtq_f32_u32(vmovl_u16(vget_high_u16(hi16))));
    }
    for (; i < size; i++)
        bg[i] = (float)blurred[i];
}

/* ---- Background Update: EMA (NEON) ---- */

void motion_bg_update(float *bg, const uint8_t *blurred, int size, float alpha)
{
    float inv = 1.0f - alpha;
    float32x4_t va = vdupq_n_f32(alpha);
    float32x4_t vi = vdupq_n_f32(inv);

    int i = 0;
    for (; i <= size - 16; i += 16) {
        uint8x16_t px = vld1q_u8(blurred + i);
        uint16x8_t lo16 = vmovl_u8(vget_low_u8(px));
        uint16x8_t hi16 = vmovl_u8(vget_high_u8(px));

        /* 4 groups of 4 floats = 16 pixels */
        float32x4_t f0 = vcvtq_f32_u32(vmovl_u16(vget_low_u16(lo16)));
        float32x4_t b0 = vld1q_f32(bg + i);
        vst1q_f32(bg + i, vfmaq_f32(vmulq_f32(vi, b0), va, f0));

        float32x4_t f1 = vcvtq_f32_u32(vmovl_u16(vget_high_u16(lo16)));
        float32x4_t b1 = vld1q_f32(bg + i + 4);
        vst1q_f32(bg + i + 4, vfmaq_f32(vmulq_f32(vi, b1), va, f1));

        float32x4_t f2 = vcvtq_f32_u32(vmovl_u16(vget_low_u16(hi16)));
        float32x4_t b2 = vld1q_f32(bg + i + 8);
        vst1q_f32(bg + i + 8, vfmaq_f32(vmulq_f32(vi, b2), va, f2));

        float32x4_t f3 = vcvtq_f32_u32(vmovl_u16(vget_high_u16(hi16)));
        float32x4_t b3 = vld1q_f32(bg + i + 12);
        vst1q_f32(bg + i + 12, vfmaq_f32(vmulq_f32(vi, b3), va, f3));
    }
    for (; i < size; i++)
        bg[i] = alpha * (float)blurred[i] + inv * bg[i];
}

/* ---- Grid-based Motion Detection (NEON inner loop) ---- */

/* Count pixels where |blurred - round(bg)| > threshold, 16 at a time */
static inline int diff_thresh_16(const uint8_t *blurred, const float *bg,
                                  uint8_t thresh)
{
    uint8x16_t px = vld1q_u8(blurred);
    uint8x16_t bg_u8 = bg_floats_to_u8(bg);
    uint8x16_t diff = vabdq_u8(px, bg_u8);
    uint8x16_t mask = vcgtq_u8(diff, vdupq_n_u8(thresh));
    return vaddvq_u8(vshrq_n_u8(mask, 7));
}

float motion_bg_diff_grid(const uint8_t *blurred, const float *bg,
                          int w, int h, int threshold,
                          int grid_cols, int grid_rows,
                          float *cell_pcts, int *total_count,
                          int *max_col, int *max_row)
{
    float max_pct = 0;
    *max_col = 0;
    *max_row = 0;
    *total_count = 0;

    uint8_t thresh = (uint8_t)(threshold > 255 ? 255 : threshold);

    for (int gr = 0; gr < grid_rows; gr++) {
        int y0 = gr * h / grid_rows;
        int y1 = (gr + 1) * h / grid_rows;
        for (int gc = 0; gc < grid_cols; gc++) {
            int x0 = gc * w / grid_cols;
            int x1 = (gc + 1) * w / grid_cols;

            int count = 0;
            int total = 0;
            for (int y = y0; y < y1; y++) {
                int row_off = y * w;
                int x = x0;

                /* NEON: 16 pixels at a time */
                for (; x <= x1 - 16; x += 16) {
                    count += diff_thresh_16(blurred + row_off + x,
                                            bg + row_off + x, thresh);
                    total += 16;
                }

                /* Scalar tail */
                for (; x < x1; x++) {
                    int i = row_off + x;
                    int diff = abs((int)blurred[i] - (int)(bg[i] + 0.5f));
                    if (diff > threshold)
                        count++;
                    total++;
                }
            }

            float pct = total > 0 ? 100.0f * count / total : 0;
            cell_pcts[gr * grid_cols + gc] = pct;
            *total_count += count;

            if (pct > max_pct) {
                max_pct = pct;
                *max_col = gc;
                *max_row = gr;
            }
        }
    }
    return max_pct;
}

/* ---- Blob Detection (unchanged — operates on grid, not pixels) ---- */

int motion_find_blobs(const float *cell_pcts, int cols, int rows,
                      float active_pct, int *blob_col, int *blob_row)
{
    uint8_t visited[64 * 64];
    int stack[64 * 64];
    int total = cols * rows;

    if (total > 64 * 64) total = 64 * 64;
    memset(visited, 0, total);

    int best_size = 0;
    int best_sum_c = 0, best_sum_r = 0;

    static const int dr[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    static const int dc[] = { 0, 0, 1,-1,  1, -1, 1,-1};

    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            int idx = r * cols + c;
            if (visited[idx] || cell_pcts[idx] < active_pct)
                continue;

            int sp = 0;
            stack[sp++] = idx;
            visited[idx] = 1;

            int size = 0;
            int sum_c = 0, sum_r = 0;

            while (sp > 0) {
                int ci = stack[--sp];
                int cr_ = ci / cols;
                int cc = ci % cols;
                size++;
                sum_c += cc;
                sum_r += cr_;

                for (int d = 0; d < 8; d++) {
                    int nr = cr_ + dr[d];
                    int nc = cc + dc[d];
                    if (nr < 0 || nr >= rows || nc < 0 || nc >= cols)
                        continue;
                    int ni = nr * cols + nc;
                    if (!visited[ni] && cell_pcts[ni] >= active_pct) {
                        visited[ni] = 1;
                        stack[sp++] = ni;
                    }
                }
            }

            if (size > best_size) {
                best_size = size;
                best_sum_c = sum_c;
                best_sum_r = sum_r;
            }
        }
    }

    if (best_size > 0) {
        *blob_col = best_sum_c / best_size;
        *blob_row = best_sum_r / best_size;
    } else {
        *blob_col = 0;
        *blob_row = 0;
    }
    return best_size;
}

/* ---- Motion Visualization (NEON) ---- */

void motion_visualize(const uint8_t *blurred, const float *bg,
                      int src_w, int src_h, int threshold,
                      int disp_w, int disp_h, uint8_t *vis_rgb)
{
    uint8_t thresh_u8 = (uint8_t)(threshold > 255 ? 255 : threshold);

    /* Fast path: horizontal 1:1 (src_w == disp_w), only vertical resample */
    if (src_w == disp_w) {
        uint8x16_t thresh_v = vdupq_n_u8(thresh_u8);
        uint8x16_t c128 = vdupq_n_u8(128);
        uint8x16_t c127 = vdupq_n_u8(127);

        for (int dy = 0; dy < disp_h; dy++) {
            int sy = dy * src_h / disp_h;
            const uint8_t *row_blur = blurred + sy * src_w;
            const float *row_bg = bg + sy * src_w;
            uint8_t *out = vis_rgb + dy * disp_w * 3;

            int dx = 0;
            for (; dx <= disp_w - 16; dx += 16) {
                uint8x16_t px = vld1q_u8(row_blur + dx);
                uint8x16_t bg_u8 = bg_floats_to_u8(row_bg + dx);
                uint8x16_t diff = vabdq_u8(px, bg_u8);
                uint8x16_t above = vcgtq_u8(diff, thresh_v);

                /* Green channel: above ? 128+min(diff,127) : gray */
                uint8x16_t green_above = vqaddq_u8(c128, vminq_u8(diff, c127));

                /* Gray: saturating diff * 12 = diff*8 + diff*4 */
                uint8x16_t gray = vqaddq_u8(vqshlq_n_u8(diff, 3),
                                            vqshlq_n_u8(diff, 2));

                uint8x16_t r = vbicq_u8(gray, above);
                uint8x16_t g = vbslq_u8(above, green_above, gray);
                uint8x16_t b = vbicq_u8(gray, above);

                uint8x16x3_t rgb = {{ r, g, b }};
                vst3q_u8(out + dx * 3, rgb);
            }

            /* Scalar tail */
            for (; dx < disp_w; dx++) {
                int si = sy * src_w + dx;
                int di = (dy * disp_w + dx) * 3;
                int diff = abs((int)blurred[si] - (int)(bg[si] + 0.5f));
                if (diff > threshold) {
                    vis_rgb[di + 0] = 0;
                    vis_rgb[di + 1] = (uint8_t)(128 + (diff > 127 ? 127 : diff));
                    vis_rgb[di + 2] = 0;
                } else {
                    uint8_t v = (uint8_t)(diff * 12 > 255 ? 255 : diff * 12);
                    vis_rgb[di + 0] = v;
                    vis_rgb[di + 1] = v;
                    vis_rgb[di + 2] = v;
                }
            }
        }
        return;
    }

    /* Generic path: nearest-neighbor resampling (scalar) */
    for (int dy = 0; dy < disp_h; dy++) {
        int sy = dy * src_h / disp_h;
        for (int dx = 0; dx < disp_w; dx++) {
            int sx = dx * src_w / disp_w;
            int si = sy * src_w + sx;
            int di = (dy * disp_w + dx) * 3;

            int diff = abs((int)blurred[si] - (int)(bg[si] + 0.5f));
            if (diff > threshold) {
                vis_rgb[di + 0] = 0;
                vis_rgb[di + 1] = (uint8_t)(128 + (diff > 127 ? 127 : diff));
                vis_rgb[di + 2] = 0;
            } else {
                uint8_t v = (uint8_t)(diff * 12 > 255 ? 255 : diff * 12);
                vis_rgb[di + 0] = v;
                vis_rgb[di + 1] = v;
                vis_rgb[di + 2] = v;
            }
        }
    }
}
