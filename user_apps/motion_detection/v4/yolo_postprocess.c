/*
 * yolo_postprocess.c - YOLOv5 post-processing for RKNN (pure C)
 *
 * Ported from Rockchip's postprocess.cc (Apache 2.0 License).
 * Changes: pure C, fixed-size arrays, embedded labels, class filtering.
 */

#include "yolo_postprocess.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* YOLOv5 anchors for each stride level */
static const int anchor0[6] = {10, 13, 16, 30, 33, 23};     /* stride 8  */
static const int anchor1[6] = {30, 61, 62, 45, 59, 119};    /* stride 16 */
static const int anchor2[6] = {116, 90, 156, 198, 373, 326}; /* stride 32 */

/* COCO 80-class labels */
static const char *COCO_LABELS[OBJ_CLASS_NUM] = {
	"person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
	"truck", "boat", "traffic light", "fire hydrant", "stop sign",
	"parking meter", "bench", "bird", "cat", "dog", "horse", "sheep",
	"cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
	"handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
	"sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
	"surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
	"knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
	"broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
	"couch", "potted plant", "bed", "dining table", "toilet", "tv",
	"laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
	"oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
	"scissors", "teddy bear", "hair drier", "toothbrush"
};

/* Security-relevant class IDs (person, vehicles, animals) */
static const int ALLOWED_CLASSES[] = {
	0,  /* person */
	1,  /* bicycle */
	2,  /* car */
	3,  /* motorcycle */
	5,  /* bus */
	7,  /* truck */
	14, /* bird */
	15, /* cat */
	16, /* dog */
	17, /* horse */
	18, /* sheep */
	19, /* cow */
	21, /* bear */
};
#define NUM_ALLOWED (sizeof(ALLOWED_CLASSES) / sizeof(ALLOWED_CLASSES[0]))

int is_allowed_class(int class_id)
{
	for (int i = 0; i < (int)NUM_ALLOWED; i++) {
		if (ALLOWED_CLASSES[i] == class_id)
			return 1;
	}
	return 0;
}

const char *coco_label(int class_id)
{
	if (class_id >= 0 && class_id < OBJ_CLASS_NUM)
		return COCO_LABELS[class_id];
	return "unknown";
}

/* --- Quantization helpers --- */

static inline int32_t clip_i32(float val, float min_v, float max_v)
{
	float f = val <= min_v ? min_v : (val >= max_v ? max_v : val);
	return (int32_t)f;
}

static inline int clamp_i(float val, int min_v, int max_v)
{
	return val > min_v ? (val < max_v ? (int)val : max_v) : min_v;
}

static int8_t qnt_f32_to_affine(float f32, int32_t zp, float scale)
{
	float dst = (f32 / scale) + zp;
	return (int8_t)clip_i32(dst, -128, 127);
}

static float deqnt_affine_to_f32(int8_t qnt, int32_t zp, float scale)
{
	return ((float)qnt - (float)zp) * scale;
}

/* --- NMS --- */

/* Max candidate boxes before NMS */
#define MAX_CANDIDATES 4096

typedef struct {
	float boxes[MAX_CANDIDATES * 4];  /* x, y, w, h */
	float probs[MAX_CANDIDATES];
	int   class_ids[MAX_CANDIDATES];
	int   order[MAX_CANDIDATES];
	int   count;
} candidate_list_t;

static float calc_iou(const float *a, const float *b)
{
	float xmin0 = a[0], ymin0 = a[1], xmax0 = a[0] + a[2], ymax0 = a[1] + a[3];
	float xmin1 = b[0], ymin1 = b[1], xmax1 = b[0] + b[2], ymax1 = b[1] + b[3];

	float w = fmaxf(0.f, fminf(xmax0, xmax1) - fmaxf(xmin0, xmin1) + 1.0f);
	float h = fmaxf(0.f, fminf(ymax0, ymax1) - fmaxf(ymin0, ymin1) + 1.0f);
	float inter = w * h;
	float area0 = (xmax0 - xmin0 + 1.0f) * (ymax0 - ymin0 + 1.0f);
	float area1 = (xmax1 - xmin1 + 1.0f) * (ymax1 - ymin1 + 1.0f);
	float uni = area0 + area1 - inter;
	return uni <= 0.f ? 0.f : (inter / uni);
}

static void nms(candidate_list_t *c, int filter_id, float threshold)
{
	for (int i = 0; i < c->count; i++) {
		int n = c->order[i];
		if (n == -1 || c->class_ids[n] != filter_id)
			continue;
		for (int j = i + 1; j < c->count; j++) {
			int m = c->order[j];
			if (m == -1 || c->class_ids[m] != filter_id)
				continue;
			float iou = calc_iou(&c->boxes[n * 4], &c->boxes[m * 4]);
			if (iou > threshold)
				c->order[j] = -1;
		}
	}
}

/* Quicksort indices by descending probability */
static void qsort_desc(float *probs, int *indices, int left, int right)
{
	if (left >= right)
		return;

	float pivot = probs[left];
	int pivot_idx = indices[left];
	int lo = left, hi = right;

	while (lo < hi) {
		while (lo < hi && probs[hi] <= pivot)
			hi--;
		probs[lo] = probs[hi];
		indices[lo] = indices[hi];
		while (lo < hi && probs[lo] >= pivot)
			lo++;
		probs[hi] = probs[lo];
		indices[hi] = indices[lo];
	}
	probs[lo] = pivot;
	indices[lo] = pivot_idx;
	qsort_desc(probs, indices, left, lo - 1);
	qsort_desc(probs, indices, lo + 1, right);
}

/* --- Per-stride anchor decoding --- */

static int process_stride(int8_t *input, const int *anchor, int grid_h, int grid_w,
                          int stride, float threshold, int32_t zp, float scale,
                          candidate_list_t *c)
{
	int grid_len = grid_h * grid_w;
	int8_t thres_i8 = qnt_f32_to_affine(threshold, zp, scale);
	int added = 0;

	for (int a = 0; a < 3; a++) {
		for (int i = 0; i < grid_h; i++) {
			for (int j = 0; j < grid_w; j++) {
				int8_t box_conf = input[(PROP_BOX_SIZE * a + 4) * grid_len + i * grid_w + j];
				if (box_conf < thres_i8)
					continue;

				int offset = (PROP_BOX_SIZE * a) * grid_len + i * grid_w + j;
				int8_t *in_ptr = input + offset;

				/* Find best class */
				int8_t max_prob = in_ptr[5 * grid_len];
				int max_id = 0;
				for (int k = 1; k < OBJ_CLASS_NUM; k++) {
					int8_t prob = in_ptr[(5 + k) * grid_len];
					if (prob > max_prob) {
						max_prob = prob;
						max_id = k;
					}
				}

				if (max_prob < thres_i8)
					continue;

				/* Class filter: skip non-security classes */
				if (!is_allowed_class(max_id))
					continue;

				if (c->count >= MAX_CANDIDATES)
					continue;

				/* Decode box */
				float bx = (deqnt_affine_to_f32(*in_ptr, zp, scale)) * 2.0f - 0.5f;
				float by = (deqnt_affine_to_f32(in_ptr[grid_len], zp, scale)) * 2.0f - 0.5f;
				float bw = (deqnt_affine_to_f32(in_ptr[2 * grid_len], zp, scale)) * 2.0f;
				float bh = (deqnt_affine_to_f32(in_ptr[3 * grid_len], zp, scale)) * 2.0f;
				bx = (bx + j) * (float)stride;
				by = (by + i) * (float)stride;
				bw = bw * bw * (float)anchor[a * 2];
				bh = bh * bh * (float)anchor[a * 2 + 1];
				bx -= bw / 2.0f;
				by -= bh / 2.0f;

				int idx = c->count;
				c->boxes[idx * 4 + 0] = bx;
				c->boxes[idx * 4 + 1] = by;
				c->boxes[idx * 4 + 2] = bw;
				c->boxes[idx * 4 + 3] = bh;
				c->probs[idx] = deqnt_affine_to_f32(max_prob, zp, scale) *
				                deqnt_affine_to_f32(box_conf, zp, scale);
				c->class_ids[idx] = max_id;
				c->count++;
				added++;
			}
		}
	}
	return added;
}

int yolo_postprocess(int8_t *input0, int8_t *input1, int8_t *input2,
                     int model_in_h, int model_in_w,
                     float conf_threshold, float nms_threshold,
                     int pad_x, int pad_y, float scale_w, float scale_h,
                     int32_t qnt_zps[3], float qnt_scales[3],
                     detection_group_t *group)
{
	/* Stack-allocate candidate list (about 100KB) */
	candidate_list_t cands;
	cands.count = 0;

	/* Decode all three stride levels */
	int grid_h0 = model_in_h / 8;
	int grid_w0 = model_in_w / 8;
	process_stride(input0, anchor0, grid_h0, grid_w0, 8,
	               conf_threshold, qnt_zps[0], qnt_scales[0], &cands);

	int grid_h1 = model_in_h / 16;
	int grid_w1 = model_in_w / 16;
	process_stride(input1, anchor1, grid_h1, grid_w1, 16,
	               conf_threshold, qnt_zps[1], qnt_scales[1], &cands);

	int grid_h2 = model_in_h / 32;
	int grid_w2 = model_in_w / 32;
	process_stride(input2, anchor2, grid_h2, grid_w2, 32,
	               conf_threshold, qnt_zps[2], qnt_scales[2], &cands);

	group->count = 0;
	if (cands.count == 0)
		return 0;

	/* Initialize order indices */
	for (int i = 0; i < cands.count; i++)
		cands.order[i] = i;

	/* Sort by descending probability */
	qsort_desc(cands.probs, cands.order, 0, cands.count - 1);

	/* NMS per unique class */
	int seen_classes[OBJ_CLASS_NUM] = {0};
	for (int i = 0; i < cands.count; i++) {
		int cid = cands.class_ids[i];
		if (!seen_classes[cid]) {
			seen_classes[cid] = 1;
			nms(&cands, cid, nms_threshold);
		}
	}

	/* Collect surviving detections */
	int out_count = 0;
	for (int i = 0; i < cands.count && out_count < MAX_DETECTIONS; i++) {
		int n = cands.order[i];
		if (n == -1)
			continue;

		float x1 = cands.boxes[n * 4 + 0] - pad_x;
		float y1 = cands.boxes[n * 4 + 1] - pad_y;
		float x2 = x1 + cands.boxes[n * 4 + 2];
		float y2 = y1 + cands.boxes[n * 4 + 3];
		int cid = cands.class_ids[n];

		detection_t *d = &group->dets[out_count];
		d->box.left   = (int)(clamp_i(x1, 0, model_in_w) / scale_w);
		d->box.top    = (int)(clamp_i(y1, 0, model_in_h) / scale_h);
		d->box.right  = (int)(clamp_i(x2, 0, model_in_w) / scale_w);
		d->box.bottom = (int)(clamp_i(y2, 0, model_in_h) / scale_h);
		d->confidence = cands.probs[i];
		d->class_id = cid;
		strncpy(d->name, coco_label(cid), sizeof(d->name) - 1);
		d->name[sizeof(d->name) - 1] = '\0';
		out_count++;
	}
	group->count = out_count;

	return 0;
}
