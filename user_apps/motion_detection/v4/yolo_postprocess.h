#ifndef YOLO_POSTPROCESS_H
#define YOLO_POSTPROCESS_H

#include <stdint.h>

#define OBJ_CLASS_NUM     80
#define MAX_DETECTIONS    64
#define NMS_THRESH        0.45f
#define BOX_THRESH        0.25f
#define PROP_BOX_SIZE     (5 + OBJ_CLASS_NUM)

typedef struct {
	int left, right, top, bottom;
} box_rect_t;

typedef struct {
	char name[16];
	box_rect_t box;
	float confidence;
	int class_id;
} detection_t;

typedef struct {
	int count;
	detection_t dets[MAX_DETECTIONS];
} detection_group_t;

/*
 * YOLOv5 post-processing: anchor decode + NMS + class filtering.
 *
 * input0/1/2: quantized int8 output tensors from RKNN (strides 8/16/32)
 * model_in_h/w: model input dimensions (640x640)
 * conf_threshold: confidence threshold (e.g. 0.25)
 * nms_threshold: NMS IoU threshold (e.g. 0.45)
 * pad_x/pad_y: letterbox padding offsets
 * scale_w/scale_h: letterbox scale factors (model coords → original coords)
 * qnt_zps[3], qnt_scales[3]: per-output quantization params
 * group: output detection results
 *
 * Returns 0 on success.
 */
int yolo_postprocess(int8_t *input0, int8_t *input1, int8_t *input2,
                     int model_in_h, int model_in_w,
                     float conf_threshold, float nms_threshold,
                     int pad_x, int pad_y, float scale_w, float scale_h,
                     int32_t qnt_zps[3], float qnt_scales[3],
                     detection_group_t *group);

/* Check if a COCO class ID is security-relevant */
int is_allowed_class(int class_id);

/* Get COCO label string for a class ID (0-79) */
const char *coco_label(int class_id);

#endif /* YOLO_POSTPROCESS_H */
