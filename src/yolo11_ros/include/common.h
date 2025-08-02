#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdint.h>
#include <cstring>  // 添加memcpy支持

#ifdef __cplusplus
extern "C" {
#endif

// 图像格式定义
typedef enum {
    IMAGE_FORMAT_RGB888 = 0,
    IMAGE_FORMAT_BGR888,
    IMAGE_FORMAT_GRAY8,
    IMAGE_FORMAT_YUV420SP_NV21,
    IMAGE_FORMAT_YUV420SP_NV12,
    IMAGE_FORMAT_YUV420P_YU12,
    IMAGE_FORMAT_YUV420P_YV12,
    IMAGE_FORMAT_YUV422SP_NV16,
    IMAGE_FORMAT_YUV422SP_NV61,
    IMAGE_FORMAT_YUV422P_YU16,
    IMAGE_FORMAT_YUV422P_YV16,
    IMAGE_FORMAT_UNDEFINED
} image_format_e;

// 矩形结构体
typedef struct {
    int left;
    int top;
    int right;
    int bottom;
} image_rect_t;

// 图像缓冲区结构体
typedef struct {
    int width;
    int height;
    image_format_e format;
    unsigned char* virt_addr;
    int size;
} image_buffer_t;

// Letterbox结构体
typedef struct {
    float scale;
    int x_pad;
    int y_pad;
} letterbox_t;

// 移除rknn_app_context_t的前向声明，因为yolo11.h中已经定义了完整结构体

#ifdef __cplusplus
}
#endif

#endif // _COMMON_H_