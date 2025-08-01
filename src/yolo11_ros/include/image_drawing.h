#ifndef _IMAGE_DRAWING_H_
#define _IMAGE_DRAWING_H_

#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

// 图像绘制函数声明（这里我们主要使用OpenCV，所以可以简化）
void draw_rectangle(image_buffer_t* img, image_rect_t* rect, int color);
void draw_text(image_buffer_t* img, const char* text, int x, int y, int color);

// 简化实现（实际绘制由OpenCV完成）
inline void draw_rectangle(image_buffer_t* img, image_rect_t* rect, int color) {
    // 在我们的实现中，绘制由OpenCV完成，这里只是占位符
    (void)img;
    (void)rect;
    (void)color;
}

inline void draw_text(image_buffer_t* img, const char* text, int x, int y, int color) {
    // 在我们的实现中，绘制由OpenCV完成，这里只是占位符
    (void)img;
    (void)text;
    (void)x;
    (void)y;
    (void)color;
}

#ifdef __cplusplus
}
#endif

#endif // _IMAGE_DRAWING_H_