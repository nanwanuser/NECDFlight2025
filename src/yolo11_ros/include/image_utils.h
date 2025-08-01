#ifndef _IMAGE_UTILS_H_
#define _IMAGE_UTILS_H_

#include "common.h"
#include <cstring>
#include <algorithm>
#include <cmath>

#ifdef __cplusplus
extern "C" {
#endif

// 获取图像大小
int get_image_size(image_buffer_t* img);

// 图像格式转换和letterbox处理
int convert_image_with_letterbox(image_buffer_t* src, image_buffer_t* dst, letterbox_t* letterbox, int bg_color);

// 简单的实现函数
inline int get_image_size(image_buffer_t* img) {
    if (!img) return 0;
    
    switch (img->format) {
        case IMAGE_FORMAT_RGB888:
        case IMAGE_FORMAT_BGR888:
            return img->width * img->height * 3;
        case IMAGE_FORMAT_GRAY8:
            return img->width * img->height;
        default:
            return img->width * img->height * 3; // 默认RGB
    }
}

// 🔥 修复的letterbox实现
inline int convert_image_with_letterbox(image_buffer_t* src, image_buffer_t* dst, letterbox_t* letterbox, int bg_color) {
    if (!src || !dst || !letterbox) return -1;
    
    // 计算缩放比例 - 保持宽高比
    float scale_x = (float)dst->width / src->width;
    float scale_y = (float)dst->height / src->height;
    float scale = std::min(scale_x, scale_y);  // 🔥 关键：取最小值保持宽高比
    
    letterbox->scale = scale;
    
    // 计算实际缩放后的尺寸
    int new_width = (int)(src->width * scale);
    int new_height = (int)(src->height * scale);
    
    // 计算padding
    letterbox->x_pad = (dst->width - new_width) / 2;
    letterbox->y_pad = (dst->height - new_height) / 2;
    
    // 🔥 对于640x480 → 640x640的情况：
    // scale = min(640/640, 640/480) = min(1.0, 1.333) = 1.0
    // new_width = 640, new_height = 480
    // x_pad = 0, y_pad = (640-480)/2 = 80
    
    // 🔥 真正的letterbox处理
    if (src->format == IMAGE_FORMAT_RGB888 && dst->format == IMAGE_FORMAT_RGB888) {
        // 初始化目标图像为背景色
        unsigned char bg_r = (bg_color >> 16) & 0xFF;
        unsigned char bg_g = (bg_color >> 8) & 0xFF;
        unsigned char bg_b = bg_color & 0xFF;
        
        // 填充背景色
        for (int i = 0; i < dst->width * dst->height; i++) {
            dst->virt_addr[i * 3 + 0] = bg_r;
            dst->virt_addr[i * 3 + 1] = bg_g;
            dst->virt_addr[i * 3 + 2] = bg_b;
        }
        
        // 🔥 关键：将源图像复制到正确位置
        if (scale == 1.0) {
            // 不需要缩放，直接复制
            for (int y = 0; y < src->height; y++) {
                int src_offset = y * src->width * 3;
                int dst_offset = ((y + letterbox->y_pad) * dst->width + letterbox->x_pad) * 3;
                memcpy(dst->virt_addr + dst_offset, src->virt_addr + src_offset, src->width * 3);
            }
        } else {
            // 需要缩放 - 简单的最近邻插值
            for (int dst_y = 0; dst_y < new_height; dst_y++) {
                for (int dst_x = 0; dst_x < new_width; dst_x++) {
                    int src_x = (int)(dst_x / scale);
                    int src_y = (int)(dst_y / scale);
                    
                    // 确保不越界
                    src_x = std::min(src_x, src->width - 1);
                    src_y = std::min(src_y, src->height - 1);
                    
                    int src_offset = (src_y * src->width + src_x) * 3;
                    int dst_offset = ((dst_y + letterbox->y_pad) * dst->width + (dst_x + letterbox->x_pad)) * 3;
                    
                    dst->virt_addr[dst_offset + 0] = src->virt_addr[src_offset + 0];
                    dst->virt_addr[dst_offset + 1] = src->virt_addr[src_offset + 1];
                    dst->virt_addr[dst_offset + 2] = src->virt_addr[src_offset + 2];
                }
            }
        }
        
        return 0;
    }
    
    // 🔥 如果格式不匹配或其他情况，回退到简单复制（不推荐，但作为后备）
    if (src->size <= dst->size) {
        memcpy(dst->virt_addr, src->virt_addr, src->size);
        // 设置合理的letterbox参数
        letterbox->scale = 1.0;
        letterbox->x_pad = 0;
        letterbox->y_pad = 0;
        return 0;
    }
    
    return -1;
}

#ifdef __cplusplus
}
#endif

#endif // _IMAGE_UTILS_H_
