#ifndef _FILE_UTILS_H_
#define _FILE_UTILS_H_

#include <stdio.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

// 从文件读取数据
int read_data_from_file(const char* filename, char** data);

// 实现函数
inline int read_data_from_file(const char* filename, char** data) {
    FILE* file = fopen(filename, "rb");
    if (!file) {
        printf("Failed to open file: %s\n", filename);
        return 0;
    }
    
    // 获取文件大小
    fseek(file, 0, SEEK_END);
    long size = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    if (size <= 0) {
        fclose(file);
        return 0;
    }
    
    // 分配内存
    *data = (char*)malloc(size);
    if (!*data) {
        fclose(file);
        return 0;
    }
    
    // 读取数据
    size_t read_size = fread(*data, 1, size, file);
    fclose(file);
    
    if (read_size != size) {
        free(*data);
        *data = NULL;
        return 0;
    }
    
    return (int)size;
}

#ifdef __cplusplus
}
#endif

#endif // _FILE_UTILS_H_