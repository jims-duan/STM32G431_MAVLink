#ifndef _LOWPASSFILTER_H
#define _LOWPASSFILTER_H

#include "main.h"

typedef struct LowPassFilterStructure LowPassFilterStructure;

// 函数类型定义
typedef float (*low_filter_update_func)(LowPassFilterStructure* obj, float raw_value);

struct LowPassFilterStructure
{
    float alpha;
    float last_value;
    low_filter_update_func update;
};

// 创建滤波器对象的函数（构造函数）
LowPassFilterStructure LowPassFilter_Create(float alpha, float last_value);

#endif

