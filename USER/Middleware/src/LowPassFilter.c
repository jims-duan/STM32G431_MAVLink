#include "LowPassFilter.h"


static float low_filter_update_impl(LowPassFilterStructure* obj, float raw_value)
{
    float filtered = obj->alpha * raw_value + (1.0f - obj->alpha) * obj->last_value;
    obj->last_value = filtered;
    return filtered;
}

// 构造函数
LowPassFilterStructure LowPassFilter_Create(float alpha, float last_value)
{
    LowPassFilterStructure obj;
    
    obj.alpha = alpha;
    obj.last_value = last_value;
    obj.update = low_filter_update_impl;
    
    return obj;
}

