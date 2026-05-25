#ifndef _SYSTEM_H
#define _SYSTEM_H

#include "main.h"

/*计算时间差*/
static inline uint32_t get_tick_diff(uint32_t current, uint32_t previous)
{
    return current - previous;
}

#endif

