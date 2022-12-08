#ifndef __FILTER_H
#define __FILTER_H

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "queue.h"

#define SMOOTH_WINDOW_SIZE 5

#pragma pack(1)
typedef struct
{
    float delta_s;
    uint8_t item_cnt;
    uint32_t max_val_addr;
    uint32_t min_val_addr;
    float val_buf[SMOOTH_WINDOW_SIZE];
    float smooth_value;
    float mid_value;
}Filter_GenParam_TypeDef;

typedef struct
{
    
}Filter_LowPassParam_TypeDef;

typedef struct
{
    
}Filter_HighPassParam_TypeDef;

typedef struct
{

}Filter_KalmenParam_TypeDef;
#pragma pack()

#endif

