#ifndef POS_DATA_H
#define POS_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* relative move on single axis */
typedef struct
{
    uint32_t time;
    float pos;
    float vel;
    float acc;
} RelMov_TypeDef;

#ifdef __cplusplus
}
#endif

#endif
