#ifndef __FILTER_PARAM_H
#define __FILTER_PARAM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DEFAULT_PARAM_SCALE 1.0f

typedef struct
{
    float p;
    float scale;
}BTF_Para_TypeDef;

/* 100Hz sample rate butterworth filter parameter section */
extern const BTF_Para_TypeDef BTF_E_2O_10Hz_100Hz[3];
extern const BTF_Para_TypeDef BTF_U_2O_10Hz_100Hz[2];

extern const BTF_Para_TypeDef BTF_E_2O_30Hz_100Hz[3];
extern const BTF_Para_TypeDef BTF_U_2O_30Hz_100Hz[2];

extern const BTF_Para_TypeDef BTF_E_3O_10Hz_100Hz[4];
extern const BTF_Para_TypeDef BTF_U_3O_10Hz_100Hz[3];

extern const BTF_Para_TypeDef BTF_E_3O_30Hz_100Hz[4];
extern const BTF_Para_TypeDef BTF_U_3O_30Hz_100Hz[3];

extern const BTF_Para_TypeDef BTF_E_4O_10Hz_100Hz[5];
extern const BTF_Para_TypeDef BTF_U_4O_10Hz_100Hz[4];

extern const BTF_Para_TypeDef BTF_E_4O_30Hz_100Hz[5];
extern const BTF_Para_TypeDef BTF_U_4O_30Hz_100Hz[4];

extern const BTF_Para_TypeDef BTF_E_5O_10Hz_100Hz[6];
extern const BTF_Para_TypeDef BTF_U_5O_10Hz_100Hz[5];

extern const BTF_Para_TypeDef BTF_E_5O_30Hz_100Hz[6];
extern const BTF_Para_TypeDef BTF_U_5O_30Hz_100Hz[5];

/* 1K sample rate butterworth filter parameter section */
extern const BTF_Para_TypeDef BTF_E_2O_30Hz_1K[3];
extern const BTF_Para_TypeDef BTF_U_2O_30Hz_1K[2];

extern const BTF_Para_TypeDef BTF_E_2O_50Hz_1K[3];
extern const BTF_Para_TypeDef BTF_U_2O_50Hz_1K[2];

extern const BTF_Para_TypeDef BTF_E_3O_30Hz_1K[4];
extern const BTF_Para_TypeDef BTF_U_3O_30Hz_1K[3];

extern const BTF_Para_TypeDef BTF_E_3O_50Hz_1K[4];
extern const BTF_Para_TypeDef BTF_U_3O_50Hz_1K[3];

extern const BTF_Para_TypeDef BTF_E_4O_30Hz_1K[5];
extern const BTF_Para_TypeDef BTF_U_4O_30Hz_1K[4];

extern const BTF_Para_TypeDef BTF_E_4O_50Hz_1K[5];
extern const BTF_Para_TypeDef BTF_U_4O_50Hz_1K[4];

extern const BTF_Para_TypeDef BTF_E_5O_30Hz_1K[6];
extern const BTF_Para_TypeDef BTF_U_5O_30Hz_1K[5];

extern const BTF_Para_TypeDef BTF_E_5O_50Hz_1K[6];
extern const BTF_Para_TypeDef BTF_U_5O_50Hz_1K[5];

#ifdef __cplusplus
}
#endif

#endif
