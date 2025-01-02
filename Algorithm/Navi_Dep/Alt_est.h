#ifndef __ALT_EST_H
#define __ALT_EST_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <Eigen>
#include "pos_data.h"

void BaroAltEstimate_Init(float baro, float delta_T, float baro_bias, float acc_bias);
RelMov_TypeDef BaroAltEstimate_Update(float baro, float acc_z);

#endif
