#ifndef __ALT_EST_H
#define __ALT_EST_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "Common/gen_physic_def/imu_data.h"

float BaroAltEstimate_Update(float baro, const M_Cbn_TypeDef Cbn, float *acc);

#endif
