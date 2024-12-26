#ifndef __ALT_EST_H
#define __ALT_EST_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <Eigen>

void BaroAltEstimate_Init(float baro_alt, float acc_z, float delta_T);
float BaroAltEstimate_Update(float baro, float acc_z);

#endif
