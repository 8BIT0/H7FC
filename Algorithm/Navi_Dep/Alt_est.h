#ifndef __ALT_EST_H
#define __ALT_EST_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <Eigen>

float BaroAltEstimate_Update(float baro, Matrix<float, 3, 1> acc);

#endif
