#ifndef __MATH_UTIL_H
#define __MATH_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

/* Cbn matrix definition */
typedef union
{
    float buf[9];
    float matrix[3][3];
} M_Cbn_TypeDef;

inline float Deg2Rad(float deg)
{
    return deg / 57.3f;
}

inline float Rad2Deg(float rad)
{
    return rad * 57.3f;
}

/* g to meter per square sec */
inline float g2Mpss(float g)
{
    return g * 9.8;
}

inline M_Cbn_TypeDef AttConvert2Cbn(float pitch, float roll, float yaw)
{
    M_Cbn_TypeDef tmp;

    memset(&tmp, 0, sizeof(M_Cbn_TypeDef));

    tmp.matrix[0][0] = cos(yaw) * cos(pitch);
    tmp.matrix[1][0] = cos(pitch) * sin(yaw);
    tmp.matrix[2][0] = -sin(pitch);
    
    tmp.matrix[0][1] = cos(yaw) * sin(pitch) *sin(roll) - cos(roll) * sin(yaw);
    tmp.matrix[1][1] = cos(yaw) * cos(roll) + sin(yaw) * sin(roll) * sin(pitch);
    tmp.matrix[2][1] = cos(pitch) * sin(roll);

    tmp.matrix[0][2] = sin(yaw) * sin(roll) + cos(roll) * cos(yaw) * sin(pitch);
    tmp.matrix[1][2] = cos(roll) * sin(yaw) * sin(pitch) - cos(yaw) * sin(roll);
    tmp.matrix[2][2] = cos(pitch) * cos(roll);
    
    return tmp;
}

#ifdef __cplusplus
}
#endif

#endif
