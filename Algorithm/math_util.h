#ifndef __MATH_UTIL_H
#define __MATH_UTIL_H

#include <stdbool.h>
#include <stdint.h>

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

#endif
