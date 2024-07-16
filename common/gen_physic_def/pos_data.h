#ifndef POS_DATA_H
#define POS_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef union
{
    /* in door flight */
    struct
    {
        uint32_t time;
        double Pos_X;
        double Pos_Y;
        double Pos_Z;
    } XYZ_Pos;

    /* out door flight */
    struct
    {
        uint32_t time;
        double Lon;
        double lat;
        double alt;
    } GPS_Pos;
} PosData_TypeDef;

typedef struct
{
    uint32_t time;
    
    float alt;
} AltData_TypeDef;

typedef union
{
    /* in door flight */
    struct
    {
        uint32_t time;
        double Vel_X;
        double Vel_Y;
        double Vel_Z;
    } XYZ_Vel;

    /* out door flight */
    struct
    {
        uint32_t time;
        double Vel_N;
        double Vel_E;
        double Vel_D;
    } GPS_Vel;
} PosVelData_TypeDef;

#ifdef __cplusplus
}
#endif

#endif
