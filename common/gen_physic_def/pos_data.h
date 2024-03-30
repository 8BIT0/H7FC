#ifndef POS_DATA_H
#define POS_DATA_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef union
{
    /* in door flight */
    struct
    {
        double Pos_X;
        double Pos_Y;
        double Pos_Z;
    } XYZ_Pos;

    /* out door flight */
    struct
    {
        double Lon;
        double lat;
        double alt;
    } GPS_Pos;
} PosData_TypeDef;

typedef union
{
    /* in door flight */
    struct
    {
        double Vel_X;
        double Vel_Y;
        double Vel_Z;
    } XYZ_Vel;

    /* out door flight */
    struct
    {
        double Vel_N;
        double Vel_E;
        double Vel_D;
    } GPS_Vel;
} PosVelData_TypeDef;

#endif
