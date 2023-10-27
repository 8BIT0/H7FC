#ifndef __SRV_SENSORMONITOR_H
#define __SRV_SENSORMONITOR_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "Srv_IMUSample.h"
#include "Srv_Baro.h"

typedef enum
{

}SrvSensorMonitor_Type_List;

typedef union
{
    uint32_t val;
    struct
    {
        uint32_t imu  : 1;
        uint32_t mag  : 1;
        uint32_t baro : 1;
        uint32_t tof  : 1;
        uint32_t gnss : 1;

        uint32_t res  : 27;
    }bit;
}SrvSensorMonitor_Reg_TypeDef;

/* bit field on init_state_reg set 1 represent error triggerd on */
typedef struct
{
    SrvSensorMonitor_Reg_TypeDef enbled_reg;
    SrvSensorMonitor_Reg_TypeDef init_state_reg;
}SrvSensorMonitorObj_TypeDef;

typedef struct
{
    
}SrvSensorMonitor_TypeDef;


#endif
