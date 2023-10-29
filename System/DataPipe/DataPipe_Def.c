#include "DataPipe.h"

DataPipeObj_TypeDef SensorInitState_smp_DataPipe = {.enable = true};
DataPipeObj_TypeDef SensorInitState_hub_DataPipe = {.enable = true};
DataPipeObj_TypeDef SensorEnableState_smp_DataPipe = {.enable = true};
DataPipeObj_TypeDef SensorEnableState_hub_DataPipe = {.enable = true};

DataPipeObj_TypeDef Receiver_Smp_DataPipe = {.enable = true};
DataPipeObj_TypeDef Receiver_hub_DataPipe = {.enable = true};

DataPipeObj_TypeDef IMU_Log_DataPipe = {.enable = true};
DataPipeObj_TypeDef IMU_Smp_DataPipe = {.enable = true};
DataPipeObj_TypeDef IMU_hub_DataPipe = {.enable = true};

DataPipeObj_TypeDef Actuator_cal_DataPipe = {.enable = true};
DataPipeObj_TypeDef Actuator_hub_DataPipe = {.enable = true};
