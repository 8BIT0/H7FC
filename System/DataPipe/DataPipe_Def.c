#include "DataPipe.h"

DataPipeObj_TypeDef Receiver_Smp_DataPipe = {.enable = true};
DataPipeObj_TypeDef Receiver_ptl_DataPipe = {.enable = true};
DataPipeObj_TypeDef Receiver_Ctl_DataPipe = {.enable = false};

DataPipeObj_TypeDef IMU_Log_DataPipe = {.enable = true};
DataPipeObj_TypeDef IMU_Smp_DataPipe = {.enable = true};
