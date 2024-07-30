#include "DataPipe.h"

DataPipeObj_TypeDef SensorInitState_smp_DataPipe = {.enable = true};
DataPipeObj_TypeDef SensorInitState_hub_DataPipe = {.enable = true};
DataPipeObj_TypeDef SensorEnableState_smp_DataPipe = {.enable = true};
DataPipeObj_TypeDef SensorEnableState_hub_DataPipe = {.enable = true};

DataPipeObj_TypeDef Receiver_Smp_DataPipe = {.enable = true};
DataPipeObj_TypeDef Receiver_hub_DataPipe = {.enable = true};

DataPipeObj_TypeDef CtlData_Log_DataPipe = {.enable = true};
DataPipeObj_TypeDef CtlData_smp_DataPipe = {.enable = true};
DataPipeObj_TypeDef CtlData_hub_DataPipe = {.enable = true};

DataPipeObj_TypeDef IMU_Log_DataPipe = {.enable = true};
DataPipeObj_TypeDef IMU_Smp_DataPipe = {.enable = true};
DataPipeObj_TypeDef IMU_hub_DataPipe = {.enable = true};

DataPipeObj_TypeDef IMU_PriRange_Smp_DataPipe = {.enable = true};
DataPipeObj_TypeDef IMU_PriRange_hub_DataPipe = {.enable = true};
#if (IMU_CNT == 2)
DataPipeObj_TypeDef IMU_SecRange_Smp_DataPipe = {.enable = true};
DataPipeObj_TypeDef IMU_SecRange_hub_DataPipe = {.enable = true};
#endif

DataPipeObj_TypeDef Actuator_Smp_DataPipe = {.enable = true};
DataPipeObj_TypeDef Actuator_hub_DataPipe = {.enable = true};

DataPipeObj_TypeDef Altitude_Log_DataPipe = {.enable = true};
DataPipeObj_TypeDef Altitude_smp_DataPipe = {.enable = true};
DataPipeObj_TypeDef Altitude_hub_DataPipe = {.enable = true};

DataPipeObj_TypeDef Attitude_Log_DataPipe = {.enable = true};
DataPipeObj_TypeDef Attitude_smp_DataPipe = {.enable = true};
DataPipeObj_TypeDef Attitude_hub_DataPipe = {.enable = true};

DataPipeObj_TypeDef Baro_Log_DataPipe = {.enable = true};
DataPipeObj_TypeDef Baro_smp_DataPipe = {.enable = true};
DataPipeObj_TypeDef Baro_hub_DataPipe = {.enable = true};

DataPipeObj_TypeDef VCP_Connect_smp_DataPipe = {.enable = true};
DataPipeObj_TypeDef VCP_Connect_hub_DataPipe = {.enable = true};

DataPipeObj_TypeDef Flow_Log_DataPipe = {.enable = true};
DataPipeObj_TypeDef Flow_smp_DataPipe = {.enable = true};
DataPipeObj_TypeDef Flow_hub_DataPipe = {.enable = true};
