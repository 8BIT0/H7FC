#include "Task_Navi.h"
#include "DataPipe.h"
#include "Srv_Baro.h"
#include "Srv_DataHub.h"

/* internal vriable */
uint32_t TaskNavi_Period = 0;
static TaskNavi_Attitude_TypeDef Attitude;

void TaskNavi_Init(uint32_t period)
{
    memset(&Attitude, 0, sizeof(Attitude));
    
    TaskNavi_Period = period;
}




