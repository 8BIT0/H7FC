#include "Task_Navi.h"
#include "DataPipe.h"
#include "Srv_Baro.h"
#include "Srv_IMUSample.h"

/* internal vriable */
uint32_t TaskNavi_Period = 0;

void TaskNavi_Init(uint32_t period)
{

    TaskNavi_Period = period;
}




