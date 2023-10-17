#include "cmsis_os.h"
#include "Task_Navi.h"
#include "DataPipe.h"
#include "Srv_Baro.h"
#include "Srv_OsCommon.h"
#include "Srv_DataHub.h"
#include "DataPipe.h"

/* internal vriable */
uint32_t TaskNavi_Period = 0;

/* data structure definition */
typedef struct
{
    uint32_t time_stamp;

    float acc[Axis_Sum];
    float gyr[Axis_Sum];
}TaskNavi_IMUData_TypeDef;

typedef struct
{
    float yaw;
    float pitch;
    float roll;
}TaskNavi_Attitude_TypeDef;
/* data structure definition */

static TaskNavi_Attitude_TypeDef Attitude;

void TaskNavi_Init(uint32_t period)
{
    memset(&Attitude, 0, sizeof(Attitude));

    TaskNavi_Period = period;
}

void TaskNavi_Core(void const *arg)
{
    uint32_t sys_time = SrvOsCommon.get_os_ms();

    while(1)
    {
        
        SrvOsCommon.precise_delay(&sys_time, TaskNavi_Period);
    }
}


