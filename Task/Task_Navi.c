#include "cmsis_os.h"
#include "Task_Navi.h"
#include "DataPipe.h"
#include "Srv_Baro.h"
#include "Srv_OsCommon.h"
#include "Srv_DataHub.h"
#include "DataPipe.h"
#include "MadgwickAHRS.h"

/* internal vriable */
uint32_t TaskNavi_Period = 0;

/* data structure definition */
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

    /* init DataPipe */
    memset(&Attitude_cmp_DataPipe, 0, sizeof(Attitude_cmp_DataPipe));
    Attitude_cmp_DataPipe.data_addr = (uint32_t)&Attitude;
    Attitude_cmp_DataPipe.data_size = sizeof(Attitude);
    Attitude_cmp_DataPipe.trans_finish_cb = ;
    DataPipe_Enable(&Attitude_cmp_DataPipe);
    
    TaskNavi_Period = period;
}

void TaskNavi_Core(void const *arg)
{
    uint32_t sys_time = SrvOsCommon.get_os_ms();
    bool imu_state = false;
    bool mag_state = false;
    uint32_t IMU_TimeStamp = 0;
    uint32_t MAG_TimeStamp = 0;
    float Acc_Scale = 0.0f;
    float Gyr_Scale = 0.0f;
    float Mag_Scale = 0.0f;
    float Acc[Axis_Sum] = {0.0f};
    float Gyr[Axis_Sum] = {0.0f};
    float Mag[Axis_Sum] = {0.0f};
    float IMU_Tempra = 0.0f;
    uint8_t IMU_Err = 0;
    uint8_t MAG_Err = 0;

    SrvDataHub.get_imu_init_state(&imu_state);
    SrvDataHub.get_mag_init_state(&mag_state);
    
    bool Attitude_Update = false;

    while(1)
    {
        if(imu_state)
        {
            SrvDataHub.get_scaled_imu(&IMU_TimeStamp, &Acc_Scale, &Gyr_Scale, \
                                      &Acc[Axis_X], &Acc[Axis_Y], &Acc[Axis_Z], \
                                      &Gyr[Axis_X], &Gyr[Axis_Y], &Gyr[Axis_Z], \
                                      &IMU_Tempra, &IMU_Err);

            Attitude_Update = true;                                
        }
        
        if(mag_state)
        {
            SrvDataHub.get_scaled_mag(&MAG_TimeStamp, &Mag_Scale, \
                                      &Mag[Axis_X], &Mag[Axis_Y], &Mag[Axis_Z], \
                                      &MAG_Err);
        }
        else
        {
            Mag[Axis_X] = 0.0f;
            Mag[Axis_Y] = 0.0f;
            Mag[Axis_Z] = 0.0f;
        }
        
        if(Attitude_Update)
        {
            /* DataPipe Attitude Data to SrvDataHub */

        }

        /* check imu data update freq on test */
        SrvOsCommon.precise_delay(&sys_time, TaskNavi_Period);
    }
}


