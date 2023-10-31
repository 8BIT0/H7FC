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
DataPipe_CreateDataObj(IMUAtt_TypeDef, Navi_Attitude);

void TaskNavi_Init(uint32_t period)
{
    /* init DataPipe */
    memset(&Attitude_cmp_DataPipe, 0, sizeof(Attitude_cmp_DataPipe));
    memset(DataPipe_DataObjAddr(Navi_Attitude), 0, sizeof(DataPipe_DataObj(Navi_Attitude)));
    Attitude_cmp_DataPipe.data_addr = DataPipe_DataObjAddr(Navi_Attitude);
    Attitude_cmp_DataPipe.data_size = DataPipe_DataSize(Navi_Attitude);
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
            /* update Attitude */
            MadgwickAHRSupdate(Gyr[Axis_X], Gyr[Axis_Y], Gyr[Axis_Z], Acc[Axis_X], Acc[Axis_Y], Acc[Axis_Z], Mag[Axis_X], Mag[Axis_Y], Mag[Axis_Z]);

            /* DataPipe Attitude Data to SrvDataHub */
            DataPipe_SendTo(&Attitude_cmp_DataPipe, &Attitude_hub_DataPipe);
        }

        /* check imu data update freq on test */
        SrvOsCommon.precise_delay(&sys_time, TaskNavi_Period);
    }
}


