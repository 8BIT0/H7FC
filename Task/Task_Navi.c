#include "cmsis_os.h"
#include "Task_Navi.h"
#include "DataPipe.h"
#include "Srv_Baro.h"
#include "Srv_OsCommon.h"
#include "Srv_DataHub.h"
#include "DataPipe.h"
#include "MadgwickAHRS.h"
#include "math_util.h"
#include "pos_data.h"

/* IMU coordinate is x->forward y->right z->down */
/*
    x Axis -> Roll  clock wise rotate positive
    y Axis -> Pitch noise up positive
    z Axis -> Yaw   anticlock wise rotate positice
*/


static void TaskNavi_FlipOver_Detect(void);

/* internal vriable */
uint32_t TaskNavi_Period = 0;

/* data structure definition */
DataPipe_CreateDataObj(IMUAtt_TypeDef, Navi_Attitude);
DataPipe_CreateDataObj(PosData_TypeDef, Navi_POS);

void TaskNavi_Init(uint32_t period)
{
    /* init DataPipe */
    memset(&Attitude_smp_DataPipe, 0, sizeof(Attitude_smp_DataPipe));
    memset(&POS_smp_DataPipe, 0, sizeof(POS_smp_DataPipe));

    memset(DataPipe_DataObjAddr(Navi_Attitude), 0, DataPipe_DataSize(Navi_Attitude));
    memset(DataPipe_DataObjAddr(Navi_POS), 0, DataPipe_DataSize(Navi_POS));
    
    Attitude_smp_DataPipe.data_addr = DataPipe_DataObjAddr(Navi_Attitude);
    Attitude_smp_DataPipe.data_size = DataPipe_DataSize(Navi_Attitude);
    DataPipe_Enable(&Attitude_smp_DataPipe);

    POS_smp_DataPipe.data_addr = DataPipe_DataObjAddr(Navi_POS);
    POS_smp_DataPipe.data_size = DataPipe_DataSize(Navi_POS);
    DataPipe_Enable(&POS_smp_DataPipe);

    TaskNavi_Period = period;
}

void TaskNavi_Core(void const *arg)
{
    uint32_t sys_time = SrvOsCommon.get_os_ms();
    bool imu_state = false;
    bool mag_state = false;
    uint32_t Org_IMU_TimeStamp = 0;
    uint32_t Flt_IMU_TimeStamp = 0;
    uint32_t MAG_TimeStamp = 0;
    float Acc_Scale = 0.0f;
    float Gyr_Scale = 0.0f;
    float Mag_Scale = 0.0f;
    IMUAtt_TypeDef attitude;
    float Flt_Acc[Axis_Sum] = {0.0f};
    float Flt_Gyr[Axis_Sum] = {0.0f};
    float Flt_Mag[Axis_Sum] = {0.0f};
    float Flt_IMU_Tempra = 0.0f;
    uint8_t IMU_Err = 0;
    uint8_t MAG_Err = 0;

    SrvDataHub.get_imu_init_state(&imu_state);
    SrvDataHub.get_mag_init_state(&mag_state);
    
    bool Attitude_Update = false;
    memset(&attitude, 0, sizeof(IMUAtt_TypeDef));
    
    while(1)
    {
        if(imu_state)
        {
            SrvDataHub.get_scaled_imu(&Flt_IMU_TimeStamp, &Acc_Scale, &Gyr_Scale, \
                                      &Flt_Acc[Axis_X], &Flt_Acc[Axis_Y], &Flt_Acc[Axis_Z], \
                                      &Flt_Gyr[Axis_X], &Flt_Gyr[Axis_Y], &Flt_Gyr[Axis_Z], \
                                      &Flt_IMU_Tempra, &IMU_Err);

            Attitude_Update = true;
        }
        
        if(mag_state)
        {
            SrvDataHub.get_scaled_mag(&MAG_TimeStamp, &Mag_Scale, \
                                      &Flt_Mag[Axis_X], &Flt_Mag[Axis_Y], &Flt_Mag[Axis_Z], \
                                      &MAG_Err);
        }
        else
        {
            Flt_Mag[Axis_X] = 0.0f;
            Flt_Mag[Axis_Y] = 0.0f;
            Flt_Mag[Axis_Z] = 0.0f;
        }
        
        if(Attitude_Update)
        {
            /* update Attitude */
            MadgwickAHRSupdate(Deg2Rad(Flt_Gyr[Axis_X]), Deg2Rad(-Flt_Gyr[Axis_Y]), Deg2Rad(-Flt_Gyr[Axis_Z]), \
                               Flt_Acc[Axis_X],           -Flt_Acc[Axis_Y],         -Flt_Acc[Axis_Z], \
                               Flt_Mag[Axis_X],           Flt_Mag[Axis_Y],          -Flt_Mag[Axis_Z]);
            
            if(MadgwickAHRS_Get_Attitude(&attitude.pitch, &attitude.roll, &attitude.yaw) && \
               MadgwickAHRS_Get_Quraterion(&attitude.q0, &attitude.q1, &attitude.q2, &attitude.q3))
            {
                attitude.time_stamp = SrvOsCommon.get_os_ms();
                DataPipe_DataObj(Navi_Attitude) = attitude;
            }
            
            TaskNavi_FlipOver_Detect();

            /* DataPipe Attitude Data to SrvDataHub */
            DataPipe_SendTo(&Attitude_smp_DataPipe, &Attitude_hub_DataPipe);

            /* DataPipe FlipOver State */
        }

        /* check imu data update freq on test */
        SrvOsCommon.precise_delay(&sys_time, TaskNavi_Period);
    }
}

static void TaskNavi_FlipOver_Detect(void)
{
    /* use roll angle detect drone up side down state */
    /* if drone flip over roll must between 180 ~ -180 */
    /* keep this value over than 1s we think is flip over */
    
}
