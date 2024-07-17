#include "Task_Navi.h"
#include "DataPipe.h"
#include "Srv_Baro.h"
#include "Srv_OsCommon.h"
#include "Srv_DataHub.h"
#include "DataPipe.h"
#include "MadgwickAHRS.h"
#include "math_util.h"
#include <Eigen>
#include <stdio.h>
#include <iostream>

#include "HW_Def.h"
#include "debug_util.h"

#define NAVI_TAG " "
#define NAVI_INFO(fmt, ...) Debug_Print(&DebugP4, NAVI_TAG, fmt, ##__VA_ARGS__)

using namespace std;
using namespace Eigen;

/* IMU coordinate is x->forward y->right z->down */
/*
    x Axis -> Roll  clock wise rotate positive
    y Axis -> Pitch noise up positive
    z Axis -> Yaw   anticlock wise rotate positice
*/
#define FlipOver_Detect_HoldingTime 500 /* unit : ms */

static bool TaskNavi_FlipOver_Detect(float roll_angle);

/* internal vriable */
TaskNavi_Monitor_TypeDef TaskNavi_Monitor;

/* data structure definition */
DataPipe_CreateDataObj(IMUAtt_TypeDef,  Navi_Attitude);
DataPipe_CreateDataObj(PosData_TypeDef, Navi_POS);
DataPipe_CreateDataObj(AltData_TypeDef, Navi_Altitude);

void TaskNavi_Init(uint32_t period)
{
    memset(&TaskNavi_Monitor, 0, sizeof(TaskNavi_Monitor_TypeDef));

    /* init DataPipe */
    memset(&Attitude_smp_DataPipe, 0, sizeof(Attitude_smp_DataPipe));
    memset(&POS_smp_DataPipe, 0, sizeof(POS_smp_DataPipe));

    memset(DataPipe_DataObjAddr(Navi_Attitude), 0, DataPipe_DataSize(Navi_Attitude));
    memset(DataPipe_DataObjAddr(Navi_POS), 0, DataPipe_DataSize(Navi_POS));
    memset(DataPipe_DataObjAddr(Navi_Altitude), 0, DataPipe_DataSize(Navi_Altitude));

    Attitude_smp_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Navi_Attitude);
    Attitude_smp_DataPipe.data_size = DataPipe_DataSize(Navi_Attitude);
    DataPipe_Enable(&Attitude_smp_DataPipe);

    POS_smp_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Navi_POS);
    POS_smp_DataPipe.data_size = DataPipe_DataSize(Navi_POS);
    DataPipe_Enable(&POS_smp_DataPipe);

    Altitude_smp_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Navi_Altitude);
    Altitude_smp_DataPipe.data_size = DataPipe_DataSize(Navi_Altitude);
    DataPipe_Enable(&Altitude_smp_DataPipe);

    TaskNavi_Monitor.period = period;

    /* eigen test */
    Matrix<float, 2, 3> matrix_23;
    Matrix<float, 3, 1> vd_3d;
    Matrix<float, 2, 1> result2;

    matrix_23 << 1, 2, 3, 4, 5, 6;
    vd_3d << 3, 2, 1;

    result2 = matrix_23 * vd_3d;
    NAVI_INFO("Eigen Test\r\n");
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 1; j++)
        {
            NAVI_INFO("%f\t", result2(i, j));
        }
        NAVI_INFO("\r\n");
    }
    /* eigen test */
}

void TaskNavi_Core(void const *arg)
{
    uint32_t sys_time = SrvOsCommon.get_os_ms();
    bool imu_state = false;
    bool mag_state = false;
    bool bar_state = false;
    uint32_t Flt_IMU_TimeStamp = 0;
    uint32_t MAG_TimeStamp = 0;
    uint32_t Baro_TimeStamp = 0;
    float Acc_Scale = 0.0f;
    float Gyr_Scale = 0.0f;
    float Mag_Scale = 0.0f;
    IMUAtt_TypeDef attitude;
    AlgoAttData_TypeDef algo_att;
    float Flt_Acc[Axis_Sum] = {0.0f};
    float Flt_Gyr[Axis_Sum] = {0.0f};
    float Flt_Mag[Axis_Sum] = {0.0f};
    float Flt_IMU_Tempra = 0.0f;
    float Bar_Pres = 0.0f;
    float Baro_Alt = 0.0f;
    float Baro_Alt_Offset = 0.0f;
    float Baro_Tempra = 0.0f;
    uint8_t IMU_Err = 0;
    uint8_t MAG_Err = 0;
    uint8_t BAR_Err = 0;
    
    bool Attitude_Update = false;
    memset(&attitude, 0, sizeof(IMUAtt_TypeDef));
    memset(&algo_att, 0, sizeof(AlgoAttData_TypeDef));

    while(1)
    {
        SrvDataHub.get_imu_init_state(&imu_state);
        SrvDataHub.get_mag_init_state(&mag_state);
        SrvDataHub.get_baro_init_state(&bar_state);

        if(imu_state)
        {
            Attitude_Update = SrvDataHub.get_scaled_imu(&Flt_IMU_TimeStamp, &Acc_Scale, &Gyr_Scale, \
                                                        &Flt_Acc[Axis_X], &Flt_Acc[Axis_Y], &Flt_Acc[Axis_Z], \
                                                        &Flt_Gyr[Axis_X], &Flt_Gyr[Axis_Y], &Flt_Gyr[Axis_Z], \
                                                        &Flt_IMU_Tempra, &IMU_Err);
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
            MadgwickAHRSupdate(&algo_att, Deg2Rad(Flt_Gyr[Axis_X]), Deg2Rad(Flt_Gyr[Axis_Y]), Deg2Rad(Flt_Gyr[Axis_Z]), \
                                          Flt_Acc[Axis_X],          Flt_Acc[Axis_Y],          Flt_Acc[Axis_Z], \
                                          Flt_Mag[Axis_X],          Flt_Mag[Axis_Y],          Flt_Mag[Axis_Z]);
            
            attitude.pitch = algo_att.pitch;
            attitude.roll  = algo_att.roll;
            attitude.yaw   = algo_att.yaw;
            attitude.q0    = algo_att.q0;
            attitude.q1    = algo_att.q1;
            attitude.q2    = algo_att.q2;
            attitude.q3    = algo_att.q3;
            
            attitude.flip_over = TaskNavi_FlipOver_Detect(attitude.roll);
            attitude.time_stamp = SrvOsCommon.get_os_ms();

            DataPipe_DataObj(Navi_Attitude) = attitude;

            /* DataPipe Attitude Data to SrvDataHub */
            DataPipe_SendTo(&Attitude_smp_DataPipe, &Attitude_hub_DataPipe);
            DataPipe_SendTo(&Attitude_smp_DataPipe, &Attitude_Log_DataPipe);
        }

        /* comput baro altitude */
        if (bar_state && Attitude_Update && \
            SrvDataHub.get_baro_altitude(&Baro_TimeStamp, &Bar_Pres, &Baro_Alt, &Baro_Alt_Offset, &Baro_Tempra, &BAR_Err))
        {
            DataPipe_DataObj(Navi_Altitude).time = SrvOsCommon.get_os_ms();
            DataPipe_DataObj(Navi_Altitude).alt = 0.0f;

            DataPipe_SendTo(&Altitude_smp_DataPipe, &Altitude_hub_DataPipe);
            DataPipe_SendTo(&Altitude_smp_DataPipe, &Altitude_Log_DataPipe);
        }

        /* check imu data update freq on test */
        SrvOsCommon.precise_delay(&sys_time, TaskNavi_Monitor.period);
    }
}

static bool TaskNavi_FlipOver_Detect(float roll_angle)
{
    /* use roll angle detect drone up side down state */
    /* if drone flip over roll must between 180 ~ -180 */
    /* keep this value over than 1s we think is flip over */
    static uint32_t FilpOver_Trigger_Time = 0;
    static uint32_t FlipOver_ResetTrigger_Time = 0;
    static bool FlipOver_State = false;

    if(!FlipOver_State)
    {
        if(((roll_angle < 180.0f) && (roll_angle > 80.0f)) || ((roll_angle > -180.0f) && (roll_angle < -80.0f)))
        {
            if(FilpOver_Trigger_Time == 0)
            {
                FilpOver_Trigger_Time = SrvOsCommon.get_os_ms();
            }
            else
            {
                if((SrvOsCommon.get_os_ms() - FilpOver_Trigger_Time) >= FlipOver_Detect_HoldingTime)
                {
                    FilpOver_Trigger_Time = 0;
                    FlipOver_State = true;
                }
            }
        }
        else
            FilpOver_Trigger_Time = 0;
    }
    else
    {
        if((roll_angle < 20.0f) && (roll_angle > -20.0f))
        {
            if(FlipOver_ResetTrigger_Time == 0)
            {
                FlipOver_ResetTrigger_Time = SrvOsCommon.get_os_ms();
            }
            else
            {
                if(SrvOsCommon.get_os_ms() - FlipOver_ResetTrigger_Time >= FlipOver_Detect_HoldingTime)
                {
                    FlipOver_ResetTrigger_Time = 0;
                    FlipOver_State = false;
                }
            }
        }
        else
            FlipOver_ResetTrigger_Time = 0;
    }

    return FlipOver_State;
}

