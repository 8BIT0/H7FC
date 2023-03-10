#include "Srv_DataHub.h"

/* internal variable */
SrvDataHub_Monitor_TypeDef SrvDataHub_Monitor;

/* Pipe Object */
DataPipe_CreateDataObj(SrvIMU_UnionData_TypeDef, PtlPriIMU_Data);
DataPipe_CreateDataObj(SrvIMU_UnionData_TypeDef, PtlSecIMU_Data);

DataPipe_CreateDataObj(SrvRecever_RCSig_TypeDef, Proto_Rc);

/* internal function */
static void SrvComProto_PipeRcTelemtryDataFinish_Callback(DataPipeObj_TypeDef *obj);

static void SrvDataHub_Init(void)
{
    memset(&DataHub, 0, sizeof(DataHub));

    /* init pipe object */
    memset(DataPipe_DataObjAddr(PtlPriIMU_Data), NULL, DataPipe_DataSize(PtlPriIMU_Data));
    IMU_Ptl_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(PtlPriIMU_Data);
    IMU_Ptl_DataPipe.data_size = DataPipe_DataSize(PtlPriIMU_Data);
    IMU_Ptl_DataPipe.trans_finish_cb = SrvComProto_PipeRcTelemtryDataFinish_Callback;
    // DataPipe_Set_RxInterval(&IMU_Ptl_DataPipe, Runtime_MsToUs(5)); /* limit pipe frequence to 200Hz */
    DataPipe_Enable(&IMU_Ptl_DataPipe);

    /* init pipe object */
    memset(DataPipe_DataObjAddr(Proto_Rc), 0, DataPipe_DataSize(Proto_Rc));
    Receiver_ptl_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Proto_Rc);
    Receiver_ptl_DataPipe.data_size = DataPipe_DataSize(Proto_Rc);
    Receiver_ptl_DataPipe.trans_finish_cb = SrvComProto_PipeRcTelemtryDataFinish_Callback;
    // DataPipe_Set_RxInterval(&Receiver_ptl_DataPipe, Runtime_MsToUs(20)); /* limit pipe frequence to 50Hz */
    DataPipe_Enable(&Receiver_ptl_DataPipe);

    memset(&SrvDataHub_Monitor, 0, sizeof(SrvDataHub_Monitor));
    SrvDataHub_Monitor.init_state = true;
}

static void SrvComProto_PipeRcTelemtryDataFinish_Callback(DataPipeObj_TypeDef *obj)
{
    if ((!SrvDataHub_Monitor.init_state) || (obj == NULL))
        return;

    if (obj == &Receiver_ptl_DataPipe)
    {
    }
    else if (obj == &IMU_Ptl_DataPipe)
    {
        SrvDataHub_Monitor.update_reg.bit.imu = true;

        if (SrvDataHub_Monitor.inuse_reg.bit.imu)
            SrvDataHub_Monitor.inuse_reg.bit.imu = false;

        SrvDataHub_Monitor.data.imu_update_time = DataPipe_DataObj(PtlPriIMU_Data).data.time_stamp;
        SrvDataHub_Monitor.data.acc_scale = DataPipe_DataObj(PtlPriIMU_Data).data.acc_scale;
        SrvDataHub_Monitor.data.gyr_scale = DataPipe_DataObj(PtlPriIMU_Data).data.gyr_scale;
        SrvDataHub_Monitor.data.imu_temp = DataPipe_DataObj(PtlPriIMU_Data).data.tempera;

        SrvDataHub_Monitor.data.flt_acc_x = DataPipe_DataObj(PtlPriIMU_Data).data.flt_acc[Axis_X];
        SrvDataHub_Monitor.data.flt_acc_y = DataPipe_DataObj(PtlPriIMU_Data).data.flt_acc[Axis_Y];
        SrvDataHub_Monitor.data.flt_acc_z = DataPipe_DataObj(PtlPriIMU_Data).data.flt_acc[Axis_Z];

        SrvDataHub_Monitor.data.flt_gyr_x = DataPipe_DataObj(PtlPriIMU_Data).data.flt_gyr[Axis_X];
        SrvDataHub_Monitor.data.flt_gyr_y = DataPipe_DataObj(PtlPriIMU_Data).data.flt_gyr[Axis_Y];
        SrvDataHub_Monitor.data.flt_gyr_z = DataPipe_DataObj(PtlPriIMU_Data).data.flt_gyr[Axis_Z];

        SrvDataHub_Monitor.data.org_acc_x = DataPipe_DataObj(PtlPriIMU_Data).data.org_acc[Axis_X];
        SrvDataHub_Monitor.data.org_acc_y = DataPipe_DataObj(PtlPriIMU_Data).data.org_acc[Axis_Y];
        SrvDataHub_Monitor.data.org_acc_z = DataPipe_DataObj(PtlPriIMU_Data).data.org_acc[Axis_Z];

        SrvDataHub_Monitor.data.org_gyr_x = DataPipe_DataObj(PtlPriIMU_Data).data.org_gyr[Axis_X];
        SrvDataHub_Monitor.data.org_gyr_y = DataPipe_DataObj(PtlPriIMU_Data).data.org_gyr[Axis_Y];
        SrvDataHub_Monitor.data.org_gyr_z = DataPipe_DataObj(PtlPriIMU_Data).data.org_gyr[Axis_Z];

        SrvDataHub_Monitor.update_reg.bit.imu = false;
    }
}

static bool SrvDataHub_Get_Raw_IMU(uint32_t *time_stamp, float *acc_x, float *acc_y, float *acc_z, float *gyr_x, float *gyr_y, float *gyr_z, float *tmpr;)
{
    if ((time_stamp == NULL) ||
        (acc_x == NULL) ||
        (acc_y == NULL) ||
        (acc_z == NULL) ||
        (gyr_x == NULL) ||
        (gyr_y == NULL) ||
        (gyr_z == NULL))
        return false;

reupdate_imu:
    SrvDataHub_Monitor.inuse_reg.bit.imu = true;
    *time_stamp = SrvDataHub_Monitor.data.imu_update_time;
    *acc_x = SrvDataHub_Monitor.data.org_acc_x;
    *acc_y = SrvDataHub_Monitor.data.org_acc_y;
    *acc_z = SrvDataHub_Monitor.data.org_acc_z;
    *gyr_x = SrvDataHub_Monitor.data.org_gyr_x;
    *gyr_y = SrvDataHub_Monitor.data.org_gyr_y;
    *gyr_z = SrvDataHub_Monitor.data.org_gyr_z;
    *tmpr = SrvDataHub_Monitor.data.imu_temp;
    if (!SrvDataHub_Monitor.inuse_reg.bit.imu)
        goto reupdate_imu;

    SrvDataHub_Monitor.inuse_reg.bit.imu = false;

    return true;
}

static bool SrvDataHub_Get_Scaled_IMU(uint32_t *time_stamp, float *acc_x, float *acc_y, float *acc_z, float *gyr_x, float *gyr_y, float *gyr_z)
{
    if ((time_stamp == NULL) ||
        (acc_x == NULL) ||
        (acc_y == NULL) ||
        (acc_z == NULL) ||
        (gyr_x == NULL) ||
        (gyr_y == NULL) ||
        (gyr_z == NULL))
        return false;

    return true;
}
