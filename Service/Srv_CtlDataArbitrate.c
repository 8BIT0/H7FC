/*
 * Auther: 8_B!T0
 * Bref: in this project we may have multiple control signal source like RC_Remote MAVLink_Attitude_Expection MAVLinkl_AngularSpeed_Expection
 *       but doing acturator control we must to make the control signal unique
 *       in this file we can make multiple control signal in but out with the only one  
 */
#include <math.h>
#include "DataPipe.h"
#include "Srv_CtlDataArbitrate.h"

#define ATTITUDE_ACCURACY 100
#define MAX_ATTITUDE_ANGLE_RANGE 60
#define MIN_ATTITUDE_ANGLE_RANGE -60

#define RC_TAKING_OVER_CONFIRM 2000 /* unit : ms */
#define TAKINGOVER_NEGOCIOATE_TIMEOUT 10000 /* unit : ms */

#define CTL_DATA_UPDATE_TIMEOUT 2000 /* unit: ms */

/* internal vriable */
static Srv_CtlArbitrateMonitor_TypeDef SrvCtlArbitrateMonitor;
static void Srv_OnPlaneComputer_TakingOver_RequireSend(ControlData_TypeDef cur_ctl_data);

/* internal function */
static void Srv_CtlData_ConvertGimbal_ToAtt(uint8_t *gimbal_percent, float *exp_pitch, float *exp_roll);
static void Srv_CtlData_ConvertGimbal_ToAngularSpeed(uint8_t *gimbal_percent, float *exp_gyr_x, float *exp_gyr_y, float *exp_gyr_z);
static void Srv_CtlData_ControlPrivilege_Req_Check(void);

/* external function */
static bool Srv_CtlDataArbitrate_Init(Srv_CtlRange_TypeDef att_range[Att_Ctl_Sum], Srv_CtlRange_TypeDef angularspeed_range[Axis_Sum]);
static void Srv_CtlDataArbitrate_Update(ControlData_TypeDef *inuse_ctl_Data);
static Srv_CtlExpectionData_TypeDef Srv_CtlDataArbitrate_GetData(void);

/* external vriable */
Srv_CtlDataArbitrate_TypeDef Srv_CtlDataArbitrate = {
    .init = Srv_CtlDataArbitrate_Init,
    .negociate_update = Srv_CtlDataArbitrate_Update,
    .get_data = Srv_CtlDataArbitrate_GetData,
};

static bool Srv_CtlDataArbitrate_Init(Srv_CtlRange_TypeDef att_range[Att_Ctl_Sum], Srv_CtlRange_TypeDef angularspeed_range[Axis_Sum])
{
    uint8_t index = 0;

    int16_t ref_gyr_range = 0;

    uint8_t pri_acc_range = 0;
    uint16_t pri_gyr_range = 0;
    
    uint8_t sec_acc_range = 0;
    uint16_t sec_gyr_range = 0;

    int16_t input_range_max = 0;
    int16_t input_range_min = 0;
    
    memset(&SrvCtlArbitrateMonitor, 0, sizeof(SrvCtlArbitrateMonitor));

    for(index = 0; index < Att_Ctl_Sum; index ++)
    {
        input_range_max = (((int16_t)(att_range[index].max * 1000)) / 1000);
        input_range_min = (((int16_t)(att_range[index].min * 1000)) / 1000);

        /* max or min is 0 */
        if((input_range_max == 0) || 
           (input_range_min == 0))
            return false;
        
        /* max lower than min */
        if(input_range_max <= input_range_min)
            return false;

        /* bigger than define max */
        if(input_range_max > MAX_ATTITUDE_ANGLE_RANGE)
            att_range[index].max = MAX_ATTITUDE_ANGLE_RANGE;

        /* lower than define min */
        if(input_range_min < MIN_ATTITUDE_ANGLE_RANGE)
            att_range[index].min = MIN_ATTITUDE_ANGLE_RANGE;

        if(att_range[index].enable_dead_zone)
        {
            input_range_max = (((int16_t)(att_range[index].dead_zone_max * 1000)) / 1000) ;
            input_range_min = (((int16_t)(att_range[index].dead_zone_min * 1000)) / 1000);
        
            if(input_range_max <= input_range_min)
                return false;
        }
    }

    /* get imu angular speed range */
    if(!SrvDataHub.get_pri_imu_range(&pri_acc_range, &pri_gyr_range) && !SrvDataHub.get_sec_imu_range(&sec_acc_range, &sec_gyr_range))
        return false;

    if((pri_gyr_range == 0) && (sec_gyr_range == 0))
        return false;

    if(pri_gyr_range || sec_gyr_range)
    {
        if(pri_gyr_range <= sec_gyr_range)
        {
            if(pri_gyr_range)
            {
                ref_gyr_range = pri_gyr_range;
            }
            else
                ref_gyr_range = sec_gyr_range;
        }
        else
            ref_gyr_range = sec_gyr_range;
    }

    for(index = 0; index < Axis_Sum; index ++)
    {
        input_range_max = (((int16_t)(angularspeed_range[index].max * 1000)) / 1000);
        input_range_min = (((int16_t)(angularspeed_range[index].min * 1000)) / 1000);

        if((input_range_max == 0) || (input_range_min == 0))
           return false;
        
        if(input_range_max <= input_range_min)
           return false;

        if(input_range_max > ref_gyr_range)
            angularspeed_range[index].max = ref_gyr_range;

        if(input_range_min < (ref_gyr_range * -1))
            angularspeed_range[index].min = (ref_gyr_range * -1);

        if(angularspeed_range[index].enable_dead_zone)
        {
            input_range_max = (((int16_t)(angularspeed_range[index].dead_zone_max * 1000)) / 1000);
            input_range_min = (((int16_t)(angularspeed_range[index].dead_zone_min * 1000)) / 1000);

            if(input_range_max <= input_range_min)
                return false;
        }
    }

    for(index = 0; index < 2; index ++)
    {
        SrvCtlArbitrateMonitor.att_ctl_range[index] = att_range[index];
    }

    for(index = 0; index < 3; index ++)
    {
        SrvCtlArbitrateMonitor.angularspeed_ctl_range[index] = angularspeed_range[index];
    }
    
    memset(&SrvCtlArbitrateMonitor.RC_CtlData, 0, sizeof(ControlData_TypeDef));
    memset(&SrvCtlArbitrateMonitor.OPC_CtlData, 0, sizeof(ControlData_TypeDef));
    memset(&SrvCtlArbitrateMonitor.InUse_CtlData, 0, sizeof(ControlData_TypeDef));

    /* set default signal source/type/control mode */
    SrvCtlArbitrateMonitor.InUse_CtlData.sig_source = ControlData_Src_RC;
    SrvCtlArbitrateMonitor.InUse_CtlData.control_mode = Attitude_Control;

    return true;
}

/* 
 *    signal from the telemetry use channel control drone
 *    signal from the on plane computer use expection physical value control drone
 */
static void Srv_CtlDataArbitrate_Update(ControlData_TypeDef *inuse_ctl_data)
{
    float *exp_pitch = NULL;
    float *exp_roll = NULL;

    float *exp_gyr_x = NULL;
    float *exp_gyr_y = NULL;
    float *exp_gyr_z = NULL;

    if(inuse_ctl_data)
    {
        /* check on plane computer online state */

        SrvDataHub.get_rc_control_data(&SrvCtlArbitrateMonitor.RC_CtlData);
        SrvDataHub.get_opc_control_data(&SrvCtlArbitrateMonitor.OPC_CtlData);
        
        /* check signal update */
        SrvCtlArbitrateMonitor.InUse_CtlData.arm_state = DRONE_ARM;
        SrvCtlArbitrateMonitor.InUse_CtlData.fail_safe = true;
        
        if(SrvCtlArbitrateMonitor.InUse_CtlData.sig_source == ControlData_Src_RC)
        {
            if(SrvCtlArbitrateMonitor.RC_CtlData.update_time_stamp)
                memcpy(&SrvCtlArbitrateMonitor.InUse_CtlData, &SrvCtlArbitrateMonitor.RC_CtlData, sizeof(ControlData_TypeDef));
        }
        else if(SrvCtlArbitrateMonitor.InUse_CtlData.sig_source == ControlData_Src_OPC)
        {
            if(SrvCtlArbitrateMonitor.OPC_CtlData.update_time_stamp)
                memcpy(&SrvCtlArbitrateMonitor.InUse_CtlData, &SrvCtlArbitrateMonitor.OPC_CtlData, sizeof(ControlData_TypeDef));
        }

        /* check any taking over ack info */
        Srv_CtlData_ControlPrivilege_Req_Check();

        /* check failsafe first */
        if(SrvCtlArbitrateMonitor.InUse_CtlData.fail_safe)
        {
            if(SrvCtlArbitrateMonitor.InUse_CtlData.sig_source == ControlData_Src_RC)
            {
                /* require on plane computer taking over control and check ack */
                Srv_OnPlaneComputer_TakingOver_RequireSend(SrvCtlArbitrateMonitor.InUse_CtlData);
            }
            else if(SrvCtlArbitrateMonitor.InUse_CtlData.sig_source == ControlData_Src_OPC)
            {
                /* hovering for 10 sec wait remote set all gimbal in the mid pos. set arm toggle at the DRONE_ARM state */
                /* under the statement up top, set ARM toggle on DRONE_DISARM then remote will taking over */
            }
        }
        
        exp_pitch = &SrvCtlArbitrateMonitor.InUse_CtlData.exp_att_pitch;
        exp_roll = &SrvCtlArbitrateMonitor.InUse_CtlData.exp_att_roll;

        exp_gyr_x = &SrvCtlArbitrateMonitor.InUse_CtlData.exp_gyr_x;
        exp_gyr_y = &SrvCtlArbitrateMonitor.InUse_CtlData.exp_gyr_y;
        exp_gyr_z = &SrvCtlArbitrateMonitor.InUse_CtlData.exp_gyr_z;

        if(SrvCtlArbitrateMonitor.InUse_CtlData.sig_source != ControlData_Src_OPC)
        {
            if(SrvCtlArbitrateMonitor.InUse_CtlData.fail_safe)
            {
                SrvCtlArbitrateMonitor.InUse_CtlData.gimbal_percent[Gimbal_Throttle] = 0;
                SrvCtlArbitrateMonitor.InUse_CtlData.gimbal_percent[Gimbal_Pitch] = 50;
                SrvCtlArbitrateMonitor.InUse_CtlData.gimbal_percent[Gimbal_Roll] = 50;
                SrvCtlArbitrateMonitor.InUse_CtlData.gimbal_percent[Gimbal_Yaw] = 50;
            }

            /* convert rc channel value into exptection attitude or angluar speed */
            Srv_CtlData_ConvertGimbal_ToAtt(SrvCtlArbitrateMonitor.InUse_CtlData.gimbal_percent, exp_pitch, exp_roll);
            Srv_CtlData_ConvertGimbal_ToAngularSpeed(SrvCtlArbitrateMonitor.InUse_CtlData.gimbal_percent, exp_gyr_x, exp_gyr_y, exp_gyr_z);
        }

        memcpy(inuse_ctl_data, &SrvCtlArbitrateMonitor.InUse_CtlData, sizeof(ControlData_TypeDef));
    }
}

/* remote gimbal only can control pitch and roll angle but yaw angle */
static void Srv_CtlData_ConvertGimbal_ToAtt(uint8_t *gimbal_percent, float *exp_pitch, float *exp_roll)
{
    float pos_trip = 0.0f;
    float neg_trip = 0.0f;
    float gimbal_percent_tmp = 0.0f;
    int16_t diff_to_idle = 0;
    int16_t dead_zone_max = 0;
    int16_t dead_zone_min = 0;

    if(gimbal_percent && exp_pitch && exp_roll)
    {
        /**************************************************************** pitch section ******************************************************************/
        pos_trip = SrvCtlArbitrateMonitor.att_ctl_range[Att_Pitch].max - SrvCtlArbitrateMonitor.att_ctl_range[Att_Pitch].idle;
        neg_trip = SrvCtlArbitrateMonitor.att_ctl_range[Att_Pitch].idle- SrvCtlArbitrateMonitor.att_ctl_range[Att_Pitch].min;
        gimbal_percent_tmp = (gimbal_percent[Gimbal_Pitch] - 50) / 50.0f;

        if((gimbal_percent[Gimbal_Pitch] - 50) > 0)
        {
            (*exp_pitch) = (pos_trip * gimbal_percent_tmp) + SrvCtlArbitrateMonitor.att_ctl_range[Att_Pitch].idle;
            diff_to_idle = (int16_t)((pos_trip * gimbal_percent_tmp) * ATTITUDE_ACCURACY);
        }
        else if((gimbal_percent[Gimbal_Pitch] - 50) < 0)
        {
            (*exp_pitch) = (neg_trip * gimbal_percent_tmp) + SrvCtlArbitrateMonitor.att_ctl_range[Att_Pitch].idle;
            diff_to_idle = (int16_t)((neg_trip * gimbal_percent_tmp) * ATTITUDE_ACCURACY);
        }
        else
            (*exp_pitch) = SrvCtlArbitrateMonitor.att_ctl_range[Att_Pitch].idle;

        if(SrvCtlArbitrateMonitor.att_ctl_range[Att_Pitch].enable_dead_zone)
        {
            dead_zone_max = (int16_t)(SrvCtlArbitrateMonitor.att_ctl_range[Att_Pitch].dead_zone_max * ATTITUDE_ACCURACY);
            dead_zone_min = (int16_t)(SrvCtlArbitrateMonitor.att_ctl_range[Att_Pitch].dead_zone_min * ATTITUDE_ACCURACY);

            if((diff_to_idle <= dead_zone_max) && (diff_to_idle >= dead_zone_min))
            {
                /* expection value under the dead zone */
                (*exp_pitch) = SrvCtlArbitrateMonitor.att_ctl_range[Att_Pitch].idle;
            }
        }

        /**************************************************************** roll section ******************************************************************/
        pos_trip = SrvCtlArbitrateMonitor.att_ctl_range[Att_Roll].max - SrvCtlArbitrateMonitor.att_ctl_range[Att_Roll].idle;
        neg_trip = SrvCtlArbitrateMonitor.att_ctl_range[Att_Roll].idle - SrvCtlArbitrateMonitor.att_ctl_range[Att_Roll].min;
        gimbal_percent_tmp = (gimbal_percent[Gimbal_Roll] - 50) / 50.0f;
 
        if((gimbal_percent[Gimbal_Roll] - 50) > 0)
        {
            (*exp_roll) = (pos_trip * gimbal_percent_tmp) + SrvCtlArbitrateMonitor.att_ctl_range[Att_Roll].idle;
            diff_to_idle = (int16_t)((pos_trip * gimbal_percent_tmp) * ATTITUDE_ACCURACY);
        }
        else if((gimbal_percent[Gimbal_Roll] - 50) < 0)
        {
            (*exp_roll) = (neg_trip * gimbal_percent_tmp) + SrvCtlArbitrateMonitor.att_ctl_range[Att_Roll].idle;
            diff_to_idle = (int16_t)((neg_trip * gimbal_percent_tmp) * ATTITUDE_ACCURACY);
        }
        else
            (*exp_roll) = SrvCtlArbitrateMonitor.att_ctl_range[Att_Roll].idle;

        if(SrvCtlArbitrateMonitor.att_ctl_range[Att_Roll].enable_dead_zone)
        {
            dead_zone_max = (int16_t)(SrvCtlArbitrateMonitor.att_ctl_range[Att_Roll].dead_zone_max * ATTITUDE_ACCURACY);
            dead_zone_min = (int16_t)(SrvCtlArbitrateMonitor.att_ctl_range[Att_Roll].dead_zone_min * ATTITUDE_ACCURACY);

            if((diff_to_idle <= dead_zone_max) && (diff_to_idle >= dead_zone_min))
            {
                /* expection value under the dead zone */
                (*exp_roll) = SrvCtlArbitrateMonitor.att_ctl_range[Att_Roll].idle;
            }
        }
    }
}

static void Srv_CtlData_ControlPrivilege_Req_Check(void)
{
    if(SrvCtlArbitrateMonitor.InUse_CtlData.sig_source == ControlData_Src_RC)
    {
        if(SrvCtlArbitrateMonitor.InUse_CtlData.aux.bit.taking_over_req)
        {
            if(SrvCtlArbitrateMonitor.RC_TakingOver_ReqHold == 0)
            {
                SrvCtlArbitrateMonitor.RC_TakingOver_ReqHold = SrvOsCommon.get_os_ms();
            }
        }
        else
        {
            if(SrvOsCommon.get_os_ms() - SrvCtlArbitrateMonitor.RC_TakingOver_ReqHold >= RC_TAKING_OVER_CONFIRM)
            {
                /* send taking over require to on plane computer */
            }
            
            SrvCtlArbitrateMonitor.RC_TakingOver_ReqHold = 0;
        }
    }
    else if(SrvCtlArbitrateMonitor.InUse_CtlData.sig_source == ControlData_Src_OPC)
    {
        /* check telemetry take over request */
        if(SrvCtlArbitrateMonitor.RC_CtlData.aux.bit.taking_over_req)
        {
            /* RC remote require to taking over */
        }
    }
}


static void Srv_CtlData_ConvertGimbal_ToAngularSpeed(uint8_t *gimbal_percent, float *exp_gyr_x, float *exp_gyr_y, float *exp_gyr_z)
{
    float pos_trip = 0.0f;
    float neg_trip = 0.0f;
    float gimbal_percent_tmp = 0.0f;

    /* noticed dead zone is useless in angular speed control */
    if(gimbal_percent && exp_gyr_x && exp_gyr_y && exp_gyr_z)
    {
        /*************************************************************** gyro x section ******************************************************************/
        pos_trip = SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_X].max - SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_X].idle;
        neg_trip = SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_X].idle - SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_X].min;
        gimbal_percent_tmp = (gimbal_percent[Gimbal_Roll] - 50) / 50.0f;

        if((gimbal_percent[Gimbal_Roll] - 50) > 0)
        {
            (*exp_gyr_x) = (pos_trip * gimbal_percent_tmp) + SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_X].idle;
        }
        else if((gimbal_percent[Gimbal_Roll] - 50) < 0)
        {
            (*exp_gyr_x) = (neg_trip * gimbal_percent_tmp) + SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_X].idle;
        }
        else
            (*exp_gyr_x) = SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_X].idle;

        /*************************************************************** gyro y section ******************************************************************/
        pos_trip = SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Y].max - SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Y].idle;
        neg_trip = SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Y].idle - SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Y].min;
        gimbal_percent_tmp = (gimbal_percent[Gimbal_Pitch] - 50) / 50.0f;

        if((gimbal_percent[Gimbal_Pitch] - 50) > 0)
        {
            (*exp_gyr_y) = (pos_trip * gimbal_percent_tmp) + SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Y].idle;
        }
        else if((gimbal_percent[Gimbal_Pitch] - 50) < 0)
        {
            (*exp_gyr_y) = (neg_trip * gimbal_percent_tmp) + SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Y].idle;
        }
        else
            (*exp_gyr_y) = SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Y].idle;

        /*************************************************************** gyro z section ******************************************************************/
        pos_trip = SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Z].max - SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Z].idle;
        neg_trip = SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Z].idle - SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Z].min;
        gimbal_percent_tmp = (gimbal_percent[Gimbal_Yaw] - 50) / 50.0f;

        if((gimbal_percent[Gimbal_Yaw] - 50) > 0)
        {
            (*exp_gyr_z) = (pos_trip * gimbal_percent_tmp) + SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Z].idle;
        }
        else if((gimbal_percent[Gimbal_Yaw] - 50) < 0)
        {
            (*exp_gyr_z) = (neg_trip * gimbal_percent_tmp) + SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Z].idle;
        }
        else
            (*exp_gyr_z) = SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Z].idle;
    }
}

static Srv_CtlExpectionData_TypeDef Srv_CtlDataArbitrate_GetData(void)
{
    Srv_CtlExpectionData_TypeDef tmp;

    memset(&tmp, 0, sizeof(Srv_CtlExpectionData_TypeDef));

    tmp.fail_safe = SrvCtlArbitrateMonitor.InUse_CtlData.fail_safe;
    tmp.arm_state = SrvCtlArbitrateMonitor.InUse_CtlData.arm_state;
    tmp.buzzer_state = SrvCtlArbitrateMonitor.InUse_CtlData.aux.bit.buzzer;
    tmp.calib_state = SrvCtlArbitrateMonitor.InUse_CtlData.aux.bit.calib;
    tmp.mode = SrvCtlArbitrateMonitor.InUse_CtlData.control_mode;
    tmp.TakingOver_stage = SrvCtlArbitrateMonitor.InUse_CtlData.aux.bit.taking_over_req;
    tmp.recover_flip_over = SrvCtlArbitrateMonitor.InUse_CtlData.aux.bit.flip_over;
    
    tmp.throttle_percent = SrvCtlArbitrateMonitor.InUse_CtlData.gimbal_percent[Gimbal_Throttle];
    tmp.exp_attitude[Att_Pitch] = SrvCtlArbitrateMonitor.InUse_CtlData.exp_att_pitch;
    tmp.exp_attitude[Att_Roll] = SrvCtlArbitrateMonitor.InUse_CtlData.exp_att_roll;
    
    tmp.exp_angularspeed[Axis_X] = SrvCtlArbitrateMonitor.InUse_CtlData.exp_gyr_x;
    tmp.exp_angularspeed[Axis_Y] = SrvCtlArbitrateMonitor.InUse_CtlData.exp_gyr_y;
    tmp.exp_angularspeed[Axis_Z] = SrvCtlArbitrateMonitor.InUse_CtlData.exp_gyr_z;

    /* for temporary */
    tmp.tunning = false;

    return tmp;
}

static void Srv_OnPlaneComputer_TakingOver_RequireSend(ControlData_TypeDef cur_ctl_data)
{
    /* call on plane computer communicate api */
}

