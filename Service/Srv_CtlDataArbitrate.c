/*
 * Auther: 8_B!T0
 * Bref: in this project we may have multiple control signal source like RC_Remote MAVLink_Attitude_Expection MAVLinkl_AngularSpeed_Expection
 *       but doing acturator control we must to make the control signal unique
 *       in this file we can make multiple control signal in but out with the only one  
 */
#include <math.h>
#include "Srv_CtlDataArbitrate.h"

#define ATTITUDE_ACCURACY 100
#define MAX_ATTITUDE_ANGLE_RANGE 60
#define MIN_ATTITUDE_ANGLE_RANGE -60

/* internal vriable */
static Srv_CtlArbitrateMonitor_TypeDef SrvCtlArbitrateMonitor;

/* internal function */
static void Srv_CtlData_ConvertGimbal_ToAtt(uint16_t *gimbal_percent, float *exp_pitch, float *exp_roll);
static void Srv_CtlData_ConvertGimbal_ToAngularSpeed(uint16_t *gimbal_percent, float *exp_gyr_x, float *exp_gyr_y, float *exp_gyr_z);

/* external function */
static bool Srv_CtlDataArbitrate_Init(Srv_CtlRange_TypeDef att_range[Att_Ctl_Sum], Srv_CtlRange_TypeDef angularspeed_range[Axis_Sum]);
static void Srv_CtlDataArbitrate_Update(void);
static Srv_CtlExpectionData_TypeDef Srv_CtlDataArbitrate_Get_Data(void);

/* external vriable */
Srv_CtlDataArbitrate_TypeDef Srv_CtlDataArbitrate = {
    .init = Srv_CtlDataArbitrate_Init,
    .negociate_update = Srv_CtlDataArbitrate_Update,
    .get_data = Srv_CtlDataArbitrate_Get_Data,
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

    for(index = 0; index < sizeof(att_range) / sizeof(att_range[0]); index ++)
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

    for(index = 0; index < sizeof(angularspeed_range) / sizeof(angularspeed_range[0]); index ++)
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

    memcpy(SrvCtlArbitrateMonitor.att_ctl_range, att_range, sizeof(att_range));
    memcpy(SrvCtlArbitrateMonitor.att_ctl_range, angularspeed_range, sizeof(angularspeed_range));

    /* set default signal source/type/control mode */
    SrvCtlArbitrateMonitor.cur_sig_source = Control_Sig_RC;
    SrvCtlArbitrateMonitor.cur_sig_type = Control_Channel_Sig;
    SrvCtlArbitrateMonitor.cur_ctl_mode = Control_Mode_Attitude;
    SrvCtlArbitrateMonitor.sig_privilege_req_source = SrvCtlArbitrateMonitor.cur_sig_source;

    return true;
}

static void Srv_CtlDataArbitrate_Update(void)
{
    uint16_t rc_ch[32] = {0};
    uint16_t rc_gimbal_percent[Srv_Gimbal_TagSum] = {0};
    uint32_t rc_time_stamp = 0;
    uint8_t rc_channel_sum = 0;
    bool arm_state = DRONE_ARM;

    SrvDataHub.get_arm_state(&arm_state);

    /* convert gimbal value to physical expection */
    SrvDataHub.get_gimbal_percent(rc_gimbal_percent);

    if(arm_state == DRONE_ARM)
    {
        /* when drone is armed we can switch control signal easily */
        /* if on plane conputer reqire to control */
    }
    else
    {
        /* when drone is disarm need to check toggle pos and input signal arm state first */
        /* then sync trottle */
        switch((uint8_t)SrvCtlArbitrateMonitor.cur_sig_source)
        {
            case Control_Sig_RC:
                SrvCtlArbitrateMonitor.cur_sig_type = Control_Channel_Sig;

                break;
            
            case Control_Sig_OnPlaneComputer:
                switch((uint8_t)SrvCtlArbitrateMonitor.cur_sig_type)
                {
                    default:
                        break;
                }
                break;

            default:
                break;
        }

        /* another control signal required control privilege doing arbitrate signal source */
        if(SrvCtlArbitrateMonitor.sig_privilege_req_source != SrvCtlArbitrateMonitor.cur_sig_source)
        {
            
            
        }
    }
}

/* remote gimbal only can control pitch and roll angle but yaw angle */
static void Srv_CtlData_ConvertGimbal_ToAtt(uint16_t *gimbal_percent, float *exp_pitch, float *exp_roll)
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
        neg_trip = SrvCtlArbitrateMonitor.att_ctl_range[Att_Pitch].min - SrvCtlArbitrateMonitor.att_ctl_range[Att_Pitch].idle;
        gimbal_percent_tmp = (gimbal_percent[Srv_RC_Pitch] - 50) / 100.0f;

        if((gimbal_percent[Srv_RC_Pitch] - 50) > 0)
        {
            (*exp_pitch) = (pos_trip * gimbal_percent_tmp) + SrvCtlArbitrateMonitor.att_ctl_range[Att_Pitch].idle;
            diff_to_idle = (int16_t)((pos_trip * gimbal_percent_tmp) * ATTITUDE_ACCURACY);
        }
        else if((gimbal_percent[Srv_RC_Pitch] - 50) < 0)
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
        neg_trip = SrvCtlArbitrateMonitor.att_ctl_range[Att_Roll].min - SrvCtlArbitrateMonitor.att_ctl_range[Att_Roll].idle;
        gimbal_percent_tmp = (gimbal_percent[Srv_RC_Roll] - 50) / 100.0f;
 
        if((gimbal_percent[Srv_RC_Roll] - 50) > 0)
        {
            (*exp_roll) = (pos_trip * gimbal_percent_tmp) + SrvCtlArbitrateMonitor.att_ctl_range[Att_Roll].idle;
            diff_to_idle = (int16_t)((pos_trip * gimbal_percent_tmp) * ATTITUDE_ACCURACY);
        }
        else if((gimbal_percent[Srv_RC_Roll] - 50) < 0)
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

static void Srv_CtlData_ConvertGimbal_ToAngularSpeed(uint16_t *gimbal_percent, float *exp_gyr_x, float *exp_gyr_y, float *exp_gyr_z)
{
    float pos_trip = 0.0f;
    float neg_trip = 0.0f;
    float gimbal_percent_tmp = 0.0f;

    /* noticed dead zone is useless in angular speed control */
    if(gimbal_percent && exp_gyr_x && exp_gyr_y && exp_gyr_z)
    {
        /*************************************************************** gyro x section ******************************************************************/
        pos_trip = SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_X].max - SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_X].idle;
        neg_trip = SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_X].min - SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_X].idle;
        gimbal_percent_tmp = (gimbal_percent[Srv_RC_Roll] - 50) / 100.0f;

        if((gimbal_percent[Srv_RC_Roll] - 50) > 0)
        {
            (*exp_gyr_x) = (pos_trip * gimbal_percent_tmp) + SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_X].idle;
        }
        else if((gimbal_percent[Srv_RC_Roll] - 50) < 0)
        {
            (*exp_gyr_x) = (neg_trip * gimbal_percent_tmp) + SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_X].idle;
        }
        else
            (*exp_gyr_x) = SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_X].idle;

        /*************************************************************** gyro y section ******************************************************************/
        pos_trip = SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Y].max - SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Y].idle;
        neg_trip = SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Y].min - SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Y].idle;
        gimbal_percent_tmp = (gimbal_percent[Srv_RC_Pitch] - 50) / 100.0f;

        if((gimbal_percent[Srv_RC_Pitch] - 50) > 0)
        {
            (*exp_gyr_y) = (pos_trip * gimbal_percent_tmp) + SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Y].idle;
        }
        else if((gimbal_percent[Srv_RC_Pitch] - 50) < 0)
        {
            (*exp_gyr_y) = (neg_trip * gimbal_percent_tmp) + SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Y].idle;
        }
        else
            (*exp_gyr_y) = SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Y].idle;

        /*************************************************************** gyro z section ******************************************************************/
        pos_trip = SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Z].max - SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Z].idle;
        neg_trip = SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Z].min - SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Z].idle;
        gimbal_percent_tmp = (gimbal_percent[Srv_RC_Yaw] - 50) / 100.0f;

        if((gimbal_percent[Srv_RC_Yaw] - 50) > 0)
        {
            (*exp_gyr_z) = (pos_trip * gimbal_percent_tmp) + SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Z].idle;
        }
        else if((gimbal_percent[Srv_RC_Yaw] - 50) < 0)
        {
            (*exp_gyr_z) = (neg_trip * gimbal_percent_tmp) + SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Z].idle;
        }
        else
            (*exp_gyr_z) = SrvCtlArbitrateMonitor.angularspeed_ctl_range[Axis_Z].idle;
    }
}

static Srv_CtlExpectionData_TypeDef Srv_CtlDataArbitrate_Get_Data(void)
{
    Srv_CtlExpectionData_TypeDef data_tmp;

    memset(&data_tmp, 0, sizeof(Srv_CtlExpectionData_TypeDef)); 

    return data_tmp;
}
