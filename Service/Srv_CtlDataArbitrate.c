#include "Srv_CtlDataArbitrate.h"

#define MAX_ATTITUDE_ANGLE_RANGE 50
#define MIN_ATTITUDE_ANGLE_RANGE -50

/* internal vriable */
static Srv_CtlArbitrateMonitor_TypeDef SrvCtlArbitrateMonitor;

/* external function */
static bool Srv_CtlDataArbitrate_Init(Srv_CtlRange_TypeDef att_range[2], Srv_CtlRange_TypeDef angularspeed_range[3]);
static void Srv_CtlDataArbitrate_Update(void);

/* external vriable */
Srv_CtlDataArbitrate_TypeDef Srv_CtlDataArbitrate = {
    .init = Srv_CtlDataArbitrate_Init,
    .negociate_update = Srv_CtlDataArbitrate_Update,
};

static bool Srv_CtlDataArbitrate_Init(Srv_CtlRange_TypeDef att_range[2], Srv_CtlRange_TypeDef angularspeed_range[3])
{
    uint8_t index = 0;
    uint8_t ref_acc_range = 0;
    uint16_t ref_gyr_range = 0;

    memset(&SrvCtlArbitrateMonitor, 0, sizeof(SrvCtlArbitrateMonitor));

    for(index = 0; index < sizeof(att_range) / sizeof(att_range[0]); index ++)
    {
        /* max or min is 0 */
        if((((int16_t)(att_range[index].max * 1000)) / 1000 == 0) || 
           (((int16_t)(att_range[index].min * 1000)) / 1000 == 0))
            return false;
        
        /* max lower than min */
        if((((int16_t)(att_range[index].max * 1000)) / 1000) <=
           (((int16_t)(att_range[index].min * 1000)) / 1000))
            return false;

        /* bigger than define max */
        if((((int16_t)(att_range[index].max * 1000)) / 1000) > MAX_ATTITUDE_ANGLE_RANGE)
            return false;

        /* lower than define min */
        if((((int16_t)(att_range[index].min * 1000)) / 1000) > MIN_ATTITUDE_ANGLE_RANGE)
            return false;
    }

    /* get imu angular speed range */
    if(!SrvDataHub.get_imu_range(&ref_acc_range, &ref_gyr_range) && ((ref_acc_range == 0) || (ref_gyr_range == 0)))
        return false;

    for(index = 0; index < sizeof(angularspeed_range) / sizeof(angularspeed_range[0]); index ++)
    {
        if((((int16_t)(angularspeed_range[index].max * 1000)) / 1000 == 0) || 
           (((int16_t)(angularspeed_range[index].min * 1000)) / 1000 == 0))
           return false;
        
        if((((int16_t)(angularspeed_range[index].max * 1000)) / 1000) <= 
           (((int16_t)(angularspeed_range[index].min * 1000)) / 1000))
           return false;
    }

    memcpy(SrvCtlArbitrateMonitor.att_ctl_range, att_range, sizeof(att_range));
    memcpy(SrvCtlArbitrateMonitor.att_ctl_range, angularspeed_range, sizeof(angularspeed_range));

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
    SrvDataHub.get_rc(&rc_time_stamp, rc_ch, &rc_channel_sum);
    SrvDataHub.get_gimbal_percent(rc_gimbal_percent);


}
