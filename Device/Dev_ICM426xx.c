#include "Dev_ICM426xx.h"

/* external function */
static ICM426xx_Sensor_TypeList DevICM426xx_Detect(bus_trans_callback trans, cs_ctl_callback cs_ctl);
static bool DevICM426xx_Config(DevICM426xxObj_TypeDef *Obj, 
                               ICM426xx_SampleRate_List rate, 
                               ICM426xx_GyrTrip_List GyrTrip, 
                               ICM426xx_AccTrip_List AccTrip);
static void DevICM426xx_PreInit(DevICM426xxObj_TypeDef *sensor_obj,
                               cs_ctl_callback cs_ctl,
                               bus_trans_callback bus_trans,
                               delay_callback delay,
                               get_time_stamp_callback get_time_stamp);

/* external variable */
DevICM426xx_TypeDef DevICM426xx = {
    .detect = DevICM426xx_Detect,
    .pre_init = DevICM426xx_PreInit,
};

static ICM426xx_Sensor_TypeList DevICM426xx_Detect(bus_trans_callback trans, cs_ctl_callback cs_ctl)
{
    uint8_t read_tmp = 0;
    uint8_t write_buff[2] = {0};
    uint8_t read_buff[2] = {0};
    bool state = false;

    if (cs_ctl == NULL || trans == NULL)
        return false;

    write_buff[0] = ICM426XX_WHO_AM_I | ICM426XX_READ_MASK;

    /* CS Low */
    cs_ctl(false);

    state = trans(write_buff, read_buff, 2);

    /* CS High */
    cs_ctl(true);

    if(!state)
        return ICM_NONE;

    switch(read_buff[1])
    {
        case ICM42605_DEV_ID: return ICM42605;
        case ICM42688P_DEV_ID: return ICM42688P;
        default: return ICM_NONE;
    }

    return ICM_NONE;
}

static bool DevICM426xx_Config(DevICM426xxObj_TypeDef *Obj, 
                               ICM426xx_SampleRate_List rate, 
                               ICM426xx_GyrTrip_List GyrTrip, 
                               ICM426xx_AccTrip_List AccTrip)
{

}

static void DevICM426xx_PreInit(DevICM426xxObj_TypeDef *sensor_obj,
                               cs_ctl_callback cs_ctl,
                               bus_trans_callback bus_trans,
                               delay_callback delay,
                               get_time_stamp_callback get_time_stamp)
{
    sensor_obj->cs_ctl = cs_ctl;
    sensor_obj->bus_trans = bus_trans;
    sensor_obj->delay = delay;
    sensor_obj->get_timestamp = get_time_stamp;
}
