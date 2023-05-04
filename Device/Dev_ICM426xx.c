#include "Dev_ICM426xx.h"

/* external function */
static ICM426xx_Sensor_TypeList DevICM426xx_Detect(bus_trans_callback trans, cs_ctl_callback cs_ctl);

/* external variable */
DevICM426xx_TypeDef DevICM426xx = {
    .detect = DevICM426xx_Detect,
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
