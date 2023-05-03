#include "Dev_ICM426xx.h"


static bool DevICM426xx_Detect(bus_trans_callback trans, cs_ctl_callback cs_ctl)
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
        return false;

    if( (read_buff[1] == ICM42605_DEV_ID) || 
        (read_buff[1] == ICM42688P_DEV_ID))
        return true;

    return false;
}
