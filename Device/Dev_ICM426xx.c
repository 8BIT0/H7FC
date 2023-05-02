#include "Dev_ICM426xx.h"


static bool DevICM426xx_Detect(bus_trans_callback trans, cs_ctl_callback cs_ctl)
{
    if((trans == NULL) || (cs_ctl == NULL))
        return false;

    /* cs low */
    cs_ctl(false);

    /* cs high */
    cs_ctl(true);

    return false;
}
