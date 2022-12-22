#ifndef __DEV_DSHOT_H
#define __DEV_DSHOT_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

typedef enum
{
    DevDshot_MotoOutput_Lock = 0,
    DevDshot_Beeper_Level_1 = 1,
    DevDshot_Beeper_Level_2,
    DevDshot_Beeper_Level_3,
    DevDshot_Beeper_Level_4,
    DevDshot_Beeper_Level_5,
    DevDshot_Info_Req,
    DevDshot_RotateDir_ClockWise,
    DevDshot_RotateDir_AntiClockWise,
    DevDshot_Disable_3D_Mode,
    DevDshot_Enable_3D_Mode,
    DevDshot_Setting_Req,
    DevDshot_Save_Setting,
}DevDshot_Command_List; 

typedef enum
{
    DevDshot_300 = 1,
    DevDshot_600,
}DevDshotType_List;

typedef struct
{
    bool (*init)();
    bool (*control)();
}DevDshot_TypeDef;

#endif
