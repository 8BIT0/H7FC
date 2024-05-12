#ifndef __DEV_DSHOT_H
#define __DEV_DSHOT_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "Bsp_GPIO.h"
#include "Bsp_Timer.h"

#define To_DShot_Obj(x) ((DevDshotObj_TypeDef *)x)

#define MHZ_TO_HZ(x) (x * 1000000)

// #define DSHOT_TIMER_CLK_HZ MHZ_TO_HZ(160)
#define DSHOT600_CLK_HZ MHZ_TO_HZ(12)
#define DSHOT300_CLK_HZ MHZ_TO_HZ(6)
#define DSHOT150_CLK_HZ MHZ_TO_HZ(3)

#define MOTOR_BIT_0 7
#define MOTOR_BIT_1 14
#define MOTOR_BITLENGTH 20

#define DSHOT_FRAME_SIZE 16
#define DSHOT_DMA_BUFFER_SIZE 18 /* resolution + frame reset (2us) */

#define DSHOT_LOCK_THROTTLE 0
#define DSHOT_MIN_THROTTLE 48
#define DSHOT_IDLE_THROTTLE 100
#define DSHOT_MAX_THROTTLE 1900
#define DSHOT_RANGE (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)

#define DSHOT_CMD_SET_SPIN_CLOCKWISE ((1 << DevDshot_RotateDir_ClockWise) | (0 << DevDshot_RotateDir_AntiClockWise))
#define DSHOT_CMD_SET_SPIN_ANTICLOCKWISE ((0 << DevDshot_RotateDir_ClockWise) | (1 << DevDshot_RotateDir_AntiClockWise))
#define DSHOT_CMD_SAVE_SETTING (1 << DevDshot_Save_Setting)

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
} DevDshot_Command_List;

typedef enum
{
    DevDshot_150 = 1,
    DevDshot_300,
    DevDshot_600,
} DevDshotType_List;

typedef struct
{
    DevDshotType_List type;
    BspTimerPWMObj_TypeDef pwm_obj;

    uint32_t ctl_buf[DSHOT_DMA_BUFFER_SIZE];
} DevDshotObj_TypeDef;

typedef struct
{
    bool (*init)(DevDshotObj_TypeDef *obj, void *timer_ins, uint32_t ch, void *pin, uint8_t dma, uint8_t stream);
    bool (*de_init)(DevDshotObj_TypeDef *obj);
    void (*command)(DevDshotObj_TypeDef *obj, uint16_t cmd);
    void (*control)(DevDshotObj_TypeDef *obj, uint16_t val);
} DevDshot_TypeDef;

extern DevDshot_TypeDef DevDshot;

#endif
