#ifndef __SRV_ACTUATOR_H
#define __SRV_ACTUATOR_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "Bsp_GPIO.h"
#include "Bsp_Timer.h"
#include "Bsp_DMA.h"
#include "Dev_Dshot.h"
#include "IO_Definition.h"

#define SRVACTUATOR_PB0_SIG_1                                   \
    (SrvActuator_PeriphSet_TypeDef)                             \
    {                                                           \
        TIM3, TIM_CHANNEL_3, Bsp_DMA_1, Bsp_DMA_Stream_0,       \
        {                                                       \
            PWM_SIG_1_PORT, PWM_SIG_1_PIN, false, GPIO_AF2_TIM3 \
        }                                                       \
    }

#define SRVACTUATOR_PB1_SIG_2                                   \
    (SrvActuator_PeriphSet_TypeDef)                             \
    {                                                           \
        TIM3, TIM_CHANNEL_4, Bsp_DMA_1, Bsp_DMA_Stream_1,       \
        {                                                       \
            PWM_SIG_2_PORT, PWM_SIG_2_PIN, false, GPIO_AF2_TIM3 \
        }                                                       \
    }

#define SRVACTUATOR_PA0_SIG_3                                   \
    (SrvActuator_PeriphSet_TypeDef)                             \
    {                                                           \
        TIM5, TIM_CHANNEL_1, Bsp_DMA_1, Bsp_DMA_Stream_2,       \
        {                                                       \
            PWM_SIG_3_PORT, PWM_SIG_3_PIN, false, GPIO_AF2_TIM5 \
        }                                                       \
    }

#define SRVACTUATOR_PA1_SIG_4                                   \
    (SrvActuator_PeriphSet_TypeDef)                             \
    {                                                           \
        TIM5, TIM_CHANNEL_2, Bsp_DMA_1, Bsp_DMA_Stream_3,       \
        {                                                       \
            PWM_SIG_4_PORT, PWM_SIG_4_PIN, false, GPIO_AF2_TIM5 \
        }                                                       \
    }

#define SRVACTUATOR_PA2_SIG_5                                   \
    (SrvActuator_PeriphSet_TypeDef)                             \
    {                                                           \
        TIM5, TIM_CHANNEL_3, Bsp_DMA_None, Bsp_DMA_Stream_None, \
        {                                                       \
            PWM_SIG_5_PORT, PWM_SIG_5_PIN, false, GPIO_AF2_TIM5 \
        }                                                       \
    }

#define SRVACTUATOR_PA3_SIG_6                                   \
    (SrvActuator_PeriphSet_TypeDef)                             \
    {                                                           \
        TIM5, TIM_CHANNEL_4, Bsp_DMA_None, Bsp_DMA_Stream_None, \
        {                                                       \
            PWM_SIG_6_PORT, PWM_SIG_6_PIN, false, GPIO_AF2_TIM5 \
        }                                                       \
    }

#define SRVACTUATOR_PD12_SIG_7                                  \
    (SrvActuator_PeriphSet_TypeDef)                             \
    {                                                           \
        TIM4, TIM_CHANNEL_1, Bsp_DMA_None, Bsp_DMA_Stream_None, \
        {                                                       \
            PWM_SIG_7_PORT, PWM_SIG_7_PIN, false, GPIO_AF2_TIM4 \
        }                                                       \
    }

#define SRVACTUATOR_PD13_SIG_8                                  \
    (SrvActuator_PeriphSet_TypeDef)                             \
    {                                                           \
        TIM4, TIM_CHANNEL_2, Bsp_DMA_None, Bsp_DMA_Stream_None, \
        {                                                       \
            PWM_SIG_8_PORT, PWM_SIG_8_PIN, false, GPIO_AF2_TIM4 \
        }                                                       \
    }

#define SRVACTUATOR_PD14_SIG_9                                  \
    (SrvActuator_PeriphSet_TypeDef)                             \
    {                                                           \
        TIM4, TIM_CHANNEL_3, Bsp_DMA_None, Bsp_DMA_Stream_None, \
        {                                                       \
            PWM_SIG_9_PORT, PWM_SIG_9_PIN, false, GPIO_AF2_TIM4 \
        }                                                       \
    }

#define SRVACTUATOR_PD15_SIG_10                                   \
    (SrvActuator_PeriphSet_TypeDef)                               \
    {                                                             \
        TIM4, TIM_CHANNEL_4, Bsp_DMA_None, Bsp_DMA_Stream_None,   \
        {                                                         \
            PWM_SIG_10_PORT, PWM_SIG_10_PIN, false, GPIO_AF2_TIM4 \
        }                                                         \
    }

#define SRVACTUATOR_PE5_SIG_11                                     \
    (SrvActuator_PeriphSet_TypeDef)                                \
    {                                                              \
        TIM15, TIM_CHANNEL_1, Bsp_DMA_None, Bsp_DMA_Stream_None,   \
        {                                                          \
            PWM_SIG_11_PORT, PWM_SIG_11_PIN, false, GPIO_AF2_TIM15 \
        }                                                          \
    }

#define SRVACTUATOR_PE6_SIG_12                                     \
    (SrvActuator_PeriphSet_TypeDef)                                \
    {                                                              \
        TIM15, TIM_CHANNEL_2, Bsp_DMA_None, Bsp_DMA_Stream_None,   \
        {                                                          \
            PWM_SIG_12_PORT, PWM_SIG_12_PIN, false, GPIO_AF2_TIM15 \
        }                                                          \
    }

#define QUAD_CONTROL_COMPONENT              \
    (SrvActuator_ModelComponentNum_TypeDef) \
    {                                       \
        4, 4, 0                             \
    }
#define HEX_CONTROL_COMPONENT               \
    (SrvActuator_ModelComponentNum_TypeDef) \
    {                                       \
        6, 6, 0                             \
    }
#define OCT_CONTROL_COMPONENT               \
    (SrvActuator_ModelComponentNum_TypeDef) \
    {                                       \
        8, 8, 0                             \
    }
#define X8_CONTROL_COMPONENT                \
    (SrvActuator_ModelComponentNum_TypeDef) \
    {                                       \
        8, 8, 0                             \
    }
#define Y6_CONTROL_CONPONENT                \
    (SrvActuator_ModelComponentNum_TypeDef) \
    {                                       \
        6, 6, 0                             \
    }
#define TRI_CONTROL_COMPONENT               \
    (SrvActuator_ModelComponentNum_TypeDef) \
    {                                       \
        4, 3, 1                             \
    }
#define TDRONE_CONTROL_COMPONENT            \
    (SrvActuator_ModelComponentNum_TypeDef) \
    {                                       \
        5, 2, 3                             \
    }

#define SRV_ACTUATOR_MAX_THROTTLE_PERCENT 80

typedef enum
{
    Model_Quad = 0,
    Model_Hex,
    Model_Oct,
    Model_X8,
    Model_Y6,
    Model_Tri,
    Model_TDrone,
} SrvActuator_Model_List;

typedef enum
{
    Actuator_PWM_Sig1 = 0,
    Actuator_PWM_Sig2,
    Actuator_PWM_Sig3,
    Actuator_PWM_Sig4,
    Actuator_PWM_Sig5,
    Actuator_PWM_Sig6,
    Actuator_PWM_Sig7,
    Actuator_PWM_Sig8,
    Actuator_PWM_Sig9,
    Actuator_PWM_Sig10,
    Actuator_PWM_Sig11,
    Actuator_PWM_Sig12,
    Actuator_PWM_SigSUM,
} SrvActuator_MotoTag_List;

typedef enum
{
    Actuator_MS_CW = 0, // moto clockwise spin
    Actuator_MS_ACW,    // moto anticlockwise spin
    Actuator_SS_CW,     // servo clockwise spin
    Actuator_SS_ACW     // servo anticlockwise spin
} SrvActuator_SpinDir_List;

typedef enum
{
    Actuator_DevType_DShot150 = DevDshot_150,
    Actuator_DevType_DShot300 = DevDshot_300,
    Actuator_DevType_DShot600 = DevDshot_600,
    Actuator_DevType_ServoPWM,
} SrvActuator_DevType_List;

typedef enum
{
    Actuator_CtlChannel_Throttle = 0,
    Actuator_CtlChannel_Roll,
    Actuator_CtlChannel_Pitch,
    Actuator_CtlChannel_Yaw,
} SrvActuator_ChannelControl_List;

typedef struct
{
    void *tim_base;
    uint32_t tim_channel;
    uint32_t dma;
    uint32_t dma_channel;
    BspGPIO_Obj_TypeDef pin;
} SrvActuator_PeriphSet_TypeDef;

typedef struct
{
    uint8_t drv_type;
    uint8_t sig_id;

    uint16_t ctl_val;
    uint16_t max_val;
    uint16_t min_val;
    uint16_t idle_val;
    uint16_t lock_val;

    SrvActuator_PeriphSet_TypeDef *periph_ptr;
    void *drv_obj;
    SrvActuator_SpinDir_List spin_dir;
} SrvActuator_PWMOutObj_TypeDef;

typedef struct
{
    uint8_t total_cnt;
    uint8_t moto_cnt;
    uint8_t servo_cnt;
} SrvActuator_ModelComponentNum_TypeDef;

typedef struct
{
    SrvActuator_ModelComponentNum_TypeDef num;
    SrvActuator_PWMOutObj_TypeDef *obj_list;
} SrcActuatorCTL_Obj_TypeDef;

typedef struct
{
    SrvActuator_Model_List model;
    bool init;
    SrcActuatorCTL_Obj_TypeDef drive_module;
} SrvActuatorObj_TypeDef;

typedef struct
{
    uint32_t time_stamp;

    uint8_t moto_cnt;
    uint8_t servo_cnt;

    uint16_t moto[8];
    uint16_t servo[8];

    uint8_t moto_dir[8];
    uint8_t servo_dir[8];
} SrvActuatorPipeData_TypeDef;

typedef struct
{
    bool (*init)(SrvActuator_Model_List model, uint8_t esc_type);
    bool (*lock)(void);
    void (*moto_control)(uint16_t *p_val);
    void (*servo_conttol)(uint8_t index, uint16_t val);
    bool (*invert_spin)(uint8_t component_index);
    bool (*get_moto_control_range)(uint8_t moto_index, int16_t *min, int16_t *idle, int16_t *max);
    bool (*get_servo_control_range)(uint8_t servo_index, int16_t *min, int16_t *idle, int16_t *max);
    SrvActuator_ModelComponentNum_TypeDef (*get_cnt)(void);
    SrvActuator_Model_List (*get_model)(void);
} SrvActuator_TypeDef;

extern SrvActuator_TypeDef SrvActuator;

#endif