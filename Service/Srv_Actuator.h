#ifndef __SRV_ACTUATOR_H
#define __SRV_ACTUATOR_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "Bsp_GPIO.h"
#include "Bsp_Timer.h"
#include "Bsp_DMA.h"
#include "Dev_Dshot.h"

#define SRVACTUATOR_PB0_SIG_1                             \
    (SrvActuator_PeriphSet_TypeDef)                       \
    {                                                     \
        TIM3, TIM_CHANNEL_3, Bsp_DMA_1, Bsp_DMA_Stream_0, \
        {                                                 \
            GPIOB, GPIO_PIN_0, GPIO_AF2_TIM3              \
        }                                                 \
    }

#define SRVACTUATOR_PB1_SIG_2                             \
    (SrvActuator_PeriphSet_TypeDef)                       \
    {                                                     \
        TIM3, TIM_CHANNEL_4, Bsp_DMA_1, Bsp_DMA_Stream_1, \
        {                                                 \
            GPIOB, GPIO_PIN_1, GPIO_AF2_TIM3              \
        }                                                 \
    }

#define SRVACTUATOR_PA0_SIG_3                             \
    (SrvActuator_PeriphSet_TypeDef)                       \
    {                                                     \
        TIM5, TIM_CHANNEL_1, Bsp_DMA_1, Bsp_DMA_Stream_2, \
        {                                                 \
            GPIOA, GPIO_PIN_0, GPIO_AF2_TIM5              \
        }                                                 \
    }

#define SRVACTUATOR_PA1_SIG_4                             \
    (SrvActuator_PeriphSet_TypeDef)                       \
    {                                                     \
        TIM5, TIM_CHANNEL_2, Bsp_DMA_1, Bsp_DMA_Stream_3, \
        {                                                 \
            GPIOA, GPIO_PIN_1, GPIO_AF2_TIM5              \
        }                                                 \
    }

#define SRVACTUATOR_PA2_SIG_5                                   \
    (SrvActuator_PeriphSet_TypeDef)                             \
    {                                                           \
        TIM5, TIM_CHANNEL_3, Bsp_DMA_None, Bsp_DMA_Stream_None, \
        {                                                       \
            GPIOA, GPIO_PIN_2, GPIO_AF2_TIM5                    \
        }                                                       \
    }

#define SRVACTUATOR_PA3_SIG_6                                   \
    (SrvActuator_PeriphSet_TypeDef)                             \
    {                                                           \
        TIM5, TIM_CHANNEL_4, Bsp_DMA_None, Bsp_DMA_Stream_None, \
        {                                                       \
            GPIOA, GPIO_PIN_3, GPIO_AF2_TIM5                    \
        }                                                       \
    }

#define SRVACTUATOR_PD12_SIG_7                                  \
    (SrvActuator_PeriphSet_TypeDef)                             \
    {                                                           \
        TIM4, TIM_CHANNEL_1, Bsp_DMA_None, Bsp_DMA_Stream_None, \
        {                                                       \
            GPIOD, GPIO_PIN_12, GPIO_AF2_TIM4                   \
        }                                                       \
    }

#define SRVACTUATOR_PD13_SIG_8                                  \
    (SrvActuator_PeriphSet_TypeDef)                             \
    {                                                           \
        TIM4, TIM_CHANNEL_2, Bsp_DMA_None, Bsp_DMA_Stream_None, \
        {                                                       \
            GPIOD, GPIO_PIN_13, GPIO_AF2_TIM4                   \
        }                                                       \
    }

#define SRVACTUATOR_PD14_SIG_9                                  \
    (SrvActuator_PeriphSet_TypeDef)                             \
    {                                                           \
        TIM4, TIM_CHANNEL_3, Bsp_DMA_None, Bsp_DMA_Stream_None, \
        {                                                       \
            GPIOD, GPIO_PIN_14, GPIO_AF2_TIM4                   \
        }                                                       \
    }

#define SRVACTUATOR_PD15_SIG_10                                 \
    (SrvActuator_PeriphSet_TypeDef)                             \
    {                                                           \
        TIM4, TIM_CHANNEL_4, Bsp_DMA_None, Bsp_DMA_Stream_None, \
        {                                                       \
            GPIOD, GPIO_PIN_15, GPIO_AF2_TIM4                   \
        }                                                       \
    }

#define SRVACTUATOR_PE5_SIG_11                                   \
    (SrvActuator_PeriphSet_TypeDef)                              \
    {                                                            \
        TIM15, TIM_CHANNEL_1, Bsp_DMA_None, Bsp_DMA_Stream_None, \
        {                                                        \
            GPIOE, GPIO_PIN_5, GPIO_AF2_TIM15                    \
        }                                                        \
    }

#define SRVACTUATOR_PE6_SIG_12                                   \
    (SrvActuator_PeriphSet_TypeDef)                              \
    {                                                            \
        TIM15, TIM_CHANNEL_2, Bsp_DMA_None, Bsp_DMA_Stream_None, \
        {                                                        \
            GPIOE, GPIO_PIN_6, GPIO_AF2_TIM15                    \
        }                                                        \
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

#pragma pack(1)
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
#pragma pak()

typedef struct
{
    bool (*init)(SrvActuator_Model_List model, uint8_t esc_type);
    bool (*lock)(void);
    void (*control)(uint16_t *p_val, uint8_t len);
    bool (*invert_spin)(uint8_t component_index);
} SrvActuator_TypeDef;

extern SrvActuator_TypeDef SrvActuator;

#endif