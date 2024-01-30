#include "Bsp_Timer.h"
#include "at32f435_437_tmr.h"

/* external function */
static bool BspTimer_PWM_Init(BspTimerPWMObj_TypeDef *obj,
                              void *instance,
                              uint32_t ch,
                              BspGPIO_Obj_TypeDef pin,
                              uint8_t dma,
                              uint8_t stream,
                              uint32_t buf_aadr,
                              uint32_t buf_size);
static void BspTimer_SetPreScale(BspTimerPWMObj_TypeDef *obj, uint32_t prescale);
static void BspTimer_SetAutoReload(BspTimerPWMObj_TypeDef *obj, uint32_t auto_reload);
static void BspTimer_PWM_Start(BspTimerPWMObj_TypeDef *obj);
static void BspTimer_DMA_Start(BspTimerPWMObj_TypeDef *obj);

BspTimerPWM_TypeDef BspTimer_PWM = {
    .init = BspTimer_PWM_Init,
    .set_prescaler = BspTimer_SetPreScale,
    .set_autoreload = BspTimer_SetAutoReload,
    .start_pwm = BspTimer_PWM_Start,
    .dma_trans = BspTimer_DMA_Start,
};

static bool BspTimer_PWM_Init(BspTimerPWMObj_TypeDef *obj,
                              void *instance,
                              uint32_t ch,
                              BspGPIO_Obj_TypeDef pin,
                              uint8_t dma,
                              uint8_t stream,
                              uint32_t buf_aadr,
                              uint32_t buf_size)
{
    return false;
}

static void BspTimer_SetPreScale(BspTimerPWMObj_TypeDef *obj, uint32_t prescale)
{

}

static void BspTimer_SetAutoReload(BspTimerPWMObj_TypeDef *obj, uint32_t auto_reload)
{

}

static void BspTimer_PWM_Start(BspTimerPWMObj_TypeDef *obj)
{
    
}

static void BspTimer_DMA_Start(BspTimerPWMObj_TypeDef *obj)
{
    
}

