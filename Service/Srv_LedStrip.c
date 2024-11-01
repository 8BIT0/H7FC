#include "Srv_OsCommon.h"
#include "Srv_LedStrip.h"
#include "Dev_WS2812.h"
#include "Bsp_Timer.h"
#include "Bsp_GPIO.h"
#include <math.h>
#include "HW_Def.h"

#define SrvLedStrip_Malloc(size) SrvOsCommon.malloc(size)
#define SrvLedStrip_Free(ptr) SrvOsCommon.free(ptr)

#if defined AT32F435_437
osSemaphoreId SrvLedStrip_Sem = NULL;
#endif

static DevWS2812Obj_TypeDef WS2812Obj = {
    .bus = WS2812_Bus_Timer,
    .port_Obj = NULL,
    .num = 6,
};

static bool Srv_LedStrip_Init(void)
{
    WS2812Obj.p_malloc = SrvOsCommon.malloc;
    WS2812Obj.p_free = SrvOsCommon.free;

    if (!DevWS2812.init(&WS2812Obj))
    {
        SrvLedStrip_Free(WS2812Obj.port_Obj);
        return false;
    }

#if defined AT32F435_437
    osSemaphoreDef(LedStrip_Sem);
    SrvLedStrip_Sem = osSemaphoreCreate(osSemaphore(LedStrip_Sem), 1);
#endif
}

static void Srv_LedStrip_Polling(void)
{

}

#if defined AT32F435_437
static void Srv_LedStrip_TransFin(void)
{
    if (SrvLedStrip_Sem)
        osSemaphoreRelease(SrvLedStrip_Sem);
}
#endif

/********************************** Timer Port Init ******************************** */
static bool Srv_LedStrip_PortInit(void *obj)
{
    BspGPIO_Obj_TypeDef strip_pin;

    memset(&strip_pin, 0, sizeof(BspGPIO_Obj_TypeDef));
    if (To_WS2812Obj_Ptr(obj)->bus == WS2812_Bus_Timer)
    {
        uint32_t perscaler = 0;
        uint32_t auto_reload = 0;
        BspTimerPWMObj_TypeDef *pwm_obj_tmp = NULL;
        volatile uint32_t tmr_clock = 0;

        To_WS2812Obj_Ptr(obj)->port_Obj = SrvLedStrip_Malloc(TimerPWMObj_Size);
        if (To_WS2812Obj_Ptr(obj)->port_Obj == NULL)
        {
            SrvLedStrip_Free(To_WS2812Obj_Ptr(obj)->port_Obj);
            return false;
        }

        pwm_obj_tmp = To_TimerPWMObj_Ptr(To_WS2812Obj_Ptr(obj)->port_Obj);
        pwm_obj_tmp->dma_callback_obj = SrvLedStrip_Malloc(sizeof(BspDMA_IrqCall_Obj_TypeDef));
        if (pwm_obj_tmp->dma_callback_obj == NULL)
        {
            SrvLedStrip_Free(pwm_obj_tmp->dma_callback_obj);
            SrvLedStrip_Free(pwm_obj_tmp);
            return false;
        }
        
        tmr_clock = BspTimer_PWM.get_clock_freq(pwm_obj_tmp);
        if (tmr_clock < WS2812_CLOCK)
        {
            SrvLedStrip_Free(pwm_obj_tmp->dma_callback_obj);
            SrvLedStrip_Free(pwm_obj_tmp);
            return false;
        }
        
        strip_pin.alternate = LED_STRIP_PIN_AF;
        strip_pin.pin = LED_STRIP_PIN;
        strip_pin.port = LED_STRIP_PORT;
        strip_pin.init_state = 0;

        perscaler = lrintf(tmr_clock / WS2812_CLOCK);
        if (!BspTimer_PWM.init(pwm_obj_tmp, LED_STRIP_TIM, LED_STRIP_TIM_CHANNEL,\
                               WS2812_PERIOD, perscaler, strip_pin,\
                               LED_STRIP_DMA, LED_STRIP_DMA_CHANNEL, \
                               (uint32_t)To_WS2812Obj_Ptr(obj)->ctl_data, WS2812_DATA_SIZE))
            return false;

        BspTimer_PWM.set_dma_pwm(pwm_obj_tmp);
        return true;
    }

    return false;
}
