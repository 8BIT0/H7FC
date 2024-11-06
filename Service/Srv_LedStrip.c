#include "Srv_OsCommon.h"
#include "Srv_LedStrip.h"
#include "Srv_DataHub.h"
#include "Dev_WS2812.h"
#include "Bsp_Timer.h"
#include "Bsp_GPIO.h"
#include <math.h>
#include "HW_Def.h"

#define SrvLedStrip_Malloc(size) SrvOsCommon.malloc(size)
#define SrvLedStrip_Free(ptr) SrvOsCommon.free(ptr)

/* internal function */
static bool Srv_LedStrip_PortInit(void *obj);
static bool Srv_LedStrip_Trans(void *port_obj);

typedef struct
{
    bool init;
#if defined AT32F435_437
    osSemaphoreId Sem;
#endif
    uint8_t num;
} SrvLedStrip_Monitor_TypeDef;

/* internal vriable */
static SrvLedStrip_Monitor_TypeDef Monitor = {
#if defined AT32F435_437
    .Sem = NULL,
#endif
    .init = false,
    .num = 0,
};

static DevWS2812Obj_TypeDef WS2812Obj = {
    .bus = WS2812_Bus_Timer,
    .port_Obj = NULL,
};

/* external function */
static bool Srv_LedStrip_Init(uint8_t led_num);
static void Srv_LedStrip_Polling(void);

/* external vriable */
SrvLedStrip_TypeDef SrvLedStrip = {
    .init = Srv_LedStrip_Init,
    .polling = Srv_LedStrip_Polling,
};

static bool Srv_LedStrip_Init(uint8_t led_num)
{
    if (led_num == 0)
        return false;

    Monitor.num = led_num;
    Monitor.init = false;

    WS2812Obj.led_num = led_num;
    WS2812Obj.p_malloc = SrvOsCommon.malloc;
    WS2812Obj.p_free = SrvOsCommon.free;
    WS2812Obj.port_init = Srv_LedStrip_PortInit;
    WS2812Obj.port_send = Srv_LedStrip_Trans;

    if (!DevWS2812.init(&WS2812Obj))
    {
        SrvLedStrip_Free(WS2812Obj.port_Obj);
        return false;
    }

#if defined AT32F435_437
    osSemaphoreDef(LedStrip_Sem);
    Monitor.Sem = osSemaphoreCreate(osSemaphore(LedStrip_Sem), 1);

    if (Monitor.Sem == NULL)
    {
        SrvLedStrip_Free(To_TimerPWMObj_Ptr(WS2812Obj.port_Obj)->dma_callback_obj);
        SrvLedStrip_Free(To_TimerPWMObj_Ptr(WS2812Obj.port_Obj));
        SrvLedStrip_Free(WS2812Obj.port_Obj);
        return false;
    }
#endif
    
    WS2812Obj.port_init = Srv_LedStrip_PortInit;
    WS2812Obj.port_send = Srv_LedStrip_Trans;
    
    for (uint8_t i = 0; i < led_num; i++)
        DevWS2812.set(&WS2812Obj, i, WS2812_NONE);

    Monitor.init = true;
    return true;
}

static void Srv_LedStrip_Breath_Test(void)
{
    uint8_t bright = 0;
    static uint32_t sys_time = 0;
    static RGB_TypeDef rgb;
    const uint8_t interval = 100;   /* 100ms */
    static bool dec = false;
    static uint8_t i = 0;

    if (!Monitor.init || \
        (DevWS2812.set == NULL) || \
        (DevWS2812.trans == NULL))
        return;

    DevWS2812.set(&WS2812Obj, 0, WS2812_NONE);
    DevWS2812.set(&WS2812Obj, 1, WS2812_NONE);
    DevWS2812.set(&WS2812Obj, 2, WS2812_NONE);
    DevWS2812.set(&WS2812Obj, 3, WS2812_NONE);
    DevWS2812.set(&WS2812Obj, 4, WS2812_NONE);
    DevWS2812.set(&WS2812Obj, 5, WS2812_NONE);
    
    if (sys_time == 0)
    {
        rgb = WS2812_GHOSTWHITE;
        sys_time = SrvOsCommon.get_os_ms();
        return;
    }

    if ((SrvOsCommon.get_os_ms() - sys_time) < interval)
        return;

    sys_time = SrvOsCommon.get_os_ms();
    rgb.bright ++;
    
    DevWS2812.set(&WS2812Obj, i, rgb);

    i ++;
    if (i >= Monitor.num)
        i = 0;
    
    DevWS2812.trans(&WS2812Obj);
}

static void Srv_LedStrip_Polling(void)
{
    if (!Monitor.init || \
        (DevWS2812.set == NULL) || \
        (DevWS2812.trans == NULL))
        return;

    // for (uint8_t i = 0; i < Monitor.num; i ++)
    // {
    //     DevWS2812.set(&WS2812Obj, 0, WS2812_SNOWWHITE);
    //     DevWS2812.set(&WS2812Obj, 1, WS2812_NONE);
    //     DevWS2812.set(&WS2812Obj, 2, WS2812_SNOWWHITE);
    //     DevWS2812.set(&WS2812Obj, 3, WS2812_NONE);
    //     DevWS2812.set(&WS2812Obj, 4, WS2812_NONE);
    //     DevWS2812.set(&WS2812Obj, 5, WS2812_SNOWWHITE);
    // }
    
    // DevWS2812.trans(&WS2812Obj);
    Srv_LedStrip_Breath_Test();
}

/********************************** Timer Port Init ******************************** */
#if defined AT32F435_437
static void Srv_LedStrip_TransFin(void)
{
    if (Monitor.Sem)
        osSemaphoreRelease(Monitor.Sem);
}
#endif

static bool Srv_LedStrip_Trans(void *port_obj)
{
    if (port_obj == NULL)
        return false;

    BspTimer_PWM.dma_trans(To_TimerPWMObj_Ptr(port_obj));
#if defined AT32F435_437
    if (Monitor.Sem)
        osSemaphoreWait(Monitor.Sem, 1);
#endif
    return true;
}

static bool Srv_LedStrip_PortInit(void *obj)
{
    BspGPIO_Obj_TypeDef strip_pin;

    memset(&strip_pin, 0, sizeof(BspGPIO_Obj_TypeDef));
    if (To_WS2812Obj_Ptr(obj)->bus == WS2812_Bus_Timer)
    {
        uint32_t perscaler = 0;
        BspTimerPWMObj_TypeDef *pwm_obj_tmp = NULL;
        volatile uint32_t tmr_clock = 0;

        To_WS2812Obj_Ptr(obj)->port_Obj = SrvLedStrip_Malloc(TimerPWMObj_Size);
        if (To_WS2812Obj_Ptr(obj)->port_Obj == NULL)
        {
            SrvLedStrip_Free(To_WS2812Obj_Ptr(obj)->port_Obj);
            return false;
        }

        pwm_obj_tmp = To_TimerPWMObj_Ptr(To_WS2812Obj_Ptr(obj)->port_Obj);
#if defined AT32F435_437
        pwm_obj_tmp->dma_callback_obj = SrvLedStrip_Malloc(sizeof(BspDMA_IrqCall_Obj_TypeDef));
        if (pwm_obj_tmp->dma_callback_obj == NULL)
        {
            SrvLedStrip_Free(pwm_obj_tmp->dma_callback_obj);
            SrvLedStrip_Free(pwm_obj_tmp);
            return false;
        }
        pwm_obj_tmp->send_callback = Srv_LedStrip_TransFin;
#endif

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

        perscaler = lrintf((tmr_clock - 1) / (WS2812_CLOCK - 1));
        if (!BspTimer_PWM.init(pwm_obj_tmp, LED_STRIP_TIM, LED_STRIP_TIM_CHANNEL,\
                               WS2812_PERIOD, perscaler, strip_pin,\
                               LED_STRIP_DMA, LED_STRIP_DMA_CHANNEL, \
                               (uint32_t)To_WS2812Obj_Ptr(obj)->buff, WS2812_DATA_SIZE * To_WS2812Obj_Ptr(obj)->led_num))
            return false;

        BspTimer_PWM.set_dma_pwm(pwm_obj_tmp);
        return true;
    }

    return false;
}

