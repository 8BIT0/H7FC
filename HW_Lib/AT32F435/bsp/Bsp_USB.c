#include "Bsp_USB.h"
#include "Bsp_GPIO.h"
#include "at32f435_437_usb.h"
#include "usb_conf.h"
#include "usb_core.h"
#include "usbd_int.h"
#include "cdc_class.h"
#include "cdc_desc.h"

#define OTG_PIN_GPIO                     GPIOA
#define OTG_PIN_GPIO_CLOCK               CRM_GPIOA_PERIPH_CLOCK

#define OTG_PIN_DP                       GPIO_PINS_12
#define OTG_PIN_DP_SOURCE                GPIO_PINS_SOURCE12

#define OTG_PIN_DM                       GPIO_PINS_11
#define OTG_PIN_DM_SOURCE                GPIO_PINS_SOURCE11

#define OTG_PIN_MUX                      GPIO_MUX_10

/* internal vriable */
static otg_core_type otg_core_struct;
static uint8_t USB_Rx_Buff[USB_RX_BUFF_SIZE] = {0};

static BspUSB_VCP_Obj_TypeDef BspUSB_Monitor = {
    .init_state = BspUSB_None_Init,
    .rx_byte_sum = 0,
    .rx_callback = NULL,
    .rx_irq_cnt = 0,
    .tx_fin_callback = NULL,
    .tx_fin_cnt = 0,
    .connect_time = 0,
};

/* internal function */
static bool BspUSB_Clock_Init(void);
static bool BspUSB_Pin_Init(void);
static void BspUSB_Rec_Callback(void);
static void BspUSB_Trans_Callback(void);
static void BspUSB_Connect_Callback(void);

/* external funtion */
static BspUSB_Error_List BspUSB_Init(uint32_t cus_data_addr);
static BspUSB_Error_List BspUSB_DeInit(void);
static BspUSB_Error_List BspUSB_Send(uint8_t *p_data, uint16_t size);
static void BspUSB_Set_Rx_Callback(BspUSB_Rx_Callback_Def callback);
static void BspUSB_Set_Tx_Callback(BspUSB_Tx_Cplt_Callback_Def callback);
static void BspUSB_Set_Connect_Callback(BspUSB_Connect_Callback_Def callback);
static bool BspUSB_CheckConnect(uint32_t sys_tick, uint32_t time_out);

/* external vriable */
BspUSB_VCP_TypeDef BspUSB_VCP = {
    .init = BspUSB_Init,
    .de_init = BspUSB_DeInit,
    .send = BspUSB_Send,
    .set_rx_callback = BspUSB_Set_Rx_Callback,
    .set_tx_cpl_callback = BspUSB_Set_Tx_Callback,
    .set_connect_callback = BspUSB_Set_Connect_Callback,
    .check_connect = BspUSB_CheckConnect,
};

static bool BspUSB_Clock_Init(void)
{
    /* enable otgfs clock */
    crm_periph_clock_enable(CRM_OTGFS1_PERIPH_CLOCK, TRUE);

    /* select usb 48m clcok source */
    switch(system_core_clock)
    {
    /* 48MHz */
    case 48000000:
        crm_usb_clock_div_set(CRM_USB_DIV_1);
        break;

    /* 72MHz */
    case 72000000:
        crm_usb_clock_div_set(CRM_USB_DIV_1_5);
        break;

    /* 96MHz */
    case 96000000:
        crm_usb_clock_div_set(CRM_USB_DIV_2);
        break;

    /* 120MHz */
    case 120000000:
        crm_usb_clock_div_set(CRM_USB_DIV_2_5);
        break;

    /* 144MHz */
    case 144000000:
        crm_usb_clock_div_set(CRM_USB_DIV_3);
        break;

    /* 168MHz */
    case 168000000:
        crm_usb_clock_div_set(CRM_USB_DIV_3_5);
        break;

    /* 192MHz */
    case 192000000:
        crm_usb_clock_div_set(CRM_USB_DIV_4);
        break;

    /* 216MHz */
    case 216000000:
        crm_usb_clock_div_set(CRM_USB_DIV_4_5);
        break;

    /* 240MHz */
    case 240000000:
        crm_usb_clock_div_set(CRM_USB_DIV_5);
        break;

    /* 264MHz */
    case 264000000:
        crm_usb_clock_div_set(CRM_USB_DIV_5_5);
        break;

    /* 288MHz */
    case 288000000:
        crm_usb_clock_div_set(CRM_USB_DIV_6);
        break;

    default:
        return false;
    }

    return true;
}

static bool BspUSB_Pin_Init(void)
{
    gpio_init_type gpio_init_struct;

    crm_periph_clock_enable(OTG_PIN_GPIO_CLOCK, TRUE);

    /* USB DP DM Pin Init */
    gpio_default_para_init(&gpio_init_struct);

    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;

    /* dp and dm */
    gpio_init_struct.gpio_pins = OTG_PIN_DP | OTG_PIN_DM;
    gpio_init(OTG_PIN_GPIO, &gpio_init_struct);

    gpio_pin_mux_config(OTG_PIN_GPIO, OTG_PIN_DP_SOURCE, OTG_PIN_MUX);
    gpio_pin_mux_config(OTG_PIN_GPIO, OTG_PIN_DM_SOURCE, OTG_PIN_MUX);

    return true;
}

static BspUSB_Error_List BspUSB_Init(uint32_t cus_data_addr)
{
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    BspUSB_Pin_Init();

    if(!BspUSB_Clock_Init())
        return BspUSB_Error_Fail;

    /* init usb */
    usbd_init(&otg_core_struct,
              USB_FULL_SPEED_CORE_ID,
              USB_ID,
              &cdc_class_handler,
              &cdc_desc_handler);

    /* enable otgfs irq */
    nvic_irq_enable(OTGFS1_IRQn, 5, 0);

    BspUSB_Monitor.cus_data_addr = cus_data_addr;
    BspUSB_Monitor.init_state = BspUSB_Error_None;

    usb_cdc_rec_cb = BspUSB_Rec_Callback;
    usb_cdc_trans_cb = BspUSB_Trans_Callback;
    usb_cdc_cnt_cb = BspUSB_Connect_Callback;

    memset(USB_Rx_Buff, 0, USB_RX_BUFF_SIZE);

    return BspUSB_Error_None;
}

static BspUSB_Error_List BspUSB_DeInit(void)
{
    crm_periph_clock_enable(CRM_OTGFS1_PERIPH_CLOCK, FALSE);
    nvic_irq_disable(OTGFS1_IRQn);

    return BspUSB_Error_None;
}

static BspUSB_Error_List BspUSB_Send(uint8_t *p_data, uint16_t size)
{
    if ((BspUSB_Monitor.init_state == BspUSB_Error_None) && p_data && size)
    {
        if(BspUSB_Monitor.tx_fin_cnt != BspUSB_Monitor.tx_cnt)
            return BspUSB_Error_Busy;

        if(usb_vcp_send_data(&otg_core_struct.dev, p_data, size) == SUCCESS)
        {
            BspUSB_Monitor.tx_cnt ++;
            return BspUSB_Error_None;
        }
        else
        {
            BspUSB_Monitor.tx_err_cnt ++;
        }
    }

    return BspUSB_Error_Fail;
}

static bool BspUSB_CheckConnect(uint32_t sys_tick, uint32_t time_out)
{
    if (sys_tick && \
        BspUSB_Monitor.connect_time && \
        (sys_tick - BspUSB_Monitor.connect_time < time_out))
        return true;

    return false;
}

void BspUSB_Irq_Callback(void)
{
    usbd_irq_handler(&otg_core_struct);
}

static void BspUSB_Set_Rx_Callback(BspUSB_Rx_Callback_Def callback)
{
    BspUSB_Monitor.rx_callback = callback;
}

static void BspUSB_Set_Tx_Callback(BspUSB_Tx_Cplt_Callback_Def callback)
{
    BspUSB_Monitor.tx_fin_callback = callback;
}

static void BspUSB_Set_Connect_Callback(BspUSB_Connect_Callback_Def callback)
{
    BspUSB_Monitor.connect_callback = callback;
}

static void BspUSB_Rec_Callback(void)
{
    uint16_t data_len = 0;

    if (BspUSB_Monitor.init_state != BspUSB_Error_None)
        return;

    data_len = usb_vcp_get_rxdata(&otg_core_struct.dev,  USB_Rx_Buff);

    if (BspUSB_Monitor.rx_callback)
        BspUSB_Monitor.rx_callback(BspUSB_Monitor.cus_data_addr, USB_Rx_Buff, data_len);

    BspUSB_Monitor.rx_irq_cnt ++;
    BspUSB_Monitor.rx_byte_sum += data_len;
}

static void BspUSB_Trans_Callback(void)
{
    if (BspUSB_Monitor.init_state != BspUSB_Error_None)
        return;

    BspUSB_Monitor.tx_fin_cnt ++;
    if (BspUSB_Monitor.tx_fin_callback)
        BspUSB_Monitor.tx_fin_callback(BspUSB_Monitor.cus_data_addr, NULL, NULL);
}

static void BspUSB_Connect_Callback(void)
{
    if (BspUSB_Monitor.init_state != BspUSB_Error_None)
        return;

    if (BspUSB_Monitor.connect_callback)
        BspUSB_Monitor.connect_callback(BspUSB_Monitor.cus_data_addr, &BspUSB_Monitor.connect_time);
}
