#include "Bsp_GPIO.h"
#include "at32f435_437.h"

#define To_GPIO_Port(x) ((BspGPIO_Port_TypeDef *)x)
#define To_ExtiGPIO_Port(x) ((BspGPIO_EXTI_Port_TypeDef *)x)

static EXTI_Callback EXTI_CallBack_List[GPIO_EXTI_SUM] = {NULL};

/* internal function */
static uint8_t BspGPIO_Get_PinSource(uint16_t pin);
static uint32_t BsPGPIO_Get_EXTI_Line(uint16_t pin);
static uint8_t BspGPIO_EXTILine_To_PinSource(uint32_t exti_line);
uint8_t BspGPIO_Get_Bit_Index(uint16_t val);
static IRQn_Type BspGPIO_Get_IRQn(uint32_t pin);

/* external function */
static bool BspGPIO_Out_Init(BspGPIO_Obj_TypeDef IO_Obj);
static void BspGPIO_De_Init(BspGPIO_Obj_TypeDef IO_Obj);
static bool BspGPIO_In_Init(BspGPIO_Obj_TypeDef IO_Obj);
static bool BspGPIO_Exti_Init(BspGPIO_Obj_TypeDef IO_Obj, EXTI_Callback callback);
static bool BspGPIO_Altnate_Init(BspGPIO_Obj_TypeDef IO_Obj, uint32_t af_mode);
static bool BspGPIO_Read(BspGPIO_Obj_TypeDef IO_Obj);
static bool BspGPIO_Set_Exti_Callback(BspGPIO_Obj_TypeDef IO_Obj, EXTI_Callback callback);
static bool BspGPIO_Set_Exti_Mode(BspGPIO_Obj_TypeDef IO_Obj, BspGPOP_ExtiMode_List mode);
static void BspGPIO_Write(BspGPIO_Obj_TypeDef IO_Obj, bool state);

BspGPIO_TypeDef BspGPIO = {
    .exti_init = BspGPIO_Exti_Init,
    .out_init = BspGPIO_Out_Init,
    .in_init = BspGPIO_In_Init,
    .alt_init = BspGPIO_Altnate_Init,
    .de_init = BspGPIO_De_Init,
    .read = BspGPIO_Read,
    .set_exti_callback = BspGPIO_Set_Exti_Callback,
    .set_exti_mode = BspGPIO_Set_Exti_Mode,
    .write = BspGPIO_Write,
};

uint8_t BspGPIO_Get_Bit_Index(uint16_t val)
{
	float f_val = (float)val;
	uint32_t u32_val = 0;
	uint8_t bit = 0;
	
	u32_val = *((unsigned long*)(&f_val));
	
	bit = (u32_val >> 23) & 0xFF;
	bit -= 127;
	
    return bit;
}

static IRQn_Type BspGPIO_Get_IRQn(uint32_t pin)
{
    switch(pin)
    {
        case GPIO_PINS_0:
            return EXINT0_IRQn;

        case GPIO_PINS_1:
            return EXINT1_IRQn;

        case GPIO_PINS_2:
            return EXINT2_IRQn;

        case GPIO_PINS_3:
            return EXINT3_IRQn;

        case GPIO_PINS_4:
            return EXINT4_IRQn;

        case GPIO_PINS_5:
        case GPIO_PINS_6:
        case GPIO_PINS_7:
        case GPIO_PINS_8:
        case GPIO_PINS_9:
            return EXINT9_5_IRQn;

        case GPIO_PINS_10:
        case GPIO_PINS_11:
        case GPIO_PINS_12:
        case GPIO_PINS_13:
        case GPIO_PINS_14:
        case GPIO_PINS_15:
            return EXINT15_10_IRQn;

        default:
            return -16;
    }
}

static uint8_t BspGPIO_Get_PinSource(uint16_t pin)
{
    if(pin > GPIO_PINS_15)
        return 16;

    return BspGPIO_Get_Bit_Index(pin);
}

static uint32_t BsPGPIO_Get_EXTI_Line(uint16_t pin)
{
    if (pin > GPIO_PINS_15)
        return 0;
    
    return (uint32_t)pin;
}

static uint8_t BspGPIO_EXTILine_To_PinSource(uint32_t exti_line)
{
    uint16_t index = 0;
    
    if(exti_line > EXINT_LINE_15)
        return 0;
    
    index = (uint16_t)exti_line;
    return BspGPIO_Get_Bit_Index(index);
}

static crm_periph_clock_type BspGPIO_Get_ExtiCLK(scfg_port_source_type exti_src)
{
    switch((uint8_t) exti_src)
    {
        case SCFG_PORT_SOURCE_GPIOA:
            return CRM_GPIOA_PERIPH_CLOCK;

        case SCFG_PORT_SOURCE_GPIOB:
            return CRM_GPIOB_PERIPH_CLOCK;

        case SCFG_PORT_SOURCE_GPIOC:
            return CRM_GPIOC_PERIPH_CLOCK;

        case SCFG_PORT_SOURCE_GPIOD:
            return CRM_GPIOD_PERIPH_CLOCK;

        case SCFG_PORT_SOURCE_GPIOE:
            return CRM_GPIOE_PERIPH_CLOCK;

        case SCFG_PORT_SOURCE_GPIOF:
            return CRM_GPIOF_PERIPH_CLOCK;

        case SCFG_PORT_SOURCE_GPIOG:
            return CRM_GPIOG_PERIPH_CLOCK;

        case SCFG_PORT_SOURCE_GPIOH:
            return CRM_GPIOH_PERIPH_CLOCK;

        default:
            return 0;
    }
}

static crm_periph_clock_type BspGPIO_Get_CLK(gpio_type *port)
{
    if(port == NULL)
        return 0;
    
    if(port == GPIOA)
        return CRM_GPIOA_PERIPH_CLOCK;

    if(port == GPIOB)
        return CRM_GPIOB_PERIPH_CLOCK;

    if(port == GPIOC)
        return CRM_GPIOC_PERIPH_CLOCK;

    if(port == GPIOD)
        return CRM_GPIOD_PERIPH_CLOCK;

    if(port == GPIOE)
        return CRM_GPIOE_PERIPH_CLOCK;

    if(port == GPIOF)
        return CRM_GPIOF_PERIPH_CLOCK;

    if(port == GPIOG)
        return CRM_GPIOG_PERIPH_CLOCK;

    if(port == GPIOH)
        return CRM_GPIOH_PERIPH_CLOCK;

    return 0;
}

static bool BspGPIO_Out_Init(BspGPIO_Obj_TypeDef IO_Obj)
{
    gpio_init_type gpio_init_struct;
    crm_periph_clock_type clk = 0;

    clk = BspGPIO_Get_CLK(To_GPIO_Port(IO_Obj.port)->port);
    if(clk == 0)
        return false;

    /* enable the clock */
    crm_periph_clock_enable(clk, TRUE);
    
    /* set default parameter */
    gpio_default_para_init(&gpio_init_struct);
    
    /* configure the led gpio */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pins = IO_Obj.pin;
    gpio_init_struct.gpio_pull = IO_Obj.init_state;
    gpio_init(To_GPIO_Port(IO_Obj.port)->port, &gpio_init_struct);
  
    return true;
}   

static bool BspGPIO_In_Init(BspGPIO_Obj_TypeDef IO_Obj)
{
    gpio_init_type gpio_init_struct;
    crm_periph_clock_type clk = 0;

    clk = BspGPIO_Get_CLK(To_GPIO_Port(IO_Obj.port)->port);
    if(clk == 0)
        return false;

    /* enable the led clock */
    crm_periph_clock_enable(clk, TRUE);
    
    /* set default parameter */
    gpio_default_para_init(&gpio_init_struct);
    
    /* configure the led gpio */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = IO_Obj.pin;
    gpio_init_struct.gpio_pull = IO_Obj.init_state;
    gpio_init(To_GPIO_Port(IO_Obj.port)->port, &gpio_init_struct);

    return true;
}

static bool BspGPIO_Exti_Init(BspGPIO_Obj_TypeDef IO_Obj, EXTI_Callback callback)
{
    exint_init_type exint_init_struct;
    uint32_t exti_line = 0;
    uint32_t pin_source = 0;
    crm_periph_clock_type clk = 0;

    clk = BspGPIO_Get_ExtiCLK(To_ExtiGPIO_Port(IO_Obj.port)->port);
    if(clk == 0)
        return false;

    crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(clk, TRUE);

    exti_line = BsPGPIO_Get_EXTI_Line(IO_Obj.pin);
    pin_source = BspGPIO_Get_PinSource(IO_Obj.pin);

    if( (exti_line == 0) || \
        (pin_source > SCFG_PINS_SOURCE15) || \
        (IO_Obj.init_state > EXINT_TRIGGER_BOTH_EDGE))
        return false;

    scfg_exint_line_config(To_ExtiGPIO_Port(IO_Obj.port)->port, pin_source);
    
    exint_default_para_init(&exint_init_struct);
    exint_init_struct.line_enable = TRUE;
    exint_init_struct.line_mode = EXINT_LINE_INTERRUPUT;
    exint_init_struct.line_select = exti_line;
    exint_init_struct.line_polarity = IO_Obj.init_state;
    exint_init(&exint_init_struct);

    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(BspGPIO_Get_IRQn(IO_Obj.pin), 5, 0);

    /* set default exti irq callback */
    EXTI_CallBack_List[pin_source] = callback;

    return true;
}

static bool BspGPIO_Altnate_Init(BspGPIO_Obj_TypeDef IO_Obj, uint32_t af_mode)
{
    gpio_init_type gpio_initstructure;
    uint32_t pin_source = 0;
    crm_periph_clock_type clk = 0;

    UNUSED(af_mode);

    clk = BspGPIO_Get_CLK(To_GPIO_Port(IO_Obj.port)->port);
    if(clk == 0)
        return false;
    
    crm_periph_clock_enable(clk, TRUE);
    gpio_default_para_init(&gpio_initstructure);

    pin_source = BspGPIO_Get_PinSource(IO_Obj.pin);

    if(pin_source == 16)
        return false;

    gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_initstructure.gpio_pull = GPIO_PULL_UP;
    gpio_initstructure.gpio_mode = GPIO_MODE_MUX;
    gpio_initstructure.gpio_pins = IO_Obj.pin;
    gpio_init(To_GPIO_Port(IO_Obj.port)->port, &gpio_initstructure);

    gpio_pin_mux_config(To_GPIO_Port(IO_Obj.port)->port, pin_source, IO_Obj.alternate);

    return true;
}

static bool BspGPIO_Read(BspGPIO_Obj_TypeDef IO_Obj)
{
    if(gpio_input_data_bit_read(To_GPIO_Port(IO_Obj.port)->port, IO_Obj.pin))
        return true;

    return false;
}

static bool BspGPIO_Set_Exti_Callback(BspGPIO_Obj_TypeDef IO_Obj, EXTI_Callback callback)
{
    uint32_t pin_source = 0;
    pin_source = BspGPIO_Get_PinSource(IO_Obj.pin);
    if(pin_source > SCFG_PINS_SOURCE15)
        return false;

    EXTI_CallBack_List[pin_source] = callback;

    return true;
}

static bool BspGPIO_Set_Exti_Mode(BspGPIO_Obj_TypeDef IO_Obj, BspGPOP_ExtiMode_List mode)
{
    exint_init_type exint_init_struct;
    exint_polarity_config_type polarity = EXINT_TRIGGER_BOTH_EDGE;
    uint32_t exti_line = 0;
    uint32_t pin_source = 0;
    crm_periph_clock_type clk = 0;

    clk = BspGPIO_Get_CLK(To_GPIO_Port(IO_Obj.port)->port);
    if(clk == 0)
        return false;

    /* disable irq first */
    nvic_irq_disable(BspGPIO_Get_IRQn(IO_Obj.pin));

    crm_periph_clock_enable(clk, TRUE);

    exti_line = BsPGPIO_Get_EXTI_Line(IO_Obj.pin);
    pin_source = BspGPIO_Get_PinSource(IO_Obj.pin);

    if( (exti_line == 0) || \
        (pin_source > SCFG_PINS_SOURCE15) || \
        (IO_Obj.init_state > EXINT_TRIGGER_BOTH_EDGE))
        return false;

    switch((uint8_t)mode)
    {
        case GPIO_Exti_Rasing:
            polarity = EXINT_TRIGGER_RISING_EDGE;
            break;

        case GPIO_Exti_Falling:
            polarity = EXINT_TRIGGER_FALLING_EDGE;
            break;

        case GPIO_Exti_TwoEdge:
            break;

        default:
            return false;
    }

    scfg_exint_line_config(To_ExtiGPIO_Port(IO_Obj.port)->port, pin_source);
    
    exint_default_para_init(&exint_init_struct);
    exint_init_struct.line_enable = TRUE;
    exint_init_struct.line_mode = EXINT_LINE_INTERRUPUT;
    exint_init_struct.line_select = exti_line;
    exint_init_struct.line_polarity = polarity;
    exint_init(&exint_init_struct);

    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(BspGPIO_Get_IRQn(IO_Obj.pin), 5, 0);

    return false;
}

static void BspGPIO_Write(BspGPIO_Obj_TypeDef IO_Obj, bool state)
{
    if(state)
    {
        To_GPIO_Port(IO_Obj.port)->port->scr = IO_Obj.pin;
    }
    else
    {
        To_GPIO_Port(IO_Obj.port)->port->clr = IO_Obj.pin;
    }
}

void BspGPIO_IRQ_Polling(uint32_t exti_line)
{
    uint8_t index = 0;

    if(exint_flag_get(exti_line) != RESET)
    {
        index = BspGPIO_EXTILine_To_PinSource(exti_line);

        if((index <= SCFG_PINS_SOURCE15) && (EXTI_CallBack_List[index] != NULL))
            EXTI_CallBack_List[index]();

        exint_flag_clear(exti_line);
    }
}

static void BspGPIO_De_Init(BspGPIO_Obj_TypeDef IO_Obj)
{
    if (IO_Obj.port && To_GPIO_Port(IO_Obj.port)->port)
        gpio_reset(To_GPIO_Port(IO_Obj.port)->port);
}


