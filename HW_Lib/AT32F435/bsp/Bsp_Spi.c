#include "Bsp_Spi.h"
#include "Bsp_GPIO.h"
#include "at32f435_437.h"

#define Dummy_Byte 0xFF
#define To_SPI_Instance(x) ((spi_type *)x)

/* internal function */
static crm_periph_clock_type BspSPI_Get_Clock(spi_type *type);
static bool BspSPI_Pin_Init(BspSPI_PinConfig_TypeDef pin);

/* external function */
static bool BspSPI_Init(BspSPI_NorModeConfig_TypeDef spi_cfg, void *spi_instance);
static bool BspSPI_Deinit(BspSPI_NorModeConfig_TypeDef spi_cfg);
static bool BspSPI_Transmit(void *instance, uint8_t *tx, uint16_t size, uint16_t time_out);
static bool BspSPI_Receive(void *instance, uint8_t *rx, uint16_t size, uint16_t time_out);
static uint16_t BspSPI_Trans_Receive(void *instance, uint8_t *tx, uint8_t *rx, uint16_t size, uint16_t time_out);
static bool BspSPI_Set_Speed(void *instance, uint32_t speed);

BspSpi_TypeDef BspSPI = {
    .init = BspSPI_Init,
    .deinit = BspSPI_Deinit,
    .trans = BspSPI_Transmit,
    .receive = BspSPI_Receive,
    .trans_receive = BspSPI_Trans_Receive,
    .set_speed = BspSPI_Set_Speed,
};

static crm_periph_clock_type BspSPI_Get_Clock(spi_type *type)
{
    switch((uint32_t)type)
    {
        case (uint32_t)SPI1:
            return CRM_SPI1_PERIPH_CLOCK;
        
        case (uint32_t)SPI2:
            return CRM_SPI2_PERIPH_CLOCK;
    
        case (uint32_t)SPI3:
            return CRM_SPI3_PERIPH_CLOCK;
    
        case (uint32_t)SPI4:
            return CRM_SPI4_PERIPH_CLOCK;

        default:
            return 0;
    }
}

static bool BspSPI_Pin_Init(BspSPI_PinConfig_TypeDef pin)
{
    BspGPIO_Obj_TypeDef GPIO_Pin_Obj;
    BspGPIO_Port_TypeDef GPIO_Port_Obj;

    if( (pin.port_clk == NULL) || \
        (pin.port_miso == NULL) || \
        (pin.port_mosi == NULL))
        return false;

    memset(&GPIO_Pin_Obj, 0, sizeof(BspGPIO_Obj_TypeDef));
    memset(&GPIO_Port_Obj, 0, sizeof(BspGPIO_Port_TypeDef));

    /* sck */
    GPIO_Port_Obj.port = pin.port_clk;
    GPIO_Pin_Obj.alternate = pin.pin_Alternate;
    GPIO_Pin_Obj.pin = pin.pin_clk;
    GPIO_Pin_Obj.port = &GPIO_Port_Obj;
    if(!BspGPIO.alt_init(GPIO_Pin_Obj, 0))
        return false;

    /* mosi */
    GPIO_Port_Obj.port = pin.port_mosi;
    GPIO_Pin_Obj.alternate = pin.pin_Alternate;
    GPIO_Pin_Obj.pin = pin.pin_mosi;
    GPIO_Pin_Obj.port = &GPIO_Port_Obj;
    if(!BspGPIO.alt_init(GPIO_Pin_Obj, 0))
        return false;

    /* miso */
    GPIO_Port_Obj.port = pin.port_miso;
    GPIO_Pin_Obj.alternate = pin.pin_Alternate;
    GPIO_Pin_Obj.pin = pin.pin_miso;
    GPIO_Pin_Obj.port = &GPIO_Port_Obj;
    if(!BspGPIO.alt_init(GPIO_Pin_Obj, 0))
        return false;

    return true;
}

static bool BspSPI_Init(BspSPI_NorModeConfig_TypeDef spi_cfg, void *spi_instance)
{
    spi_init_type spi_init_struct;
    crm_periph_clock_type clk = 0;

    if((spi_cfg.Instance == NULL) || !BspSPI_Pin_Init(spi_cfg.Pin))
        return false;

    clk = BspSPI_Get_Clock(spi_cfg.Instance);
    if(clk == 0)
        return false;

    crm_periph_clock_enable(clk, TRUE);
    spi_default_para_init(&spi_init_struct);

    spi_init_struct.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
    spi_init_struct.master_slave_mode = SPI_MODE_MASTER;
    spi_init_struct.mclk_freq_division = spi_cfg.BaudRatePrescaler;
    spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_MSB;
    spi_init_struct.frame_bit_num = SPI_FRAME_8BIT;
    spi_init_struct.clock_polarity = spi_cfg.CLKPolarity;
    spi_init_struct.clock_phase = spi_cfg.CLKPhase;
    spi_init_struct.cs_mode_selection = SPI_CS_SOFTWARE_MODE;

    spi_init(To_SPI_Instance(spi_cfg.Instance), &spi_init_struct);
    spi_enable(To_SPI_Instance(spi_cfg.Instance), TRUE);

    *(uint32_t *)spi_instance = (uint32_t *)spi_cfg.Instance;

    return true;
}

static bool BspSPI_Deinit(BspSPI_NorModeConfig_TypeDef spi_cfg)
{
    uint32_t clk = 0;

    if(spi_cfg.Instance == NULL)
        return false;

    clk = BspSPI_Get_Clock(spi_cfg.Instance);
    if(clk == 0)
        return false;

    crm_periph_clock_enable(clk, FALSE);
    spi_enable(To_SPI_Instance(spi_cfg.Instance), FALSE);

    return true;
}

static bool BspSPI_Transmit(void *instance, uint8_t *tx, uint16_t size, uint16_t time_out)
{
    uint64_t start_time = System_GetTick();

    if((instance == NULL) || (tx == NULL) || (size == 0))
        return false;

    for(uint16_t i = 0; i < size; i++)
    {
        while(spi_i2s_flag_get(To_SPI_Instance(*(uint32_t *)instance), SPI_I2S_TDBE_FLAG) == RESET)
        {
            if ((System_GetTick() - start_time) > time_out)
                return false;
        }
        spi_i2s_data_transmit(To_SPI_Instance(*(uint32_t *)instance), tx[i]);
        
        while(spi_i2s_flag_get(To_SPI_Instance(*(uint32_t *)instance), SPI_I2S_RDBF_FLAG) == RESET)
        {
            if ((System_GetTick() - start_time) > time_out)
                return false;
        }
        spi_i2s_data_receive(To_SPI_Instance(*(uint32_t *)instance));
    }

    return true;
}

static bool BspSPI_Receive(void *instance, uint8_t *rx, uint16_t size, uint16_t time_out)
{
    uint64_t start_time = System_GetTick();

    if((instance == NULL) || (rx == NULL) || (size == 0))
        return false;

    for(uint16_t i = 0; i < size; i++)
    {
        while(spi_i2s_flag_get(To_SPI_Instance(*(uint32_t *)instance), SPI_I2S_TDBE_FLAG) == RESET)
        {
            if ((System_GetTick() - start_time) > time_out)
                return false;
        }
        spi_i2s_data_transmit(To_SPI_Instance(*(uint32_t *)instance), Dummy_Byte);
        
        while(spi_i2s_flag_get(To_SPI_Instance(*(uint32_t *)instance), SPI_I2S_RDBF_FLAG) == RESET)
        {
            if ((System_GetTick() - start_time) > time_out)
                return false;
        }
        rx[i] = spi_i2s_data_receive(To_SPI_Instance(*(uint32_t *)instance));
    }

    return true;
}

static uint16_t BspSPI_Trans_Receive(void *instance, uint8_t *tx, uint8_t *rx, uint16_t size, uint16_t time_out)
{
    uint64_t start_time = System_GetTick();

    if((instance == NULL) || (tx == NULL) || (rx == NULL) || (size == 0))
        return 0;

    for(uint16_t i = 0; i < size; i++)
    {
        while(spi_i2s_flag_get(To_SPI_Instance(*(uint32_t *)instance), SPI_I2S_TDBE_FLAG) == RESET)
        {
            if ((System_GetTick() - start_time) > time_out)
                return 0;
        }
        spi_i2s_data_transmit(To_SPI_Instance(*(uint32_t *)instance), tx[i]);

        while(spi_i2s_flag_get(To_SPI_Instance(*(uint32_t *)instance), SPI_I2S_RDBF_FLAG) == RESET)
        {
            if ((System_GetTick() - start_time) > time_out)
                return 0;
        }
        rx[i] = spi_i2s_data_receive(To_SPI_Instance(*(uint32_t *)instance));
    }

    return size;
}

static bool BspSPI_Set_Speed(void *instance, uint32_t speed)
{
    UNUSED(instance);
    UNUSED(speed);

    return false;
}

