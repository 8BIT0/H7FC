#include "HW_Def.h"

// MOTOR 1 B06
// MOTOR 2 B07
// MOTOR 3 A03
// MOTOR 4 B01

// OSD_CS 1 B12

// ADC_BATT 1 A00
// ADC_CURR 1 A01

// timer A08 AF1
// # pin A08: TIM1 CH1 (AF1)

// timer B06 AF2
// # pin B06: TIM4 CH1 (AF2)
// timer B07 AF2
// # pin B07: TIM4 CH2 (AF2)
// timer A03 AF1
// # pin A03: TIM2 CH4 (AF1)
// timer B01 AF2
// # pin B01: TIM3 CH4 (AF2)

// dma ADC 1 11
// # ADC 1: DMA2 Channel 0 Channel 5

// dma pin A08 7
// # pin A08: DMA2 Channel 0 Channel 42
// dma pin B06 0
// # pin B06: DMA1 Channel 0 Channel 67
// dma pin B07 1
// # pin B07: DMA1 Channel 0 Channel 68
// dma pin A03 2
// # pin A03: DMA1 Channel 0 Channel 59
// dma pin B01 3
// # pin B01: DMA1 Channel 0 Channel 64

const uint8_t HWVer[3] = {0, 0, 4};

/* Led 1 C13 */
BspGPIO_Port_TypeDef Led1_Port = {
    .port = GPIOC,
};

BspGPIO_Obj_TypeDef Led1 = {
    .port = (void *)&Led1_Port,
    .pin = GPIO_PINS_13,
    .init_state = GPIO_PULL_NONE,
};

/* Led 2 C14 */
BspGPIO_Port_TypeDef Led2_Port = {
    .port = GPIOC,
};

BspGPIO_Obj_TypeDef Led2 = {
    .port = (void *)&Led2_Port,
    .pin = GPIO_PINS_14,
    .init_state = GPIO_PULL_NONE,
};

/* on board receiver uart tx port B11 */
BspGPIO_Port_TypeDef Uart3_Tx_Port = {
    .port = GPIOB,
};

BspGPIO_Obj_TypeDef Uart3_TxPin = {
    .port = (void *)&Uart3_Tx_Port,
    .pin = GPIO_PINS_11,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_7,
};

/* on board receiver uart rx port B10 */
BspGPIO_Port_TypeDef Uart3_Rx_Port = {
    .port = GPIOB,
};

BspGPIO_Obj_TypeDef Uart3_RxPin = {
    .port = (void *)&Uart3_Rx_Port,
    .pin = GPIO_PINS_10,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_7,
};

/* uart 1 tx pin A09 */
BspGPIO_Port_TypeDef Uart1_Tx_Port = {
    .port = GPIOA,
};

BspGPIO_Obj_TypeDef Uart1_TxPin = {
    .port = (void *)&Uart1_Tx_Port,
    .pin = GPIO_PINS_9,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_7,
};

/* uart 1 rx pin A10 */
BspGPIO_Port_TypeDef Uart1_Rx_Port = {
    .port = GPIOA,
};

BspGPIO_Obj_TypeDef Uart1_RxPin = {
    .port = (void *)&Uart1_Rx_Port,
    .pin = GPIO_PINS_10,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_7,
};

/* uart 2 tx pin A02 */
BspGPIO_Port_TypeDef Uart2_Tx_Port = {
    .port = GPIOA,
};

BspGPIO_Obj_TypeDef Uart2_TxPin = {
    .port = (void *)&Uart2_Tx_Port,
    .pin = GPIO_PINS_2,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_7,
};

/* uart 2 rx pin B00 */
BspGPIO_Port_TypeDef Uart2_Rx_Port = {
    .port = GPIOB,
};

BspGPIO_Obj_TypeDef Uart2_RxPin = {
    .port = (void *)&Uart2_Rx_Port,
    .pin = GPIO_PINS_0,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_6,
};

/* uart 5 tx pin B09 */
BspGPIO_Port_TypeDef Uart5_Tx_Port = {
    .port = GPIOB,
};

BspGPIO_Obj_TypeDef Uart5_TxPin = {
    .port = (void *)&Uart5_Tx_Port,
    .pin = GPIO_PINS_9,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_8,
};

/* uart 5 rx pin B08 */
BspGPIO_Port_TypeDef Uart5_Rx_Port = {
    .port = GPIOB,
};

BspGPIO_Obj_TypeDef Uart5_RxPin = {
    .port = (void *)&Uart5_Rx_Port,
    .pin = GPIO_PINS_8,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_8,
};

/* uart 7 tx pin B04 */
BspGPIO_Port_TypeDef Uart7_Tx_Port = {
    .port = GPIOB,
};

BspGPIO_Obj_TypeDef Uart7_TxPin = {
    .port = (void *)&Uart7_Tx_Port,
    .pin = GPIO_PINS_4,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_8,
};

/* uart 7 rx pin B03 */
BspGPIO_Port_TypeDef Uart7_Rx_Port = {
    .port = GPIOB,
};

BspGPIO_Obj_TypeDef Uart7_RxPin = {
    .port = (void *)&Uart7_Rx_Port,
    .pin = GPIO_PINS_3,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_8,
};

/* IMU CS Pin A04 */
BspGPIO_Port_TypeDef PriIMU_CS_Port = {
    .port = GPIOA,
};

BspGPIO_Obj_TypeDef PriIMU_CSPin = {
    .init_state = GPIO_PULL_UP,
    .pin = GPIO_PINS_4,
    .port = &PriIMU_CS_Port,
};

/* IMU INT Pin A15 */
BspGPIO_EXTI_Port_TypeDef PriIMU_INT_Port = {
    .port = SCFG_PORT_SOURCE_GPIOA,
};

BspGPIO_Obj_TypeDef PriIMU_INTPin = {
    .port = (void *)&PriIMU_INT_Port,
    .pin = GPIO_PINS_15,
    .init_state = EXINT_TRIGGER_FALLING_EDGE,
};

/* IMU Bus Pin (SPI1) */
/* SPI_SCK  1 A05 */
/* SPI_MISO 1 A06 */
/* SPI_MOSI 1 A07 */
BspSPI_PinConfig_TypeDef PriIMU_BusPin = {
    .port_mosi = (void *)GPIOA,
    .port_miso = (void *)GPIOA,
    .port_clk = (void *)GPIOA,
    
    .pin_mosi = GPIO_PINS_7,
    .pin_miso = GPIO_PINS_6,
    .pin_clk = GPIO_PINS_5,

    .pin_Alternate = GPIO_MUX_5,
};

BspSPI_NorModeConfig_TypeDef PriIMU_BusCfg = {
    .Instance = PriIMU_SPI_BUS,
    .CLKPolarity = SPI_CLOCK_POLARITY_HIGH,
    .CLKPhase = SPI_CLOCK_PHASE_2EDGE,
    .BaudRatePrescaler = SPI_MCLK_DIV_8,
    .work_mode = BspSPI_Mode_Master,
};

/* OSD chip cs B12 */
BspGPIO_Port_TypeDef OSD_CS_Port = {
    .port = GPIOB,
};

BspGPIO_Obj_TypeDef OSD_CSPin = {
    .port = (void *)&OSD_CS_Port,
    .pin = GPIO_PINS_12,
    .init_state = GPIO_PULL_UP,
};

/* external flash chip cs B05 */
BspGPIO_Port_TypeDef ExtFlash_CS_Port = {
    .port = GPIOB,
};

BspGPIO_Obj_TypeDef ExtFlash_CSPin = {
    .port = (void *)&ExtFlash_CS_Port,
    .pin = GPIO_PINS_5,
    .init_state = GPIO_PULL_UP,
};

/* external flash chip bus Pin (SPI2) */
/* SPI_SCK  2 B13 */
/* SPI_MISO 2 B14 */
/* SPI_MOSI 2 B15 */
BspSPI_PinConfig_TypeDef ExtFlash_SPIPin = {
    .port_mosi = (void *)GPIOB,
    .port_miso = (void *)GPIOB,
    .port_clk = (void *)GPIOB,
    
    .pin_mosi = GPIO_PINS_15,
    .pin_miso = GPIO_PINS_14,
    .pin_clk = GPIO_PINS_13,

    .pin_Alternate = GPIO_MUX_5,
};

/* PWM IO */
BspGPIO_Port_TypeDef PWM_1_Port = {
    .port = GPIOB,
};

BspGPIO_Port_TypeDef PWM_2_Port = {
    .port = GPIOB,
};

BspGPIO_Port_TypeDef PWM_3_Port = {
    .port = GPIOA,
};

BspGPIO_Port_TypeDef PWM_4_Port = {
    .port = GPIOB,
};

/* I2c 2 SCL H02 */
static BspGPIO_Port_TypeDef I2c2_SCL_Port = {
    .port = GPIOH,
};

/* I2C 2 SDA H03 */
static BspGPIO_Port_TypeDef I2c2_SDA_Port = {
    .port = GPIOH,
};

static BspIIC_PinConfig_TypeDef I2c2_BusPin = {
    .port_sck = &I2c2_SCL_Port,
    .port_sda = &I2c2_SDA_Port,
    .pin_sck = GPIO_PINS_2,
    .pin_sda = GPIO_PINS_3,

    .pin_Alternate = GPIO_MUX_4,
};

BspIICObj_TypeDef Baro_BusCfg = {
    .init = false,
    .Pin = &I2c2_BusPin,
    .instance_id = BspIIC_Instance_I2C_2,
};

void PriIMU_Dir_Tune(float *gyr, float *acc)
{
    double gyr_tmp[Axis_Sum] = {0.0};
    double acc_tmp[Axis_Sum] = {0.0};

    if (gyr && acc)
    {
        for (uint8_t i = Axis_X; i < Axis_Sum; i++)
        {
            gyr_tmp[i] = gyr[i];
            acc_tmp[i] = acc[i];
        }

        gyr[Axis_X] = -gyr_tmp[Axis_Y];
        gyr[Axis_Y] = -gyr_tmp[Axis_X];
        gyr[Axis_Z] = -gyr_tmp[Axis_Z];

        acc[Axis_X] = -acc_tmp[Axis_Y];
        acc[Axis_Y] = -acc_tmp[Axis_X];
        acc[Axis_Z] = -acc_tmp[Axis_Z];
    }
}

void *Baro_Bus_Instance = NULL;
BspGPIO_Obj_TypeDef *p_Baro_CS = NULL;

/* debug print port uart4 */
#define DEBUG_PORT_BAUDRATE 460800
#define DEBUG_P7_PORT UART7
#define DEBUG_P7_TX_PIN_INIT_STATE GPIO_PULL_NONE
#define DEBUG_P7_RX_PIN_INIT_STATE GPIO_PULL_NONE
#define DEBUG_P7_TX_PIN_ALT GPIO_MUX_8
#define DEBUG_P7_RX_PIN_ALT GPIO_MUX_8
#define DEBUG_P7_TX_DMA Bsp_DMA_None
#define DEBUG_P7_TX_DMA_STREAM Bsp_DMA_Stream_None
#define DEBUG_P7_RX_DMA Bsp_DMA_None
#define DEBUG_P7_RX_DMA_STREAM Bsp_DMA_Stream_None
#define DEBUG_P7_TX_PIN GPIO_PINS_4
#define DEBUG_P7_RX_PIN GPIO_PINS_3
#define DEBUG_P7_TX_PORT &Uart7_Tx_Port
#define DEBUG_P7_RX_PORT &Uart7_Rx_Port

static BspUARTObj_TypeDef Debug_Port_Obj = {
    .instance = DEBUG_P7_PORT,
    .baudrate = DEBUG_PORT_BAUDRATE,
    .tx_io = {
        .init_state = DEBUG_P7_TX_PIN_INIT_STATE,
        .pin = DEBUG_P7_TX_PIN,
        .port = DEBUG_P7_TX_PORT,
        .alternate = DEBUG_P7_TX_PIN_ALT,
    }, 
    .rx_io = {
        .init_state = DEBUG_P7_RX_PIN_INIT_STATE,
        .pin = DEBUG_P7_RX_PIN,
        .port = DEBUG_P7_RX_PORT,
        .alternate = DEBUG_P7_RX_PIN_ALT,
    }, 
    .pin_swap = false,
    .rx_dma = DEBUG_P7_RX_DMA,
    .rx_stream = DEBUG_P7_RX_DMA_STREAM,
    .tx_dma = DEBUG_P7_TX_DMA,
    .tx_stream = DEBUG_P7_TX_DMA_STREAM,
    .rx_buf = NULL,
    .rx_size = 0,
};

DebugPrintObj_TypeDef DebugPort = {
    .port_obj = &Debug_Port_Obj,
};

