#include "HW_Def.h"

BspGPIO_Port_TypeDef DBG_PC_Port = {
    .port = GPIOC,
};

DebugPinObj_TypeDef Debug_PC8 = {
    .port = (void *)&DBG_PC_Port,
    .pin = GPIO_PINS_8,
    .init_state = GPIO_PULL_NONE,
};

DebugPinObj_TypeDef Debug_PC0 = {
    .port = (void *)&DBG_PC_Port,
    .pin = GPIO_PINS_0,
    .init_state = GPIO_PULL_NONE,
};

/* Led 1 */
BspGPIO_Port_TypeDef Led1_Port = {
    .port = GPIOB,
};

BspGPIO_Obj_TypeDef Led1 = {
    .port = (void *)&Led1_Port,
    .pin = GPIO_PINS_5,
    .init_state = GPIO_PULL_NONE,
};

/* on board receiver uart tx port */
BspGPIO_Port_TypeDef Uart3_Tx_Port = {
    .port = GPIOB,
};

BspGPIO_Obj_TypeDef Uart3_TxPin = {
    .port = (void *)&Uart3_Tx_Port,
    .pin = GPIO_PINS_10,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_7,
};

/* on board receiver uart rx port */
BspGPIO_Port_TypeDef Uart3_Rx_Port = {
    .port = GPIOB,
};

BspGPIO_Obj_TypeDef Uart3_RxPin = {
    .port = (void *)&Uart3_Rx_Port,
    .pin = GPIO_PINS_11,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_7,
};

/* uart 1 tx pin */
BspGPIO_Port_TypeDef Uart1_Tx_Port = {
    .port = GPIOA,
};

BspGPIO_Obj_TypeDef Uart1_TxPin = {
    .port = (void *)&Uart1_Tx_Port,
    .pin = GPIO_PINS_9,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_7,
};

/* uart 1 rx pin */
BspGPIO_Port_TypeDef Uart1_Rx_Port = {
    .port = GPIOA,
};

BspGPIO_Obj_TypeDef Uart1_RxPin = {
    .port = (void *)&Uart1_Rx_Port,
    .pin = GPIO_PINS_10,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_7,
};

/* uart 6 tx port */
BspGPIO_Port_TypeDef Uart6_Tx_Port = {
    .port = GPIOC,
};

BspGPIO_Obj_TypeDef Uart6_TxPin = {
    .port = (void *)&Uart6_Tx_Port,
    .pin = GPIO_PINS_6,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_8,
};

/* uart 6 rx port */
BspGPIO_Port_TypeDef Uart6_Rx_Port = {
    .port = GPIOC,
};

BspGPIO_Obj_TypeDef Uart6_RxPin = {
    .port = (void *)&Uart6_Rx_Port,
    .pin = GPIO_PINS_7,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_8,
};

/* uart 4 tx pin */
BspGPIO_Port_TypeDef Uart4_Tx_Port = {
    .port = GPIOA,
};

BspGPIO_Obj_TypeDef Uart4_TxPin = {
    .port = (void *)&Uart4_Tx_Port,
    .pin = GPIO_PINS_0,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_8,
};

/* uart 4 rx pin */
BspGPIO_Port_TypeDef Uart4_Rx_Port = {
    .port = GPIOA,
};

BspGPIO_Obj_TypeDef Uart4_RxPin = {
    .port = (void *)&Uart4_Rx_Port,
    .pin = GPIO_PINS_1,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_8,
};

/* IMU CS Pin */
BspGPIO_Port_TypeDef PriIMU_CS_Port = {
    .port = GPIOA,
};

BspGPIO_Obj_TypeDef PriIMU_CSPin = {
    .init_state = GPIO_PULL_UP,
    .pin = GPIO_PINS_4,
    .port = &PriIMU_CS_Port,
};

/* IMU INT Pin */
BspGPIO_EXTI_Port_TypeDef PriIMU_INT_Port = {
    .port = SCFG_PORT_SOURCE_GPIOC,
};

BspGPIO_Obj_TypeDef PriIMU_INTPin = {
    .port = (void *)&PriIMU_INT_Port,
    .pin = GPIO_PINS_4,
    .init_state = EXINT_TRIGGER_FALLING_EDGE,
};

/* Baro Bus CS Pin */
BspGPIO_Port_TypeDef Baro_CS_Port = {
    .port = GPIOB,
};

BspGPIO_Obj_TypeDef Baro_CSPin = {
    .init_state = GPIO_PULL_UP,
    .pin = GPIO_PINS_3,
    .port = &Baro_CS_Port,
};

/* Baro Bus Pin (SPI3) */
BspSPI_NorModeConfig_TypeDef Baro_BusCfg = {
    .Instance = Baro_SPI_BUS,
    .CLKPolarity = SPI_CLOCK_POLARITY_HIGH,
    .CLKPhase = SPI_CLOCK_PHASE_2EDGE,
    .BaudRatePrescaler = SPI_MCLK_DIV_8,
    .work_mode = BspSPI_Mode_Master,
    .Pin = {
        .port_mosi = (void *)GPIOC,
        .port_miso = (void *)GPIOC,
        .port_clk = (void *)GPIOC,
        
        .pin_mosi = GPIO_PINS_12,
        .pin_miso = GPIO_PINS_11,
        .pin_clk = GPIO_PINS_10,

        .pin_Alternate = GPIO_MUX_6,
    },
};

/* IMU Bus Pin (SPI1) */
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

/* external flash chip cs */
BspGPIO_Port_TypeDef ExtFlash_CS_Port = {
    .port = GPIOB,
};

BspGPIO_Obj_TypeDef ExtFlash_CSPin = {
    .port = (void *)&ExtFlash_CS_Port,
    .pin = GPIO_PINS_12,
    .init_state = GPIO_PULL_UP,
};

/* external flash chip bus Pin (SPI2) */
BspSPI_PinConfig_TypeDef ExtFlash_SPIPin = {
    .port_mosi = (void *)GPIOB,
    .port_miso = (void *)GPIOB,
    .port_clk = (void *)GPIOB,
    
    .pin_mosi = GPIO_PINS_15,
    .pin_miso = GPIO_PINS_14,
    .pin_clk = GPIO_PINS_13,

    .pin_Alternate = GPIO_MUX_5,
};

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
    .port = GPIOA,
};

BspGPIO_Port_TypeDef PWM_5_Port = {
    .port = GPIOC,
};

BspGPIO_Port_TypeDef PWM_6_Port = {
    .port = GPIOA,
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
        gyr[Axis_Y] = gyr_tmp[Axis_X];
        gyr[Axis_Z] = gyr_tmp[Axis_Z];

        acc[Axis_X] = -acc_tmp[Axis_Y];
        acc[Axis_Y] = acc_tmp[Axis_X];
        acc[Axis_Z] = acc_tmp[Axis_Z];
    }
}

/* debug print port uart4 */
#define DEBUG_PORT_BAUDRATE 460800
#define DEBUG_P4_PORT UART4
#define DEBUG_P4_TX_PIN_INIT_STATE GPIO_PULL_NONE
#define DEBUG_P4_RX_PIN_INIT_STATE GPIO_PULL_NONE
#define DEBUG_P4_TX_PIN_ALT GPIO_MUX_8
#define DEBUG_P4_RX_PIN_ALT GPIO_MUX_8
#define DEBUG_P4_TX_DMA Bsp_DMA_None
#define DEBUG_P4_TX_DMA_STREAM Bsp_DMA_Stream_None
#define DEBUG_P4_RX_DMA Bsp_DMA_None
#define DEBUG_P4_RX_DMA_STREAM Bsp_DMA_Stream_None
#define DEBUG_P4_TX_PIN GPIO_PINS_0
#define DEBUG_P4_RX_PIN GPIO_PINS_1
#define DEBUG_P4_TX_PORT &Uart4_Tx_Port
#define DEBUG_P4_RX_PORT &Uart4_Rx_Port

static BspUARTObj_TypeDef Debug_Port4_Obj = {
    .instance = DEBUG_P4_PORT,
    .baudrate = DEBUG_PORT_BAUDRATE,
    .tx_io = {
        .init_state = DEBUG_P4_TX_PIN_INIT_STATE,
        .pin = DEBUG_P4_TX_PIN,
        .port = DEBUG_P4_TX_PORT,
        .alternate = DEBUG_P4_TX_PIN_ALT,
    }, 
    .rx_io = {
        .init_state = DEBUG_P4_RX_PIN_INIT_STATE,
        .pin = DEBUG_P4_RX_PIN,
        .port = DEBUG_P4_RX_PORT,
        .alternate = DEBUG_P4_RX_PIN_ALT,
    }, 
    .pin_swap = false,
    .rx_dma = DEBUG_P4_RX_DMA,
    .rx_stream = DEBUG_P4_RX_DMA_STREAM,
    .tx_dma = DEBUG_P4_TX_DMA,
    .tx_stream = DEBUG_P4_TX_DMA_STREAM,
    .rx_buf = NULL,
    .rx_size = 0,
};

DebugPrintObj_TypeDef DebugPort = {
    .port_obj = &Debug_Port4_Obj,
};
