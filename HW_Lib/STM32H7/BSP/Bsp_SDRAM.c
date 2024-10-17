#include "Bsp_SDRAM.h"
#include "Bsp_GPIO.h"

#define To_SDRAM_Handle(x) ((SDRAM_HandleTypeDef *)x)

#define SDRAM_TIMEOUT                           ((uint32_t)0x1000)

#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000) 
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200) 

static uint32_t BspSDRAM_Get_BankArea(uint8_t bank_area)
{
    switch (bank_area)
    {
        case BspSDRAM_Bank_1: return FMC_SDRAM_BANK1;
        case BspSDRAM_Bank_2: return FMC_SDRAM_BANK2;
        default: return FMC_SDRAM_BANK2;
    }
}

static uint32_t BspSDRAM_Get_BankNum(uint8_t bank_num)
{
    switch (bank_num)
    {
        case BspSDRAM_BankNum_2: return FMC_SDRAM_INTERN_BANKS_NUM_2;
        case BspSDRAM_BankNum_4: return FMC_SDRAM_INTERN_BANKS_NUM_4;
        default: return FMC_SDRAM_INTERN_BANKS_NUM_4;
    }
}

static uint32_t BspSDRAM_Get_RowBits(uint8_t row_bits)
{
    switch (row_bits)
    {
        case BspSDRAM_Row_11Bits: return FMC_SDRAM_ROW_BITS_NUM_11;
        case BspSDRAM_Row_12Bits: return FMC_SDRAM_ROW_BITS_NUM_12;
        case BspSDRAM_Row_13Bits: return FMC_SDRAM_ROW_BITS_NUM_13;
        default: return FMC_SDRAM_ROW_BITS_NUM_11;
    }
}

static uint32_t BspSDRAM_Get_ColumnBits(uint8_t column_bits)
{
    switch (column_bits)
    {
        case BspSDRAM_Column_8Bits:  return FMC_SDRAM_COLUMN_BITS_NUM_8;
        case BspSDRAM_Column_9Bits:  return FMC_SDRAM_COLUMN_BITS_NUM_9;
        case BspSDRAM_Column_10Bits: return FMC_SDRAM_COLUMN_BITS_NUM_10;
        case BspSDRAM_Column_11Bits: return FMC_SDRAM_COLUMN_BITS_NUM_11;
        default: return FMC_SDRAM_COLUMN_BITS_NUM_8;
    }
}

static uint32_t BspSDRAM_Bus_Width(uint8_t bus_width)
{
    switch (bus_width)
    {
        case BspSDRAM_BusWidth_8:  return FMC_SDRAM_MEM_BUS_WIDTH_8;
        case BspSDRAM_BusWidth_16: return FMC_SDRAM_MEM_BUS_WIDTH_16;
        case BspSDRAM_BusWidth_32: return FMC_SDRAM_MEM_BUS_WIDTH_32;
        default: return FMC_SDRAM_MEM_BUS_WIDTH_8;
    }
}

__attribute__((weak)) void BspSDRAM_Pin_Init(void){return;}

bool BspSDRAM_Init(BspSDRAMObj_TypeDef *obj)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    FMC_SDRAM_TimingTypeDef SdramTiming = {0};
    FMC_SDRAM_CommandTypeDef Command;
    volatile uint32_t tmpmrd = 0;
    
    if ((obj == NULL) || \
        (obj->hdl == NULL) || \
        (obj->bank_num > BspSDRAM_BankNum_4) || \
        (obj->bank_area > BspSDRAM_Bank_2) || \
        (obj->bus_width > BspSDRAM_BusWidth_32) || \
        (obj->row_bits > BspSDRAM_Row_13Bits) || \
        (obj->column_bits > BspSDRAM_Column_11Bits))
        return false;

    obj->init_state = false;

    __HAL_RCC_FMC_CLK_ENABLE();
    
    memset(obj->hdl, 0, SDRAM_HandleType_Size);

    /* fmc gpio init */
    BspSDRAM_Pin_Init();

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FMC;
    PeriphClkInitStruct.FmcClockSelection = RCC_FMCCLKSOURCE_D1HCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
        return false;

	/* hsdram1.Init */
	To_SDRAM_Handle(obj->hdl)->Instance                = FMC_SDRAM_DEVICE;
	To_SDRAM_Handle(obj->hdl)->Init.SDBank             = BspSDRAM_Get_BankArea(obj->bank_area);
	To_SDRAM_Handle(obj->hdl)->Init.ColumnBitsNumber   = BspSDRAM_Get_ColumnBits(obj->column_bits);
	To_SDRAM_Handle(obj->hdl)->Init.RowBitsNumber      = BspSDRAM_Get_RowBits(obj->row_bits);
	To_SDRAM_Handle(obj->hdl)->Init.MemoryDataWidth    = BspSDRAM_Bus_Width(obj->bus_width);
	To_SDRAM_Handle(obj->hdl)->Init.InternalBankNumber = BspSDRAM_Get_BankNum(obj->bank_num);
	To_SDRAM_Handle(obj->hdl)->Init.CASLatency         = FMC_SDRAM_CAS_LATENCY_3;
	To_SDRAM_Handle(obj->hdl)->Init.WriteProtection    = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
	To_SDRAM_Handle(obj->hdl)->Init.SDClockPeriod      = FMC_SDRAM_CLOCK_PERIOD_2;
	To_SDRAM_Handle(obj->hdl)->Init.ReadBurst          = FMC_SDRAM_RBURST_ENABLE;
	To_SDRAM_Handle(obj->hdl)->Init.ReadPipeDelay      = FMC_SDRAM_RPIPE_DELAY_1;

	/* SdramTiming */
	SdramTiming.LoadToActiveDelay       = 2;
	SdramTiming.ExitSelfRefreshDelay    = 7;
	SdramTiming.SelfRefreshTime         = 4;
	SdramTiming.RowCycleDelay           = 7;
	SdramTiming.WriteRecoveryTime       = 2;
	SdramTiming.RPDelay                 = 2;
	SdramTiming.RCDDelay                = 2;
    
    if (HAL_SDRAM_Init(To_SDRAM_Handle(obj->hdl), &SdramTiming) != HAL_OK)
        return false;

    /* config sdram */
	/* Configure a clock configuration enable command */
	Command.CommandMode            = FMC_SDRAM_CMD_CLK_ENABLE;
	Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
	Command.AutoRefreshNumber      = 1;
	Command.ModeRegisterDefinition = 0;

	if (HAL_SDRAM_SendCommand(To_SDRAM_Handle(obj->hdl), &Command, SDRAM_TIMEOUT) != HAL_OK)
        return false;

	HAL_Delay(1);

	/* Configure a PALL (precharge all) command */ 
	Command.CommandMode            = FMC_SDRAM_CMD_PALL;		// 预充电命令
	Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;	// 选择要控制的区域
	Command.AutoRefreshNumber      = 1;
	Command.ModeRegisterDefinition = 0;

	if (HAL_SDRAM_SendCommand(To_SDRAM_Handle(obj->hdl), &Command, SDRAM_TIMEOUT) != HAL_OK)
        return false;

	/* Configure a Auto-Refresh command */ 
	Command.CommandMode 				= FMC_SDRAM_CMD_AUTOREFRESH_MODE;	// 使用自动刷新
	Command.CommandTarget 				= FMC_SDRAM_CMD_TARGET_BANK1;          // 选择要控制的区域
	Command.AutoRefreshNumber			= 8;                                // 自动刷新次数
	Command.ModeRegisterDefinition 	= 0;

	HAL_SDRAM_SendCommand(To_SDRAM_Handle(obj->hdl), &Command, SDRAM_TIMEOUT);	// 发送控制指令

	/* Program the external memory mode register */
	tmpmrd = (uint32_t)(SDRAM_MODEREG_BURST_LENGTH_2            | \
                        SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL     | \
                        SDRAM_MODEREG_CAS_LATENCY_3             | \
                        SDRAM_MODEREG_OPERATING_MODE_STANDARD   | \
                        SDRAM_MODEREG_WRITEBURST_MODE_SINGLE);

	Command.CommandMode            = FMC_SDRAM_CMD_LOAD_MODE;	// 加载模式寄存器命令
	Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;	// 选择要控制的区域
	Command.AutoRefreshNumber      = 1;
	Command.ModeRegisterDefinition = tmpmrd;

	if ((HAL_SDRAM_SendCommand(To_SDRAM_Handle(obj->hdl), &Command, SDRAM_TIMEOUT) != HAL_OK) || \
        (HAL_SDRAM_ProgramRefreshRate(To_SDRAM_Handle(obj->hdl), 918) != HAL_OK))
        return false;
	
    obj->init_state = true;
    return true;
}

