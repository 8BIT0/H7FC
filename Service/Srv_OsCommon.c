#include "../FCHW_Config.h"
#include "Srv_OsCommon.h"
#include "kernel.h"
#include "HW_Def.h"
#if (SDRAM_EN == ON)
#include "Bsp_SDRAM.h"
#endif
#include "cmsis_gcc.h"
#include "shell_port.h"

typedef struct
{
    uint32_t available_size;
    uint32_t heap_remain;
    uint32_t block_num;

    uint32_t malloc_cnt;
    uint32_t malloc_failed_cnt;
    uint32_t free_cnt;
    uint32_t free_faile_cnt;

    bool sdram_state;
    uint32_t sdram_size;
    uint32_t sdram_base_addr;
    uint32_t ext_mem_size;
}SrvOsCommon_HeapMonitor_TypeDef;

/* internal vriable */
static bool first_call = true;
static SrvOsCommon_HeapMonitor_TypeDef OsHeap_Monitor = {0};

/* external function */
static void* SrvOsCommon_Malloc(uint32_t size);
static void SrvOsCommon_Free(void *ptr);
static void SrvOsCommon_Delay(uint32_t ms);
static void SrvOsCommon_DelayUntil(uint32_t *prev_time, uint32_t ms);
static bool SrvOsCommon_Init(void);

/* external vriable */
uint8_t ucHeap[ configTOTAL_HEAP_SIZE ] __attribute__((section(".OsHeap_Section")));

#if (SDRAM_EN == ON)
extern uint32_t __sdram_s1_s;
extern uint32_t __sdram_s1_e;

/* define heap_5 region */
const HeapRegion_t xHeapRegions[] = {
    {ucHeap, configTOTAL_HEAP_SIZE},
    {(uint8_t *)((uint32_t)&__sdram_s1_s), FC_SDRAM_Size}
};
#endif

SrvOsCommon_TypeDef SrvOsCommon = {
    .init = SrvOsCommon_Init,
    .get_os_ms = osKernelSysTick,
    .delay_ms = SrvOsCommon_Delay,
    .precise_delay = SrvOsCommon_DelayUntil,
    .malloc = SrvOsCommon_Malloc,
    .free = SrvOsCommon_Free,
    .enter_critical = vPortEnterCritical,
    .exit_critical = vPortExitCritical,
    .get_heap_status = vPortGetHeapStats,
    .get_systimer_current_tick = Kernel_Get_SysTimer_TickUnit,
    .get_systimer_period = Kernel_Get_PeriodValue,
    .set_systimer_tick_value = Kernel_Set_SysTimer_TickUnit,
    .set_systimer_period = Kernel_Set_PeriodValue,
    .systimer_tick_to_us = Kernel_TickVal_To_Us,
    .systimer_disable = Kernel_DisableTimer_IRQ,
    .systimer_enable = Kernel_EnableTimer_IRQ,
    .disable_all_irq = __disable_irq,
    .enable_all_irq = __enable_irq,
    .reboot = Kernel_reboot,
};

static bool SrvOsCommon_Init(void)
{
#if (SDRAM_EN == ON)
    bool state = false;
    BspSDRAMObj_TypeDef sdram_obj;

    vPortDefineHeapRegions(xHeapRegions);

    OsHeap_Monitor.sdram_state = false;
    sdram_obj.hdl = malloc(SDRAM_HandleType_Size);
    if (sdram_obj.hdl)
    {
        sdram_obj.mem_size      = FC_SDRAM_Size;
        sdram_obj.base_addr     = FC_SDRAM_Base_Addr;
        sdram_obj.bank_num      = BspSDRAM_BankNum_4;
        sdram_obj.bank_area     = BspSDRAM_Bank_1;
        sdram_obj.bus_width     = BspSDRAM_BusWidth_16;
        sdram_obj.column_bits   = BspSDRAM_Column_9Bits;
        sdram_obj.row_bits      = BspSDRAM_Row_13Bits;

        /* init sdram if have */
        if (BspSDRAM_Init(&sdram_obj))
        {
            /* sdram test */
            OsHeap_Monitor.sdram_state = true;
            OsHeap_Monitor.sdram_size = FC_SDRAM_Size;
            OsHeap_Monitor.sdram_base_addr = FC_SDRAM_Base_Addr;

            for (uint16_t i = 0; i < 10; i++)
            {
                *(volatile uint16_t *)(OsHeap_Monitor.sdram_base_addr + i * sizeof(uint16_t)) = i;
                if (*(volatile uint16_t *)(OsHeap_Monitor.sdram_base_addr + i * sizeof(uint16_t)) != i)
                {
                    state = false;
                    OsHeap_Monitor.sdram_state = false;
                    break;
                }

                /* reset value */
                *(volatile uint16_t *)(OsHeap_Monitor.sdram_base_addr + i * sizeof(uint16_t)) = 0;
            }
        }
    }

    return state;
#else
    return true;
#endif
}

static void* SrvOsCommon_Malloc(uint32_t size)
{
    void *req_tmp = NULL;
    SrvOs_HeapStatus_TypeDef status; 
    uint32_t malloc_num = 0;
    memset(&status, 0, sizeof(SrvOs_HeapStatus_TypeDef));

    /* check heap status first */
    vPortGetHeapStats(&status);
    malloc_num = status.xNumberOfSuccessfulAllocations;
    
    if(first_call)
    {
        memset(&OsHeap_Monitor, 0, sizeof(SrvOsCommon_HeapMonitor_TypeDef));
        first_call = false;
    }

    if(status.xAvailableHeapSpaceInBytes)
    {
        req_tmp = pvPortMalloc(size);
        
        /* recheck heap status after os heap malloc */
        vPortGetHeapStats(&status);

        OsHeap_Monitor.available_size = status.xAvailableHeapSpaceInBytes;
        OsHeap_Monitor.block_num = status.xNumberOfFreeBlocks;
    
        if(!req_tmp && (status.xNumberOfSuccessfulAllocations - malloc_num != 1))
        {
            req_tmp = NULL;
            OsHeap_Monitor.malloc_failed_cnt ++;
        }
        else
        {
            memset(req_tmp, 0, size);
            OsHeap_Monitor.malloc_cnt ++;
        }
    }

    return req_tmp;
}

static void SrvOsCommon_Free(void *ptr)
{
    uint32_t free_cnt = 0;
    SrvOs_HeapStatus_TypeDef status;

    memset(&status, 0, sizeof(SrvOs_HeapStatus_TypeDef));

    if(ptr)
    {
        vPortGetHeapStats(&status);
        free_cnt = status.xNumberOfSuccessfulFrees;
        
        vPortFree(ptr);
        
        /* check heap status after os heap free */
        vPortGetHeapStats(&status);
        OsHeap_Monitor.available_size = status.xAvailableHeapSpaceInBytes;
        OsHeap_Monitor.block_num = status.xNumberOfFreeBlocks;

        if(status.xNumberOfSuccessfulFrees - free_cnt == 1)
        {
            OsHeap_Monitor.free_cnt++;
            ptr = NULL;
        }
        else
            OsHeap_Monitor.free_faile_cnt ++;
    }
}

static void SrvOsCommon_Delay(uint32_t ms)
{
    osDelay(ms);
}

static void SrvOsCommon_DelayUntil(uint32_t *prev_time, uint32_t ms)
{
    if (prev_time && ms)
        osDelayUntil(prev_time, ms);
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, reboot, Kernel_reboot, System ReBoot);

