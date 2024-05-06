#include "Srv_OsCommon.h"
#include "kernel.h"
#include "cmsis_gcc.h"

typedef struct
{
    uint32_t available_size;
    uint32_t heap_remain;
    uint32_t block_num;

    uint32_t malloc_cnt;
    uint32_t malloc_failed_cnt;
    uint32_t free_cnt;
    uint32_t free_faile_cnt;
}SrvOsCommon_HeapMonitor_TypeDef;

/* internal vriable */
static bool first_call = true;
static SrvOsCommon_HeapMonitor_TypeDef OsHeap_Monitor = {0};

/* external vriable */
uint8_t ucHeap[ configTOTAL_HEAP_SIZE ] __attribute__((section(".OsHeap_Section")));

/* external function */
static void* SrvOsCommon_Malloc(uint32_t size);
static void SrvOsCommon_Free(void *ptr);
static int32_t SrvOsCommon_Delay(uint32_t ms);
static void SrvOsCommon_DelayUntil(uint32_t *prev_time, uint32_t ms);

SrvOsCommon_TypeDef SrvOsCommon = {
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
};

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

static int32_t SrvOsCommon_Delay(uint32_t ms)
{
    if (osDelay(ms) == osOK)
        return 1;

    return 0;
}

static void SrvOsCommon_DelayUntil(uint32_t *prev_time, uint32_t ms)
{
    if (prev_time && ms)
        osDelayUntil(prev_time, ms);
}
