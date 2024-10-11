/*
 * Auther: 8_B!T0
 * Reference by CmBacktrace
 * github link : https://github.com/armink/CmBacktrace
 */
#include "trace_analysiser.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug_util.h"
#include "HW_Def.h"

extern uint32_t _sstack;
extern uint32_t _estack;

static uint32_t trace_get_sp(void)
{
    register uint32_t result;
    __asm volatile ("MOV %0, sp\n" : "=r" (result) );
    return(result);
}

static uint32_t trace_get_psp(void)
{
    register uint32_t result;
    __asm volatile ("MRS %0, psp\n" : "=r" (result) );
    return(result);
}

static uint32_t trace_get_msp(void)
{
    register uint32_t result;
    __asm volatile ("MRS %0, msp\n" : "=r" (result) );
    return(result);
}

static bool trace_overflow(uint32_t p_stack, uint32_t stack_s_addr, uint32_t stack_depth)
{
    DEBUG_INFO("stack_addr : %08X\r\n", p_stack);
    DEBUG_INFO("stack_start: %08X\r\n", stack_s_addr);
    DEBUG_INFO("stack_end  : %08x\r\n", stack_depth + stack_s_addr);
    DEBUG_INFO("stack_depth: %d\r\n", stack_depth);

    p_stack += sizeof(size_t) * 8;

    if ((p_stack < stack_s_addr) || \
        (p_stack > (stack_depth + stack_s_addr)))
    {
        DEBUG_INFO("Stack Overflow\r\n");
        return true;
    }

    return false;
}

void trace_analysiser(uint32_t lr, uint32_t sp)
{
    DEBUG_INFO("HARD FAULT\r\n");
    volatile TaskHandle_t p_CurTCB = 0;
    volatile uint32_t err_stack = sp;
    volatile uint32_t cur_sp = trace_get_sp();
    volatile uint32_t msp = trace_get_msp();
    volatile uint32_t psp = trace_get_psp();

    /* check mode */
    if (lr & (1 << 2))
    {
        DEBUG_INFO("THREAD MODE\r\n");

        p_CurTCB = xTaskGetCurrentTaskHandle();

        /* check thread name */
        DEBUG_INFO("Thread Name: %s\r\n", pcTaskGetName(p_CurTCB));
        err_stack = psp;
        trace_overflow(err_stack, (uint32_t)xTaskGetCurrentTaskStack(p_CurTCB), xTaskGetCurrentTaskStackDeph(p_CurTCB));
    }
    else
    {
        DEBUG_INFO("MAIN STACK\r\n");
        trace_overflow(err_stack, _sstack, _estack);
    }

    DEBUG_INFO("sp      : %08x\r\n", sp);
    DEBUG_INFO("cur_sp  : %08x\r\n", cur_sp);
    DEBUG_INFO("psp     : %08x\r\n", psp);
    DEBUG_INFO("msp     : %08x\r\n", msp);

    /* dump regular register r0 r1 r2 r3 r12 lr pc psr */
    DEBUG_INFO("r0  : %08x\r\n", ((uint32_t *)err_stack)[0]);
    DEBUG_INFO("r1  : %08x\r\n", ((uint32_t *)err_stack)[1]);
    DEBUG_INFO("r2  : %08x\r\n", ((uint32_t *)err_stack)[2]);
    DEBUG_INFO("r3  : %08x\r\n", ((uint32_t *)err_stack)[3]);
    DEBUG_INFO("r12 : %08x\r\n", ((uint32_t *)err_stack)[4]);
        
    DEBUG_INFO("lr  : %08x\r\n", ((uint32_t *)err_stack)[5]);
    DEBUG_INFO("pc  : %08x\r\n", ((uint32_t *)err_stack)[6]);
    DEBUG_INFO("psr : %08x\r\n", ((uint32_t *)err_stack)[7]);
}

