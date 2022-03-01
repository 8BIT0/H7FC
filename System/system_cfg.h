#ifndef __SYSTEM_CFG_H
#define __SYSTEM_CFG_H

#define ITCM_CODE //__attribute__((section(".tcm_code")))

#define RUNTIEM_MAX_TICK_FRQ RUNTIME_TICK_FRQ_40K

#define KERNEL_STACK_SIZE 32 * 1024
#define EXTERNAL_RAM_MODULE 1
#define EXTERNAL_STORAGE_MODULE 1

#endif
