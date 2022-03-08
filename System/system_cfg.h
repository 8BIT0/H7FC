#ifndef __SYSTEM_CFG_H
#define __SYSTEM_CFG_H

#define ITCM_CODE //__attribute__((section(".tcm_code")))

#define US_IN_1S REAL_1S
#define SCHEDULER_TIMEBASE US_IN_1S

#define EXTERNAL_RAM_MODULE 1
#define EXTERNAL_STORAGE_MODULE 1

#endif
