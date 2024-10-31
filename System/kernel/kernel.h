#ifndef __KERNEL_H
#define __KERNEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct
{
    uint8_t BitSize;
    uint8_t ByteSize;
    uint32_t UID[8];
} Kernel_UID_TypeDef;

void Kernel_reboot(void);
bool Kernel_Init(void);
bool Kernel_EnableTimer_IRQ(void);
bool Kernel_DisableTimer_IRQ(void);
bool Kernel_Set_PeriodValue(uint32_t value);
bool Kernel_Set_SysTimer_TickUnit(uint32_t unit);
uint32_t Kernel_Get_SysTimer_TickUnit(void);
uint32_t Kernel_Get_PeriodValue(void);
uint32_t Kernel_TickVal_To_Us(void);
Kernel_UID_TypeDef Kernel_Get_UID(void);

#ifdef __cplusplus
}
#endif

#endif
