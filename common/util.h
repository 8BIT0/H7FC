#ifndef __UTIL_H
#define __UTIL_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define Kb * 1024
#define Mb * (1024 Kb)

#define UTIL_SET_BIT(x) (1 << x)
#define DEG_2_REG(x) (x / 57.29578f)
#define REG_2_DEG(x) (x * 57.29578f)

int16_t Common_CRC16(const uint8_t *pBuf, const uint32_t len);
uint8_t Get_Bit_Index(uint16_t val);
uint8_t Get_OnSet_Bit_Num(uint32_t value);

#endif
