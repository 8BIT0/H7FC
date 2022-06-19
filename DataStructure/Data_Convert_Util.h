#ifndef __DATA_CONVERT_UTIL_H
#define __DATA_CONVERT_UTIL_H

#include <stdint.h>

uint32_t LEndian2Word(const uint8_t *ptr);
uint16_t LEndian2HalfWord(const uint8_t *ptr);

uint32_t BEndian2Word(const uint8_t *ptr);
uint16_t BEndian2HalfWord(const uint8_t *ptr);

#endif
