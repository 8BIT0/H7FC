#ifndef __DATA_CONVERT_UTIL_H
#define __DATA_CONVERT_UTIL_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

uint32_t LEndian2Word(const uint8_t *ptr);
uint16_t LEndian2HalfWord(const uint8_t *ptr);
bool LEndianHalfWord2BytesArray(const uint16_t target, uint8_t *p_bytes);
bool LEndianWord2BytesArray(const uint32_t target, uint8_t *p_bytes);

uint32_t BEndian2Word(const uint8_t *ptr);
uint16_t BEndian2HalfWord(const uint8_t *ptr);

#endif
