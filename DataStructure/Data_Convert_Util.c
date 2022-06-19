#include "Data_Convert_Util.h"

/******************************************************* Small Endain Convert **************************************************************/

uint32_t LEndian2Word(const uint8_t *ptr)
{
    uint32_t tmp = 0;

    tmp |= ptr[0];
    tmp |= ptr[1] << 8;
    tmp |= ptr[2] << 16;
    tmp |= ptr[3] << 24;

    return tmp;
}

uint16_t LEndian2HalfWord(const uint8_t *ptr)
{
    uint16_t tmp = 0;

    tmp |= ptr[0];
    tmp |= ptr[1] << 8;

    return tmp;
}

/******************************************************* Big Endain Convert ****************************************************************/

uint32_t BEndian2Word(const uint8_t *ptr)
{
    uint32_t tmp = 0;

    return tmp;
}

uint16_t BEndian2HalfWord(const uint8_t *ptr)
{
    uint16_t tmp = 0;

    return tmp;
}
