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

bool LEndianWord2BytesArray(const uint32_t target, uint8_t *p_bytes)
{
    if (p_bytes)
    {
        p_bytes[0] = target & 0x000000FF;
        p_bytes[1] = (target & 0x0000FF00) >> 8;
        p_bytes[2] = (target & 0x00FF0000) >> 16;
        p_bytes[3] = (target & 0xFF000000) >> 24;
        return true;
    }

    return false;
}

bool LEndianHalfWord2BytesArray(const uint16_t target, uint8_t *p_bytes)
{
    if (p_bytes)
    {
        p_bytes[0] = target & 0x00FF;
        p_bytes[1] = (target & 0xFF00) >> 8;
        return true;
    }

    return false;
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
