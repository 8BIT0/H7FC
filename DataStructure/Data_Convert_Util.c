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
        p_bytes[0] = ((uint8_t *)&target)[0];
        p_bytes[1] = ((uint8_t *)&target)[1];
        p_bytes[2] = ((uint8_t *)&target)[2];
        p_bytes[3] = ((uint8_t *)&target)[3];
        return true;
    }

    return false;
}

bool LEndianHalfWord2BytesArray(const uint16_t target, uint8_t *p_bytes)
{
    if (p_bytes)
    {
        p_bytes[0] = ((uint8_t *)&target)[0];
        p_bytes[1] = ((uint8_t *)&target)[1];
        return true;
    }

    return false;
}

/******************************************************* Big Endain Convert ****************************************************************/

uint16_t BEndian2HalfWord(const uint8_t *p_data)
{
    uint16_t tmp = 0;

    if (p_data == NULL)
        return 0;

    ((uint8_t *)&tmp)[0] = p_data[1];
    ((uint8_t *)&tmp)[1] = p_data[0];

    return tmp;
}

uint32_t BEndian2Word(const uint8_t *p_data)
{
    uint32_t tmp = 0;
    uint8_t byte_index = sizeof(uint32_t) - 1;

    if (p_data == NULL)
        return 0;

    for (uint8_t i = 0; i < sizeof(uint32_t); i++)
    {
        ((uint8_t *)&tmp)[i] = p_data[byte_index - i];
    }

    return tmp;
}