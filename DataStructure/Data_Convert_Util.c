#include "Data_Convert_Util.h"

/******************************************************* Small Endain Convert **************************************************************/

uint32_t LEndian2Word(const uint8_t *ptr)
{
    uint32_t tmp = 0;

    for (uint8_t i = 0; i < sizeof(uint32_t); i++)
    {
        ((uint8_t *)&tmp)[i] = ptr[sizeof(uint32_t) - i - 1];
    }

    return tmp;
}

uint16_t LEndian2HalfWord(const uint8_t *ptr)
{
    uint16_t tmp = 0;

    for (uint8_t i = 0; i < sizeof(uint16_t); i++)
    {
        ((uint8_t *)&tmp)[i] = ptr[sizeof(uint16_t) - i - 1];
    }

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
