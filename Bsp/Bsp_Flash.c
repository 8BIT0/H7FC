#include "Bsp_Flash.h"

#define BSP_FLASH_ADDR_ALIGN_SIZE 4

#define BSP_FLASH_SECTOR_0
#define BSP_FLASH_SECTOR_1
#define BSP_FLASH_SECTOR_2
#define BSP_FLASH_SECTOR_3
#define BSP_FLASH_SECTOR_4
#define BSP_FLASH_SECTOR_5
#define BSP_FLASH_SECTOR_6
#define BSP_FLASH_SECTOR_7

/* external function */
static bool BspFlash_Init(void);
static bool BspFlash_Read_From_Addr(uint32_t addr, uint32_t *p_data, uint32_t size);
static bool BspFlash_Write_To_Addr(uint32_t addr, uint32_t *p_data, uint32_t size);
static bool BspFlash_Erase_Sector(uint32_t addr);

static uint32_t BspFlash_Get_Sectior(uint32_t addr)
{
    uint32_t sector = 0;

    if(addr)
    {

    }

    return sector;
}

static bool BspFlash_Init(void)
{
    return true;
}

static bool BspFlash_ReadWord(uint32_t addr, uint32_t *p_data)
{
    if(addr && ((addr % BSP_FLASH_ADDR_ALIGN_SIZE) == 0))
    {
        p_data = *(uint32_t *)addr;
        return true;
    }

    return false;
}

static bool BspFlash_Read_From_Addr(uint32_t addr, uint32_t *p_data, uint32_t size)
{
    if(addr && ((addr % BSP_FLASH_ADDR_ALIGN_SIZE) == 0) && p_data && size && ((size % BSP_FLASH_ADDR_ALIGN_SIZE) == 0))
    {
        for(uint32_t i = 0; i < size; i++)
        {
            if(BspFlash_ReadWord(addr, &p_data[i]))
            {
                addr += BSP_FLASH_ADDR_ALIGN_SIZE;
            }
            else
                return false;
        }

        return true;
    }

    return false;
}

static bool BspFlash_Write_To_Addr(uint32_t addr, uint32_t *p_data, uint32_t size)
{
    if(addr && ((addr % BSP_FLASH_ADDR_ALIGN_SIZE) == 0) && p_data && size && ((size % BSP_FLASH_ADDR_ALIGN_SIZE) == 0))
    {

    }

    return false;
}

static bool BspFlash_Erase_Sector(uint32_t addr)
{

}

