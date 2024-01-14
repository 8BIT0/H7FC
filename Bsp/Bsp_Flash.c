#include "Bsp_Flash.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_flash.h"

#define BSP_FLASH_ADDR_ALIGN_SIZE 4
#define BSP_FLASH_WRITE_UNIT 32 /* unit : byte */

/* internal function */
static bool BspFlash_ReadWord(uint32_t addr, uint8_t *p_data);
static bool BspFlash_Get_Sector(uint32_t addr, uint32_t *p_bank, uint32_t *p_sector);

/* external function */
static bool BspFlash_Init(void);
static void BspFlash_DeInit(void);
static bool BspFlash_Read_From_Addr(uint32_t addr, uint8_t *p_data, uint32_t size);
static bool BspFlash_Write_To_Addr(uint32_t addr, uint8_t *p_data, uint32_t size);
static bool BspFlash_Erase(uint32_t addr, uint32_t len);
static uint8_t BspFlash_Get_AlignSize(void);

BspFlash_TypeDef BspFlash = {
    .init = BspFlash_Init,
    .de_init = BspFlash_DeInit,
    .erase = BspFlash_Erase,
    .read = BspFlash_Read_From_Addr,
    .write = BspFlash_Write_To_Addr,
    .get_align_size = BspFlash_Get_AlignSize,
};

static bool BspFlash_Init(void)
{
    FLASH_OBProgramInitTypeDef ob_init;
    uint32_t bank_wrp_status = 0xFFF;
    
    /* Allow Access to option bytes sector */
    HAL_FLASH_OB_Unlock();
    
    /* Allow Access to Flash control registers and user Flash */
    HAL_FLASH_Unlock();
    
    /* Disable FLASH_WRP_SECTORS write protection */
    ob_init.OptionType = OPTIONBYTE_WRP;
    ob_init.Banks      = FLASH_BANK_1;
    ob_init.WRPState   = OB_WRPSTATE_DISABLE;
    ob_init.WRPSector  = OB_WRP_SECTOR_ALL;
    HAL_FLASHEx_OBProgram(&ob_init);
    /* Start the Option Bytes programming process */
    if (HAL_FLASH_OB_Launch() != HAL_OK)
        return false;
    
    /* Check if FLASH_WRP_SECTORS write protection is disabled */
    ob_init.Banks = FLASH_BANK_1;
    HAL_FLASHEx_OBGetConfig(&ob_init);
    bank_wrp_status = ob_init.WRPSector & OB_WRP_SECTOR_ALL;
    if (bank_wrp_status)
        /* Write  protection is not disabled */
        return false;
    
    /* Disable FLASH_WRP_SECTORS write protection */
    ob_init.OptionType = OPTIONBYTE_WRP;
    ob_init.Banks      = FLASH_BANK_2;
    ob_init.WRPState   = OB_WRPSTATE_DISABLE;
    ob_init.WRPSector  = OB_WRP_SECTOR_ALL;
    HAL_FLASHEx_OBProgram(&ob_init);
    /* Start the Option Bytes programming process */
    if (HAL_FLASH_OB_Launch() != HAL_OK)
        return false;
    
    /* Check if FLASH_WRP_SECTORS write protection is disabled */
    ob_init.Banks = FLASH_BANK_2;
    HAL_FLASHEx_OBGetConfig(&ob_init);
    bank_wrp_status = ob_init.WRPSector & OB_WRP_SECTOR_ALL;
    if (bank_wrp_status)
        /* Write  protection is not disabled */
        return false;
    
    /* Prevent Access to option bytes sector */
    HAL_FLASH_OB_Lock();
    
    return true;
}

static void BspFlash_DeInit(void)
{
    HAL_FLASH_Lock();
    HAL_FLASH_OB_Lock();
}

static bool BspFlash_Read_From_Addr(uint32_t addr, uint8_t *p_data, uint32_t size)
{
    uint8_t remain_size = 0;
    uint32_t read_tmp = 0;

    if(addr && ((addr % BSP_FLASH_ADDR_ALIGN_SIZE) == 0) && p_data && size)
    {
        for(uint32_t i = 0; i < size; i++)
        {
            p_data[i] = ((__IO uint8_t *)addr)[i];
            __DSB();
        }

        return true;
    }

    return false;
}

static bool BspFlash_Write_To_Addr(uint32_t addr, uint8_t *p_data, uint32_t size)
{
    uint8_t read_tmp[BSP_FLASH_WRITE_UNIT] = {0};
    uint8_t write_tmp[BSP_FLASH_WRITE_UNIT] = {0};
    uint8_t remain_size = 0;
    
    if(addr && ((addr % BSP_FLASH_WRITE_UNIT) == 0) && p_data && size)
    {
        if ((addr < FLASH_BASE_ADDR) || \
            (addr + size >= FLASH_BASE_ADDR + FLASH_SIZE))
            return false;

        for (uint32_t i = 0; i < (size / BSP_FLASH_WRITE_UNIT); i ++)
        {
            memcpy(write_tmp, p_data, BSP_FLASH_WRITE_UNIT);
            if (HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr, (uint32_t)p_data))
                return false;
            
            if(!BspFlash_Read_From_Addr(addr, read_tmp, BSP_FLASH_WRITE_UNIT))
                return false;

            if(memcmp(write_tmp, read_tmp, BSP_FLASH_WRITE_UNIT) != 0)
                return false;           

            addr += BSP_FLASH_WRITE_UNIT;
            p_data += BSP_FLASH_WRITE_UNIT;
        }

        remain_size = size % BSP_FLASH_WRITE_UNIT;
        if(remain_size)
        {
            if(!BspFlash_Read_From_Addr(addr, write_tmp, sizeof(write_tmp)))
                return false;

            memcpy(write_tmp, p_data, remain_size);
            if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr, write_tmp) != HAL_OK)
                return false;
        
            if(!BspFlash_Read_From_Addr(addr, read_tmp, sizeof(read_tmp)))
                return false;

            if(memcmp(write_tmp, read_tmp, BSP_FLASH_WRITE_UNIT) != 0)
                return false;
        }

        return true;
    }

    return false;
}

static bool BspFlash_Erase(uint32_t addr, uint32_t len)
{
    uint32_t PageError = 0;
    uint8_t sector_number;
    uint32_t erase_addr, erase_len;
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef eraseinitstruct;
    
    // if ((addr < FLASH_BASE_ADDR) || (addr + len >= FLASH_BASE_ADDR + FLASH_SIZE) || (addr - FLASH_BASE_ADDR) % FLASH_SECTOR_SIZE)
    if ((addr < FLASH_BASE_ADDR) || (addr + len >= FLASH_BASE_ADDR + FLASH_SIZE))
        return false;
    
    erase_addr = addr;
    erase_len = len;
    
    if (erase_addr < FLASH_BANK2_BASE && erase_addr + erase_len >= FLASH_BANK2_BASE)
    {
        erase_len = FLASH_BANK2_BASE - erase_addr;
        sector_number = erase_len / FLASH_SECTOR_SIZE;
        if (0 != erase_len % FLASH_SECTOR_SIZE)
        {
            sector_number += 1;
        }
        
        if (0 == BspFlash_Get_Sector(erase_addr, &eraseinitstruct.Banks, &eraseinitstruct.Sector))
            return false;

        eraseinitstruct.TypeErase = FLASH_TYPEERASE_SECTORS;
        eraseinitstruct.NbSectors = sector_number;
        eraseinitstruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
        
        if(HAL_FLASHEx_Erase(&eraseinitstruct, &PageError) != HAL_OK)
            return false;
        
        erase_addr = FLASH_BANK2_BASE;
        erase_len = len - erase_len;
    }
    
    sector_number = erase_len / FLASH_SECTOR_SIZE;
    if (0 != erase_len % FLASH_SECTOR_SIZE)
    {
        sector_number += 1;
    }
    
    if (!BspFlash_Get_Sector(erase_addr, &eraseinitstruct.Banks, &eraseinitstruct.Sector))
        return false;
    
    eraseinitstruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseinitstruct.NbSectors = sector_number;
    eraseinitstruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    
    if(HAL_FLASHEx_Erase(&eraseinitstruct, &PageError) != HAL_OK)
        return false;
    
    return true;
}

static uint8_t BspFlash_Get_AlignSize(void)
{
    return BSP_FLASH_ADDR_ALIGN_SIZE;
}

static bool BspFlash_Get_Sector(uint32_t addr, uint32_t *p_bank, uint32_t *p_sector)
{
    if(addr)
    {
        if ((addr < FLASH_BASE_ADDR) || \
            (addr >= FLASH_BASE_ADDR + FLASH_SIZE) || \
            (NULL == p_bank) || \
            (NULL == p_sector))
            return false;
        
        addr -= FLASH_BASE_ADDR;
        if (addr < FLASH_SECTOR_7_OFFSET_ADDR + FLASH_SECTOR_SIZE)
        {
            *p_bank = FLASH_BANK_1;
        }
        else
        {
            *p_bank = FLASH_BANK_2;
            addr -= (FLASH_SECTOR_7_OFFSET_ADDR + FLASH_SECTOR_SIZE);
        }
        
        if (addr < FLASH_SECTOR_1_OFFSET_ADDR)
        {
            *p_sector = FLASH_SECTOR_0;
        }
        else if ((addr >= FLASH_SECTOR_1_OFFSET_ADDR) && (addr < FLASH_SECTOR_2_OFFSET_ADDR))
        {
            *p_sector = FLASH_SECTOR_1;
        }
        else if ((addr >= FLASH_SECTOR_2_OFFSET_ADDR) && (addr < FLASH_SECTOR_3_OFFSET_ADDR))
        {
            *p_sector = FLASH_SECTOR_2;
        }
        else if ((addr >= FLASH_SECTOR_3_OFFSET_ADDR) && (addr < FLASH_SECTOR_4_OFFSET_ADDR))
        {
            *p_sector = FLASH_SECTOR_3;
        }
        else if ((addr >= FLASH_SECTOR_4_OFFSET_ADDR) && (addr < FLASH_SECTOR_5_OFFSET_ADDR))
        {
            *p_sector = FLASH_SECTOR_4;
        }
        else if ((addr >= FLASH_SECTOR_5_OFFSET_ADDR) && (addr < FLASH_SECTOR_6_OFFSET_ADDR))
        {
            *p_sector = FLASH_SECTOR_5;
        }
        else if ((addr >= FLASH_SECTOR_6_OFFSET_ADDR) && (addr < FLASH_SECTOR_7_OFFSET_ADDR))
        {
            *p_sector = FLASH_SECTOR_6;
        }
        else if ((addr >= FLASH_SECTOR_7_OFFSET_ADDR) && (addr < FLASH_SECTOR_7_OFFSET_ADDR + FLASH_SECTOR_SIZE))
        {
            *p_sector = FLASH_SECTOR_7;
        }

        return true;
    }

    return false;
}
