#include "Bsp_Flash.h"
#include "at32f435_437_flash.h"

/* internal vriable */
uint8_t BspFlash_Cache_tmp[FLASH_SECTION_SIZE] = {0};
uint8_t BspFlash_Rx_Cache_tmp[FLASH_SECTION_SIZE] = {0};

/* internal function */
static uint32_t BspFlash_Get_Section_Addr(uint32_t addr);
static uint32_t BspFlash_Get_Section_Size(uint8_t sector_id);
static bool BspFlash_NoneCheck_Write(uint32_t addr, uint8_t *p_data, uint32_t len);

/* external function */
static bool BspFlash_Init(void);
static bool BspFlash_DeInit(void);
static bool BspFlash_Erase(uint32_t addr, uint32_t len);
static bool BspFlash_Read(uint32_t addr, uint8_t *p_data, uint32_t len);
static bool BspFlash_Write(uint32_t addr, uint8_t *p_data, uint32_t len);

BspFlash_TypeDef BspFlash = {
    .de_init = BspFlash_DeInit,
    .init = BspFlash_Init,
    .erase = BspFlash_Erase,
    .read = BspFlash_Read,
    .write = BspFlash_Write,
};

static bool BspFlash_Init(void)
{
    memset(BspFlash_Cache_tmp, 0, FLASH_SECTION_SIZE);
    return true;
}

static bool BspFlash_DeInit(void)
{
    return true;
}

static bool BspFlash_Erase(uint32_t addr, uint32_t len)
{
    flash_status_type status;
    uint32_t start_sec = 0;
    uint32_t end_sec = 0;
    uint8_t erase_cnt = 1;
    uint16_t write_offset = 0;
    uint16_t write_len = 0;

    if ((addr >= FLASH_BASE) && ((addr + len) < FLASH_BLOCK_END_ADDR))
    {
        start_sec = BspFlash_Get_Section_Addr(addr);
        end_sec = BspFlash_Get_Section_Addr(addr + len);
        erase_cnt += (end_sec - start_sec) / FLASH_SECTION_SIZE;
        if ((end_sec - start_sec) % FLASH_SECTION_SIZE)
            erase_cnt += 1;
        write_offset = addr - start_sec;

        if (write_offset)
        {
            write_len = FLASH_SECTION_SIZE - write_offset;
            if (!BspFlash_Read(end_sec, BspFlash_Cache_tmp, FLASH_SECTION_SIZE))
                return false;
        }

        flash_unlock();

        for (uint8_t i = 0; i < erase_cnt; i++)
        {
            /* wait for operation to be completed */
            status = flash_operation_wait_for(ERASE_TIMEOUT);
            
            if ((status == FLASH_PROGRAM_ERROR) || (status == FLASH_EPP_ERROR))
            {
                flash_flag_clear(FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);
            }
            else if (status == FLASH_OPERATE_TIMEOUT)
            {
                flash_lock();
                return false;
            }

            /* erase section first */
            status = flash_sector_erase(start_sec);
            if(status != FLASH_OPERATE_DONE)
            {
                flash_lock();
                return false;
            }

            start_sec = BspFlash_Get_Section_Addr(start_sec + FLASH_SECTION_SIZE);
        }

        if (write_offset)
        {
            if (!BspFlash_Write(end_sec + write_offset, BspFlash_Cache_tmp + write_offset, write_len))
            {
                flash_lock();
                return false;
            }
        }

        flash_lock();
        return true;
    }

    return false;
}

static bool BspFlash_Read(uint32_t addr, uint8_t *p_data, uint32_t len)
{
    if ((addr >= FLASH_BASE) && ((addr + len) <= FLASH_BLOCK_END_ADDR) && p_data && len)
    {
        for (uint32_t i = 0; i < len; i++)
        {
            p_data[i] = ((__IO uint8_t *)addr)[i];
            __DSB();
        }

        return true;
    }

    return false;
}

static bool BspFlash_Write(uint32_t addr, uint8_t *p_data, uint32_t len)
{
    uint32_t start_sec = 0;
    volatile uint32_t end_sec = 0;
    uint32_t write_cnt = 1;
    uint32_t write_offset = 0;
    uint32_t write_len = FLASH_SECTION_SIZE;
    flash_status_type status = FLASH_OPERATE_DONE;

    if ((addr >= FLASH_BASE) && ((addr + len) <= FLASH_BLOCK_END_ADDR) && p_data && len)
    {
        flash_unlock();

        start_sec = BspFlash_Get_Section_Addr(addr);
        end_sec = BspFlash_Get_Section_Addr(addr + len);

        write_cnt += (end_sec - start_sec) / FLASH_SECTION_SIZE;
        if ((end_sec - start_sec) % FLASH_SECTION_SIZE)
            write_cnt ++;

        /* read data from section first */
        if (!BspFlash_Read(start_sec, BspFlash_Cache_tmp, FLASH_SECTION_SIZE))
            return false;
 
        if (addr - start_sec)
            write_offset = addr - start_sec;

        for(uint32_t i = 0; i < write_cnt; i++)
        {
            memcpy(&BspFlash_Cache_tmp[write_offset], p_data, write_len);
            
            /* wait for operation to be completed */
            status = flash_operation_wait_for(ERASE_TIMEOUT);
            
            if((status == FLASH_PROGRAM_ERROR) || (status == FLASH_EPP_ERROR))
            {
                flash_flag_clear(FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);
            }
            else if(status == FLASH_OPERATE_TIMEOUT)
            {
                flash_lock();
                return false;
            }

            /* erase section first */
            status = flash_sector_erase(start_sec);
            if(status != FLASH_OPERATE_DONE)
            {
                flash_lock();
                return false;
            }

            /* write data */
            if (!BspFlash_NoneCheck_Write(start_sec, BspFlash_Cache_tmp, write_len))
            {
                flash_lock();
                return false;
            }
            else
            {
                BspFlash_Read(start_sec, BspFlash_Rx_Cache_tmp, write_len);

                if (memcmp(BspFlash_Rx_Cache_tmp, BspFlash_Cache_tmp, write_len) != 0)
                {
                    flash_lock();
                    return false;
                }
            }

            if (len <= write_len)
                break;

            /* update section addr */
            start_sec = BspFlash_Get_Section_Addr(start_sec + write_len);

            write_offset = 0;
            len -= write_len;
            p_data += write_len;

            if(len <= FLASH_SECTION_SIZE)
            {
                write_len = len;
            }
            else
                write_len = FLASH_SECTION_SIZE;
        }

        flash_lock();
        return true;
    }

    return false;
}

static bool BspFlash_NoneCheck_Write(uint32_t addr, uint8_t *p_data, uint32_t len)
{
    flash_status_type status = FLASH_OPERATE_BUSY;

    if ((addr >= FLASH_BASE) && ((addr + len) <= FLASH_BLOCK_END_ADDR) && p_data && len)
    {
        for(uint32_t i = 0; i < len; i ++)
        {
            status = flash_byte_program(addr, p_data[i]);
            while (status != FLASH_OPERATE_DONE)
            {
                status = flash_operation_status_get();
            }
            
            addr ++;
        }

        return true;
    }

    return false;
}

static uint32_t BspFlash_Get_Section_Addr(uint32_t addr)
{
    if ((addr < FLASH_BASE) || (addr > FLASH_BLOCK_END_ADDR))
        return 0;

    return (addr / FLASH_SECTION_SIZE) * FLASH_SECTION_SIZE;
}

static uint32_t BspFlash_Get_Section_Size(uint8_t sector_id)
{
    return FLASH_SECTION_SIZE;
}

static uint32_t BspFlash_Get_Align_Size(void)
{
    return FLASH_ALIGN_SIZE;
}
