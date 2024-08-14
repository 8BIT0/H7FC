#include "Bsp_GPIO.h"
#include "Bsp_IIC.h"
#include "at32f435_437.h"
#include "i2c_application.h"

#define I2C_TIMEOUT     100

#define I2Cx_CLK_10K    0xB170FFFF   //10K
#define I2Cx_CLK_50K    0xC0E06969   //50K
#define I2Cx_CLK_100K   0x80504C4E   //100K
#define I2Cx_CLK_200K   0x30F03C6B   //200K

/* external function */
static bool BspIIC_Init(BspIICObj_TypeDef *obj);
static bool BspIIC_DeInit(BspIICObj_TypeDef *obj);
static bool BspIIC_Read(BspIICObj_TypeDef *obj, uint16_t dev_addr, uint16_t reg, uint8_t *p_buf, uint16_t len);
static bool BspIIC_Write(BspIICObj_TypeDef *obj, uint16_t dev_addr, uint16_t reg, uint8_t *p_buf, uint16_t len);

BspIIC_TypeDef BspIIC = {
    .init = BspIIC_Init,
    .de_init = BspIIC_DeInit,
    .read = BspIIC_Read,
    .write = BspIIC_Write,
};

static bool BspIIC_Pin_Init(BspIICObj_TypeDef *obj)
{
    BspGPIO_Obj_TypeDef Pin;
    bool pin_state = false;

    memset(&Pin, 0, sizeof(BspGPIO_Obj_TypeDef));

    if (obj && \
        obj->Pin->port_sck && \
        obj->Pin->port_sda && \
        obj->Pin->pin_sck && \
        obj->Pin->pin_sda)
    {
        Pin.open_drain = true;
        Pin.alternate = obj->Pin->pin_Alternate;
        Pin.init_state = false;

        /* sck init */
        Pin.port = obj->Pin->port_sck;
        Pin.pin = obj->Pin->pin_sck;
        pin_state = BspGPIO.alt_init(Pin, 0);

        /* sda init */
        Pin.port = obj->Pin->port_sda;
        Pin.pin = obj->Pin->pin_sda;
        pin_state &= BspGPIO.alt_init(Pin, 0);
    }

    return pin_state;
}

static bool BspIIC_PeriphClk_Init(BspIICObj_TypeDef *obj)
{
    if (obj)
    {
        switch ((uint8_t)obj->instance_id)
        {
            case BspIIC_Instance_I2C_1:
                crm_periph_clock_enable(CRM_I2C1_PERIPH_CLOCK, TRUE);
                return true;

            case BspIIC_Instance_I2C_2:
                crm_periph_clock_enable(CRM_I2C2_PERIPH_CLOCK, TRUE);
                return true;

            case BspIIC_Instance_I2C_3:
                crm_periph_clock_enable(CRM_I2C3_PERIPH_CLOCK, TRUE);
                return true;

            default: return false;
        }
    }

    return false;
}

static bool BspIIC_Init(BspIICObj_TypeDef *obj)
{
    if(obj)
    {
        obj->handle = NULL;
        switch ((uint8_t)obj->instance_id)
        {
            case BspIIC_Instance_I2C_1: obj->handle = I2C1; break;
            case BspIIC_Instance_I2C_2: obj->handle = I2C2; break;
            case BspIIC_Instance_I2C_3: obj->handle = I2C3; break;
            default: return false;
        }

        if (!BspIIC_Pin_Init(obj))
            return false;
    
        if (!BspIIC_PeriphClk_Init(obj))
            return false;

        if (obj->handle == NULL)
            return false;

        i2c_reset((i2c_type *)obj->handle);
        i2c_init((i2c_type *)obj->handle, 0x0F, I2Cx_CLK_100K);
        i2c_own_address1_set((i2c_type *)obj->handle, I2C_ADDRESS_MODE_7BIT, obj->addr);
        i2c_enable((i2c_type *)obj->handle, TRUE);
        
        return true;
    }

    return false;
}

static bool BspIIC_DeInit(BspIICObj_TypeDef *obj)
{
    if(obj)
    {
        i2c_reset((i2c_type *)obj->handle);
        memset(obj, 0, sizeof(BspIICObj_TypeDef));
    }

    return false;
}

static bool BspIIC_Read(BspIICObj_TypeDef *obj, uint16_t dev_addr, uint16_t reg, uint8_t *p_buf, uint16_t len)
{
    if(obj && p_buf && len)
    {
        if (i2c_memory_read(obj->handle, I2C_MEM_ADDR_WIDIH_8, dev_addr, reg, p_buf, len, I2C_TIMEOUT) != I2C_OK)
            return false;

        return true;
    }

    return false;
}

static bool BspIIC_Write(BspIICObj_TypeDef *obj, uint16_t dev_addr, uint16_t reg, uint8_t *p_buf, uint16_t len)
{
    if(obj && p_buf && len)
    {
        if (i2c_memory_write(obj->handle, I2C_MEM_ADDR_WIDIH_8, dev_addr, reg, p_buf, len, I2C_TIMEOUT) != I2C_OK)
            return false;

        return true;
    }

    return false;
}
