#include "Bsp_IIC.h"

/* internal vriable */
static I2C_HandleTypeDef* BspIIC_HandleList[BspIIC_Instance_I2C_Sum] = {0};

/* external function */
static bool BspIIC_Init(BspIICObj_TypeDef *obj);
static bool BspIIC_DeInit(BspIICObj_TypeDef *obj);
static bool BspIIC_Read(BspIICObj_TypeDef *obj, uint32_t dev_addr, uint32_t reg, uint8_t *p_buf, uint16_t len);
static bool BspIIC_Write(BspIICObj_TypeDef *obj, uint32_t dev_addr, uint32_t reg, uint8_t *p_buf, uint16_t len);


BspIIC_TypeDef BspIIC = {
    .init = BspIIC_Init,
    .de_init = BspIIC_DeInit,
    .read = BspIIC_Read,
    .write = BspIIC_Write,
};

static bool BspIIC_Init(BspIICObj_TypeDef *obj)
{
    obj->init = false;
    BspGPIO_Obj_TypeDef sda_obj_tmp;
    BspGPIO_Obj_TypeDef sck_obj_tmp;

    memset(&sda_obj_tmp, 0, sizeof(sda_obj_tmp));
    memset(&sck_obj_tmp, 0, sizeof(sck_obj_tmp));

    if(obj && obj->Pin->port_sck && obj->Pin->port_sda)
    {
        switch((uint8_t)(obj->instance_id))
        {
            case BspIIC_Instance_I2C_2:
                obj->handle.Instance = I2C2;
                obj->handle.Init.Timing = 0x10909CEC;
                obj->handle.Init.OwnAddress1 = 0;
                obj->handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
                obj->handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
                obj->handle.Init.OwnAddress2 = 0;
                obj->handle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
                obj->handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
                obj->handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
                
                if (HAL_I2C_Init(&(obj->handle)) != HAL_OK)
                    return false;

                if (HAL_I2CEx_ConfigAnalogFilter(&(obj->handle), I2C_ANALOGFILTER_ENABLE) != HAL_OK)
                    return false;

                if (HAL_I2CEx_ConfigDigitalFilter(&(obj->handle), 0) != HAL_OK)
                    return false;

                obj->PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
                obj->PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
                if (HAL_RCCEx_PeriphCLKConfig(&(obj->PeriphClkInitStruct)) != HAL_OK)
                    return false;

                __HAL_RCC_I2C2_CLK_ENABLE();

                sda_obj_tmp.alternate = obj->Pin->pin_Alternate;//GPIO_AF4_I2C2;
                sck_obj_tmp.alternate = obj->Pin->pin_Alternate;//GPIO_AF4_I2C2;

                sda_obj_tmp.pin = obj->Pin->pin_sda;
                sck_obj_tmp.pin = obj->Pin->pin_sck;

                sda_obj_tmp.port = obj->Pin->port_sda;
                sck_obj_tmp.port = obj->Pin->port_sck;

                BspGPIO.alt_init(sda_obj_tmp, GPIO_MODE_AF_OD);
                BspGPIO.alt_init(sck_obj_tmp, GPIO_MODE_AF_OD);

                /* I2C2 interrupt Init */
                HAL_NVIC_SetPriority(I2C2_ER_IRQn, 5, 0);
                HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);

                BspIIC_HandleList[BspIIC_Instance_I2C_2] = &(obj->handle);
                obj->init = true;
                return true;

            default:
                return false;
        }
    }

    return false;
}

static bool BspIIC_DeInit(BspIICObj_TypeDef *obj)
{
    if(obj)
    {
        if(obj->init)
        {
            switch(obj->instance_id)
            {
                case BspIIC_Instance_I2C_2:
                    __HAL_RCC_I2C2_CLK_DISABLE();
                
                    HAL_GPIO_DeInit(obj->Pin->port_sck, obj->Pin->pin_sck);
                    HAL_GPIO_DeInit(obj->Pin->port_sda, obj->Pin->pin_sda);

                    HAL_NVIC_DisableIRQ(I2C2_ER_IRQn);
                    obj->init = false;
                    break;

                default:
                    return false;
            }
        }

        return true;
    }

    return false;
}

static bool BspIIC_Read(BspIICObj_TypeDef *obj, uint32_t dev_addr, uint32_t reg, uint8_t *p_buf, uint16_t len)
{
    if(obj && p_buf && len)
    {
        HAL_I2C_Mem_Read(&(obj->handle), dev_addr, reg, I2C_MEMADD_SIZE_8BIT, p_buf, len, 100);
    }

    return false;
}

static bool BspIIC_Write(BspIICObj_TypeDef *obj, uint32_t dev_addr, uint32_t reg, uint8_t *p_buf, uint16_t len)
{
    if(obj && p_buf && len)
    {
        HAL_I2C_Mem_Write(&(obj->handle), dev_addr, reg, I2C_MEMADD_SIZE_8BIT, p_buf, len, 100);
    }

    return false;
}

I2C_HandleTypeDef *BspIIC_Get_HandlePtr(BspIIC_Instance_List index)
{
    if((index > 0) && (index < BspIIC_Instance_I2C_Sum))
    {
        return BspIIC_HandleList[index];
    }

    return NULL;
}

