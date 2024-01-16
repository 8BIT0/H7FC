#include "Bsp_IIC.h"

#define To_IIC_Handle_Ptr(x) ((I2C_HandleTypeDef *)x)
#define To_IIC_PeriphCLKInitType(x) ((RCC_PeriphCLKInitTypeDef *)x)

/* internal vriable */
static I2C_HandleTypeDef* BspIIC_HandleList[BspIIC_Instance_I2C_Sum] = {0};

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

static bool BspIIC_Init(BspIICObj_TypeDef *obj)
{
    obj->init = false;
    BspGPIO_Obj_TypeDef sda_obj_tmp;
    BspGPIO_Obj_TypeDef sck_obj_tmp;

    memset(&sda_obj_tmp, 0, sizeof(sda_obj_tmp));
    memset(&sck_obj_tmp, 0, sizeof(sck_obj_tmp));

    if(obj && obj->Pin->port_sck && obj->Pin->port_sda)
    {
        if( (obj->PeriphClkInitStruct == NULL) || \
            (obj->handle == NULL))
            return false;

        switch((uint8_t)(obj->instance_id))
        {
            case BspIIC_Instance_I2C_2:
                To_IIC_PeriphCLKInitType(obj->PeriphClkInitStruct)->PeriphClockSelection = RCC_PERIPHCLK_I2C2;
                To_IIC_PeriphCLKInitType(obj->PeriphClkInitStruct)->I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
                if (HAL_RCCEx_PeriphCLKConfig(To_IIC_PeriphCLKInitType(obj->PeriphClkInitStruct)) != HAL_OK)
                    return false;
 
                sda_obj_tmp.alternate = obj->Pin->pin_Alternate;//GPIO_AF4_I2C2;
                sck_obj_tmp.alternate = obj->Pin->pin_Alternate;//GPIO_AF4_I2C2;

                sda_obj_tmp.pin = obj->Pin->pin_sda;
                sck_obj_tmp.pin = obj->Pin->pin_sck;

                sda_obj_tmp.port = obj->Pin->port_sda;
                sck_obj_tmp.port = obj->Pin->port_sck;

                BspGPIO.alt_init(sda_obj_tmp, GPIO_MODE_AF_OD);
                BspGPIO.alt_init(sck_obj_tmp, GPIO_MODE_AF_OD);

                __HAL_RCC_I2C2_CLK_ENABLE();
                
                To_IIC_Handle_Ptr(obj->handle)->Instance = I2C2;
                To_IIC_Handle_Ptr(obj->handle)->Init.Timing = 0x00702991;//0x10909CEC;
                To_IIC_Handle_Ptr(obj->handle)->Init.OwnAddress1 = 0;
                To_IIC_Handle_Ptr(obj->handle)->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
                To_IIC_Handle_Ptr(obj->handle)->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
                To_IIC_Handle_Ptr(obj->handle)->Init.OwnAddress2 = 0;
                To_IIC_Handle_Ptr(obj->handle)->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
                To_IIC_Handle_Ptr(obj->handle)->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
                To_IIC_Handle_Ptr(obj->handle)->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
                
                if (HAL_I2C_Init(To_IIC_Handle_Ptr(obj->handle)) != HAL_OK)
                    return false;

                if (HAL_I2CEx_ConfigAnalogFilter(To_IIC_Handle_Ptr(obj->handle), I2C_ANALOGFILTER_ENABLE) != HAL_OK)
                    return false;

                if (HAL_I2CEx_ConfigDigitalFilter(To_IIC_Handle_Ptr(obj->handle), 0) != HAL_OK)
                    return false;

                /* I2C2 interrupt Init */
                HAL_NVIC_SetPriority(I2C2_ER_IRQn, 5, 0);
                HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);

                BspIIC_HandleList[BspIIC_Instance_I2C_2] = To_IIC_Handle_Ptr(obj->handle);
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

static bool BspIIC_Read(BspIICObj_TypeDef *obj, uint16_t dev_addr, uint16_t reg, uint8_t *p_buf, uint16_t len)
{
    if(obj && p_buf && len)
    {
        if(HAL_I2C_Mem_Read(obj->handle, dev_addr, reg, I2C_MEMADD_SIZE_8BIT, p_buf, len, 100) == HAL_OK)
            return true;
    }

    return false;
}

static bool BspIIC_Write(BspIICObj_TypeDef *obj, uint16_t dev_addr, uint16_t reg, uint8_t *p_buf, uint16_t len)
{
    if(obj && p_buf && len)
    {
        if(HAL_I2C_Mem_Write(obj->handle, dev_addr, reg, I2C_MEMADD_SIZE_8BIT, p_buf, len, 100) == HAL_OK)
            return true;
    }

    return false;
}

void *BspIIC_Get_HandlePtr(BspIIC_Instance_List index)
{
    if((index > 0) && (index < BspIIC_Instance_I2C_Sum))
    {
        return BspIIC_HandleList[index];
    }

    return NULL;
}

