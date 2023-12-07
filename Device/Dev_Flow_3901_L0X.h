#ifndef DEV_FLOW_3901_L0X_H
#define DEV_FLOW_3901_L0X_H

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#define MATEK_3901_L0X_BAUDRATE 115200

/* working range 2cm ~ 2m */

typedef enum
{
    DevFlow3901L0X_None_Error = 0,
    DevFlow3901L0X_ObjData_Error,
} DevFlow3901L0X_Error_List;

typedef struct
{
    uint32_t timestamp;
    float pos[3];
    uint8_t xy_quality;
    uint8_t z_quality;
} DevFlow3901L0XData_TypeDef;

typedef struct
{
    bool init;
    uint8_t err;

    uint32_t (*get_timestamp)(void);

    DevFlow3901L0XData_TypeDef data;

    uint16_t buf_total_size;
    uint16_t buf_cur_size;
    /* [ { timestamp (uint32_t) | flow data 1 } , { timestamp (uint32_t) | flow data 2 } , { | } , .......] */
    uint16_t *p_buf;
} DevFlow3901L0XObj_TypeDef;

typedef struct
{
    bool (*init)(DevFlow3901L0XObj_TypeDef *obj);
    bool (*push_data)(DevFlow3901L0XObj_TypeDef *obj, uint8_t *p_data, uint16_t len);
    DevFlow3901L0XData_TypeDef (*get_data)(DevFlow3901L0XObj_TypeDef obj);
} DevFlow3901L0X_TypeDef;

#endif
