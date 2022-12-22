#ifndef __DEV_DSHOT_H
#define __DEV_DSHOT_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

typedef enum
{
    DevDshot_Rotate_Invert = 9,
}DevDshot_Command_List; 

typedef enum
{
    DevDshot_300 = 1,
    DevDshot_600,
}DevDshotType_List;

typedef struct
{
    bool (*init)();
    bool (*control)();
}DevDshot_TypeDef;

#endif
