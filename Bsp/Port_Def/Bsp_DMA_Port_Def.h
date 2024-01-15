#ifndef __BSP_DMA_PORT_DEF_H
#define __BSP_DMA_PORT_DEF_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    bool (*regist)(int8_t dma, int8_t stream, void *obj);
    bool (*unregist)(int8_t dma, int8_t stream);
    void *(*get_handle)(int8_t dma, int8_t stream);
    void *(*get_instance)(int8_t dma, int8_t stream);
    void (*enable_irq)(int8_t dma, int8_t stream, uint32_t preempt, uint32_t sub);
} BspDMA_TypeDef;

#endif
