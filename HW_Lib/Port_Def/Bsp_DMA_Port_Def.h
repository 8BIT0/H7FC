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

typedef void (*BspDMA_Pipe_TransFin_Cb)(void *hdl);
typedef void (*BspDMA_Pipe_TransErr_Cb)(void *hdl);

typedef struct
{
    bool (*init)(BspDMA_Pipe_TransFin_Cb fin_cb, BspDMA_Pipe_TransErr_Cb err_cb);
    bool (*trans)(uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
    void *(*get_hanle)(void);
} BspDMA_Pipe_TypeDef;

#endif
