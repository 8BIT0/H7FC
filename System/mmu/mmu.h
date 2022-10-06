#ifndef __MMU_H
#define __MMU_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define BLOCK_ALIGMENT_SIZE 4
#define BLOCK_ALIGMENT_MASK BLOCK_ALIGMENT_SIZE - 1
#define PHY_MEM_SIZE 128 * 1024

#define MINIMUM_BLOCK_SIZE BLOCK_ALIGMENT_SIZE * 2

typedef uint32_t MemSize_t;
typedef uint32_t MemBlock_Addr;

/* memory block structure  */
typedef struct _MemBlock_TypeDef
{
    /* pointer to next free memory block */
    struct _MemBlock_TypeDef *nxtFree;

    /* request size */
    uint32_t size;
} MemBlock_TypeDef;

typedef struct
{
    MemSize_t phy_size;
    MemSize_t used_size;
    MemSize_t remain_size;
    MemSize_t total_size;

    MemBlock_TypeDef *FreeBlock;
    uint32_t FreeBlock_Num;

    uint32_t req_t;
    uint32_t fre_t;

    bool init;
} Mem_Monitor_TypeDef;

void *MMU_Malloc(uint16_t size);
void MMU_Free(void *ptr);
Mem_Monitor_TypeDef MMU_Get_Monitor(void);

#endif
