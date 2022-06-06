#ifndef __SDCARD_H
#define __SDCARD_H

#include "stdbool.h"
#include "string.h"
#include "sdcard_standard.h"

#define SDCARD_TIMEOUT_INIT_MILLIS 200
#define SDCARD_MAX_CONSECUTIVE_FAILURES 8

typedef enum
{
    Card_Insert = 1,
    Card_NoFound = 0,
} SDCard_InsertState;

typedef enum
{
    SDCARD_BLOCK_OPERATION_READ,
    SDCARD_BLOCK_OPERATION_WRITE,
    SDCARD_BLOCK_OPERATION_ERASE
} sdcardBlockOperation_e;

typedef enum
{
    SDCARD_OPERATION_IN_PROGRESS,
    SDCARD_OPERATION_BUSY,
    SDCARD_OPERATION_SUCCESS,
    SDCARD_OPERATION_FAILURE
} sdcardOperationStatus_e;

typedef enum
{
    // In these states we run at the initialization 400kHz clockspeed:
    SDCARD_STATE_NOT_PRESENT = 0,
    SDCARD_STATE_RESET,
    SDCARD_STATE_CARD_INIT_IN_PROGRESS,
    SDCARD_STATE_INITIALIZATION_RECEIVE_CID,

    // In these states we run at full clock speed
    SDCARD_STATE_READY,
    SDCARD_STATE_READING,
    SDCARD_STATE_SENDING_WRITE,
    SDCARD_STATE_WAITING_FOR_WRITE,
    SDCARD_STATE_WRITING_MULTIPLE_BLOCKS,
    SDCARD_STATE_STOPPING_MULTIPLE_BLOCK_WRITE
} sdcardState_e;

typedef struct sdcardMetadata_s
{
    uint32_t numBlocks; /* Card capacity in 512-byte blocks*/
    uint16_t oemID;
    uint8_t manufacturerID;

    char productName[5];

    uint32_t productSerial;
    uint8_t productRevisionMajor;
    uint8_t productRevisionMinor;

    uint16_t productionYear;
    uint8_t productionMonth;
} sdcardMetadata_t;

typedef void (*sdcard_operationCompleteCallback_c)(sdcardBlockOperation_e operation, uint32_t blockIndex, uint8_t *buffer, uint32_t callbackData);
typedef void (*sdcard_profilerCallback_c)(sdcardBlockOperation_e operation, uint32_t blockIndex, uint32_t duration);

typedef struct
{
    struct
    {
        uint8_t *buffer;
        uint32_t blockIndex;
        uint8_t chunkIndex;

        sdcard_operationCompleteCallback_c callback;
        uint32_t callbackData;
    } pendingOperation;

    uint32_t operationStartTime;
    uint8_t failureCount;

    uint8_t version;
    bool highCapacity;

    uint32_t multiWriteNextBlock;
    uint32_t multiWriteBlocksRemain;

    sdcardState_e state;

    sdcardMetadata_t metadata;
    sdcardCSD_t csd;

#if (SDCARD_USE_SDIO)
    dmaIdentifier_e dmaIdentifier;
    uint8_t useCache;
#endif

    bool enabled;
    bool detectionInverted;
} SDCard_Dev_s;

typedef struct
{
    void (*sdcard_init)(void);
    bool (*sdcard_readBlock)(uint32_t blockIndex, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData);
    sdcardOperationStatus_e (*sdcard_beginWriteBlocks)(uint32_t blockIndex, uint32_t blockCount);
    sdcardOperationStatus_e (*sdcard_writeBlock)(uint32_t blockIndex, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData);
    bool (*sdcard_poll)(void);
    bool (*sdcard_isFunctional)(void);
    bool (*sdcard_isInitialized)(void);
    const sdcardMetadata_t *(*sdcard_getMetadata)(void);
} SDCard_FuncTable_s;

SDCard_InsertState SDCardDrv_InsertDetect(void);

bool sdcard_isInserted(void);
void sdcard_init(void);
bool sdcard_readBlock(uint32_t blockIndex, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData);
bool sdcard_beginWriteBlocks(uint32_t blockIndex, uint32_t blockCount);
bool sdcard_writeBlock(uint32_t blockIndex, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData);
bool sdcard_poll(void);
bool sdcard_isFunctional(void);
bool sdcard_isInitialized(void);
const sdcardMetadata_t *sdcard_getMetadata(void);
#endif
