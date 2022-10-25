#ifndef __LOGFILE_H
#define __LOGFILE_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define BASE_SIZE_UNIT 1024
#define FILE_SIZE_K(x) (x * BASE_SIZE_UNIT)
#define FILE_SIZE_M(x) (x * FILE_SIZE_K(BASE_SIZE_UNIT))
#define CONVERT_EXTEND_FILE_NAME ".txt"

#define FILE_GET_B(x) (x % BASE_SIZE_UNIT)
#define FILE_GET_KB(x) ((x / FILE_SIZE_K(1)) % FILE_SIZE_M(1))
#define FILE_GET_MB(x) (x % FILE_SIZE_M(1))

#define MIN_LOG_FILENAME_LEN 5
#define EXTEND_FILETYPE_NAME ".log"
#define EXTEND_FILETYPE_NAME_LEN strlen(EXTEND_FILETYPE_NAME)

typedef struct
{
    uint64_t total_byte;
    uint16_t b;
    uint16_t kb;
    uint16_t mb;
} FileSize_TypeDef;

typedef struct
{
    FILE *log_file;
    FILE *cnv_log_file;

    char *path;
    char *file_name;

    FileSize_TypeDef logfile_size;
    FileSize_TypeDef cnvfile_size;

    uint64_t cs_error_cnt; /* check sum error count */
    uint64_t error_data_block_cnt;
    uint64_t decode_data_block_cnt;

    uint8_t *bin_data;
} LogFileObj_TypeDef;

#endif
