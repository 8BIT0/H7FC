#ifndef __FILE_DECODE_H
#define __FILE_DECODE_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "../inc/logfile.h"

#define DEFAULT_DECOMPESS_BUF_SIZE 1024
#define LOG_COMPESS_HEADER 0xCA
#define LOG_COMPESS_ENDER 0xED

typedef struct
{
    uint8_t *buff;
    uint16_t size;
}decompess_io_stream;

decompess_io_stream *LogFile_Decompess_Init(const LogFileObj_TypeDef file);
bool LogFile_Decode(decompess_io_stream *stream, LogFileObj_TypeDef *file);

#endif
