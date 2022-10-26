#include "../inc/var_def.h"
#include "../inc/file_decode.h"

static uint16_t LogFile_Decode_IMUData(uint8_t *data, uint16_t size);

bool LogFile_Decode(LogFileObj_TypeDef *file)
{
    LogData_Header_TypeDef header;

    if ((file == NULL) || (file->bin_data == NULL) || (file->logfile_size.total_byte == 0))
        return false;

    /* create convert file */

    for (uint64_t i = 0; i < file->logfile_size.total_byte; i++)
    {
        if (file->bin_data[i] == LOG_HEADER)
        {
            memcpy(&header, &file->bin_data[i], LOG_HEADER_SIZE);

            switch (header.type)
            {
            case LOG_DATATYPE_IMU:
                i += LOG_HEADER_SIZE;
                if ((uint8_t)header.size == LOG_IMU_DATA_SIZE)
                {
                    if (LogFile_Decode_IMUData(&file->bin_data[i], LOG_IMU_DATA_SIZE) != LOG_IMU_DATA_SIZE)
                    {
                    }
                }
                else
                {
                    printf("        %d         %d\r\n", header.size, LOG_IMU_DATA_SIZE);
                    file->error_data_block_cnt++;
                    file->abandon_byte_cnt += LOG_HEADER_SIZE;
                }
                break;

            default:
                file->error_data_block_cnt++;
                break;
            }
        }
        else
        {
            file->abandon_byte_cnt++;
        }
    }

    /* close convert file */

    return true;
}

static uint16_t LogFile_Decode_IMUData(uint8_t *data, uint16_t size)
{
    if ((data == NULL) || (size == 0))
        return -1;

    /* check sum */

    /* check value range */

    return size;
}
