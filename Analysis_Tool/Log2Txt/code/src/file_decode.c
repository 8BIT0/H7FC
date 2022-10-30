#include "../inc/var_def.h"
#include "../inc/file_decode.h"

static uint16_t LogFile_Decode_IMUData(FILE *cnv_file, uint8_t *data, uint16_t size);

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
            i += LOG_HEADER_SIZE;
            file->decode_remain -= LOG_HEADER_SIZE;

            switch (header.type)
            {
            case LOG_DATATYPE_IMU:
                if ((uint8_t)header.size == LOG_IMU_DATA_SIZE)
                {
                    if (file->decode_remain >= LOG_IMU_DATA_SIZE)
                    {
                        if (LogFile_Decode_IMUData(file->cnv_log_file, &file->bin_data[i], file->decode_remain) > 0)
                        {
                            file->decode_remain -= LOG_IMU_DATA_SIZE;
                            i += LOG_IMU_DATA_SIZE - 1;
                        }
                    }
                    else
                        return true;
                }
                else
                {
                    file->error_data_block_cnt++;
                    file->abandon_byte_cnt += LOG_HEADER_SIZE;
                }
                break;

            default:
                file->error_data_block_cnt++;
                break;
            }
        }
    }

    /* close convert file */
    fclose(file->cnv_log_file);

    return true;
}

uint64_t err = 0;
uint64_t done = 0;

static uint16_t LogFile_Decode_IMUData(FILE *cnv_file, uint8_t *data, uint16_t size)
{
    uint16_t chk_sum = 0;
    IMU_LogUnionData_TypeDef IMU_Data;
    char log_string[512] = {'\0'};
    uint16_t log_size = 0;

    if ((cnv_file == NULL) || (data == NULL) || (size == 0) || (size < sizeof(IMU_Data)))
        return -1;

    memcpy(&IMU_Data, data, sizeof(IMU_Data));

    /* check sum */
    for (uint8_t i = 0; i < (sizeof(IMU_Data) - sizeof(uint16_t)); i++)
    {
        chk_sum += IMU_Data.buff[i];
    }

    if (chk_sum == IMU_Data.data.chk_sum)
    {
        printf("Rt: %lld\t Gyr[X]:%f\t Gyr[Y]:%f \tGyr[Z]:%f \tAcc[X]:%f \tAcc[Y]:%f \tAcc[Z]:%f\r\n",
               IMU_Data.data.time_stamp,
               IMU_Data.data.gyr[Axis_X],
               IMU_Data.data.gyr[Axis_Y],
               IMU_Data.data.gyr[Axis_Z],
               IMU_Data.data.acc[Axis_X],
               IMU_Data.data.acc[Axis_Y],
               IMU_Data.data.acc[Axis_Z]);

        done++;

        /* write convert file */
        fprintf(cnv_file, "%lld %f %f %f %f %f %f\r\n",
                IMU_Data.data.time_stamp,
                IMU_Data.data.gyr[Axis_X],
                IMU_Data.data.gyr[Axis_Y],
                IMU_Data.data.gyr[Axis_Z],
                IMU_Data.data.acc[Axis_X],
                IMU_Data.data.acc[Axis_Y],
                IMU_Data.data.acc[Axis_Z]);

        return sizeof(IMU_Data);
    }
    else
    {
        err++;
        return -1;
    }
}
