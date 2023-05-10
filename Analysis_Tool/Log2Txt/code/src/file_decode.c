#include "../inc/var_def.h"
#include "../inc/file_decode.h"

uint32_t err = 0;
uint32_t done = 0;

static uint16_t LogFile_Decode_IMUData(FILE *cnv_file, uint8_t *data, uint16_t size);
// c:/Users/HUAV/Desktop/8B!T0/H7FC_V0.02/Analysis_Tool/Log2Txt/logfile/imu2.log
bool LogFile_Decode(LogFileObj_TypeDef *file)
{
    LogData_Header_TypeDef header;
    uint16_t offset = 0;
    uint64_t header_cnt = 0;
    uint64_t log_imu_cnt = 0;
    uint64_t frame_start = 0;
    uint64_t frame_end = 0;

    if ((file == NULL) || (file->bin_data == NULL) || (file->logfile_size.total_byte == 0))
        return false;

    /* create convert file */
    for (uint64_t i = 0; i < file->logfile_size.total_byte; i++)
    {
        if (file->bin_data[i] == LOG_HEADER)
        {
            if((file->bin_data[i + 1] == LOG_DATATYPE_IMU) && (file->bin_data[i + 2] == LOG_IMU_DATA_SIZE))
            {
                if(i - frame_start == LOG_IMU_DATA_SIZE)
                {
                    log_imu_cnt ++;
                    LogFile_Decode_IMUData(file->cnv_log_file, &file->bin_data[frame_start], LOG_IMU_DATA_SIZE);
                }

                frame_start = i + LOG_HEADER_SIZE;
                header_cnt ++;
            }
        }
        else
            file->decode_remain --;
    }

    printf("[INFO]\tfind %lld header\r\n", header_cnt);
    printf("[INFO]\tfind %lld log imu data\r\n", log_imu_cnt);
    printf("[INFO]\tDecode Success Count:%d\tError Count:%d\r\n", done, err);

    /* close convert file */
    fclose(file->cnv_log_file);

    return true;
}

static uint16_t LogFile_Decode_IMUData(FILE *cnv_file, uint8_t *data, uint16_t size)
{
    uint16_t chk_sum = 0;
    IMU_LogUnionData_TypeDef IMU_Data;
    uint16_t log_size = 0;

    if ((cnv_file == NULL) || (data == NULL) || (size == 0) || (size < sizeof(IMU_Data)))
        return -1;

    memcpy(IMU_Data.buff, data, sizeof(IMU_Data));

    /* check sum */
    for (uint8_t i = 0; i < (sizeof(IMU_Data) - sizeof(uint16_t)); i++)
    {
        chk_sum += IMU_Data.buff[i];
    }

    if (chk_sum == IMU_Data.data.chk_sum)
    {
        done++;
        fprintf(cnv_file, "%lld %lld %f %f %f %f %f %f %f %f %f %f %f %f\r\n",
                IMU_Data.data.time_stamp,
                IMU_Data.data.cycle_cnt,
                IMU_Data.data.org_gyr[Axis_X],
                IMU_Data.data.org_gyr[Axis_Y],
                IMU_Data.data.org_gyr[Axis_Z],
                IMU_Data.data.org_acc[Axis_X],
                IMU_Data.data.org_acc[Axis_Y],
                IMU_Data.data.org_acc[Axis_Z],
                IMU_Data.data.flt_gyr[Axis_X],
                IMU_Data.data.flt_gyr[Axis_Y],
                IMU_Data.data.flt_gyr[Axis_Z],
                IMU_Data.data.flt_acc[Axis_X],
                IMU_Data.data.flt_acc[Axis_Y],
                IMU_Data.data.flt_acc[Axis_Z]);

        return sizeof(IMU_Data);
    }
    else
    {
        err++;

        printf("[ ERROR DECODE ] %lld %lld\r\n",
                IMU_Data.data.time_stamp,
                IMU_Data.data.cycle_cnt);

        return -1;
    }
}
