#include "../inc/var_def.h"
#include "../inc/file_decode.h"
#include "minilzo.h"

/* create a 64K buffer */
static uint8_t decompess_file_buff[4096] __attribute__((align(32))) = {0};
static decompess_io_stream decompess_stream = {.size = DEFAULT_DECOMPESS_BUF_SIZE};

#define HEAP_ALLOC(var,size) \
    lzo_align_t __LZO_MMODEL var [ ((size) + (sizeof(lzo_align_t) - 1)) / sizeof(lzo_align_t) ]

static HEAP_ALLOC(wrkmem, LZO1X_1_MEM_COMPRESS);

uint32_t err = 0;
uint32_t done = 0;

static uint16_t LogFile_Decode_IMUData(FILE *cnv_file, uint8_t *data, uint16_t size);

decompess_io_stream *LogFile_Decompess_Init(const LogFileObj_TypeDef file)
{
    uint32_t first_header_match = 0;
    uint32_t first_ender_match = 0;

    uint32_t compess_header_cnt = 0;
    uint32_t compess_ender_cnt = 0;

    uint32_t cur_header_pos = 0;

    uint32_t cur_pck_size = 0;
    uint32_t check_pck_size = 0;
    uint32_t err_pck_cnt = 0;
    uint32_t nor_pck_cnt = 0;

    uint32_t decompess_err_cnt = 0;
    uint32_t decompess_len = 0;

    uint8_t compess_buff[2048] __attribute__((align(32))) = {0};

    static uint8_t lst_cyc = 0;
    static uint64_t lst_Rt = 0;
    static uint64_t start_rt = 0;
    static uint64_t end_rt = 0;
    uint32_t test = 0;
    uint32_t pck_lost_cnt = 0;
    uint32_t decompess_pck_cnt = 0;

    uint32_t bad_ender_cnt = 0;
    uint64_t compess_pck_byte = 0;
    IMU_LogUnionData_TypeDef IMU_Data;
    LogData_Header_TypeDef header;

    /* init decompress module */
    if (lzo_init() != LZO_E_OK)
        return NULL;

    /* decompess file down below */
    for(uint32_t i = 0; i < file.logfile_size.total_byte; i++)
    {
        if(file.bin_data[i] == LOG_COMPESS_HEADER)
        {
            if(compess_header_cnt == 0)
                first_header_match = i;

            ((uint8_t *)&cur_pck_size)[0] = file.bin_data[i + 1];
            ((uint8_t *)&cur_pck_size)[1] = file.bin_data[i + 2];
            ((uint8_t *)&cur_pck_size)[2] = file.bin_data[i + 3];
            ((uint8_t *)&cur_pck_size)[3] = file.bin_data[i + 4];

            cur_header_pos = i;
            
            /* percheck ender */
            if((i + 5 + cur_pck_size) < file.logfile_size.total_byte)
            {
                if(file.bin_data[i + 5 + cur_pck_size] == LOG_COMPESS_ENDER)
                {
                    i += 5 + cur_pck_size;
                    compess_pck_byte += 5 + cur_pck_size;
                }
                else
                {
                    bad_ender_cnt++;
                }
            }
            else
                goto Decode_EOF;

            compess_header_cnt ++;
        }

        if(compess_header_cnt && (file.bin_data[i] == LOG_COMPESS_ENDER))
        {
            if(compess_header_cnt == 1)
                first_ender_match = i;
            
            check_pck_size = i - cur_header_pos - 5;

            // printf("[INFO] Pack Length check_len: %d\tpck_len: %d\r\n", check_pck_size, cur_pck_size);
            if(check_pck_size != cur_pck_size)
            {
                err_pck_cnt ++;
            }
            else
            {
                nor_pck_cnt ++;
            
                memcpy(compess_buff, &file.bin_data[cur_header_pos + 5], check_pck_size);

                /* decompess data */
                if(lzo1x_decompress(compess_buff, check_pck_size, decompess_file_buff, &decompess_len, NULL) == LZO_E_OK)
                {
                    /* decode data */
                    for(uint16_t offset = 0; offset < decompess_len; offset += sizeof(LogData_Header_TypeDef) + sizeof(IMU_LogUnionData_TypeDef))
                    {
                        memcpy(&header, decompess_file_buff + offset, sizeof(LogData_Header_TypeDef));
                        memset(IMU_Data.buff, 0, sizeof(IMU_LogUnionData_TypeDef));
                        memcpy(IMU_Data.buff, decompess_file_buff + offset + sizeof(LogData_Header_TypeDef), sizeof(IMU_LogUnionData_TypeDef));

                        // printf("[INFO] Runtime %lld \t Cycle Cnt %d\r\n", IMU_Data.data.time, IMU_Data.data.cyc);
                    
                        if(lst_Rt)
                        {
                            if(((IMU_Data.data.time - lst_Rt) > 500) && ((IMU_Data.data.cyc - lst_cyc) > 1))
                            {
                                test ++;

                                printf("[INFO]  Occur Pack Index %d\r\n", nor_pck_cnt);
                                printf("[INFO]  Current Decompess Size: %lld\r\n", compess_pck_byte);
                                printf("[INFO]  runtime:\t\t%d\ttime_diff:\t\t%d\r\n", IMU_Data.data.time, IMU_Data.data.time - lst_Rt);
                                printf("[INFO]  cycle:\t\t\t%d\tCycle_diff:\t\t%d\r\n", IMU_Data.data.cyc, IMU_Data.data.cyc - lst_cyc);
                                printf("\r\n");

                                pck_lost_cnt += IMU_Data.data.cyc - lst_cyc;
                            }
                            else
                                decompess_pck_cnt ++;
                        }
                        else
                            start_rt = IMU_Data.data.time;

                        end_rt = IMU_Data.data.time;
                        lst_Rt = IMU_Data.data.time;
                        lst_cyc = IMU_Data.data.cyc;
                    }
                }
                else
                    decompess_err_cnt ++;
            }

            compess_ender_cnt ++;
        }
    }

Decode_EOF:
    printf("\r\n");
    
    printf("[INFO]  test                          : %d\r\n", test);
    
    printf("[INFO]  Bad Ender                     : %d\r\n", bad_ender_cnt);
    printf("[INFO]  Total Lost Pack Number        : %d\r\n", pck_lost_cnt);
    printf("[INFO]  Decompess Pack Number         : %d\r\n", decompess_pck_cnt);

    printf("[INFO]  StartRunTime                  : %d\r\n", start_rt);
    printf("[INFO]  EndRunTime                    : %d\r\n", end_rt);

    printf("[INFO]  Total Byte                    : %lld\r\n", file.logfile_size.total_byte);
    printf("[INFO]  Compess Comput                : %lld\r\n", compess_pck_byte);
    printf("[INFO]  Decompess Error Num           : %d\r\n", decompess_err_cnt);

    printf("[INFO]  Error  Length Pack Num        : %d\r\n", err_pck_cnt);
    printf("[INFO]  Normal Length Pack Num        : %d\r\n", nor_pck_cnt);

    printf("[INFO]  First Match Compess Header At : %d\r\n", first_header_match);
    printf("[INFO]  First Match Compess Ender  At : %d\r\n", first_ender_match);

    printf("[INFO]  Match Compess Header          : %d\r\n", compess_header_cnt);
    printf("[INFO]  Match Compess Ender           : %d\r\n", compess_ender_cnt);

    return &decompess_stream;
}

bool LogFile_Decode(decompess_io_stream *stream, LogFileObj_TypeDef *file)
{
    LogData_Header_TypeDef header;
    uint16_t offset = 0;
    uint64_t imu_header_cnt = 0;
    uint64_t log_imu_cnt = 0;
    uint64_t imU_data_start = 0;

    if ((stream == NULL) || (stream->size == 0))
        return false;

    /* create convert file */
    for (uint64_t i = 0; i < stream->size; i++)
    {
        if (stream->buff[i] == LOG_HEADER)
        {
            if((stream->buff[i + 1] == LOG_DATATYPE_IMU) && (stream->buff[i + 2] == LOG_IMU_DATA_SIZE))
            {
                if(i - imU_data_start >= LOG_IMU_DATA_SIZE)
                {
                    log_imu_cnt ++;
                    LogFile_Decode_IMUData(file->cnv_log_file, &stream->buff[imU_data_start], LOG_IMU_DATA_SIZE);
                }

                imU_data_start = i + LOG_HEADER_SIZE;
                imu_header_cnt ++;
            }
        }
        else
            file->decode_remain --;
    }

    printf("[INFO]\tfind %lld imu header\r\n", imu_header_cnt);
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

    // if (chk_sum == IMU_Data.data.chk_sum)
    // {
    //     done++;
    //     fprintf(cnv_file, "%f %f %f %f %f %f %f %f %f %f %f %f %f %lld\r\n",
    //             IMU_Data.data.time_stamp / 1000.0f,
    //             IMU_Data.data.org_gyr[Axis_X],
    //             IMU_Data.data.org_gyr[Axis_Y],
    //             IMU_Data.data.org_gyr[Axis_Z],
    //             IMU_Data.data.org_acc[Axis_X],
    //             IMU_Data.data.org_acc[Axis_Y],
    //             IMU_Data.data.org_acc[Axis_Z],
    //             IMU_Data.data.flt_gyr[Axis_X],
    //             IMU_Data.data.flt_gyr[Axis_Y],
    //             IMU_Data.data.flt_gyr[Axis_Z],
    //             IMU_Data.data.flt_acc[Axis_X],
    //             IMU_Data.data.flt_acc[Axis_Y],
    //             IMU_Data.data.flt_acc[Axis_Z],
    //             IMU_Data.data.cycle_cnt);

    //     return sizeof(IMU_Data);
    // }
    // else
    // {
    //     err++;

    //     printf("[ ERROR DECODE ] %lld %lld\r\n",
    //             IMU_Data.data.time_stamp,
    //             IMU_Data.data.cycle_cnt);

    //     return -1;
    // }
}
