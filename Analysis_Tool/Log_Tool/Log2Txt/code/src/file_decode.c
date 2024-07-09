#include "../inc/var_def.h"
#include "../inc/file_decode.h"
#include "minilzo.h"

/* create a 4M buffer */
static uint8_t decompess_file_buff[4 * 1024 * 1024] __attribute__((align(32))) = {0};
static decompess_io_stream decompess_stream = {.size = 0};

#define HEAP_ALLOC(var,size) \
    lzo_align_t __LZO_MMODEL var [ ((size) + (sizeof(lzo_align_t) - 1)) / sizeof(lzo_align_t) ]

static HEAP_ALLOC(wrkmem, LZO1X_1_MEM_COMPRESS);

uint32_t err = 0;
uint32_t done = 0;

static uint16_t LogFile_Decode_IMUData(FILE *cnv_file, uint8_t *data, uint16_t size);

decompess_io_stream *LogFile_Decompess_Init(LogFileObj_TypeDef *file)
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

    uint32_t stream_size = 0;
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

    uint32_t match_imu_frame_cnt = 0;

    /* init decompress module */
    if (lzo_init() != LZO_E_OK)
        return NULL;

    /* decompess file down below */
    for(uint32_t i = 0; i < file->logfile_size.total_byte; i++)
    {
        if(file->bin_data[i] == LOG_COMPESS_HEADER)
        {
            if(compess_header_cnt == 0)
                first_header_match = i;

            ((uint8_t *)&cur_pck_size)[0] = file->bin_data[i + 1];
            ((uint8_t *)&cur_pck_size)[1] = file->bin_data[i + 2];
            ((uint8_t *)&cur_pck_size)[2] = file->bin_data[i + 3];
            ((uint8_t *)&cur_pck_size)[3] = file->bin_data[i + 4];

            cur_header_pos = i;
            
            /* percheck ender */
            if((i + 5 + cur_pck_size) < file->logfile_size.total_byte)
            {
                if(file->bin_data[i + 5 + cur_pck_size] == LOG_COMPESS_ENDER)
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

        if(compess_header_cnt && (file->bin_data[i] == LOG_COMPESS_ENDER))
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
            
                memcpy(compess_buff, &file->bin_data[cur_header_pos + 5], check_pck_size);

                /* decompess data */
                if(lzo1x_decompress(compess_buff, check_pck_size, decompess_file_buff, &decompess_len, NULL) == LZO_E_OK)
                {
                    stream_size += decompess_len;
                    /* decode data */
                    for(uint32_t offset = 0; offset < decompess_len; offset += sizeof(LogData_Header_TypeDef) + sizeof(IMU_LogUnionData_TypeDef))
                    {
                        memcpy(&header, decompess_file_buff + offset, sizeof(LogData_Header_TypeDef));
                        memset(IMU_Data.buff, 0, sizeof(IMU_LogUnionData_TypeDef));
                        memcpy(IMU_Data.buff, decompess_file_buff + offset + sizeof(LogData_Header_TypeDef), sizeof(IMU_LogUnionData_TypeDef));

                        if((header.type == LOG_DATATYPE_IMU) && (header.size == LOG_IMU_DATA_SIZE))
                            match_imu_frame_cnt ++;

                                    done++;
                        fprintf(file->cnv_log_file, "%lld %f %f %f %f %f %f %f %f %f %f %f %f %lld\r\n",
                                IMU_Data.data.time,
                                (float)((int16_t)IMU_Data.data.org_gyr[Axis_X] / IMU_Data.data.gyr_scale),
                                (float)((int16_t)IMU_Data.data.org_gyr[Axis_Y] / IMU_Data.data.gyr_scale),
                                (float)((int16_t)IMU_Data.data.org_gyr[Axis_Z] / IMU_Data.data.gyr_scale),
                                (float)((int16_t)IMU_Data.data.org_acc[Axis_X] / IMU_Data.data.acc_scale),
                                (float)((int16_t)IMU_Data.data.org_acc[Axis_Y] / IMU_Data.data.acc_scale),
                                (float)((int16_t)IMU_Data.data.org_acc[Axis_Z] / IMU_Data.data.acc_scale),
                                (float)((int16_t)IMU_Data.data.flt_gyr[Axis_X] / IMU_Data.data.gyr_scale),
                                (float)((int16_t)IMU_Data.data.flt_gyr[Axis_Y] / IMU_Data.data.gyr_scale),
                                (float)((int16_t)IMU_Data.data.flt_gyr[Axis_Z] / IMU_Data.data.gyr_scale),
                                (float)((int16_t)IMU_Data.data.flt_acc[Axis_X] / IMU_Data.data.acc_scale),
                                (float)((int16_t)IMU_Data.data.flt_acc[Axis_Y] / IMU_Data.data.acc_scale),
                                (float)((int16_t)IMU_Data.data.flt_acc[Axis_Z] / IMU_Data.data.acc_scale),
                                IMU_Data.data.cyc);

                        printf("[INFO] Runtime %lld \t Cycle Cnt %d\r\n", IMU_Data.data.time, IMU_Data.data.cyc);
 
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
    decompess_stream.buff = decompess_file_buff;
    decompess_stream.size = stream_size;

    printf("\r\n");
    
    printf("[INFO]  test                          : %d\r\n", test);
    
    printf("[INFO]  Bad Ender                     : %d\r\n", bad_ender_cnt);
    printf("[INFO]  Total Lost Pack Number        : %d\r\n", pck_lost_cnt);
    printf("[INFO]  Decompess Pack Number         : %d\r\n", decompess_pck_cnt);

    printf("[INFO]  StartRunTime                  : %d\r\n", start_rt);
    printf("[INFO]  EndRunTime                    : %d\r\n", end_rt);

    printf("[INFO]  Total Byte                    : %lld\r\n", file->logfile_size.total_byte);
    printf("[INFO]  Compess Comput                : %lld\r\n", compess_pck_byte);
    printf("[INFO]  Decompess Error Num           : %d\r\n", decompess_err_cnt);

    printf("[INFO]  Error  Length Pack Num        : %d\r\n", err_pck_cnt);
    printf("[INFO]  Normal Length Pack Num        : %d\r\n", nor_pck_cnt);

    printf("[INFO]  First Match Compess Header At : %d\r\n", first_header_match);
    printf("[INFO]  First Match Compess Ender  At : %d\r\n", first_ender_match);

    printf("[INFO]  Match Compess Header          : %d\r\n", compess_header_cnt);
    printf("[INFO]  Match Compess Ender           : %d\r\n", compess_ender_cnt);

    printf("[INFO]  Match IMU Header              : %d\r\n", match_imu_frame_cnt);

    /* close convert file */
    fclose(file->cnv_log_file);

    return &decompess_stream;
}

