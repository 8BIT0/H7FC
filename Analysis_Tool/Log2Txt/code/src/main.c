#include "../inc/var_def.h"
#include "../inc/logfile.h"
#include "../inc/file_decode.h"
#include <sys/stat.h>

#define MAX_LOAD_MB_SIZE 64

static LogFileObj_TypeDef LogFile;

static bool File_Check_ExtendType(char *ext_type)
{
    char *ext;

    if (ext_type)
    {
        if ((ext_type[0] == '\'') && (ext_type[strlen(ext_type) - 1] == '\''))
        {
            ext_type[strlen(ext_type) - 1] = '\0';

            sscanf(ext_type, "'%s'", ext_type);
            ext_type[strlen(ext_type)] = '\0';
        }

        ext = ext_type + strlen(ext_type) - EXTEND_FILETYPE_NAME_LEN;

        if (memcmp(ext, EXTEND_FILETYPE_NAME, strlen(EXTEND_FILETYPE_NAME)) != 0)
            printf("[Error]\tFile Type Error\r\n");

        return true;
    }

    printf("\r\n");

    return false;
}

static bool Load_File(char *path, LogFileObj_TypeDef *obj)
{
    uint16_t file_path_len = 0;
    struct stat file_stat;
    bool state = false;
    uint16_t offset = 0;
    char path_tmp[128] = "";
    char *logfile_name_tmp = NULL;
    char *cnvfile_name_tmp = NULL;
    char cnvfile_path[1024] = "";

    if (path && obj)
    {
        file_path_len = strlen(path);

        if ((file_path_len > MIN_LOG_FILENAME_LEN) && File_Check_ExtendType(path))
        {
            /* open log file */
            obj->log_file = fopen(path, "rb");

            if (obj->log_file)
            {
                /* separate file name and path */
                for (uint16_t i = 0; i < strlen(path); i++)
                {
                    if (path[i] == '/')
                        offset = i + 1;
                }

                memset(path_tmp, '\0', offset);
                memcpy(path_tmp, path, offset);

                logfile_name_tmp = malloc(strlen(path) - offset);
                cnvfile_name_tmp = malloc(strlen(path) - offset - strlen(EXTEND_FILETYPE_NAME) + strlen(CONVERT_EXTEND_FILE_NAME));

                memset(logfile_name_tmp, '\0', strlen(path) - offset);
                memset(cnvfile_name_tmp, '\0', strlen(path) - offset - strlen(EXTEND_FILETYPE_NAME) + strlen(CONVERT_EXTEND_FILE_NAME));

                memcpy(logfile_name_tmp, path + offset, strlen(path) - offset);
                memcpy(cnvfile_name_tmp, path + offset, strlen(path) - offset - strlen(EXTEND_FILETYPE_NAME));
                memcpy(cnvfile_name_tmp + strlen(CONVERT_EXTEND_FILE_NAME) - 1, CONVERT_EXTEND_FILE_NAME, strlen(CONVERT_EXTEND_FILE_NAME));

                obj->path = path_tmp;
                obj->log_file_name = logfile_name_tmp;
                obj->cnv_file_name = cnvfile_name_tmp;

                /* read log data into buff */
                stat(path, &file_stat);
                obj->logfile_size.total_byte = file_stat.st_size;
                obj->decode_remain = file_stat.st_size;
                obj->logfile_size.b = FILE_GET_B(obj->logfile_size.total_byte);
                obj->logfile_size.kb = FILE_GET_KB(obj->logfile_size.total_byte);
                obj->logfile_size.mb = FILE_GET_MB(obj->logfile_size.total_byte);

                memcpy(cnvfile_path, obj->path, strlen(obj->path));
                memcpy(cnvfile_path + strlen(obj->path), obj->cnv_file_name, strlen(obj->cnv_file_name));

                printf("\r\n");
                printf("[INFO]\tFile Path\t\t\t%s\r\n", obj->path);
                printf("[INFO]\tConvert File path:\t\t%s\r\n", cnvfile_path);

                printf("[INFO]\tLogFile Name:\t\t\t%s\r\n", obj->log_file_name);
                printf("[INFO]\tCNVFile Name:\t\t\t%s\r\n", obj->cnv_file_name);
                printf("[INFO]\tFile Total Byte Size:\t\t%lld\r\n", obj->logfile_size.total_byte);
                printf("\r\n");

                /* create convert file */
                obj->cnv_log_file = fopen(cnvfile_path, "w");

                if (!obj->cnv_log_file)
                {
                    printf("[ERROR]\tConvert File Create Error\r\n");
                    return false;
                }
                else
                    printf("[INFO]\tConvert File Create Success\r\n");

                if (obj->logfile_size.mb < MAX_LOAD_MB_SIZE)
                {
                    obj->bin_data = malloc(obj->logfile_size.total_byte);

                    if (obj->bin_data == NULL)
                    {
                        free(obj->bin_data);
                        printf("[Error]\tMemory Malloc Failed\r\n");
                        printf("\r\n\r\n");
                    }
                    else
                    {
                        /* Load All Data */
                        for (uint64_t i = 0; i < obj->logfile_size.total_byte; i++)
                        {
                            fscanf(obj->log_file, "%c", &obj->bin_data[i]);
                        }

                        printf("[INFO]\tFile Load Finished\r\n");
                        if (fclose(obj->log_file) == 0)
                        {
                            printf("[INFO]\tFile Closed\r\n");
                        }
                        else
                        {
                            printf("[Error]\tFile Close Error\r\n");
                            state = false;
                        }
                    }
                }

                state = true;
            }
            else
            {
                printf("[Error]\tOpen Target File Error :%s\r\n", path);
                printf("\r\n\r\n");
            }
        }

        memset(path, '\0', strlen(path));
    }

    return state;
}

int main()
{
    char path[1024] = "";
    memset(&LogFile, 0, sizeof(LogFile));

    while (!Load_File(path, &LogFile))
    {
        /* waiting for path input */
        printf("[INFO]\tType LogFile Path : \t");
        scanf("%s", path);
    }

    // printf("\r\n");
    // printf("[INFO]\tFile Path\t\t\t%s\r\n", LogFile.path);
    // printf("[INFO]\tFile Name:\t\t\t%s\r\n", LogFile.file_name);
    // printf("[INFO]\tFile Total Byte Size:\t\t%lld\r\n", LogFile.logfile_size.total_byte);

    LogFile_Decode(&LogFile);

    while (1)
    {
    }
    return 0;
}