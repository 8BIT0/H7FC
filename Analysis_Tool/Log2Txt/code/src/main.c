#include "../inc/var_def.h"
#include "../inc/logfile.h"
#include <sys/stat.h>

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
    char *path_tmp = NULL;
    char *logfile_name_tmp = NULL;
    char *cnvfile_name_tmp = NULL;

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

                path_tmp = malloc(offset);
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
                obj->logfile_size.b = FILE_GET_B(obj->logfile_size.total_byte);
                obj->logfile_size.kb = FILE_GET_KB(obj->logfile_size.total_byte);
                obj->logfile_size.mb = FILE_GET_MB(obj->logfile_size.total_byte);

                printf("\r\n");
                printf("[INFO]\tFile Path\t\t\t%s\r\n", obj->path);
                printf("[INFO]\tLogFile Name:\t\t\t%s\r\n", obj->log_file_name);
                printf("[INFO]\tCNVFile Name:\t\t\t%s\r\n", obj->cnv_file_name);
                printf("[INFO]\tFile Total Byte Size:\t\t%lld\r\n", obj->logfile_size.total_byte);

                printf("\r\n");
                printf("[INFO]\tFile Byte Size:\t\t\t%d\r\n", obj->logfile_size.b);
                printf("[INFO]\tFile KByte Size:\t\t%d\r\n", obj->logfile_size.kb);
                printf("[INFO]\tFile MByte Size:\t\t%d\r\n", obj->logfile_size.mb);

                /* create convert file */

                if (obj->logfile_size.total_byte)
                {
                    obj->bin_data = malloc(obj->logfile_size.total_byte);
                    if (obj->bin_data)
                    {
                    }
                    else
                    {
                        printf("[Error]\tMemory Malloc Failed\r\n");
                        printf("\r\n\r\n");
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
    char *path;
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

    // printf("\r\n");
    // printf("[INFO]\tFile Byte Size:\t\t\t%d\r\n", LogFile.logfile_size.b);
    // printf("[INFO]\tFile KByte Size:\t\t%d\r\n", LogFile.logfile_size.kb);
    // printf("[INFO]\tFile MByte Size:\t\t%d\r\n", LogFile.logfile_size.mb);

    while (1)
    {
    }
    return 0;
}