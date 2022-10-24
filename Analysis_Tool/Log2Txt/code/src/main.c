#include "../inc/var_def.h"

#define MIN_LOG_FILENAME_LEN 5
#define EXTEND_FILETYPE_NAME ".log"
#define EXTEND_FILETYPE_NAME_LEN strlen(EXTEND_FILETYPE_NAME)

static bool File_Check_ExtendType(char *ext_type)
{
    char *ext;

    if (ext_type)
    {
        ext = ext_type + strlen(ext_type) - EXTEND_FILETYPE_NAME_LEN;

        if (strcmp(ext, EXTEND_FILETYPE_NAME) == 0)
        {
            printf("[INFO]\tFile Type Checked\r\n");
            return true;
        }
        else
            printf("[Error]\tFile Type Error\r\n");
    }

    return false;
}

static bool Load_File(char *path)
{
    uint16_t file_path_len = 0;

    if (path)
    {
        file_path_len = strlen(path);

        if ((file_path_len > MIN_LOG_FILENAME_LEN) && File_Check_ExtendType(path))
        {
            /* open log file */

            /* read log data into buff */

            return true;
        }
        else
        {
            printf("[Error]\tLogFile Path input :\t%s\r\n", path);
            printf("\r\n\r\n");
        }

        memset(path, '\0', strlen(path));
    }

    return false;
}

int main()
{
    char *path;

    do
    {
        /* waiting for path input */
        printf("[INFO]\tType LogFile Path : \t");
        scanf("%s", path);
        printf("\r\n");
    } while (!Load_File(path));

    while (1)
    {
    }
    return 0;
}