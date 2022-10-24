#include "../inc/var_def.h"

#define MIN_LOG_FILENAME_LEN 5
#define EXTEND_FILETYPE_NAME ".log"
#define EXTEND_FILETYPE_NAME_LEN strlen(EXTEND_FILETYPE_NAME)

bool Load_File(char *path)
{
    uint16_t file_path_len = 0;

    if (path)
    {
        file_path_len = strlen(path);

        if (file_path_len > MIN_LOG_FILENAME_LEN)
        {
            printf("[Error]\tLogFile Path input :\t%s\r\n", path);
            memset(path, '\0', strlen(path));
            printf("\r\n\r\n");
        }
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
    } while (!Load_File(path));

    while (1)
    {
    }
    return 0;
}