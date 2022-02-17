#include "storage.h"
#include "Dev_W25Qxx.h"

bool Storage_Init(void)
{
#if (EXTERNAL_STORAGE_MODULE == 1)
    bool ExModule_Init = false;

#endif
}