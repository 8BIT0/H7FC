#ifndef __STORAGE_DEF_H
#define __STORAGE_DEF_H

#include "util.h"
#include "../../FCHW_Config.h"

#define InternalFlash_BootDataSec_Size  (4 Kb)
#define InternalFlash_SysDataSec_Size   (16 Kb)
#define InternalFlash_UserDataSec_Size  (32 Kb)

#define Storage_TabSize                 Flash_Storage_TabSize
#define Storage_InfoPageSize            Flash_Storage_InfoPageSize

#define Storage_Assert(x)               while(x)

#define Format_Retry_Cnt                5
#define ExternalModule_ReInit_Cnt       5

#define Storage_ErrorCode_ToStr(x)      #x

#define From_Start_Address              0

#define BootSection_Block_Size          (4 Kb)
#define BootTab_Num 1

#define Storage_OnChip_Max_Capacity     256
#define Storage_ReserveBlock_Size       128

#define Storage_ExtFlash_Max_Capacity   (1 Kb)

#define INTERNAL_STORAGE_PAGE_TAG       "[InternalFlash Storage]"
#define EXTERNAL_STORAGE_PAGE_TAG       "[ExternalFlash Storage]"
#define INTERNAL_PAGE_TAG_SIZE          strlen(INTERNAL_STORAGE_PAGE_TAG)
#define EXTERNAL_PAGE_TAG_SIZE          strlen(EXTERNAL_STORAGE_PAGE_TAG)

#define STORAGE_NAME_LEN                41
#define STORAGE_ITEM_HEAD_TAG           0xAA
#define STORAGE_ITEM_END_TAG            0xBB
#define STORAGE_SLOT_HEAD_TAG           0xEF0110EF
#define STORAGE_SLOT_END_TAG            0xFE1001FE
#define STORAGE_DATA_ALIGN              4
#define STORAGE_MIN_BYTE_SIZE           1
#define STORAGE_FREEITEM_NAME           "Item_Avaliable"

#endif
