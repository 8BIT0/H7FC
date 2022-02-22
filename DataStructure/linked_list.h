#ifndef __LINKED_LIST
#define __LINKED_LIST

#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#define DEFAULT_ARRANGE_MODE by_order
#define DEFAULT_COMPARE_CALLBACK NULL
#define MINIMUN_SEARCH_POS 1

typedef enum
{
    list_obj_error = -2,
    item_obj_error = -1,
    item_no_error = 0,
    list_no_error = 0,
    item_id_error,
} list_error_code;

typedef enum
{
    to_front,
    by_order,
    by_condition,
} list_arrangement_mode;

typedef enum
{
    pre_callback = 0,
    sub_callback,
} listtrv_callback_serial;

typedef void *(*item_compare_callback)(void *arg_1, void *arg_2);
typedef void (*item_datareset_callback)(void *data);

typedef struct list_item
{
    list_arrangement_mode mode;
    item_compare_callback compare_callback;

    struct list_item *prv;
    struct list_item *nxt;

    void *data;
} item_obj;

typedef item_obj list_obj;

typedef int (*list_traverse_callback)(item_obj *item, void *arg_1, void *arg_2);

void List_Init(list_obj *list, item_obj *first_item, list_arrangement_mode mode, item_compare_callback callback);
void List_ItemInit(item_obj *obj, void *arg);
void List_InsertByOrder(list_obj *list, item_obj *item);
void List_InsertByCondition(list_obj *list, item_obj *item);
void List_Insert_Item(list_obj *list, item_obj *item);

list_error_code List_traverse(list_obj *list, list_traverse_callback callback, void *arg, listtrv_callback_serial cb_serial);
list_error_code List_traverse_HaltByCondition(list_obj *list, list_traverse_callback callback, void *arg, listtrv_callback_serial cb_serial, int condition);
list_error_code List_ItemClear(item_obj *item);
list_error_code List_Delete_Item(item_obj *item, item_datareset_callback callback);
list_error_code List_DecBelowID(item_obj *obj);
list_error_code List_DecID(list_obj *list);

item_obj *List_CheckAt(list_obj *list, uint16_t layer);
item_obj *List_Chk_FirstItem(list_obj *list);
item_obj *List_Chk_LastItem(list_obj *list);
item_obj *List_PopFirst(list_obj *list);

int16_t List_GetFront_Len(item_obj *list);
int16_t List_GetBack_Len(item_obj *list);
int16_t List_GetLen(list_obj *list);

#endif
