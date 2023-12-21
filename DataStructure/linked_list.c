#include "linked_list.h"
#include <string.h>

void List_Init(list_obj *list, item_obj *first_item, list_arrangement_mode mode, item_compare_callback callback)
{
    // if ((first_item != NULL) && (first_item->data != NULL))
    // {
    list->data = first_item->data;
    list->nxt = first_item->nxt;

    first_item->compare_callback = callback;
    first_item->mode = mode;
    // }
    // else
    //     list->nxt = NULL;

    list->mode = mode;
    list->compare_callback = callback;

    list->prv = NULL;
}

static void Item_SwapLRData(item_obj *item_L, item_obj *item_R)
{
    void *data_tmp;

    data_tmp = item_L->data;
    item_L->data = item_R->data;
    item_R->data = data_tmp;
}

static void List_ItemSwapLRItem(item_obj *item)
{
    item_obj *item_tmp = NULL;

    if (item != NULL)
    {
        item_tmp = item->prv;

        item->prv = item->nxt;
        item->nxt = item_tmp;
    }
}

// minimun check pos is 1
item_obj *List_CheckAt(list_obj *list, uint16_t layer)
{
    item_obj *item_tmp = NULL;

    if (layer == 1)
    {
        return list;
    }

    if (layer < MINIMUN_SEARCH_POS)
    {
        return NULL;
    }
    else
    {
        item_tmp = (item_obj *)List_CheckAt(list->nxt, layer--);
    }

    return item_tmp;
}

item_obj *List_Chk_FirstItem(list_obj *list)
{
    if (list != NULL)
    {
        if (list->prv == NULL)
        {
            return list;
        }
        else
            return List_Chk_FirstItem(list->prv);
    }

    return NULL;
}

item_obj *List_Chk_LastItem(list_obj *list)
{
    if (list != NULL)
    {
        if (list->nxt == NULL)
        {
            return list;
        }
        else
            return List_Chk_LastItem(list->nxt);
    }

    return NULL;
}

list_error_code List_ItemClear(item_obj *item)
{
    if (item != NULL)
    {
        item->data = NULL;
        item->prv = NULL;
        item->nxt = NULL;

        return list_no_error;
    }

    return list_obj_error;
}

void List_ItemInit(item_obj *obj, void *arg)
{
    if (List_ItemClear(obj) == list_no_error)
    {
        obj->data = arg;
    }
}

void List_InsertToPrv(list_obj *list, item_obj *item)
{
    if (list->nxt != NULL)
    {
        List_InsertToPrv(list->prv, item);
    }
    else
    {
        list->prv = item;
        item->nxt = list;
    }
}

void List_InsertByOrder(list_obj *list, item_obj *item)
{
    if (list->nxt != NULL)
    {
        List_InsertByOrder(list->nxt, item);
    }
    else
    {
        list->nxt = item;
        item->prv = list;
    }
}

void List_InsertByCondition(list_obj *list, item_obj *item)
{
    if ((list->compare_callback != NULL) && (list->data != NULL) && (item->data != NULL))
    {
        volatile void *cmp_out = list->compare_callback(list->data, item->data);

        if (cmp_out == NULL)
        {
            return;
        }

        if (cmp_out == item->data)
        {
            Item_SwapLRData(list, item);
        }
    }
    else
        return;

    if (list->nxt != NULL)
    {
        List_InsertByCondition(list->nxt, item);
    }
    else
    {
        list->nxt = item;
        item->prv = list;
        item->nxt = NULL;
    }
}

void List_Insert_Item(list_obj *list, item_obj *item)
{
    if ((list != NULL) && (item != NULL))
    {
        item->compare_callback = list->compare_callback;
        item->mode = list->mode;

        switch (list->mode)
        {
        case to_front:
            List_InsertToPrv(list, item);
            break;

        case by_order:
            List_InsertByOrder(list, item);
            break;

        case by_condition:
            List_InsertByCondition(list, item);
            break;

        default:
            return;
            break;
        }
    }
    else if ((list == NULL) && (item != NULL))
    {
        list = item;
    }
}

list_error_code List_traverse_HaltByCondition(list_obj *list, list_traverse_callback callback, void *arg, listtrv_callback_serial cb_serial, int condition)
{
    if (list != NULL)
    {
        if ((callback != NULL) && (cb_serial == pre_callback))
        {
            if (condition == callback(list, list->data, arg))
                return list_no_error;
        }

        if (list->nxt != NULL)
        {
            List_traverse(list->nxt, callback, arg, cb_serial);
        }

        if ((callback != NULL) && (cb_serial == sub_callback))
        {
            if (condition == callback(list, list->data, arg))
                return list_no_error;
        }
    }
    else
        return list_obj_error;

    return list_no_error;
}

list_error_code List_traverse(list_obj *list, list_traverse_callback callback, void *arg, listtrv_callback_serial cb_serial)
{
    if (list != NULL)
    {
        if ((callback != NULL) && (cb_serial == pre_callback))
        {
            callback(list, list->data, arg);
        }

        if (list->nxt != NULL)
        {
            List_traverse(list->nxt, callback, arg, cb_serial);
        }

        if ((callback != NULL) && (cb_serial == sub_callback))
        {
            callback(list, list->data, arg);
        }
    }
    else
        return list_obj_error;

    return list_no_error;
}

list_error_code List_DecBelowID(item_obj *obj)
{
    if (obj != NULL)
    {
        List_DecBelowID(obj->nxt);

        return item_no_error;
    }

    return item_obj_error;
}

list_error_code List_Delete_Item(item_obj *item, item_datareset_callback callback)
{
    if (item != NULL)
    {
        if (callback != NULL)
        {
            callback(item);
        }

        if ((item->prv != NULL) && (item->nxt != NULL))
        {
            item->prv->nxt = item->nxt;
            item->nxt->prv = item->prv;
            item->prv = NULL;
            item->nxt = NULL;
        }
        else if ((item->prv != NULL) && (item->nxt == NULL))
        {
            item->prv->nxt = NULL;
            item->prv = NULL;
            item->nxt = NULL;
        }
        else if ((item->prv == NULL) && (item->nxt != NULL))
        {
            item = item->nxt;
        }
        else if ((item->prv == NULL) && (item->nxt == NULL))
        {
            item->data = NULL;
            item = NULL;
        }

        return list_no_error;
    }

    return list_obj_error;
}

item_obj *List_PopFirst(list_obj *list)
{
    item_obj *item_tmp = NULL;

    if (list != NULL)
    {
        if (list->prv != NULL)
        {
            List_PopFirst(list->prv);
        }
        else
        {
            item_tmp = list;
            item_tmp->nxt = NULL;

            list = list->nxt;
            list->prv = NULL;
        }
    }

    return item_tmp;
}

int16_t List_GetFront_Len(item_obj *list)
{
    int16_t fnt_len = 0;

    if (list != NULL)
    {
        if (list->prv == NULL)
        {
            return 0;
        }
        else
        {
            fnt_len = List_GetFront_Len(list->prv);
            fnt_len++;
        }
    }
    else
        return list_obj_error;

    return fnt_len;
}

int16_t List_GetBack_Len(item_obj *list)
{
    uint16_t bck_len = 0;

    if (list != NULL)
    {
        if (list->nxt == NULL)
        {
            return 0;
        }
        else
        {
            bck_len = List_GetBack_Len(list->nxt);
            bck_len++;
        }
    }
    else
        return list_obj_error;

    return bck_len;
}

int16_t List_GetLen(list_obj *list)
{
    uint16_t len = 0;

    if (list != NULL)
    {
        len = List_GetFront_Len(list);
        len += List_GetBack_Len(list);
    }
    else
        return list_obj_error;

    return len;
}
