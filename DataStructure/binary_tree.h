#ifndef __BINARY_TREE_H
#define __BINARY_TREE_H

#include "stdint.h"
#include "stdio.h"
#include "stddef.h"
#include "stdbool.h"
#include "string.h"
#include "stdlib.h"

#define INVALID_SEARCH -1
#define BALANCE 1

typedef enum
{
    full_tree,
    complete_tree,
    uncomplete_tree,
} bintree_type;

typedef enum
{
    trim_ll = 0,
    trim_rr,
    trim_lr,
    trim_rl,

    trim_style_sum,
} trim_style_list;

typedef enum
{
    dir_none = 0,
    dir_left = 1,
    dir_right,
} direction_e;

typedef enum
{
    pre_trv = 1,
    mid_trv,
    bck_trv,
} traverse_type;

#pragma pack(4)
typedef struct node
{
    struct node *F_Node;
    struct node *L_Node;
    struct node *R_Node;

    char *name;
    void *data_ptr;
} node_template;

typedef struct
{
    bool isBalance;
    uint8_t layer;
    uint8_t hi_bias;
    node_template *occur_node_ptr;
    direction_e occur_dir;
} balance_check_ouput_s;

typedef void (*unbalance_callback)(node_template *root_tmp);
typedef void (*display_callback)(node_template *root_tmp);
typedef uint32_t (*compare_callback)(void *eq_l, void *eq_r);
typedef uint32_t (*search_callback)(void *arg, uint16_t len);
#define tree_traverse_callback display_callback

void Tree_Node_Init(node_template *node, char *node_name, void *data);
balance_check_ouput_s Tree_Balance_Checker(node_template *relative_root, int8_t *depth, unbalance_callback cb_func);
void Tree_InsertNode(node_template *relative_root, node_template *node, compare_callback callback);
void Tree_Structure_Dsp(node_template *relative_root, display_callback dsp_func, traverse_type type);
void Tree_Printf_NodeName(node_template *node);
uint8_t Tree_GetDepth(node_template *relative_root);
void Tree_SwapLR(node_template *relative_root);
node_template *Tree_ReSetRoot(node_template *relative_root);

#endif
