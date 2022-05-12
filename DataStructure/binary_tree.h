#ifndef __BINARY_TREE_H
#define __BINARY_TREE_H

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

typedef uint32_t Gen_Handle;
typedef Gen_Handle data_handle;
typedef Gen_Handle Tree_Handle;
typedef Gen_Handle TreeNode_Handle;

#define TreeNodeHandleToObj(x) ((TreeNode_TypeDef *)x)

/* return smaller data handler */
typedef data_handle (*Tree_Callback)(data_handle node_data, data_handle insert_data);
typedef uint8_t (*Tree_Search_Callback)(data_handle node_data, data_handle search_data);
typedef void (*Tree_Traverse_Callback)(data_handle data);

typedef enum
{
    Tree_Search_L = 1, /* search left node */
    Tree_Search_R,     /* search right node */
    Tree_Search_D,     /* search matched data */
    Tree_Search_E,     /* search error */
} Tree_Search_State_List;

typedef enum
{
    Tree_Pre_Traverse = 1,
    Tree_Mid_Traverse,
    Tree_Bck_Traverse,
} Tree_TraverseType_List;

#pragma pack(1)

typedef struct
{
    Gen_Handle node_hdl;
    Tree_Search_State_List state;
} TreeSearch_Out_TypeDef;

typedef struct Node_TypeDef
{
    char *name;

    data_handle data;
    int16_t balance_factory;

    struct Node_TypeDef *F_Node;
    struct Node_TypeDef *L_Node;
    struct Node_TypeDef *R_Node;

    Tree_Callback insert_callback;
    Tree_Search_Callback search_callback;
    Tree_Callback compare_callback;
} TreeNode_TypeDef;

typedef struct
{
    char *name;
    TreeNode_TypeDef *root_node;

    Tree_Callback insert_callback;
    Tree_Search_Callback search_callback;
    Tree_Callback compare_callback;
} Tree_TypeDef;

#pragma pack()

typedef struct
{
    Tree_TypeDef *(*Create)(char *name, Tree_Callback insert, Tree_Search_Callback search, Tree_Callback compare);
    bool (*Insert)(Tree_TypeDef *tree, char *node_name, data_handle data_addr);
    TreeNode_Handle (*Search)(Tree_TypeDef *tree, data_handle data_addr);
    void (*Traverse)(Tree_TypeDef *tree, Tree_TraverseType_List type, Tree_Traverse_Callback callback);
} BinaryTree_TypeDef;

extern BinaryTree_TypeDef BalanceTree;

#endif
