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

/* return smaller data handler */
typedef data_handle (*Tree_Callback)(data_handle node_data, data_handle insert_data);
typedef void (*Tree_Traverse_Callback)(data_handle data);

typedef enum
{
    Tree_Pre_Traverse = 1,
    Tree_Mid_Traverse,
    Tree_Bck_Traverse,
} Tree_TraverseType_List;

#pragma pack(1)

typedef struct Node_TypeDef
{
    char *name;

    data_handle data;
    int16_t balance_factory;

    struct Node_TypeDef *F_Node;
    struct Node_TypeDef *L_Node;
    struct Node_TypeDef *R_Node;

    Tree_Callback insert_callback;
    Tree_Callback search_callback;
    Tree_Callback compare_callback;
} TreeNode_TypeDef;

typedef struct
{
    char *name;
    TreeNode_TypeDef *root_node;

    Tree_Callback insert_callback;
    Tree_Callback search_callback;
    Tree_Callback compare_callback;
} Tree_TypeDef;

#pragma pack()

typedef struct
{
    Tree_TypeDef *(*Create)(char *name, Tree_Callback insert, Tree_Callback search, Tree_Callback compare);
    bool (*Insert)(Tree_TypeDef *tree, char *node_name, data_handle data_addr);
    TreeNode_TypeDef (*Search)(Tree_TypeDef *tree, data_handle data_addr);
    void (*Traverse)(Tree_TypeDef *tree, Tree_TraverseType_List type, Tree_Traverse_Callback callback);
} BinaryTree_TypeDef;

extern BinaryTree_TypeDef BlanceTree;

#endif
