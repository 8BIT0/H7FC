/*
coder: 8_B!T0
bref:
use binary tree structure estabilsh a error log tree make error trigger and seach effcient
*/
#include "binary_tree.h"
#include "Srv_OsCommon.h"
#include <string.h>

#define TreeMalloc(x) SrvOsCommon.malloc(x)
#define TreeFree(x) SrvOsCommon.free(x)

static Tree_TypeDef *BinaryTree_Create(char *name, Tree_Callback insert, Tree_Search_Callback search, Tree_Callback compare);
static void Tree_Traverse(Tree_TypeDef *tree, Tree_TraverseType_List type, Tree_Traverse_Callback callback);
static bool Tree_Insert(Tree_TypeDef *tree, char *node_name, data_handle data_addr);
static TreeNode_Handle Tree_Search(Tree_TypeDef *tree, data_handle data);

BinaryTree_TypeDef BalanceTree = {
    .Create = BinaryTree_Create,
    .Insert = Tree_Insert,
    .Traverse = Tree_Traverse,
    .Search = Tree_Search,
};

static Tree_TypeDef *BinaryTree_Create(char *name, Tree_Callback insert, Tree_Search_Callback search, Tree_Callback compare)
{
    Tree_TypeDef *tree_tmp = NULL;

    tree_tmp = (Tree_TypeDef *)TreeMalloc(sizeof(Tree_TypeDef));

    if (tree_tmp)
    {
        if ((insert != NULL) && (search != NULL) && (compare != NULL))
        {
            tree_tmp->name = name;
            tree_tmp->search_callback = search;
            tree_tmp->insert_callback = insert;
            tree_tmp->compare_callback = compare;
            tree_tmp->root_node = NULL;
        }
        else
        {
            TreeFree(tree_tmp);
            tree_tmp = NULL;
        }
    }

    return tree_tmp;
}

static int16_t Tree_Get_Depth(TreeNode_TypeDef *target)
{
    int16_t l_depth = 0;
    int16_t r_depth = 0;

    if (target != NULL)
    {
        l_depth = Tree_Get_Depth(target->L_Node) + 1;
        r_depth = Tree_Get_Depth(target->R_Node) + 1;

        return (l_depth >= r_depth) ? l_depth : r_depth;
    }

    return 0;
}

static int16_t Tree_Get_BalanceFactory(TreeNode_TypeDef *target)
{
    if (target)
        return Tree_Get_Depth(target->L_Node) - Tree_Get_Depth(target->R_Node);

    return 0;
}

static TreeNode_TypeDef *Tree_Search_RootNode(TreeNode_TypeDef *node)
{
    if (node)
    {
        if (node->F_Node)
        {
            return Tree_Search_RootNode(node->F_Node);
        }
        else
            return node;
    }

    return NULL;
}

static TreeNode_TypeDef *TreeNode_RightRotate(TreeNode_TypeDef *node)
{
    TreeNode_TypeDef *o_r = NULL; // old root node
    TreeNode_TypeDef *n_r = NULL; // new root node
    TreeNode_TypeDef *f_n = NULL; // father node

    if (node)
    {
        o_r = node;
        n_r = node->L_Node;
        f_n = node->F_Node;

        if (f_n)
        {
            /* old root data < father node data */
            if (node->compare_callback(f_n->data, n_r->data) == n_r->data)
            {
                f_n->L_Node = n_r;
            }
            else if (node->compare_callback(f_n->data, n_r->data) == n_r->data)
            {
                f_n->R_Node = n_r;
            }
        }
        n_r->F_Node = f_n;

        o_r->L_Node = n_r->R_Node;
        if (n_r->R_Node)
        {
            n_r->R_Node->F_Node = o_r;
        }
        n_r->R_Node = o_r;
        o_r->F_Node = n_r;
    }

    return n_r;
}

static TreeNode_TypeDef *TreeNode_LeftRotate(TreeNode_TypeDef *node)
{
    TreeNode_TypeDef *o_r = NULL; // old root node
    TreeNode_TypeDef *n_r = NULL; // new root node
    TreeNode_TypeDef *f_n = NULL; // father node

    if (node)
    {
        o_r = node;
        n_r = node->R_Node;
        f_n = node->F_Node;

        if (f_n)
        {
            if (node->compare_callback(f_n->data, n_r->data) == n_r->data)
            {
                f_n->L_Node = n_r;
            }
            else if (node->compare_callback(f_n->data, n_r->data) == f_n->data)
            {
                f_n->R_Node = n_r;
            }
        }
        n_r->F_Node = f_n;

        o_r->R_Node = n_r->L_Node;
        if (n_r->L_Node)
        {
            n_r->L_Node->F_Node = o_r;
        }
        n_r->L_Node = o_r;
        o_r->F_Node = n_r;
    }

    return n_r;
}

static void TreeNoe_Dynamic_Trim(TreeNode_TypeDef *root)
{
    int16_t bf = 0;
    int16_t lbf = 0;
    int16_t rbf = 0;

    if (root == NULL)
        return;

    bf = Tree_Get_BalanceFactory(root);
    if (bf == 2) // LL
    {
        lbf = Tree_Get_BalanceFactory(root->L_Node);
        if (lbf == -1) // LR
        {
            root->L_Node = TreeNode_LeftRotate(root->L_Node);
        }

        root = TreeNode_RightRotate(root);
    }
    else if (bf == -2) // RR
    {
        rbf = Tree_Get_BalanceFactory(root->R_Node);
        if (rbf == 1) // RL
        {
            root->R_Node = TreeNode_RightRotate(root->R_Node);
        }

        root = TreeNode_LeftRotate(root);
    }
}

static bool TreeNode_Insert(TreeNode_TypeDef *root, TreeNode_TypeDef *node)
{
    data_handle hdl_tmp = 0;

    if (root == NULL || node == NULL)
        return false;

    hdl_tmp = root->insert_callback(root->data, node->data);

    /* insert callback return smaller data`s addr */
    if (hdl_tmp == node->data)
    {
        /* insert node data smaller than root/target_node data */
        if (root->L_Node != NULL)
        {
            return TreeNode_Insert(root->L_Node, node);
        }
        else
        {
            node->F_Node = root;
            root->L_Node = node;
        }
    }
    else if (hdl_tmp == root->data)
    {
        /* insert node data bigger than root/target_node data  */
        if (root->R_Node != NULL)
        {
            return TreeNode_Insert(root->R_Node, node);
        }
        else
        {
            node->F_Node = root;
            root->R_Node = node;
        }
    }
    else
        return false;

    return true;
}

static void TreeNode_Update_BalanceFactory(TreeNode_TypeDef *node)
{
    if (node)
    {
        node->balance_factory = Tree_Get_BalanceFactory(node);

        TreeNode_Update_BalanceFactory(node->L_Node);
        TreeNode_Update_BalanceFactory(node->R_Node);

        if ((node->balance_factory == 2) || (node->balance_factory == -2))
        {
            /* dynamic balance trim */
            TreeNoe_Dynamic_Trim(node);
        }
    }
}

static void Tree_RootUpdate(Tree_TypeDef *tree)
{
    TreeNode_TypeDef *root = NULL;

    if (tree && tree->root_node)
    {
        root = Tree_Search_RootNode(tree->root_node);

        if (root != NULL)
        {
            tree->root_node = root;

            /* test code */
            TreeNode_Update_BalanceFactory(tree->root_node);
        }
    }
}

static bool TreeNode_Init(TreeNode_TypeDef *node, char *name, data_handle data, Tree_Callback insert, Tree_Search_Callback search, Tree_Callback compare)
{
    if (node && data)
    {
        if ((insert == NULL) || (search == NULL) || (compare == NULL))
            return false;

        node->name = name;
        node->data = data;
        node->compare_callback = compare;
        node->insert_callback = insert;
        node->search_callback = search;

        return true;
    }

    return false;
}

static bool Tree_Insert(Tree_TypeDef *tree, char *node_name, data_handle data_addr)
{
    /* create node first */
    TreeNode_TypeDef *node_tmp = NULL;

    if (tree == NULL)
        return false;

    node_tmp = (TreeNode_TypeDef *)TreeMalloc(sizeof(TreeNode_TypeDef));
    TreeNode_Init(node_tmp, node_name, data_addr, tree->insert_callback, tree->search_callback, tree->compare_callback);

    if (node_tmp)
    {
        /* update root node */
        Tree_RootUpdate(tree);

        if (tree->root_node != NULL)
        {
            TreeNode_Insert(tree->root_node, node_tmp);
        }
        else
            tree->root_node = node_tmp;

        /* update root node */
        Tree_RootUpdate(tree);

        return true;
    }

    return false;
}

static void TreeNode_Pre_Traverse(TreeNode_TypeDef *node, Tree_Traverse_Callback callback)
{
    if (node != NULL)
    {
        // process root node
        if (callback != NULL)
            callback(node->data);

        // traverse left node
        TreeNode_Pre_Traverse(node->L_Node, callback);
        // traverse right node
        TreeNode_Pre_Traverse(node->R_Node, callback);
    }
}

static void TreeNode_Mid_Traverse(TreeNode_TypeDef *node, Tree_Traverse_Callback callback)
{
    if (node != NULL)
    {
        // traverse left node
        TreeNode_Mid_Traverse(node->L_Node, callback);

        // process root node
        if (callback != NULL)
            callback(node->data);

        // traverse right node
        TreeNode_Mid_Traverse(node->R_Node, callback);
    }
}

static void TreeNode_Bck_Traverse(TreeNode_TypeDef *node, Tree_Traverse_Callback callback)
{
    if (node != NULL)
    {
        // traverse left node
        TreeNode_Bck_Traverse(node->L_Node, callback);
        // traverse right node
        TreeNode_Bck_Traverse(node->R_Node, callback);

        // precess root node
        if (callback != NULL)
            callback(node->data);
    }
}

static void Tree_Traverse(Tree_TypeDef *tree, Tree_TraverseType_List type, Tree_Traverse_Callback callback)
{
    if (tree && tree->root_node)
    {
        switch ((uint8_t)type)
        {
        case Tree_Pre_Traverse:
            TreeNode_Pre_Traverse(tree->root_node, callback);
            break;

        case Tree_Mid_Traverse:
            TreeNode_Mid_Traverse(tree->root_node, callback);
            break;

        case Tree_Bck_Traverse:
            TreeNode_Bck_Traverse(tree->root_node, callback);
            break;

        default:
            break;
        }
    }
}

static TreeSearch_Out_TypeDef TreeNode_Search(TreeNode_TypeDef *node, data_handle data)
{
    TreeSearch_Out_TypeDef search_out;

    memset(&search_out, 0, sizeof(search_out));

    search_out.state = Tree_Search_E;
    search_out.node_hdl = 0;

    if (node != NULL && node->search_callback != NULL)
    {
        search_out.state = node->search_callback(node->data, data);

        switch (search_out.state)
        {
        case Tree_Search_D:
            /* data matach */
            search_out.node_hdl = (TreeNode_Handle)node;
            break;

        case Tree_Search_L:
            /* search left node */
            return TreeNode_Search(node->L_Node, data);

        case Tree_Search_R:
            /* search right node */
            return TreeNode_Search(node->R_Node, data);

        case Tree_Search_E:
        default:
            /* error match */
            break;
        }
    }

    return search_out;
}

static TreeNode_Handle Tree_Search(Tree_TypeDef *tree, data_handle data)
{
    TreeSearch_Out_TypeDef search_out;

    search_out.state = Tree_Search_E;
    search_out.node_hdl = 0;

    if (tree == NULL || data == 0 || tree->search_callback == NULL)
        return 0;

    search_out = TreeNode_Search(tree->root_node, data);

    if (search_out.state == Tree_Search_D)
    {
        return search_out.node_hdl;
    }
    else
        return 0;
}
