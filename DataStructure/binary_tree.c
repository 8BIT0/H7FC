/*
coder: 8_B!T0
bref:
to make search and sort function more effcient. so i create this file
in order to make doubly linked list can be mutual converted with binary tree
and make some search function effcient.
*/
#include "binary_tree.h"
#include <string.h>

static bool Tree_DynamicTrim(node_template *relative_root);
static void Tree_RotateLeft(node_template *root, node_template *a);
static void Tree_RotateRight(node_template *root, node_template *a);

void Tree_Node_Init(node_template *node, char *node_name, void *data)
{
    if (node == NULL)
        return;

    node->data_ptr = data;

    node->F_Node = NULL;
    node->L_Node = NULL;
    node->R_Node = NULL;

    node->name = node_name;
}

node_template *Tree_ReSetRoot(node_template *tree)
{
    if (tree == NULL)
        return NULL;

    if (tree->F_Node == NULL)
        return tree;
    else
        return Tree_ReSetRoot(tree->F_Node);
}

void Tree_InsertNode(node_template *relative_root, node_template *node, compare_callback callback)
{
    uint32_t cmp_out = 0;

    if ((node == NULL) || (callback == NULL) || (relative_root == node))
        return;

    // need traverse process compare all node in current root
    cmp_out = callback(relative_root->data_ptr, node->data_ptr);

    if (cmp_out)
    {
        // relative root data is bigger then node data
        if (cmp_out == (uint32_t)relative_root->data_ptr)
        {
            if (relative_root->L_Node != NULL)
            {
                Tree_InsertNode(relative_root->L_Node, node, callback);
            }
            else
            {
                if (relative_root == node)
                    return;

                node->F_Node = relative_root;
                relative_root->L_Node = node;
            }
        }
        else if (cmp_out == (uint32_t)node->data_ptr)
        {
            if (relative_root->R_Node != NULL)
            {
                Tree_InsertNode(relative_root->R_Node, node, callback);
            }
            else
            {
                if (relative_root == node)
                    return;

                node->F_Node = relative_root;
                relative_root->R_Node = node;
            }
        }

        Tree_DynamicTrim(relative_root);
    }
}

bool Tree_DeleteNode(node_template *node)
{
    if ((node->F_Node != NULL) || (node != NULL))
    {
        if (node->F_Node->L_Node == node)
        {
            node->F_Node->L_Node = NULL;
        }
        else if (node->F_Node->R_Node == node)
        {
            node->F_Node->R_Node = NULL;
        }

        node->F_Node = NULL;
        return true;
    }

    return false;
}

void Tree_Printf_NodeName(node_template *node)
{
    if (node->name != NULL)
    {
        printf("%s\n", node->name);
    }
}

// traverse rule : root left right
void Tree_Pre_Traverse(node_template *relative_node, tree_traverse_callback callback)
{
    if (relative_node != NULL)
    {
        // process root node
        if (callback != NULL)
        {
            callback(relative_node);
        }

        // traverse left node
        Tree_Pre_Traverse(relative_node->L_Node, callback);
        // traverse right node
        Tree_Pre_Traverse(relative_node->R_Node, callback);
    }
}

// traverse rule : left root right
void Tree_Mid_Traverse(node_template *relative_node, tree_traverse_callback callback)
{
    if (relative_node != NULL)
    {
        // traverse left node
        Tree_Mid_Traverse(relative_node->L_Node, callback);

        // process root node
        if (callback != NULL)
        {
            callback(relative_node);
        }

        // traverse right node
        Tree_Mid_Traverse(relative_node->R_Node, callback);
    }
}

// traverse rule : left right root
void Tree_Bck_Traverse(node_template *relative_node, tree_traverse_callback callback)
{
    if (relative_node != NULL)
    {
        // traverse left node
        Tree_Bck_Traverse(relative_node->L_Node, callback);
        // traverse right node
        Tree_Bck_Traverse(relative_node->R_Node, callback);

        // precess root node
        if (callback != NULL)
        {
            callback(relative_node);
        }
    }
}

void Tree_Structure_Dsp(node_template *relative_root, display_callback dsp_func, traverse_type type)
{
    switch (type)
    {
    case pre_trv:
        printf("\r\npre traverse\r\n");
        Tree_Pre_Traverse(relative_root, dsp_func);
        break;

    case mid_trv:
        printf("\r\nmid traverse\r\n");
        Tree_Mid_Traverse(relative_root, dsp_func);
        break;

    case bck_trv:
        printf("\r\nbck traverse\r\n");
        Tree_Bck_Traverse(relative_root, dsp_func);
        break;

    default:
        break;
    }
}

uint8_t Tree_GetDepth(node_template *relative_root)
{
    uint8_t left_depth_tmp = 0;
    uint8_t right_depth_tmp = 0;

    if (relative_root == NULL)
    {
        return 0;
    }

    left_depth_tmp = Tree_GetDepth(relative_root->L_Node) + 1;
    right_depth_tmp = Tree_GetDepth(relative_root->R_Node) + 1;

    return (left_depth_tmp > right_depth_tmp) ? left_depth_tmp : right_depth_tmp;
}

void Tree_SwapLR(node_template *relative_root)
{
    node_template *node_tmp = NULL;

    if (relative_root != NULL)
    {
        node_tmp = relative_root->L_Node;

        relative_root->L_Node = relative_root->R_Node;
        relative_root->R_Node = node_tmp;

        Tree_SwapLR(relative_root->L_Node);
        Tree_SwapLR(relative_root->R_Node);
    }
}

balance_check_ouput_s Tree_Balance_Checker(node_template *relative_root, int8_t *depth, unbalance_callback cb_func)
{
    balance_check_ouput_s output_tmp;
    int8_t left_depth = 0;
    int8_t right_depth = 0;
    int8_t bias_bwt_LR = 0;

    output_tmp.isBalance = false;
    output_tmp.layer = 0;
    output_tmp.hi_bias = 0;
    output_tmp.occur_dir = dir_none;
    output_tmp.occur_node_ptr = NULL;

    // remember the empty tree is balance
    if (relative_root == NULL)
    {
        *depth = 0;
        output_tmp.isBalance = true;
        return output_tmp;
    }
    else
    {
        if (Tree_Balance_Checker(relative_root->L_Node, &left_depth, cb_func).isBalance)
        {
            if (Tree_Balance_Checker(relative_root->R_Node, &right_depth, cb_func).isBalance)
            {
                *depth = left_depth > right_depth ? (left_depth + 1) : (right_depth + 1);

                // if left sub tree and right sub tree both in balance
                bias_bwt_LR = abs(left_depth - right_depth);

                if (bias_bwt_LR <= 1)
                {
                    output_tmp.isBalance = true;
                    output_tmp.hi_bias = bias_bwt_LR;
                    output_tmp.layer = *depth;
                    return output_tmp;
                }
                else
                {
                    output_tmp.isBalance = false;
                    output_tmp.hi_bias = bias_bwt_LR;
                    output_tmp.layer = *depth;
                    return output_tmp;
                }
            }
            else
            {
                output_tmp.isBalance = false;
                output_tmp.layer = *depth;
                output_tmp.occur_dir = dir_right;
                output_tmp.occur_node_ptr = relative_root->R_Node;

                if (cb_func != NULL)
                {
                    cb_func(relative_root->R_Node);
                }

                return output_tmp;
            }
        }
        else
        {
            output_tmp.isBalance = false;
            output_tmp.layer = *depth;
            output_tmp.occur_dir = dir_left;
            output_tmp.occur_node_ptr = relative_root->L_Node;

            if (cb_func != NULL)
            {
                cb_func(relative_root->L_Node);
            }

            return output_tmp;
        }
    }
}

static int8_t Tree_GetBalance_Gain(node_template *relative_node)
{
    if (relative_node == NULL)
        return 0;

    return Tree_GetDepth(relative_node->L_Node) - Tree_GetDepth(relative_node->R_Node);
}

static bool Tree_DynamicTrim(node_template *relative_root)
{
    node_template *node_tmp;
    trim_style_list trim_type;
    int8_t DepthDif = 0;

    if (relative_root == NULL)
        return false;

    while (relative_root != NULL)
    {
        DepthDif = Tree_GetBalance_Gain(relative_root);

        if ((DepthDif > 2) || (DepthDif < -2))
            return false;

        if (DepthDif == 2)
        {
            if (relative_root->L_Node->R_Node != NULL)
            {
                Tree_RotateLeft(relative_root->F_Node, relative_root->L_Node);
            }

            Tree_RotateRight(relative_root->F_Node, relative_root);
        }
        else if (DepthDif == -2)
        {
            if (relative_root->R_Node->L_Node != NULL)
            {
                Tree_RotateRight(relative_root->F_Node, relative_root->R_Node);
            }

            Tree_RotateLeft(relative_root->F_Node, relative_root);
        }

        relative_root = relative_root->F_Node;
    }

    return true;
}

static void Tree_RotateLeft(node_template *root, node_template *a)
{
    if (a == NULL)
        return;

    node_template *b = a->R_Node;
    node_template *d = b->L_Node;

    a->R_Node = d;
    if (d != NULL)
    {
        d->F_Node = a;
    }

    b->F_Node = a->F_Node;

    if (b->F_Node != NULL)
    {
        if (b->F_Node->L_Node == a)
        {
            b->F_Node->L_Node = b;
        }
        else
        {
            b->F_Node->R_Node = b;
        }
    }
    else
    {
        // b node is root
        root = b;
    }

    a->F_Node = b;
    b->L_Node = a;
}

static void Tree_RotateRight(node_template *root, node_template *a)
{
    if (a == NULL)
        return;

    node_template *b = a->L_Node;
    node_template *d = b->R_Node;

    a->L_Node = d; // this step make left node equle to right node
    if (d != NULL)
    {
        d->F_Node = a;
    }

    b->F_Node = a->F_Node;

    if (b->F_Node != NULL)
    {
        if (b->F_Node->L_Node == a)
        {
            b->F_Node->L_Node = b;
        }
        else
        {
            b->F_Node->R_Node = b;
        }
    }
    else
    {
        // b node is root
        root = b;
    }

    a->F_Node = b;
    b->R_Node = a;
}

uint32_t Tree_Search(node_template *Root_Ptr, void *node_data, search_callback mth_callback, compare_callback cmp_callback)
{
    uint32_t nxt_node = 0;
    node_template *node_tmp = NULL;

    if (Root_Ptr == NULL || node_data == NULL || cmp_callback == NULL)
        return ERROR_MATCH;

    node_tmp = Root_Ptr;

    nxt_node = cmp_callback(node_tmp->data_ptr, node_data);

    if (nxt_node == (uint32_t)node_tmp)
    {
        if (mth_callback != NULL)
            mth_callback(node_data);

        return (uint32_t)((node_template *)nxt_node);
    }
    else if (nxt_node > ERROR_MATCH)
    {
        return Tree_Search((node_template *)nxt_node, node_data, mth_callback, cmp_callback);
    }
    else
        return ERROR_MATCH;
}
