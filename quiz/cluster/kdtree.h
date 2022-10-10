//
// Created by Andrei on 08-Oct-22.
//

#ifndef KDTREE_H
#define KDTREE_H

#include "../../render/render.h"


struct Node{
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    Node(std::vector<float> arr, int setId)
            :	point(arr), id(setId), left(NULL), right(NULL)
    {}

    ~Node()
    {
        delete left;
        delete right;
    }
};

struct kdTree
{
    Node* root;

    kdTree()
    : root(NULL)
    {}

    ~kdTree()
    {
        delete root;
    }

    void insertHelper(Node** node, int depth, std::vector<float> point, int id)
    {
        if (*node == NULL)
        {
            *node = new Node(point, id);
        }
        else
        {
            int cd = depth % 2;
            if (point[cd] < ((*node)->point[cd]))
            {
                insertHelper(&((*node)->left), depth + 1, point, id);
            }
            else
            {
                insertHelper(&((*node)->right), depth + 1, point, id);
            }
        }
    }


    void insert(std::vector<float> point, int id)
    {
        Node** node = &root;
        int depth = 0;
        while (*node != NULL)
        {
            int cd = depth % 2;
            if (point[cd] < (*node)->point[cd])
                node = &(*node)->left;
            else
                node = &(*node)->right;
            depth++;
        }
        *node = new Node(point, id);
    }

    std::vector<int> search(std::vector<float> target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(target, distanceTol, root, 0, ids);
        return ids;
    }

    void searchHelper(std::vector<float> target, float distanceTol, Node* node, int depth, std::vector<int>& ids)
    {
        if (node != NULL)
        {
            if ((node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol)) &&
                (node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol)))
            {
                float distance = sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) +
                                      (node->point[1] - target[1]) * (node->point[1] - target[1]));
                if (distance <= distanceTol)
                    ids.push_back(node->id);
            }

            // check across boundary
            if ((target[depth % 2] - distanceTol) < node->point[depth % 2])
                searchHelper(target, distanceTol, node->left, depth + 1, ids);
            if ((target[depth % 2] + distanceTol) > node->point[depth % 2])
                searchHelper(target, distanceTol, node->right, depth + 1, ids);
        }
    }
};

#endif //KDTREE_H
