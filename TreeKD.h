//
// Created by Andrei on 09-Oct-22.
//

#include <utility>

#include "render/render.h"

#ifndef LIDAR_KDTREE_H
#define LIDAR_KDTREE_H

struct Node{
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    Node(std::vector<float> &arr, int setId)
            :	point(std::move(arr)), id(setId), left(nullptr), right(nullptr)
    {}

    ~Node()
    {
        delete left;
        delete right;
    }
};

struct TreeKD
{
    Node* root;

    TreeKD()
    : root(nullptr)
    {}

    ~TreeKD()
    {
        delete root;
    }

    void insertHelper(Node** node, int depth, std::vector<float>& point, int id)
    {
        if (*node == nullptr)
        {
            *node = new Node(point, id);
        }
        else
        {
            int cd = depth % 3;
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
        insertHelper(&root, 0, point, id);
    }

    void searchHelper(std::vector<float>& target, Node* node, int depth, float distanceTol, std::vector<int>& ids) const
    {
        if (node != nullptr)
        {
            if ((node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol)) &&
                (node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol)) &&
                (node->point[2] >= (target[2] - distanceTol) && node->point[2] <= (target[2] + distanceTol)))
            {
                float distance = sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) +
                                      (node->point[1] - target[1]) * (node->point[1] - target[1]) +
                                      (node->point[2] - target[2]) * (node->point[2] - target[2]));
                if (distance <= distanceTol)
                {
                    ids.push_back(node->id);
                }
            }

            if ((target[depth % 3] - distanceTol) < node->point[depth % 3])
            {
                searchHelper(target, node->left, depth + 1, distanceTol, ids);
            }
            if ((target[depth % 3] + distanceTol) > node->point[depth % 3])
            {
                searchHelper(target, node->right, depth + 1, distanceTol, ids);
            }
        }

    }

    [[nodiscard]] std::vector<int> search(std::vector<float> target, float distanceTol) const
    {
        std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);
        return ids;
    }
};

#endif //LIDAR_KDTREE_H
