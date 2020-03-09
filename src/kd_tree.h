#pragma once
#include <vector>
#include<queue>
#include <pcl/common/common.h>

struct Node {
    std::vector<float> point;
    int id;
    Node *left;
    Node *right;

    Node(std::vector<float> arr, int setId)
        : point(arr), id(setId), left(NULL), right(NULL) {}
};

struct KdTree {
    Node *root;
    KdTree() : root(NULL) {}

    void insert(std::vector<float> point, int id) {
        if (root == NULL) {
            root = new Node(point, id);
            return;
        }
        Node *cur = root;
        int depth = 0;
        while (cur != NULL) {
            if (cur->point[depth % 2] > point[depth % 2])
                if (cur->left == NULL) {
                    cur->left = new Node(point, id);
                    break;
                }
                else
                    cur = cur->left;
            else {
                if (cur->right == NULL) {
                    cur->right = new Node(point, id);
                    break;
                }
                else
                    cur = cur->right;
            }
            depth++;
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol) {
        std::vector<int> ids;
        std::queue<Node *> queue;
        std::queue<int> depthQueue;
        queue.push(root);
        depthQueue.push(0);
        while (!queue.empty()) {
            if (queue.front() == NULL) {
                depthQueue.pop();
                queue.pop();
                continue;
            }
            int depth = depthQueue.front();
            depthQueue.pop();
            const Node *cur = queue.front();
            queue.pop();
            float distance = powf(target[0] - cur->point[0], 2) +
                powf(target[1] - cur->point[1], 2);
            if (distance > distanceTol * distanceTol) {
                float axisDistance = target[depth % 2] - cur->point[depth % 2];
                if (axisDistance < 0)
                    queue.push(cur->left);
                else
                    queue.push(cur->right);
                depthQueue.push(depth + 1);
            }
            else {
                queue.push(cur->left);
                queue.push(cur->right);
                depthQueue.push(depth + 1);
                depthQueue.push(depth + 1);

                ids.push_back(cur->id);
            }
        }
        return ids;
    }
};

template <typename PointT>
void proximity(const typename pcl::PointCloud<PointT>::Ptr &cloud, std::vector<bool>& hasProcessed
    , size_t pointIndex, std::vector<int>& cluster,KdTree *tree, float distanceTol) {
    std::vector<int> neighbour =
        tree->search({ cloud->points[pointIndex].x
            ,cloud->points[pointIndex].y
            ,cloud->points[pointIndex].z }
    , distanceTol);
    for (auto neighbourIndex : neighbour) {
        if (hasProcessed[neighbourIndex])
            continue;
        hasProcessed[neighbourIndex] = true;
        cluster.push_back(neighbourIndex);
        proximity<PointT>(cloud, hasProcessed, neighbourIndex, cluster, tree, distanceTol);
    }
}

template <typename PointT>
std::vector<std::vector<int>>
euclideanCluster(const typename pcl::PointCloud<PointT>::Ptr &cloud,KdTree *tree,
    float distanceTol) {

    std::vector<std::vector<int>> clusters;
    std::vector<bool> hasProcessed(cloud->points.size(), false);
    for (size_t pointIndex = 0; pointIndex < cloud->points.size(); pointIndex++) {
        if (hasProcessed[pointIndex])
            continue;
        clusters.push_back({(int)pointIndex});
        hasProcessed[pointIndex] = true;
        proximity<PointT>(cloud, hasProcessed, pointIndex, clusters.back(), tree, distanceTol);
    }

    return clusters;
}