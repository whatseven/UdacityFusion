/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <queue>

// Structure to represent node of kd tree
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
        } else
          cur = cur->left;
      else {
        if (cur->right == NULL) {
          cur->right = new Node(point, id);
          break;
        } else
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
      } else {
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
