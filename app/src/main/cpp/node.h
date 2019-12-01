#ifndef NODE_H
#define NODE_H
#include <vector>
#include <opencv2/opencv.hpp>

struct Node
{
    Point3d position;
    std::vector<Node *> connections;
};

#endif