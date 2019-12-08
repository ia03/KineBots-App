#ifndef NODE_H
#define NODE_H
#include <vector>
#include <opencv2/opencv.hpp>

struct PosNode
{
    double g = 0.0;
    double h = 0.0;
    double f = 0.0;
    PosNode *parent = nullptr;
    Point3d position;
    std::vector<PosNode *> connections;
};

#endif