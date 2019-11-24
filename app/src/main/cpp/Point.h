#ifndef POINT_H
#define POINT_H

struct Point
{
    Point() = default;
    int x;
    int y;

    Point(int x, int y) : x(x), y(y) {};
    Point &operator=(const Point &val);
};

struct Point3d
{
    double x;
    double y;
    double z;

    Point3d &operator-=(const Point3d &val);
};
#endif