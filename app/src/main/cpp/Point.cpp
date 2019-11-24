#include "Point.h"

Point &Point::operator=(const Point &val)
{
    x = val.x;
    y = val.y;
    return *this;
}

Point3d &Point3d::operator-=(const Point3d &val)
{
    x -= val.x;
    y -= val.y;
    z -= val.z;
    return *this;
}