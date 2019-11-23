#include "Point.h"

Point3d &Point3d::operator-=(Point3d &val)
{
    x -= val.x;
    y -= val.y;
    z -= val.z;
    return *this;
}