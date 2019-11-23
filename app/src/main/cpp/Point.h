struct Point
{
    int x;
    int y;

    Point(int x, int y) : x(x), y(y) {};
};

struct Point3d
{
    double x;
    double y;
    double z;

    Point3d &operator-=(Point3d &val);
};