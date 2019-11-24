#ifndef LINE_H
#define LINE_H
#include "CameraLine.h"
#include <opencv2/opencv.hpp>

using namespace cv;

struct Line
{
	Point3d point_1;
	Point3d point_2;
};

#endif