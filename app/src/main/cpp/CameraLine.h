#ifndef CAMERALINE_H
#define CAMERALINE_H
#include <opencv2/opencv.hpp>
#include "Line.h"
#include <jni.h>
#include <limits>

using namespace cv;

struct CameraLine
{
	// Links to the actual 3D line that this camera line represents.
	const Line *line_3d = nullptr;

	CameraLine(JNIEnv *env, const jintArray &jni_coordinates);

	CameraLine() = default;

	bool operator<(const CameraLine &right_val) const;

	Point point_1;
	Point point_2;

	double error_percent = std::numeric_limits<double>::max();

	double size() const;

	double margin_of_error(const double percent) const;
};
#endif