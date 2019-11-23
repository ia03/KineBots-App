#pragma once
#include "Point.h"
#include "Line.h"
#include "jni.h"
#include <limits>

struct CameraLine
{
	// Links to the actual 3D line that this camera line represents.
	const Line *line_3d = nullptr;

	CameraLine(const Vec4i &coordinates);

	CameraLine() = default;

	bool operator<(const CameraLine &right_val) const;

	Point point_1;
	Point point_2;

	double error_percent = std::numeric_limits<double>::max();

	double size() const;

	double margin_of_error(const double percent) const;
};