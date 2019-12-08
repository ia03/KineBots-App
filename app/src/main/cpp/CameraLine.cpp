#include "CameraLine.h"
#include "utils.h"
#include <jni.h>

CameraLine::CameraLine(JNIEnv *env, const jdoubleArray &jni_coordinates)
{
	jdouble *coordinates = env->GetDoubleArrayElements(jni_coordinates, nullptr);

	point_1 = Point(coordinates[0], coordinates[1]);
	point_2 = Point(coordinates[2], coordinates[3]);
}

bool CameraLine::operator<(const CameraLine &right_val) const
{
	return error_percent > right_val.error_percent;
}

double CameraLine::size() const
{
	return distance<Point>(point_1, point_2);
}

double CameraLine::margin_of_error(const double percent) const
{
	return size() * percent;
}