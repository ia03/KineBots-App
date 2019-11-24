#include "CameraLine.h"
#include "utils.h"
#include <jni.h>

CameraLine::CameraLine(JNIEnv *env, const jintArray &jni_coordinates)
{
	jint *coordinates = env->GetIntArrayElements(jni_coordinates, nullptr);

	point_1 = Point(coordinates[0], coordinates[1]);
	point_2 = Point(coordinates[2], coordinates[3]);
}

bool CameraLine::operator<(const CameraLine &right_val) const
{
	return error_percent > right_val.error_percent;
}

double CameraLine::size() const
{
	return distance(point_1, point_2);
}

double CameraLine::margin_of_error(const double percent) const
{
	return size() * percent;
}