#define _USE_MATH_DEFINES
#include <cmath>
#include "Dir3d.h"
#include "utils.h"

typedef Point3d DirVec3d;

Dir3d::Dir3d(double roll, double pitch, double yaw) : roll(roll),
pitch(pitch), yaw(yaw)
{}

Dir3d Dir3d::operator-() const
{
	Dir3d result;
	result.pitch = -pitch;
	result.yaw = -yaw;
	result.roll = -roll;
	return result;
}

DirVec3d Dir3d::direction_vector() const
{
	DirVec3d result;
	result.x = cos(yaw) * cos(pitch);
	result.y = sin(yaw) * cos(pitch);
	result.z = sin(-pitch);
	return result;
}

Dir3d Dir3d::operator+(const Dir3d &right_val) const
{
	Dir3d result;
	result.pitch = add_euler_angle(pitch, right_val.pitch);
	result.roll = add_euler_angle(roll, right_val.roll);
	result.yaw = add_euler_angle(yaw, right_val.yaw);
	return result;
}