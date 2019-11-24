#ifndef DIR3D_H
#define DIR3D_H
#include <opencv2/opencv.hpp>

using namespace cv;

typedef Point3d DirVec3d;

// SI units should be used. Therefore:
// 1. Angles should be in radians.
// 2. Distances should be in metres.

// ENU (East North Down) convention.
// Default yaw (0) is east.
struct Dir3d
{
	Dir3d(double pitch, double roll, double yaw);
	Dir3d() = default;

	double roll;
	double pitch;
	double yaw;

	// Returns this angle in the form of a direction vector.
	DirVec3d direction_vector() const;

	// Negation operator.
	Dir3d operator-() const;

	Dir3d operator+(const Dir3d &right_val) const;
};

#endif