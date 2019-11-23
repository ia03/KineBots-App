#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include "WatWorld.h"
#include "Dir3d.h"
#include <list>

std::vector<Dir3d> read_gyros(const std::string &filename);

std::list<Line> read_lines(const std::string &filename);

void write_lines(const std::string &filename, const std::list<Line> &lines);

double angle_to(double point);

// Rotates a point around the origin in 3D.
Point3d rotate(Point3d point, const Dir3d &angle);

// Locates the camera based on the directions to 2 known points.
Point3d get_camera_pos(const Point3d &point_1, DirVec3d dir_vec_1,
	const Point3d &point_2, DirVec3d dir_vec_2);

double distance(const Point &point_1, const Point &point_2);

double add_euler_angle(const double a, const double b);

template<typename T>
bool in_margin_of_error(const T value, const T margin_of_error,
	const T target)
{
	return (target - margin_of_error) <= value && value
		<= (target + margin_of_error);
}