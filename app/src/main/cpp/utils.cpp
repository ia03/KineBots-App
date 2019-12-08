#define _USE_MATH_DEFINES
#include <cmath>
#include "utils.h"
#include "WatWorld.h"
#include <string>
#include <fstream>

std::vector<Dir3d> read_gyros(const std::string &filename)
{
	std::vector<Dir3d> gyros;
	std::ifstream gyros_file(filename);
	double roll, pitch, yaw;
	while (gyros_file >> roll >> pitch >> yaw)
	{
		Dir3d gyro;
		gyro.roll = roll;
		gyro.pitch = pitch;
		gyro.yaw = yaw;
		gyros.push_back(gyro);
	}

	gyros_file.close();
	return gyros;
}

std::list<Line> read_lines(const std::string &filename)
{
	std::list<Line> lines;
	std::ifstream lines_file(filename);
	double x1, y1, z1, x2, y2, z2;
	while (lines_file >> x1 >> y1 >> z1 >> x2 >> y2 >> z2)
	{
		Line line;
		line.point_1.x = x1;
		line.point_1.y = y1;
		line.point_1.z = z1;
		line.point_2.x = x2;
		line.point_2.y = y2;
		line.point_2.z = z2;
		lines.push_back(line);
	}

	lines_file.close();
	return lines;
}

void write_lines(const std::string &filename, const std::list<Line> &lines)
{
	std::ofstream lines_file(filename);

	for (auto const &line : lines)
	{
		lines_file << line.point_1.x << ' ' << line.point_1.y << ' '
			<< line.point_1.z << ' ' << line.point_2.x << ' '
			<< line.point_2.y << ' ' << line.point_2.z << '\n';
	}
	
	lines_file.close();
}

Point3d rotate(Point3d point, const Dir3d &angle)
{
	double cos_a = cos(angle.yaw);
	double sin_a = sin(angle.yaw);

	double cos_b = cos(angle.pitch);
	double sin_b = sin(angle.pitch);

	double cos_c = cos(angle.roll);
	double sin_c = sin(angle.roll);

	double axx = cos_a * cos_b;
	double axy = cos_a * sin_b * sin_c - sin_a * cos_c;
	double axz = cos_a * sin_b * cos_c + sin_a * sin_c;

	double ayx = sin_a * cos_b;
	double ayy = sin_a * sin_b * sin_c + cos_a * cos_c;
	double ayz = sin_a * sin_b * cos_c - cos_a * sin_c;

	double azx = -sin_b;
	double azy = cos_b * sin_c;
	double azz = cos_b * cos_c;

	double px = point.x;
	double py = point.y;
	double pz = point.z;

	point.x = axx * px + axy * py + axz * pz;
	point.y = ayx * px + ayy * py + ayz * pz;
	point.z = azx * px + azy * py + azz * pz;

	return point;
}

Point3d get_camera_pos(const Point3d &point_1, DirVec3d dir_vec_1,
	const Point3d &point_2, DirVec3d dir_vec_2)
{
	// See http://quickmathintuitions.org/quick-method-to-find-line-of-shortest-distance-for-skew-lines/

	// The leftmost value is t1.
	// The rightmost value is t2.
	Mat_<double> coeff_mat(2, 2, CV_64FC1);
	Mat_<double> const_mat(2, 1, CV_64FC1);
	Mat_<double> time_mat(2, 1, CV_64FC1);
	dir_vec_1 = -dir_vec_1;
	dir_vec_2 = -dir_vec_2;

	// First equation.
	coeff_mat(0, 0) = pow(dir_vec_1.x, 2) + pow(dir_vec_1.y, 2)
		+ pow(dir_vec_1.z, 2);
	coeff_mat(0, 1) = -dir_vec_2.x * dir_vec_1.x - dir_vec_2.y
		* dir_vec_1.y - dir_vec_2.z * dir_vec_1.z;
	const_mat(0, 0) = point_2.x * dir_vec_1.x - point_1.x
		* dir_vec_1.x + point_2.y * dir_vec_1.y - point_1.y * dir_vec_1.y
		+ point_2.z * dir_vec_1.z - point_1.z * dir_vec_1.z;

	// Second equation.
	coeff_mat(1, 0) = dir_vec_1.x * dir_vec_2.x + dir_vec_1.y
		* dir_vec_2.y + dir_vec_1.z * dir_vec_2.z;
	coeff_mat(1, 1) = -pow(dir_vec_2.x, 2) - pow(dir_vec_2.y, 2)
		- pow(dir_vec_2.z, 2);
	const_mat(1, 0) = point_2.x * dir_vec_2.x - point_1.x
		* dir_vec_2.x + point_2.y * dir_vec_2.y - point_1.y * dir_vec_2.y
		+ point_2.z * dir_vec_2.z - point_1.z * dir_vec_2.z;

	solve(coeff_mat, const_mat, time_mat);

	double time_1 = time_mat(0, 0);
	double time_2 = time_mat(1, 0);

	// Closest point on the first line.
	Vec3d closest_on_line_1 = static_cast<Vec3d>(point_1) + (time_1
		* static_cast<Vec3d>(dir_vec_1));

	// Closest point on the second line.
	Vec3d closest_on_line_2 = static_cast<Vec3d>(point_2) + (time_2
		* static_cast<Vec3d>(dir_vec_2));

	// Find averages.
	return static_cast<Point3d>((closest_on_line_1 + closest_on_line_2) / 2);
}

double add_euler_angle(const double a, const double b)
{
	double result = a + b;

	if (result > M_PI)
	{
		result -= 2 * M_PI;
	}
	else if (result < -M_PI)
	{
		result += 2 * M_PI;
	}

	return result;
}