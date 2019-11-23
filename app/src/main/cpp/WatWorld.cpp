#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <vector>
#include "WatWorld.h"
#include "CameraLine.h"
#include "SeenLine.h"
#include "utils.h"
#include "Line.h"

Point3d WatWorld::get_position() const
{
	return position;
}

void WatWorld::add_line(const Line &line)
{
	lines.push_back(line);
}

void WatWorld::set_lines(const std::list<Line> &input_lines)
{
	lines = input_lines;
}

Dir3d WatWorld::angle_to(Point3d point) const
{
	Dir3d result;

	// Use the coordinate system relative to the robot.
	point -= position;
	//point = rotate(point, -heading);
	
	result.yaw = add_euler_angle(atan2(point.y, point.x), -heading.yaw);
	//result.yaw = atan2(point.y, point.x);

	//result.pitch = add_euler_angle(atan2(-point.z, point.x), -heading.pitch);
	result.pitch = add_euler_angle(atan2(-point.z, sqrt(pow(point.x, 2) + pow(
		point.y, 2))), -heading.pitch);
	//result.pitch = atan2(-point.z, point.x);

	return result;
}

void WatWorld::set_heading(const double pitch, const double roll,
	const double yaw)
{
	heading.pitch = pitch;
	heading.roll = roll;
	heading.yaw = yaw;
}

void WatWorld::set_heading(const Dir3d &new_heading)
{
	heading = new_heading;
}

void WatWorld::set_position(const double x, const double y, const double z)
{
	position.x = x;
	position.y = y;
	position.z = z;
}

Point WatWorld::point(const Dir3d &dir) const
{
	Point result;
	//int x_offset = static_cast<int>(-round(dir.yaw * hor_pixels_per_radian));
	//int y_offset = static_cast<int>(round(dir.pitch * vert_pixels_per_radian));
	int x_offset = static_cast<int>(-round(dir.yaw * hor_pixels_per_radian));
	int y_offset = static_cast<int>(round(dir.pitch * vert_pixels_per_radian));

	result.x = (image_width / 2) + x_offset;
	result.y = (image_height / 2) + y_offset;

	return result;
}

Point3d WatWorld::point_on_ground(Point point)
{
	Point3d result;
	Dir3d line = dir(point);
	line.pitch = add_euler_angle(line.pitch, heading.pitch);
	line.yaw = add_euler_angle(line.yaw, heading.yaw);

	DirVec3d cam_to_point = line.direction_vector();
	double t = -cam_raise / cam_to_point.z;
	result.x = position.x + cam_to_point.x * t;
	result.y = position.y + cam_to_point.y * t;
	result.z = position.z - cam_raise;
	return result;
}

Line WatWorld::gen_line(const CameraLine &cam_line)
{
	Line result;
	result.point_1 = point_on_ground(cam_line.point_1);
	result.point_2 = point_on_ground(cam_line.point_2);
	return result;
}

SeenLine WatWorld::gen_seen_line(CameraLine &cam_line)
{
	SeenLine result;

	for (const Line &line : lines)
	{
		if (match(cam_line, line, margin_of_error))
		{
			result.line = cam_line.line_3d;
			result.added = true;
			matched_cam_lines.push_back(cam_line);
		}
	}

	if (!result.added)
	{
		result.line = new const Line(gen_line(cam_line));
	}
	result.matrix = Mat(raw_frame.size(), CV_8UC3);
	result.point_1 = cam_line.point_1;
	result.point_2 = cam_line.point_2;
	line(result.matrix, cam_line.point_1, cam_line.point_2,
		Scalar(255, 255, 255), line_thickness);
	return result;
}

Dir3d WatWorld::dir(const Point &point) const
{
	Dir3d result;
	result.yaw = ((image_width / 2) - point.x) * radian_per_hor_pixels;
	result.pitch = (point.y - (image_height / 2)) * radian_per_vert_pixels;
	return result;
}

void WatWorld::update_position()
{
	auto best_cam_line = std::max_element(matched_cam_lines.begin(),
		matched_cam_lines.end());

	if (best_cam_line != matched_cam_lines.end())
	{
		const Line *longest_line = best_cam_line->line_3d;
		position = get_camera_pos(longest_line->point_1,
			(dir(best_cam_line->point_1) + heading).direction_vector(),
			longest_line->point_2,
			(dir(best_cam_line->point_2) + heading).direction_vector());
	}
	else
	{
		std::cout << "Fatal error. No lines found.\n";
		return;
	}
}

void WatWorld::process_recorded_frame(const Mat &frame,
	const Dir3d &new_heading)
{
	raw_frame = frame;
	heading = process_heading(new_heading);
	std::vector<Vec4i> reducedLines = hough_lines(frame);
	seen_lines.clear();
	matched_cam_lines.clear();

	for (const Vec4i &reduced : reducedLines)
	{
		CameraLine camera_line(reduced);
		seen_lines.push_back(gen_seen_line(camera_line));
	}

	
	draw();
	update_position();
}

void WatWorld::process_frame(const Mat &frame, const Dir3d &new_heading)
{
	heading = process_heading(new_heading);
	std::vector<Vec4i> reducedLines = reduced_lines(frame);
	matched_cam_lines.clear();
/*
	Mat result = Mat::zeros(frame.rows, frame.cols, CV_8UC3);

	for (Vec4i reduced : reducedLines) {
		line(result, Point(reduced[0], reduced[1]), Point(reduced[2], reduced[3]),
			Scalar(255, 255, 255), 2);
	}
	imshow("Result", result);
	waitKey(0);
*/

	for (const Vec4i &reduced : reducedLines)
	{
		CameraLine camera_line(reduced);

		for (const Line &line : lines)
		{
			if (match(camera_line, line, margin_of_error))
			{
				matched_cam_lines.push_back(camera_line);
			}
		}

	}

	update_position();
}

bool WatWorld::match(CameraLine &camera_line, const Line &line,
	const double margin_of_error_percent, bool switched)
{
	const double size = camera_line.size();
	if (size < cam_line_min)
	{
		return false;
	}

	Point expected_point_1 = point(angle_to(line.point_1));
	Point expected_point_2 = point(angle_to(line.point_2));
	//const double error_margin = camera_line.margin_of_error(
	//	margin_of_error_percent);
	const double error_margin = distance(expected_point_1, expected_point_2)
		* margin_of_error_percent;
	const double dist_1 = distance(
		camera_line.point_1, expected_point_1);
	const double dist_2 = distance(
		camera_line.point_2, expected_point_2);
	if (in_margin_of_error(dist_1, error_margin, 0.0) && in_margin_of_error(
		dist_2, error_margin, 0.0))
	{
		camera_line.line_3d = &line;
		camera_line.error_percent = (dist_1 + dist_2) / size;
		return true;
	}
	else if (!switched)
	{
		std::swap(camera_line.point_1, camera_line.point_2);
		return match(camera_line, line, margin_of_error_percent, true);
	}

	return false;
}

Dir3d WatWorld::process_heading(Dir3d new_heading) const
{
	new_heading.pitch = add_euler_angle(new_heading.pitch, cam_pitch);
	return new_heading;
}

void WatWorld::handle_clicked_line(SeenLine &seen_line)
{
	if (!seen_line.added)
	{
		lines.push_back(*seen_line.line);
		delete seen_line.line;
		seen_line.line = &*(--lines.end());
		seen_line.added = true;
	}
	else
	{
		const Line *copied_line = new Line(*seen_line.line);
		std::list<Line>::iterator old_line_iter;
		for (auto it = lines.begin(); it != lines.end(); it++)
		{
			if (&(*it) == seen_line.line)
			{
				old_line_iter = it;
				break;
			}
		}
		lines.erase(old_line_iter);
		seen_line.line = copied_line;
		seen_line.added = false;
	}
}

void WatWorld::draw()
{
	rendered_frame = raw_frame;

	for (const SeenLine &seen_line : seen_lines)
	{
		Scalar color;
		// Green if added, red if not.
		if (seen_line.added)
		{
			color = Scalar(0, 255, 0);
		}
		else
		{
			color = Scalar(255, 0, 0);
		}

		line(rendered_frame, seen_line.point_1, seen_line.point_2, color,
			line_thickness);
	}

	imshow("Processed Frame", rendered_frame);
	setMouseCallback("Processed Frame", click_handler, this);
}

void WatWorld::on_click(int event, int x, int y)
{
	if (event != EVENT_LBUTTONDOWN)
	{
		return;
	}

	for (SeenLine &seen_line : seen_lines)
	{
		Vec3b pixel = seen_line.matrix.at<Vec3b>(y, x);
		if (!(pixel[0] == 0 && pixel[1] == 0 && pixel[2] == 0))
		{
			handle_clicked_line(seen_line);
			draw();
			break;
		}
	}
}

void click_handler(int event, int x, int y, int flags, void *instance)
{
	WatWorld *ptr = reinterpret_cast<WatWorld *>(instance);
	ptr->on_click(event, x, y);
}