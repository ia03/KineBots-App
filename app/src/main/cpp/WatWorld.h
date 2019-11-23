#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include "Dir3d.h"
#include "CameraLine.h"
#include "SeenLine.h"
#include "Line.h"
#include "Point.h"
#include <list>

typedef Point3d DirVec3d;

class WatWorld
{
	const double margin_of_error = 0.3;
	const double horizontal_fov = 1.59;
	const double vertical_fov = 1.30;
	const int image_height = 1080;
	const int image_width = 1920;
	const double vert_pixels_per_radian = static_cast<double>(image_height)
		/ vertical_fov;
	const double hor_pixels_per_radian = static_cast<double>(image_width)
		/ horizontal_fov;
	const double radian_per_vert_pixels = vertical_fov
		/ static_cast<double>(image_height);
	const double radian_per_hor_pixels = horizontal_fov
		/ static_cast<double>(image_width);
	const double cam_line_min = 50.0;
	const double cam_raise = 0.28;
	const double cam_pitch = 0.75;
	const double line_thickness = 4;

	Dir3d heading;
	Point3d position;

	std::list<Line> lines;
	std::vector<Point3d> navdots;
	
	std::vector<SeenLine> seen_lines;
	std::vector<CameraLine> matched_cam_lines;  // Reset every frame.


public:
	WatWorld() = default;

	Point3d get_position() const;

	void add_line(const Line &line);

	void set_lines(const std::list<Line> &input_lines);
	
	Dir3d angle_to(Point3d point) const;

	void set_heading(const double pitch, const double roll, const double yaw);

	void set_heading(const Dir3d &new_heading);

	void set_position(const double x, const double y, const double z);

	Point point(const Dir3d &dir) const;

	Point3d point_on_ground(Point point);

	Line gen_line(const CameraLine &cam_line);

	SeenLine gen_seen_line(CameraLine &cam_line);

	Dir3d dir(const Point &point) const;
	
	void update_position();

	// cam_lines is an output argument.
	void process_recorded_frame(const Mat &frame,
		const Dir3d &new_heading);

	void process_frame(const Mat &frame, const Dir3d &new_heading);

	// Checks if this camera line matches a line in 3D space and, if so,
	// assigns that line to the camera line.
	bool match(CameraLine &camera_line, const Line &line,
		const double margin_of_error_percent, bool switched = false);

	Dir3d process_heading(Dir3d heading) const;

	void handle_clicked_line(SeenLine &seen_line);

	void draw();

	void on_click(int event, int x, int y);
};

static void click_handler(int event, int x, int y, int flags, void *instance);