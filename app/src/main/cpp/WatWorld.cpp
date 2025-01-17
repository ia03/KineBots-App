#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <vector>
#include "WatWorld.h"
#include "CameraLine.h"
#include "SeenLine.h"
#include "utils.h"
#include "Line.h"
#include "Motor_direction.h"

WatWorld world;

Point3d WatWorld::get_position() const
{
	return position;
}

void WatWorld::add_line(const Line &line)
{
    lines.push_back(line);
}

void WatWorld::add_line(const double x1, const double y1, const double z1,
	const double x2, const double y2, const double z2)
{
    Line line;
    line.point_1 = Point3d(x1, y1, z1);
    line.point_2 = Point3d(x2, y2, z2);
    add_line(line);
}

void WatWorld::add_node(std::string key, double x, double y, double z)
{
	nodes[key] = PosNode();
	nodes[key].position = Point3d(x, y, z);
}

void WatWorld::add_connection(std::string key_1, std::string key_2)
{
	nodes[key_1].connections.push_back(&nodes[key_2]);
	nodes[key_2].connections.push_back(&nodes[key_1]);
}

void WatWorld::set_lines(const std::list<Line> &input_lines)
{
	lines = input_lines;
}

void WatWorld::rotate_lines(const Dir3d &angle)
{
	for (auto &line : lines)
	{
		line.point_1 = rotate(line.point_1, angle);
		line.point_2 = rotate(line.point_2, angle);
	}
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
	result.point_1 = cam_line.point_1;
	result.point_2 = cam_line.point_2;
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
		//std::cout << "Fatal error. No lines found.\n";
		return;
	}
}

void WatWorld::process_frame(JNIEnv *env, jobject lines_obj,
		const Dir3d &new_heading)
{
	jclass lines_class = env->GetObjectClass(lines_obj);
	jmethodID lines_rows_id = env->GetMethodID(lines_class, "rows", "()I");
	jmethodID lines_get_id = env->GetMethodID(lines_class, "get", "(II)[D");
	jint rows = env->CallIntMethod(lines_obj, lines_rows_id);

	set_heading(process_heading(new_heading));
	matched_cam_lines.clear();

	for (int i = 0; i < static_cast<int>(rows); i++)
	{
		jobject line = env->CallObjectMethod(lines_obj, lines_get_id,
				static_cast<jint>(i), 0);
		CameraLine camera_line(env, reinterpret_cast<jdoubleArray>(line));

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
	const double error_margin = distance<Point>(expected_point_1,
		expected_point_2) * margin_of_error_percent;
	const double dist_1 = distance<Point>(
		camera_line.point_1, expected_point_1);
	const double dist_2 = distance<Point>(
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



extern "C"
JNIEXPORT jdoubleArray JNICALL
Java_com_watworld_kinebots_MainActivity_getPos(JNIEnv *env, jobject thiz,
	jobject lines)
{
	jclass this_class = env->GetObjectClass(thiz);
	jfieldID orientation_angles_id = env->GetFieldID(this_class,
		"orientationAngles", "[F");
	jobject orientation_angles_obj = env->GetObjectField(thiz,
		orientation_angles_id);
	jfloatArray *orientation_angles_jarray = reinterpret_cast<jfloatArray *>(
		&orientation_angles_obj);
	float *orientation_angles = env->GetFloatArrayElements(
		*orientation_angles_jarray, nullptr);
	float roll = orientation_angles[1];
	float pitch = orientation_angles[2];
	float yaw = orientation_angles[0];
	env->ReleaseFloatArrayElements(*orientation_angles_jarray,
		orientation_angles, 0);
	Dir3d heading(roll, pitch, yaw);

	world.process_frame(env, lines, heading);
	Point3d position = world.get_position();

	jdoubleArray result = env->NewDoubleArray(3);
	jdouble result_data[3];
	result_data[0] = position.x;
	result_data[1] = position.y;
	result_data[2] = position.z;
	env->SetDoubleArrayRegion(result, 0, 3, result_data);

	return result;
}

Motor_direction WatWorld::get_motor_direction()
{

	add_node("1", 1, 0, 0);
	add_node("2", 2, 0, 0);
	add_node("3", 3, 0, 0);
	add_connection("1", "2");
	add_connection("1", "3");
	add_connection("2", "3");
	calculate_path(&nodes["3"]);
	return motor_direction;
}

void WatWorld::calculate_path(PosNode *goal)
{
	// The starting node is the one that's closest to the current position.
	PosNode *starting_node = &(std::min_element(nodes.begin(), nodes.end(),
		[this](const auto &node_1, const auto &node_2) -> bool {
        return distance(node_1.second.position, position) < distance(
        	node_2.second.position, position);
	})->second);
	std::vector<PosNode *> open_nodes;
	std::vector<PosNode *> closed_nodes;

	open_nodes.push_back(starting_node);
	while (!open_nodes.empty())
	{
		std::vector<PosNode *>::iterator current_node = std::min_element(
			open_nodes.begin(), open_nodes.end(), [](PosNode *node_1,
				PosNode *node_2) -> bool {
				return node_1->f < node_2->f;
			});
		closed_nodes.push_back(*current_node);
		open_nodes.erase(current_node);

		// Pathfinding is complete.
		if (*current_node == goal)
		{
			PosNode *traced_node = *current_node;
			while (traced_node != nullptr)
			{
				path.push_back(traced_node);
				traced_node = traced_node->parent;
			}
			std::reverse(path.begin(), path.end());
			return;
		}
		for (auto &node : (*current_node)->connections)
		{
			if (std::find(closed_nodes.begin(),
				closed_nodes.end(), node) != closed_nodes.end())
			{
				continue;
			}

			double new_g = (*current_node)->g + distance(node->position,
				(*current_node)->position);
			if ((std::find(open_nodes.begin(), open_nodes.end(),
				node) != open_nodes.end()) && new_g > node->g)
			{
				continue;
			}
			node->parent = (*current_node);
			node->g = new_g;
			node->h = distance(node->position, goal->position);
			node->f = node->g + node->h;
			open_nodes.push_back(node);
		}
	}

}

extern "C"
JNIEXPORT void JNICALL
Java_com_watworld_kinebots_MainActivity_addLine(JNIEnv *env, jobject thiz,
    jdouble x1, jdouble y1, jdouble z1, jdouble x2, jdouble y2, jdouble z2)
{
    world.add_line(x1, y1, z1, x2, y2, z2);
}

extern "C"
JNIEXPORT void JNICALL
Java_com_watworld_kinebots_MainActivity_setRotation(JNIEnv *env, jobject thiz,
	jdouble roll, jdouble pitch, jdouble yaw)
{
	Dir3d dir(roll, pitch, yaw);
	world.rotate_lines(dir);
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_watworld_kinebots_MainActivity_getMotorDirection(JNIEnv *env,
	jobject thiz)
{
	return static_cast<jint>(static_cast<int>(world.get_motor_direction()));
}

extern "C"
JNIEXPORT void JNICALL
Java_com_watworld_kinebots_MainActivity_addNode(JNIEnv *env, jobject thiz,
	jstring key, jdouble x, jdouble y, jdouble z)
{
	std::string converted_key = env->GetStringUTFChars(key, nullptr);
	world.add_node(converted_key, x, y, z);
}

extern "C"
JNIEXPORT void JNICALL
Java_com_watworld_kinebots_MainActivity_addConnection(JNIEnv *env, jobject thiz,
	jstring key1, jstring key2)
{
	std::string converted_key_1 = env->GetStringUTFChars(key1, nullptr);
	std::string converted_key_2 = env->GetStringUTFChars(key2, nullptr);
	world.add_connection(converted_key_1, converted_key_2);
}