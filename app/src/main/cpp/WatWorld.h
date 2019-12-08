#ifndef WATWORLD_H
#define WATWORLD_H
#include <vector>
#include <queue>
#include "Dir3d.h"
#include "CameraLine.h"
#include "SeenLine.h"
#include "Line.h"
#include "Motor_direction.h"
#include "PosNode.h"
#include <opencv2/opencv.hpp>
#include <list>
#include <string>
#include <unordered_map>

using namespace cv;

typedef Point3d DirVec3d;

class WatWorld
{
    const double margin_of_error = 0.2;
    const double horizontal_fov = 1; // ~1
    const double vertical_fov = 0.837758041;  // ~.66, .5585
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
    const double cam_raise = 0;
    const double cam_pitch = 0;

    Dir3d heading;
    Point3d position;
    Motor_direction motor_direction;

    std::list<Line> lines;
    std::unordered_map<std::string, PosNode> nodes;
    std::vector<PosNode *> path;

    std::vector<CameraLine> matched_cam_lines;  // Reset every frame.


public:
    WatWorld() = default;

    Point3d get_position() const;

    void add_line(const Line &line);

    void add_line(const double x1, const double y1, const double z1,
        const double x2, const double y2, const double z2);

    void add_node(std::string key, double x, double y, double z);

    void add_connection(std::string key_1, std::string key_2);

    void set_lines(const std::list<Line> &input_lines);

    void rotate_lines(const Dir3d &angle);
    
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

    void process_frame(JNIEnv *env, jobject lines, const Dir3d &new_heading);

    // Checks if this camera line matches a line in 3D space and, if so,
    // assigns that line to the camera line.
    bool match(CameraLine &camera_line, const Line &line,
        const double margin_of_error_percent, bool switched = false);

    Dir3d process_heading(Dir3d heading) const;

    Motor_direction get_motor_direction();

    void calculate_path(PosNode *goal);
};

extern "C"
JNIEXPORT jdoubleArray JNICALL
Java_com_watworld_kinebots_MainActivity_getPos(JNIEnv *env, jobject thiz,
    jobject lines);

extern "C"
JNIEXPORT void JNICALL
Java_com_watworld_kinebots_MainActivity_addLine(JNIEnv *env, jobject thiz,
    jdouble x1, jdouble y1, jdouble z1, jdouble x2, jdouble y2, jdouble z2);

extern "C"
JNIEXPORT void JNICALL
Java_com_watworld_kinebots_MainActivity_setRotation(JNIEnv *env, jobject thiz,
    jdouble roll, jdouble pitch, jdouble yaw);

extern "C"
JNIEXPORT jint JNICALL
Java_com_watworld_kinebots_MainActivity_getMotorDirection(JNIEnv *env,
    jobject thiz);

extern "C"
JNIEXPORT void JNICALL
Java_com_watworld_kinebots_MainActivity_addNode(JNIEnv *env, jobject thiz,
    jstring key, jdouble x, jdouble y, jdouble z);

extern "C"
JNIEXPORT void JNICALL
Java_com_watworld_kinebots_MainActivity_addConnection(JNIEnv *env, jobject thiz,
    jstring key1, jstring key2);
#endif
