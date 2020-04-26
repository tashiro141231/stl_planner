#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <vector>
#include <string>
#include <iostream>
#include <cmath>

#include "stl_planner/nav_core.h"

unsigned char* map_to_raw(nav_msgs::OccupancyGrid map);
nav_msgs::Path path_to_rospath(std::vector<Point> path, std::string global_frame);
double CalcDirection(Point a, Point b);
Point pose_to_point(geometry_msgs::Pose pose);
double quaternion_to_theta(geometry_msgs::Quaternion orientation);
