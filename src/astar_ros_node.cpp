/**
 * @file astar_ros_node.cpp
 * @brief Astar ROS node.
 * @author Yosuke TASHIRO, Kotaro HIHARA
 * @date 2020.07.26
*/

#include <string>
#include <vector>

#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"

#include "stl_planner/planner_base_ros.h"
#include "stl_planner/astar_ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "AStar_ROS_node");
  ros::NodeHandle n;
  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);
  AStarROS astar_node(buffer);

  astar_node.main_loop();

  return 0;
}

