/**
 * @file state_machine_node.cpp
 * @brief State Machine ROS node.
 * @author Yosuke TASHIRO, Kotaro HIHARA
 * @date 2020.04.19
*/

#include <string>
#include <vector>

#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"

#include "stl_planner/state_machine_ros.h"

using namespace smr;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "State_machine");
  ros::NodeHandle n;
  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);

  StateMachineROS smr(buffer);
  ROS_INFO("Initialied stl_planner.");

  smr.main_loop();

  return 0;
}
