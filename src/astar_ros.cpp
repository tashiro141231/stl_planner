
#include "ros/ros.h"

#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Path.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"

#include <string>
#include <vector>
#include <iostream>

#include <stl_planner/astar_ros.h>
#include <stl_planner/planner_base_ros.h>

AStarROS::AStarROS(tf2_ros::Buffer& tf) : PlannerBaseROS(tf)
 {
  Initialize();
  ros::NodeHandle nh;
  pub_gp_ = nh.advertise<nav_msgs::Path>("global_plan", 1000);
}

void AStarROS::setDWAParams(){
  ;
}

void AStarROS::setMap(int width, int height, double resolution, Point lower_left, unsigned char* map) {
  gp.setMap(width, height, resolution, lower_left, map);
}

void AStarROS::setCostMap(int width, int height, double resolution, Point lower_left, unsigned char* map) {
  ROS_INFO("Konichiha");
}

void AStarROS::setGoal(geometry_msgs::PoseStamped msg) {
  double x,y,theta;
  x = msg.pose.position.x;
  y = msg.pose.position.y;
  theta = quaternion_to_theta(msg.pose.orientation);
  Point goal;
  goal.x = x;
  goal.y = y;
  goal.theta = theta;
  gp.setStartGoal(getCurrentPos(), goal);
  nav_msgs::Path p = path_to_rospath(gp.calc_path_astar(), getGlobalFrame()); //calc path to goal

  PubGlobalPath(p);
  ROS_INFO("outing");
}

void AStarROS::setCurrentPositionToPlanner(Point point) {
  gp.setStartPoint(point);
}

void AStarROS::PubGlobalPath(nav_msgs::Path path) {
  pub_gp_.publish(path);
}

void AStarROS::main_loop() {
  ros::Rate loop_rate(getLoopRate());
  while(ros::ok()) {
    UpdateCurrentPosition();
    ros::spinOnce();
    // gp.calc_path();
    loop_rate.sleep();
  }
}
