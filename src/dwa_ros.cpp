
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

#include <stl_planner/dwa_ros.h>
#include <stl_planner/planner_base_ros.h>

DWA_ROS::DWA_ROS(tf2_ros::Buffer& tf) : PlannerBaseROS(tf)
 {
  Initialize();
}

void DWA_ROS::setDWAParams(){
  ;
}

void DWA_ROS::setGoal(geometry_msgs::PoseStamped msg) {
  double x,y,theta;
  geometry_msgs::PoseStamped target;
  target = msg;
  x = msg.pose.position.x;
  y = msg.pose.position.y;
  theta = quaternion_to_theta(msg.pose.orientation);
  Point goal;
  goal.x = x;
  goal.y = y;
  goal.theta = theta;
  dwa.setStartGoal(getCurrentPos(), goal);
  pub_goal_.publish(target);
  nav_msgs::Path p = path_to_rospath(dwa.calc_path_dwa(), getGlobalFrame()); //calc path to goal

  PubLocalPath(p);
  ROS_INFO("outing");
}

void DWA_ROS::setMap(int width, int height, double resolution, Point lower_left, unsigned char* map) {
  dwa.SetMap(width, height, resolution, lower_left, map);
}

void DWA_ROS::setCostMap(int width, int height, double resolution, Point lower_left, unsigned char* map) {
  ROS_INFO("Konnichiha cost map ha jissou sitenaiyo!!");
  dwa.SetCostMap(getCostMapWidth(), getCostMapHeight(), getCostMapResolution(), getCostMapLowerLeft(), getCostgMapRaw());
}

void DWA_ROS::setCurrentPositionToPlanner(Point point) {
  dwa.setCurrentPosition(point);
}

void DWA_ROS::PubLocalPath(nav_msgs::Path path) {
  pub_dwa_path_.publish(path);
}

void LPlannerROS::PubGlobalPath(nav_msgs::Path path) {
  pub_gp_.publish(path);
}

void LPlannerROS::PubVelOmgOutput(double v, double w) {
  geometry_msgs::Twist out;
  out.linear.x = v;
  out.angular.z = w;
  pub_dwa_vw_.publish(out);
}

void DWA_ROS::main_loop() {
  ros::Rate loop_rate(getLoopRate());
  while(ros::ok()) {
  UpdateCurrentPosition();
    if(dwa.goalCheck()) {
      PubVelOmgOutput(0,0);
    }
    ros::spinOnce();
    if(dwa.UpdateVW()) {
      V = dwa.getVelOut();
      W = dwa.getOmgOut();
      PubVelOmgOutput(V, W);
    }
    loop_rate.sleep();
  }
}
