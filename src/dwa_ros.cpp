
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
   planner_initialized_ = false;
  PlannerInitialize();
}

void DWA_ROS::PlannerInitialize() {
  if(!planner_initialized_) {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    pub_gp_ = nh.advertise<nav_msgs::Path>("dwa_global_path", 1000);
    pub_dwa_vw_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1); 
    pub_goal_ = nh.advertise<geometry_msgs::PoseStamped>("dwa_planner/goal", 1);
    pub_dwa_path_ = nh.advertise<nav_msgs::Path>("dwa_local_path", 1000);

    planner_initialized_ = true;
  }
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
  nav_msgs::Path p = path_to_rospath(dwa.getPath(), getGlobalFrame()); //calc path to goal

  PubLocalPath(p);
  ROS_INFO("outing");
}

void DWA_ROS::setMap(int width, int height, double resolution, Point lower_left, unsigned char* map) {
  dwa.setMap(width, height, resolution, lower_left, map);
}

void DWA_ROS::setCostMap(int width, int height, double resolution, Point lower_left, unsigned char* map) {
  ROS_INFO("Konnichiha cost map ha jissou sitenaiyo!!");
  dwa.setCostMap(getCostMapWidth(), getCostMapHeight(), getCostMapResolution(), getCostMapLowerLeft(), getCostMapRaw());
}

void DWA_ROS::setCurrentPositionToPlanner(Point point) {
  dwa.setCurrentPosition(point);
}

void DWA_ROS::PubLocalPath(nav_msgs::Path path) {
  pub_dwa_path_.publish(path);
}

void DWA_ROS::PubGlobalPath(nav_msgs::Path path) {
  pub_gp_.publish(path);
}

void DWA_ROS::PubVelOmgOutput(double v, double w) {
  geometry_msgs::Twist out;
  out.linear.x = v;
  out.angular.z = w;
  pub_dwa_vw_.publish(out);
}

void DWA_ROS::main_loop() {
  ros::Rate loop_rate(getLoopRate());
  double V=0;
  double W=0;
  while(ros::ok()) {
    std::cout<<"v= " <<V <<"  w= " << W<<std::endl;
  UpdateCurrentPosition();
    if(dwa.goalCheck()) {
      PubVelOmgOutput(0,0);
    }else{
    }
    ros::spinOnce();
    std::cout<<"vw_update_check"<<std::endl;
    if(dwa.UpdateVW()) {
      std::cout<<"running"<<std::endl;
      //nav_msgs::Path p = path_to_rospath(dwa.getPath(), getGlobalFrame());
      //PubLocalPath(p);
      V = dwa.getVelOut();
      W = dwa.getOmgOut();
      PubVelOmgOutput(V, W);
    }else{
      PubVelOmgOutput(0,0);
      std::cout<<"stop"<<std::endl;
      std::cout<<"robot radius"<<dwa.robot_radius<<std::endl;
    }
    loop_rate.sleep();
  }
}
