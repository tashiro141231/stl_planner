
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

#include <stl_planner/pp_ros.h>
#include <stl_planner/planner_base_ros.h>

PurePursuit_ROS::PurePursuit_ROS(tf2_ros::Buffer& tf) : PlannerBaseROS(tf)
 {
   planner_initialized_ = false;
  PlannerInitialize();
}

void PurePursuit_ROS::PlannerInitialize() {
  if(!planner_initialized_) {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    pub_pp_vw_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1); 
    //pub_goal_ = nh.advertise<geometry_msgs::PoseStamped>("purepursuit_planner/goal", 1);
    pub_pp_path_ = nh.advertise<nav_msgs::Path>("purepursuit_local_path", 1000);

    double max_vel, min_vel, max_acc, max_w, min_w, max_dw, dt;
    double v_resolution, w_resolution, predict_time;
    double goal_gain, speed_gain, ob_gain, robot_radius;

    private_nh.getParam("pp/max_vel", max_vel);
    private_nh.getParam("pp/min_vel", min_vel);
    private_nh.getParam("pp/max_acc", max_acc);
    private_nh.getParam("pp/max_w", max_w);
    private_nh.getParam("pp/min_w", min_w);

    pp.Initialize(max_vel, min_vel, max_acc, max_w, min_w) ;
    planner_initialized_ = true;
  }
}

void PurePursuit_ROS::setGoal(geometry_msgs::PoseStamped msg) {
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
  pp.setStartGoal(getCurrentPos(), goal);
  pub_goal_.publish(target);
  //nav_msgs::Path p = path_to_rospath(pp.getPath(), getGlobalFrame()); //calc path to goal
  //PubLocalPath(p);
  //ROS_INFO("outing");
}

void PurePursuit_ROS::setMap(int width, int height, double resolution, Point lower_left, unsigned char* map) {
  //pp.setMap(width, height, resolution, lower_left, map);
}

void PurePursuit_ROS::setCostMap(int width, int height, double resolution, Point lower_left, unsigned char* map) {
  //ROS_INFO("Konnichiha cost map ha jissou sitenaiyo!!");
  //pp.setCostMap(getCostMapWidth(), getCostMapHeight(), getCostMapResolution(), getCostMapLowerLeft(), getCostMapRaw());
}

void PurePursuit_ROS::setCurrentPositionToPlanner(Point point) {
  pp.setCurrentPosition(point);
}

void PurePursuit_ROS::PubLocalPath(nav_msgs::Path path) {
  //pub_pp_path_.publish(path);
}

void PurePursuit_ROS::PubGlobalPath(nav_msgs::Path path) {
  //pub_gp_.publish(path);
}

void PurePursuit_ROS::PubVelOmgOutput(double v, double w) {
  geometry_msgs::Twist out;
  out.linear.x = v;
  out.angular.z = w;
  pub_pp_vw_.publish(out);
}

void PurePursuit_ROS::main_loop() {
  ros::Rate loop_rate(getLoopRate());
  double V=0;
  double W=0;
  while(ros::ok()) {
    std::cout<<"v= " <<V <<"  w= " << W<<std::endl;
  UpdateCurrentPosition();
    if(pp.goalCheck()) {
      PubVelOmgOutput(0,0);
    }else{
    }
    ros::spinOnce();
    std::cout<<"vw_update_check"<<std::endl;
    if(pp.UpdateVW()) {
      std::cout<<"running"<<std::endl;
      //nav_msgs::Path p = path_to_rospath(pp.getPath(), getGlobalFrame());
      //PubLocalPath(p);
      V = pp.getVelOut();
      W = pp.getOmgOut();
      PubVelOmgOutput(V, W);
      //sleep(60);
    }else{
      PubVelOmgOutput(0,0);
      std::cout<<"stop"<<std::endl;
    }
    loop_rate.sleep();
  }
}
