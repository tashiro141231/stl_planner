

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

#include <stl_planner/line_planner_ros.h>
#include <stl_planner/planner_base_ros.h>


LinePlannerROS::LinePlannerROS(tf2_ros::Buffer& tf) : PlannerBaseROS(tf)
{
  planner_initialized_ = false;
  PlannerInitialize();
}

void LinePlannerROS::PlannerInitialize() {
  if(!planner_initialized_) {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    pub_gp_ = nh.advertise<nav_msgs::Path>("line_planner_path", 1000);
    pub_line_vw_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1); 
    pub_goal_ = nh.advertise<geometry_msgs::PoseStamped>("line_planner/goal", 1);

    double max_vel, min_vel,max_w,acc, rate;
    double Ko, Kp, Ke, max_eta;
    private_nh.getParam("line_planner/loop_rate", rate);
    private_nh.getParam("line_planner/max_vel", max_vel);
    private_nh.getParam("line_planner/min_vel", min_vel);
    private_nh.getParam("line_planner/vel_acc", acc);
    private_nh.getParam("line_planner/max_w", max_w);
    private_nh.getParam("line_planner/gain_omega", Ko);
    private_nh.getParam("line_planner/gain_phi", Kp);
    private_nh.getParam("line_planner/gain_eta", Ke);
    private_nh.getParam("line_planner/max_eta", max_eta);

    setLoopRate(rate);
    setMaxVel(max_vel);
    setMinVel(min_vel);
    setMaxOmega(max_w);
    setMinOmega(-max_w);
    setMaxAcc(acc);
    lp.Initialize(max_vel, min_vel, acc, max_w, max_eta, Ko, Kp, Ke, rate);

    planner_initialized_ = true;
  }
}

void LinePlannerROS::setMap(int width, int height, double resolution, Point lower_left, unsigned char* map) {
}

void LinePlannerROS::setCostMap(int width, int height, double resolution, Point lower_left, unsigned char* map) {
  ROS_INFO("Konichiha");
}

void LinePlannerROS::setGoal(geometry_msgs::PoseStamped msg) {
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
  lp.setStartGoal(getCurrentPos(), goal);
  pub_goal_.publish(target);
  ROS_INFO("Received goal.");
  //nav_msgs::Path p = path_to_rospath(gp.calc_path_astar(), getGlobalFrame()); //calc path to goal

}

void LinePlannerROS::setCurrentPositionToPlanner(Point point) {
  lp.setCurrentPosition(point);
}

void LinePlannerROS::setOdomVelOmega(double vel, double omega) {
  lp.setCurrentVelOmega(vel, omega);
}

void LinePlannerROS::PubGlobalPath(nav_msgs::Path path) {
  pub_gp_.publish(path);
}

void LinePlannerROS::PubVelOmgOutput(double v, double w) {
  geometry_msgs::Twist out;
  out.linear.x = v;
  out.angular.z = w;
  pub_line_vw_.publish(out);
}


void LinePlannerROS::main_loop() {
  double rate = getLoopRate(); 
  ros::Rate loop_rate(rate);
  double V=0;
  double W=0;

  while(ros::ok()) {
    UpdateCurrentPosition();
    if(lp.goalCheck()) {
      PubVelOmgOutput(0,0);
      lp.setCurrentVelOmega(0, 0);
    }
    ros::spinOnce();
    if(lp.UpdateVW()) {
      V = lp.getVelOut();
      W = lp.getOmgOut();
      PubVelOmgOutput(V, W);
    }
    loop_rate.sleep();
  }
  PubVelOmgOutput(0, 0);
}
