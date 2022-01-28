
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
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include <string>
#include <vector>
#include <iostream>

#include <stl_planner/dwa_ros.h>
#include <stl_planner/planner_base_ros.h>

DWA_ROS::DWA_ROS(tf2_ros::Buffer& tf) : PlannerBaseROS(tf)
 {
  planner_initialized_ = false;
  stop_navigation_=false;
  stop_mode_ =false;
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

    pub_navigation_state_ = nh.advertise<std_msgs::String>("dwa_navigation_state",1000);
    sub_stop_mode_ = nh.subscribe<std_msgs::Bool>("stop_mode", 1,&DWA_ROS::StopModeCallback,this);
    sub_stop_navigation_ = nh.subscribe<std_msgs::Bool>("stop_navigation", 1,&DWA_ROS::StopNavigationCallback,this);    

    double max_vel, min_vel, max_acc, max_w, min_w, max_dw, dt;
    double v_resolution, w_resolution, predict_time;
    double goal_gain, speed_gain, ob_gain, robot_radius;

    private_nh.getParam("dwa/max_vel", max_vel);
    private_nh.getParam("dwa/min_vel", min_vel);
    private_nh.getParam("dwa/max_acc", max_acc);
    private_nh.getParam("dwa/max_w", max_w);
    private_nh.getParam("dwa/min_w", min_w);
    private_nh.getParam("dwa/max_dw", max_dw);
    private_nh.getParam("dwa/dt", dt);
    private_nh.getParam("dwa/v_resolution", v_resolution);
    private_nh.getParam("dwa/w_resolution", w_resolution);
    private_nh.getParam("dwa/predict_time", predict_time);
    private_nh.getParam("dwa/goal_gain", goal_gain);
    private_nh.getParam("dwa/speed_gain", speed_gain);
    private_nh.getParam("dwa/ob_gain", ob_gain);
    private_nh.getParam("dwa/robot_radius", robot_radius);


    dwa.Initialize(max_vel, min_vel, max_acc, max_w, min_w, 
            max_dw, dt, v_resolution, w_resolution, predict_time, 
            goal_gain, speed_gain, ob_gain, robot_radius) ;

    planner_initialized_ = true;
    time_goalset=0;
    now=0;
    waiting=false;
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
  time_goalset=time(nullptr);
  waiting=true;
  PubLocalPath(p);
  //ROS_INFO("outing");
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

void DWA_ROS::StopModeCallback(std_msgs::Bool stop_mode){
  stop_mode_=stop_mode.data;
  dwa.stop_mode_=stop_mode_;
}

void DWA_ROS::StopNavigationCallback(std_msgs::Bool stop_navigation){
  stop_navigation_=stop_navigation.data;
  dwa.stop_navigation_=stop_navigation.data;
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
      //sleep(60);
    }else{
      PubVelOmgOutput(0,0);
      std::cout<<"stop"<<std::endl;
    }
    loop_rate.sleep();
  }
}
