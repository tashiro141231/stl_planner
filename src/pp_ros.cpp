
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
#include <time.h>


#include <stl_planner/pp_ros.h>
#include <stl_planner/planner_base_ros.h>

PurePursuit_ROS::PurePursuit_ROS(tf2_ros::Buffer& tf) : PlannerBaseROS(tf)
{
  planner_initialized_ = false;
  stop_navigation_=false;
  stop_mode_ =false;
  PlannerInitialize();
}

void PurePursuit_ROS::PlannerInitialize() {
  if(!planner_initialized_) {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    pub_pp_vw_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1); 
    //pub_goal_ = nh.advertise<geometry_msgs::PoseStamped>("purepursuit_planner/goal", 1);
    pub_pp_path_ = nh.advertise<nav_msgs::Path>("purepursuit_local_path", 1000);
    pub_navigation_state_ = nh.advertise<std_msgs::String>("pp_ros_navigation_state",1000);
    astar_sub_ = nh.subscribe<nav_msgs::Path>("global_plan", 1,&PurePursuit_ROS::GlobalPlanCallback,this);
    sub_stop_mode_ = nh.subscribe<std_msgs::Bool>("stop_mode", 1,&PurePursuit_ROS::StopModeCallback,this);
    sub_stop_navigation_ = nh.subscribe<std_msgs::Bool>("stop_navigation", 1,&PurePursuit_ROS::StopNavigationCallback,this);    
    double max_vel, min_vel, max_acc, stop_dec,max_w, min_w, max_dw, dt,set_vel,wp_range,goal_range;
    double stop_vel,stop_dist,look_dist;
    double v_resolution, w_resolution, predict_time;
    double goal_gain, speed_gain, ob_gain, robot_radius;

    private_nh.getParam("line_planner/loop_rate", rate_);
    private_nh.getParam("pp/max_vel", max_vel);
    private_nh.getParam("pp/min_vel", min_vel);
    private_nh.getParam("pp/max_acc", max_acc);
    private_nh.getParam("pp/stop_dec", stop_dec);
    private_nh.getParam("pp/max_w", max_w);
    private_nh.getParam("pp/min_w", min_w);
    private_nh.getParam("pp/max_dw", max_dw);
    private_nh.getParam("pp/set_vel", set_vel);
    private_nh.getParam("pp/wp_range", wp_range);
    private_nh.getParam("pp/goal_range", goal_range);
    private_nh.getParam("pp/stop_vel", stop_vel);
    private_nh.getParam("pp/look_dist", look_dist);
    private_nh.getParam("pp/stop_dist", stop_dist);
    private_nh.getParam("pp/predict_time", predict_time);
    dt=1/rate_;

    pp.Initialize(max_vel, min_vel, max_acc,stop_dec, max_w, min_w,set_vel,wp_range,goal_range,max_dw,dt,stop_vel,look_dist,stop_dist,predict_time) ;
    planner_initialized_ = true;
    time_goalset=0;
    now=0;
    waiting=false;
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
  time_goalset=time(nullptr);
  waiting=true;
  //pub_goal_.publish(target);
  
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
  pub_pp_path_.publish(path);
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

void PurePursuit_ROS::GlobalPlanCallback(nav_msgs::Path path){
  std::cout<<"path callback"<<std::endl;
  pp.global_path=path;
}

void PurePursuit_ROS::StopModeCallback(std_msgs::Bool stop_mode){
  stop_mode_=stop_mode.data;
  pp.stop_mode_=stop_mode_;
}

void PurePursuit_ROS::StopNavigationCallback(std_msgs::Bool stop_navigation){
  stop_navigation_=stop_navigation.data;
  pp.stop_navigation_=stop_navigation.data;
}

void PurePursuit_ROS::main_loop() {
  //ros::Rate loop_rate(getLoopRate());
  ros::Rate loop_rate(getLoopRate());
  double V=0;
  double W=0;
  while(ros::ok()) {
    UpdateCurrentPosition();
    now = time(nullptr);
    if(pp.goalCheck()&&waiting) {//ゴール判定
      if(stop_mode_){//stop_modeの時止まる(updateVWのところでも0に近づいてるはずだが)
        V=0;W=0;
      }else{}//stom_mode以外でゴールした時は最後のvw維持
      navigation_state_.data="goal";
      ROS_INFO("goal");
      waiting=false;
      pub_navigation_state_.publish(navigation_state_);
    }else if(now-time_goalset>10&&waiting){//timeout処理.止まる.ちなゴールが置かれたら再びwaitingに戻る
      navigation_state_.data="timeout";
      waiting=false;
      pub_navigation_state_.publish(navigation_state_);
      V=0;W=0;
      ROS_INFO("timeout");
    }else{//goalでもtimeoutでもない時
      if(pp.UpdateVW()&&waiting){//goalあってwaitingの時vwを更新
        ROS_INFO("running");
        V=pp.getVelOut();
        W=pp.getOmgOut();
      }else{}
    }
    PubVelOmgOutput(V, W);
    PubLocalPath(pp.getPath());
    ROS_INFO("V: %f ,        W: %f",V,W);
    //ROS_INFO("W: %f",W);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
