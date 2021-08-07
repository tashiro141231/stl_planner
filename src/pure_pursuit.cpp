/**
 * @file purepursuit.cpp
 * @brief PurePursuit Planer for STELLA.
 * @author Kotaro HIHARA
 * @date 2021.07.20
*/
#include "stl_planner/pure_pursuit.h"
#include <algorithm>

PP_Planner::PP_Planner(){
  current_path_ = {{0,0,0}};
}

void PP_Planner::Initialize(double max_vel, double min_vel, double max_acc, double max_w, double min_w) {
  is_set_goal_ = false;
  max_vel_ = max_vel;//[m/s]
  min_vel_ = min_vel;
  max_acc_ = max_acc;
  max_w_ = max_w;//[rad/s]
  min_w_ = min_w;
  target_vel_=max_vel_*0.3;
}


void PP_Planner::setStartPoint(Point start) {
  start_ = start;
}

void PP_Planner::setGoalPoint(Point goal) {
  goal_ = goal;
}

void PP_Planner::setStartGoal(Point start, Point goal) {
  start_ = start;
  goal_ = goal;
  is_set_goal_ = true;
}

void PP_Planner::setCurrentPosition(Point point) {
  current_pos_ = point;
  std::cout << "Set CurrentPosition x: " << current_pos_.x << " y: " << current_pos_.y << " theta: " << current_pos_.theta * 180 / M_PI << std::endl;
  if(is_set_goal_) {
    //std::cout << "Start x: " << start_.x << " y: " << start_.y << " theta: " << start_.theta * 180 / M_PI << std::endl;
    std::cout << "Goal x: " << goal_.x << " y: " << goal_.y << " theta: " << goal_.theta * 180 / M_PI << std::endl;
  }
}


void PP_Planner::SetParams(double v_max, double v_min, double max_acc, double w_max, double w_min) {
  max_vel_ = v_max;
  min_vel_ = v_min;
  max_w_ = w_max;
  min_w_ = w_min;
  max_acc_ = max_acc;
}

bool PP_Planner::UpdateVW() {
  std::cout << "Goal: " << goal_.x << "," << goal_.y <<std::endl;
  std::cout << "target: " << target_point_.x <<","<<target_point_.x << std::endl;
  
  if(is_set_goal_) {
    pure_pursuit(); 
    std::cout << "V: " << current_vel_ << " W: " << current_omega_ << std::endl;
    return true;
  }
  return false;
}

std::vector<Point> PP_Planner::getPath(){
  return current_path_;
}

double PP_Planner::getVelOut() {
  return current_vel_;
}

double PP_Planner::getOmgOut() {
  return current_omega_;
}

bool PP_Planner::isRobotPlannning() {
  return is_set_goal_;
}

bool PP_Planner::goalCheck() {
  if(is_set_goal_) {
    double dis = hypot((current_pos_.x - goal_.x), (current_pos_.y - goal_.y));
    if(dis < 0.2) {
      is_set_goal_ = false;
      return true;
    }
  }
  return false;
}

double PP_Planner::angle_correct(double theta){
  if(theta>M_PI){
    while(theta>M_PI){
      theta -= 2*M_PI;
    }
  }
  else if(theta<M_PI){
    while(theta<-M_PI){
      theta += 2*M_PI;
    }
  }else{}
  return theta;
}

geometry_msgs::Point PP_Planner::select_target(){
  geometry_msgs::Point target;
  std::cout<<"path size"<<global_path.poses.size()<<std::endl;
  for(int i=0;i<global_path.poses.size();i++){
    geometry_msgs::Point p = global_path.poses[i].pose.position;
    if( sqrt(pow((p.y-current_pos_.y),2)+pow((p.x-current_pos_.x),2))< look_ahead_distance_){
      target=p;
    }
  }
  if (global_path.poses.size()<1){
    target.x=goal_.x;target.y=goal_.y;target.z=0;
  };
  return target;
}

void PP_Planner::pure_pursuit(){
  target_point_=select_target();
  //alpha_ = atan((goal_.y-current_pos_.y)/(goal_.x-current_pos_.x));
  //dist = sqrt(pow((goal_.y-current_pos_.y),2)+pow((goal_.x-current_pos_.x),2));
  alpha_ = atan((target_point_.y-current_pos_.y)/(target_point_.x-current_pos_.x));
  dist = sqrt(pow((target_point_.y-current_pos_.y),2)+pow((target_point_.x-current_pos_.x),2));
  current_vel_=target_vel_;
  current_omega_=2*target_vel_*sin(alpha_)/dist;
  std::cout<<"target_omega = "<<2*target_vel_*sin(alpha_)/dist<<std::endl;
  //current_vel_=0;current_omega_=0;
}

