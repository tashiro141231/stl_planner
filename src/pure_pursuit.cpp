/**
 * @file purepursuit.cpp
 * @brief PurePursuit Planer for STELLA.
 * @author Kotaro HIHARA
 * @date 2021.07.20
*/
#include "stl_planner/pure_pursuit.h"
#include <algorithm>
#include <math.h>

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
  target_vel_=max_vel_*0.8;
  stop_min_vel_=0.1;
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
  std::cout << "target: " << target_point_.x <<","<<target_point_.y<< std::endl;
  
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
  geometry_msgs::Point nearest;
  double nearest_d;
  int nearest_ind;
  geometry_msgs::Point p;
  for(int i=0;i<global_path.poses.size();i++){
     p = global_path.poses[i].pose.position;
    if(i==0){
      nearest_d= sqrt(pow((p.y-current_pos_.y),2)+pow((p.x-current_pos_.x),2));
      nearest_ind=0;
      nearest = p;
    }
    if( sqrt(pow((p.y-current_pos_.y),2)+pow((p.x-current_pos_.x),2))< nearest_d){
      nearest_d= sqrt(pow((p.y-current_pos_.y),2)+pow((p.x-current_pos_.x),2));
      nearest=p;
    }
    if(target.x==0&&target.y==0){
      target.x=goal_.x;target.y=goal_.y;target.z=0;
    }
  }

  for(int i=0;i<global_path.poses.size();i++){
    p = global_path.poses[i].pose.position;
    if( sqrt(pow((p.y-current_pos_.y),2)+pow((p.x-current_pos_.x),2))< look_ahead_distance_){
      target=p;
    }
    if(target.x==0&&target.y==0){
      target.x=goal_.x;target.y=goal_.y;target.z=0;
    }
  }
  if (global_path.poses.size()<1){
    target.x=goal_.x;target.y=goal_.y;target.z=0;
  }
  return target;
}

void PP_Planner::pure_pursuit(){
  target_point_=select_target();
  theta_ = current_pos_.theta;
  alpha_=atan2((target_point_.y-current_pos_.y),(target_point_.x-current_pos_.x))-theta_;
  while(abs(theta_)>M_PI){
    if(theta_>M_PI){
      theta_-=2*M_PI;
    }else{
      theta_+=2*M_PI;
    }
  }
  dist_ = sqrt(pow((target_point_.y-current_pos_.y),2)+pow((target_point_.x-current_pos_.x),2));
  //stop_gain
  target_vel_=max_vel_*0.8;//0.4m/s
  if(stop_mode_&&dist_<2&&dist_>0.5){
    std::cout<<"stop_mode"<<std::endl;
    target_vel_=(target_vel_-stop_min_vel_)*(dist_-0.5)/(2-0.5);
  }else if(stop_mode_&&dist_<0.5){
    std::cout<<"stop_mode"<<std::endl;
    target_vel_=stop_min_vel_;//0.1m/s
  }else{}
  current_vel_=target_vel_;
  current_omega_=2*target_vel_*sin(alpha_)/dist_;
  if(abs(alpha_)<=(60*M_PI/180)){
  }else if(alpha_>(60*M_PI/180)||abs(alpha_)>M_PI/2){
    current_vel_=0;
    current_omega_=max_w_*0.2;
  }else if(alpha_<(-60*M_PI/180)){
    current_vel_=0;
    current_omega_=min_w_*0.2;
  }else{}
  std::cout<<"alpha = "<<alpha_*180/M_PI<<std::endl;
  std::cout<<"target_omega = "<<current_omega_<<std::endl;
  //current_vel_=0;current_omega_=0;
}

