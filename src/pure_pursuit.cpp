/**
 * @file purepursuit.cpp
 * @brief PurePursuit Planer for STELLA.
 * @author Kotaro HIHARA
 * @date 2021.07.20
*/
#include "stl_planner/pure_pursuit.h"
#include <algorithm>
#include <math.h>
#include <algorithm>


PP_Planner::PP_Planner(){
}

void PP_Planner::Initialize(double max_vel, double min_vel, double max_acc,double stop_dec, double max_w, double min_w,double set_vel,double wp_range,double goal_range, double max_dw,double dt,double stop_vel,double look_dist,double stop_dist,double predict_time) {
  is_set_goal_ = false;
  max_vel_ = max_vel;//[m/s]
  min_vel_ = min_vel;
  max_acc_ = max_acc;
  stop_dec_=stop_dec;
  max_w_ = max_w;//[rad/s]
  min_w_ = min_w;
  set_vel_=set_vel;
  wp_range_=wp_range;
  goal_range_=goal_range;
  max_dw_=max_dw;
  stop_vel_=stop_vel;
  look_dist_=look_dist;
  stop_dist_=stop_dist;
  predict_time_=predict_time;

  target_vel_=0;
  target_omega_=0;
  r_=0;
  dt_=dt;
  stop_navigation_=0;
  dist_fix_=true;
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
    //std::cout << "Goal x: " << goal_.x << " y: " << goal_.y << " theta: " << goal_.theta * 180 / M_PI << std::endl;
  }
}

void PP_Planner::updateSetVel(double current_set_vel){
  set_vel_=current_set_vel;
}


void PP_Planner::SetParams(double v_max, double v_min, double max_acc,double stop_dec, double w_max, double w_min,double set_vel,double wp_range,double goal_range,double max_dw,double dt,double stop_vel,double look_dist,double stop_dist,double predict_time) {
  max_vel_ = v_max;
  min_vel_ = v_min;
  max_w_ = w_max;
  min_w_ = w_min;
  max_acc_ = max_acc;
  stop_dec_ = stop_dec;
  set_vel_=set_vel;
  wp_range_ = wp_range;
  goal_range_ = goal_range;
  max_dw_=max_dw;
  dt_=dt;
  stop_vel_=stop_vel;
  look_dist_=look_dist;
  stop_dist_=stop_dist;
  predict_time_=predict_time;
}

bool PP_Planner::UpdateVW() {
  //std::cout << "Goal: " << goal_.x << "," << goal_.y <<std::endl;
  //std::cout << "target: " << target_point_.x <<","<<target_point_.y<< std::endl;
  if(is_set_goal_) {
    pure_pursuit(); 
    return true;
  }
  return false;
}

nav_msgs::Path PP_Planner::getPath(){
  nav_msgs::Path path;
  geometry_msgs::PoseStamped pose;
  double x,y,theta;
  predict_size_=(1/dt_)*predict_time_;
  x=0;y=0;theta=0;
  //x=current_pos_.x;y=current_pos_.y;theta=current_pos_.theta;
  path.header.frame_id="/base_link";
  //path.header.frame_id="/map";
  path.header.stamp=ros::Time::now();
  path.poses.resize(predict_size_);
  geometry_msgs::Quaternion geometry_quat;
  tf::Quaternion quat;
  for(int i=0;i<predict_size_;i++){
    x+=current_vel_*dt_*cos(theta);
    y+=current_vel_*dt_*sin(theta);
    theta+=current_omega_*dt_;
    /*
    quat = tf::createQuaternionFromRPY(0,0,theta);
    quaternionTFToMsg(quat,geometry_quat);
    pose.pose.position.x=x;
    pose.pose.position.y=y;
    pose.pose.orientation = geometry_quat;
    path.poses.push_back(pose);
    */
    path.poses[i].pose.position.x=x;
    path.poses[i].pose.position.y=y;
  }
  return path;
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
  bool goal_check =false;
  bool overtake =false;
  double range_=wp_range_;
  if(stop_mode_)range_=goal_range_;
  if(is_set_goal_) {
    double dis = hypot((current_pos_.x - goal_.x), (current_pos_.y - goal_.y));
    if(dis < range_) {
      is_set_goal_ = false;
      goal_check=true;
    }
    double th=atan2((current_pos_.y-goal_.y),(current_pos_.x-goal_.x))-goal_.theta;
    while(abs(th)>M_PI){
      if(th>M_PI){
        th-=2*M_PI;
      }else{
        th+=2*M_PI;
      }
    }
    overtake=(abs(th)<M_PI/2);//追い越し判定
    //goal_check=goal_check||overtake;//距離か追い越しの条件をクリアしてたらクリア.は？
  }
  if(goal_check)is_set_goal_ = false;
  return goal_check;
}

bool PP_Planner::goalYawCheck() {
  double diff_goal;
  diff_goal=goal_.theta-current_pos_.theta;
  if(diff_goal>goalyaw_threshold){
    current_omega_=0.5*yaw_adjust_omega;
  }else if(diff_goal<-(goalyaw_threshold)){
    current_omega_=-0.5*yaw_adjust_omega;
  }else{}
  return abs(diff_goal)<goalyaw_threshold;
}

double PP_Planner::diff_goalYaw() {
    return goal_.theta-current_pos_.theta;
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

  //グローバルパスがない時
  if (global_path.poses.size()<1){
    target.x=goal_.x;target.y=goal_.y;target.z=0;
  }
  return target;
}

void PP_Planner::pure_pursuit(){
  //グローバルパスある時は追従点を選択
  //target_point_=select_target();
  target_point_.x=goal_.x; target_point_.y=goal_.y,target_point_.z=0;
  dist_ = sqrt(pow((target_point_.y-current_pos_.y),2)+pow((target_point_.x-current_pos_.x),2));
  //追従点が遠い時線形補間して近いところに追従
  if(dist_>look_dist_&&dist_fix_){
    target_point_.x=current_pos_.x+(target_point_.x-current_pos_.x)*look_dist_/dist_;
    target_point_.y=current_pos_.y+(target_point_.y-current_pos_.y)*look_dist_/dist_;
  }
  theta_ = current_pos_.theta;
  alpha_=atan2((target_point_.y-current_pos_.y),(target_point_.x-current_pos_.x))-theta_;
  while(abs(alpha_)>M_PI){
    if(alpha_>M_PI){
      alpha_-=2*M_PI;
    }else{
      alpha_+=2*M_PI;
    }
  }
  r_=abs(dist_/(2*sin(alpha_)));
  //stop_gain
  target_vel_=set_vel_;
  //target_omega_=2*target_vel_*sin(alpha_)/dist_;

  //stop_mode真でstop_dist以内から減速
  if(stop_mode_&&dist_<stop_dist_&&dist_>0.2){
    target_vel_=(target_vel_-stop_vel_)*(dist_-0.2)/(1-0.2)+stop_vel_;
    target_vel_=std::max(target_vel_,stop_vel_);
  }else if(stop_mode_&&dist_<0.2){
    target_vel_=stop_vel_;
  }else{}

  //曲率が大きい時減速
  if(r_<0.5)target_vel_=stop_vel_;

  if(stop_navigation_){
    target_vel_=0;
  }

  //accel制限
  if(abs(target_vel_ - current_vel_) < max_acc_ * dt_) {
  }
  else if(target_vel_>current_vel_){
    target_vel_=current_vel_+max_acc_*dt_;
  }else if(target_vel_<current_vel_){
    target_vel_=current_vel_-max_acc_*dt_;
    //stop_navigation専用減速度適用
    if(stop_navigation_)target_vel_=current_vel_-stop_dec_*dt_;
  }else{}

  //w加速制限
  target_omega_=2*target_vel_*sin(alpha_)/dist_;
  if(target_omega_>current_omega_){
    target_omega_=std::min(target_omega_,current_omega_+max_dw_*dt_);
  }else{
    target_omega_=std::max(target_omega_,current_omega_-max_dw_*dt_);
  }

  current_vel_=target_vel_;
  current_omega_=target_omega_;
  //角度が大きすぎる時の旋回
  /*
  if(abs(alpha_)<=(80*M_PI/180)){
  }else if(alpha_>(80*M_PI/180)||abs(alpha_)>M_PI){
    current_vel_=0;
    current_omega_=max_w_*0.2;
  }else if(alpha_<(-80*M_PI/180)){
    current_vel_=0;
    current_omega_=min_w_*0.2;
  }else{}
  */

  current_omega_=std::max(min_w_,current_omega_);
  current_omega_=std::min(max_w_,current_omega_);

  std::cout<<"alpha = "<<alpha_*180/M_PI<<std::endl;
  //std::cout<<"target_omega = "<<current_omega_<<std::endl;
  //current_vel_=0;current_omega_=0;
  if(stop_navigation_)current_omega_=0;
}

