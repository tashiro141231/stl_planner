/**
 * @file lp.cpp
 * @brief Local Planer for STELLA.
 * @author Kotaro HIHARA
 * @date 2020.04.28
*/
#include "stl_planner/lp.h"
#include <algorithm>

LPlanner::LPlanner(){
  current_path_ = {{0,0,0}};
}

void LPlanner::Initialize(double max_vel, double min_vel, double max_acc, double max_w, double min_w, 
  double max_dw, double dt, double v_resolution, double w_resolution, double predict_time, 
  double goal_gain, double speed_gain, double ob_gain, double robot_radius) {
  is_set_goal_ = false;
   max_vel_ = max_vel;//[m/s]
   min_vel_ = min_vel;
  max_acc_ = max_acc;

  max_w_ = max_w;//[rad/s]
  min_w_ = min_w;
  max_dw_ = max_dw;//[rad/ss]
  v_resolution_ = v_resolution;
  w_resolution_ = w_resolution;
  dt_ =dt;
  predict_time_ = predict_time;
  goal_gain_ = goal_gain;
  speed_gain_ = speed_gain;
  ob_gain_  = ob_gain;
  robot_radius_ = robot_radius;
}


void LPlanner::setStartPoint(Point start) {
  start_ = start;
}

void LPlanner::setGoalPoint(Point goal) {
  goal_ = goal;
}

void LPlanner::setStartGoal(Point start, Point goal) {
  start_ = start;
  goal_ = goal;
  is_set_goal_ = true;
}

void LPlanner::setCurrentPosition(Point point) {
  current_pos_ = point;
  std::cout << "Set CurrentPosition x: " << current_pos_.x << " y: " << current_pos_.y << " theta: " << current_pos_.theta * 180 / M_PI << std::endl;
  if(is_set_goal_) {
    std::cout << "Start x: " << start_.x << " y: " << start_.y << " theta: " << start_.theta * 180 / M_PI << std::endl;
    std::cout << "Goal x: " << goal_.x << " y: " << goal_.y << " theta: " << goal_.theta * 180 / M_PI << std::endl;
  }
}

void LPlanner::setMap(int width, int height, double resolution, Point lower_left, unsigned char* map) {
  width_ = width;
  height_ = height;
  max_x_ = width;
  max_y_ = height;
  resolution_ = resolution;
  o_map_ = rawmap_to_point(lower_left, map);
}
void LPlanner::setCostMap(int width, int height, double reslution, Point lower_left, unsigned char *map) {
  ;
}

std::vector<Node> LPlanner::rawmap_to_point(Point lower_left, unsigned char* map) {
  std::vector<Node> buff_map;
  Node buff;

  for(int itr = 0; itr < width_*height_; itr++) {
    if(map[itr] != 0xFF && map[itr] != 0x00) {
      buff.cost = map[itr];
      buff.x = (int)(lower_left.x) + (int)(itr % width_)*resolution_;
      buff.y = (int)(lower_left.y) + (int)(itr / width_)*resolution_;
      if(buff.cost>0){
        buff_map.push_back(buff);
      }else{}
    }
  }
  return buff_map;
}

void LPlanner::SetParams(double v_max, double v_min, double max_acc, double w_max, double w_min, double v_reso, double w_reso, double dtime, double prediction_time) {
  max_vel_ = v_max;
  min_vel_ = v_min;
  max_w_ = w_max;
  min_w_ = w_min;
  max_acc_ = max_acc;
  dt_ = dtime;
  predict_time_ = prediction_time;
}

State LPlanner::motion(State x, double v, double w){
  x.yaw += w * dt_;
  x.x += v * cos(x.yaw) * dt_;
  x.y += v * sin(x.yaw) * dt_;
  x.v = v;
  x.w = w;
  return x;
}

bool LPlanner::UpdateVW() {
  if(is_set_goal_) {
    dwa_control(); 
    std::cout << "V: " << current_vel_ << " W: " << current_omega_ << std::endl;
    return true;
  }
  return false;
}

DW LPlanner::calc_dynamic_window(State state){
    //Dynamic window from robot specification
  std::vector<double> Vs ={min_vel_, max_vel_,
          min_w_, max_w_};
    //Dynamic window from motion model
  std::vector<double> Vd = {state.v - max_acc_ * dt_,
        state.v + max_acc_ * dt_,
        state.w - max_dw_ * dt_,
        state.w + max_dw_ * dt_};
    
    //[vmin,vmax, yawrate min, yawrate max]
   DW dw = {std::max(Vs[0], Vd[0]), std::min(Vs[1], Vd[1]),
         std::max(Vs[2], Vd[2]), std::min(Vs[3], Vd[3])};
  return dw;
}

std::vector<State> LPlanner::calc_trajectory(State x,double v,double w){   
  std::vector<State> traj ={};
  State x_ = x;
  double time = 0;
  while(time<=predict_time_){
     x_ = motion(x_,v,w);
     traj.push_back(x_);
     time += dt_;
  }
  return traj;
}

void LPlanner::calc_path_dwa(State state, DW dw, Point goal,std::vector<Node> ob){
  State xinit ={};
  cost_min_ = cost_limit_;
  v_min_ = 0;//current_vel?
  w_min_ = 0;//current_omega?
  std::vector<State> best_traj ={state};
  std::vector<State> traj ;

  std::cout<<"w_max:"<<dw.w_max<<" w_min:"<<dw.w_min<<std::endl;
  double limit = 0.0001;
  
  for(double v=dw.v_min;v<dw.v_max;v+=v_resolution_){
    for(double w = dw.w_min;w<dw.w_max;w+=w_resolution_){
      traj = calc_trajectory(state,v,w);
      goal_cost_ = std::max(goal_gain_*calc_to_goal_cost(traj,goal),limit);
      //speed_cost = speed_gain_*(max_vel_-traj[traj.size()-1].v);
      speed_cost_ = std::max(speed_gain_*(max_vel_-v),limit);
      ob_cost_ = std::max(ob_gain_*calc_obstacle_cost(traj,ob),limit);
      cost_final_ = goal_cost_+speed_cost_+ob_cost_;
      if (cost_min_ >= cost_final_){
        cost_min_ = cost_final_;
        to_goal_min_ = goal_cost_;
        speed_min_ = speed_cost_;
        ob_min_ = ob_cost_;
        v_min_ = v;
        w_min_ = w;
        best_traj = traj;
      }else{}
    }
  }
  /*
  //avoid stop spin
  if (v_min_==0){
    v_min_=0.001;
    w_min_=-0.5; 
  }
  */
  std::cout<<"speed_gain_"<<speed_gain_<<std::endl;
  std::cout<<"max_vel_"<<max_vel_<<std::endl;
  std::cout<<"v_min"<<v_min_<<std::endl;
  std::cout<<"goal:"<<to_goal_min_<<" speed:"<<speed_min_<<" ob:"<<ob_min_<<std::endl;
  std::vector<Point> path ={};
  for(int i =0;i<best_traj.size();i++){
    Point buff;
    buff.x = best_traj[i].x;
    buff.y = best_traj[i].y;
    buff.theta = best_traj[i].yaw;
    path.push_back(buff);
  }
  vel_out_ = v_min_;
  omega_out_ = w_min_;
  current_vel_ = v_min_;
  current_omega_ = w_min_;
  current_path_=path;
}

std::vector<Point> LPlanner::getPath(){
  return current_path_;
}


double LPlanner::calc_obstacle_cost(std::vector<State> traj, std::vector<Node> ob){
  int skip_n = 2;
  float inf = std::numeric_limits<float>::infinity();
  double minr = inf;
  for(int ii=0;ii<traj.size();ii+=skip_n){
    for(int i=0;i<ob.size();i++){
      double dx = traj[ii].x-ob[i].x;
      double dy = traj[ii].y-ob[i].y;

      double r = sqrt(std::pow(dx,2)+std::pow(dy,2));
      if (r<=robot_radius_){
        //return inf;
      }else{}
      if(minr>=r){
        minr =r;
      } else{}
    }
  }
  if(minr<robot_radius_)
  minr = 0.0001;
  double cost= 1/minr;
  return cost;
}

double LPlanner::calc_to_goal_cost(std::vector<State> traj,Point goal){
   double goal_magnitude = std::sqrt(std::pow(goal.x,2)+ std::pow(goal.y,2));
   double traj_x = traj[traj.size()-1].x;
   double traj_y = traj[traj.size()-1].y;
   double traj_magnitude = std::sqrt(std::pow(traj_x,2) + std::pow(traj_y,2));
   double dot_product = (goal.x*traj_x) + (goal.y*traj_y);
   double error = dot_product / (goal_magnitude*traj_magnitude);
   double error_angle = std::acos(error);
   double cost = error_angle;
  return cost;
}

void LPlanner::dwa_control(){
  State state = {current_pos_.x,current_pos_.y,current_pos_.theta,current_vel_,current_omega_};
  DW dw = calc_dynamic_window(state);
  calc_path_dwa(state,dw,goal_,o_map_);
}

double LPlanner::getVelOut() {
  return current_vel_;
}

double LPlanner::getOmgOut() {
  return current_omega_;
}

bool LPlanner::isRobotPlannning() {
  return is_set_goal_;
}

bool LPlanner::goalCheck() {
  if(is_set_goal_) {
    double dis = hypot((current_pos_.x - goal_.x), (current_pos_.y - goal_.y));
    if(dis < 0.2) {
      is_set_goal_ = false;
      return true;
    }
  }
  return false;
}
