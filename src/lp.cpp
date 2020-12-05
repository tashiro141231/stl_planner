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
    //std::cout << "Start x: " << start_.x << " y: " << start_.y << " theta: " << start_.theta * 180 / M_PI << std::endl;
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
  o_map_double_ = rawmap_to_point_double(lower_left, map);
}
void LPlanner::setCostMap(int width, int height, double resolution, Point lower_left, unsigned char *map) {
  cost_width_ = width;
  cost_height_ = height;
  cost_max_x_ = width;
  cost_max_y_ = height;
  cost_resolution_ = resolution;
  //o_costmap_ = rawmap_to_point(lower_left, map);
  o_costmap_double_ = rawmap_to_point_double(lower_left, map);
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

std::vector<dNode> LPlanner::rawmap_to_point_double(Point lower_left, unsigned char* map) {
  std::vector<dNode> buff_map;
  dNode buff;
  for(int itr = 0; itr < width_*height_; itr++) {
    if(map[itr] != 0xFF && map[itr] != 0x00) {
      buff.cost = map[itr];
      buff.x = (lower_left.x) + (itr % width_)*resolution_;
      buff.y = (lower_left.y) + (itr / width_)*resolution_;
      if(buff.cost>0){
        buff_map.push_back(buff);
      }else{}
    }
  }
  return buff_map;
}

std::vector<dNode> LPlanner::rawcostmap_to_point_double(Point lower_left, unsigned char* map) {
  std::vector<dNode> buff_map;
  dNode buff;
  for(int itr = 0; itr < cost_width_*cost_height_; itr++) {
    if(map[itr] != 0xFF && map[itr] != 0x00) {
      buff.cost = map[itr];
      buff.x = (lower_left.x) + (itr % cost_width_)*cost_resolution_;
      buff.y = (lower_left.y) + (itr / cost_width_)*cost_resolution_;
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

void LPlanner::calc_path_dwa(State state, DW dw, Point goal,std::vector<dNode> ob,std::vector<dNode> costmapob){
  State xinit ={};
  cost_min_ = cost_limit_;
  v_min_ = 0;//current_vel?
  w_min_ = 0;//current_omega?
  std::vector<State> best_traj ={state};
  std::vector<State> traj,traj0;
  //std::cout<<"w_max:"<<dw.w_max<<" w_min:"<<dw.w_min<<std::endl;
  double limit = 0.0001;
  dw.v_min=0;
  int count = 0;
  for(double v=dw.v_min;v<dw.v_max;v+=v_resolution_){
    for(double w = dw.w_min;w<dw.w_max;w+=w_resolution_){
      collision_ = 0;
      traj = calc_trajectory(state,v,w);
      goal_cost_ = goal_gain_*calc_to_goal_cost(traj,goal);
      speed_cost_ = speed_gain_*(max_vel_-v);
      ob_cost_ = ob_gain_*calc_obstacle_cost(traj,ob);
      costmapob_cost_ = ob_gain_*calc_obstacle_cost(traj,costmapob);
      ob_cost_ = std::max(ob_cost_,costmapob_cost_);
      //ob_cost_ = 0;
      std::vector<double> n = cost_normalize(goal_cost_,speed_cost_,ob_cost_);
      //goal_cost_=n[0]; speed_cost_=n[1]; ob_cost_=n[2];
      cost_final_ = goal_cost_+speed_cost_+ob_cost_;
      //collision_ = 0;
      if (cost_min_ >= cost_final_ && collision_==0){
        count++;
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
  traj0 = calc_trajectory(state,0.1,0);
  double goal_cost0 = goal_gain_*calc_to_goal_cost(traj0,goal);
  std::cout<<"goal:"<<to_goal_min_<<std::endl;
  std::cout<<"goal0:"<<goal_cost0<<std::endl;
  std::cout<<"x_y_th="<<best_traj[29].x<<best_traj[29].y<<best_traj[29].yaw<<std::endl;
  std::cout<<"00_x_y_th="<<traj0[29].x<<traj0[29].y<<traj0[29].yaw<<std::endl;
    std::cout<<"count"<<count<<std::endl;
  */

  //avoid stop spin
  if (v_min_==0){
    v_min_=0.001;
    w_min_=-0.5; 
  }
  
  //std::cout<<"v_out"<<v_min_<<"w_out"<<w_min_<<std::endl;
  std::cout<<"goal:"<<to_goal_min_<<" speed:"<<speed_min_<<" ob:"<<ob_min_<<std::endl;
  std::vector<Point> path ={};
  for(int i =0;i<best_traj.size();i++){
    Point buff;
    buff.x = best_traj[i].x;
    buff.y = best_traj[i].y;
    buff.theta = best_traj[i].yaw;
    path.push_back(buff);
  }
  current_vel_ = v_min_;
  current_omega_ = w_min_;
  current_path_=path;
}

std::vector<Point> LPlanner::getPath(){
  return current_path_;
}


double LPlanner::calc_obstacle_cost(std::vector<State> traj, std::vector<dNode> ob){
  int skip_n = 2;
  float inf = std::numeric_limits<float>::infinity();
  inf = 100000;
  double minr = inf;
  for(int ii=0;ii<traj.size();ii+=skip_n){
    for(int i=0;i<ob.size();i++){
      double dx = traj[ii].x-ob[i].x;
      double dy = traj[ii].y-ob[i].y;

      double r = sqrt(std::pow(dx,2)+std::pow(dy,2));
      if(minr>=r){
        minr =r;
      } else{}
    }
  }
  if(minr<robot_radius_){
    collision_ = 1;
    return inf;
  }else{}
  if(minr>=100)
  return 0;
  double cost= 1/minr;
  return cost;
}

/*
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
}*/


double LPlanner::calc_to_goal_cost(std::vector<State> traj,Point goal){
  double goal_magnitude = std::sqrt(std::pow(goal.x,2)+ std::pow(goal.y,2));
  double traj_x = traj[traj.size()-1].x;
  double traj_y = traj[traj.size()-1].y;
  double traj_th = traj[traj.size()-1].yaw;
  double angle_to_goal = std::atan2(goal.y-traj_y,goal.x-traj_x);
  double score_angle = std::abs(angle_correct(angle_to_goal-traj_th)); 
  return score_angle;
}

void LPlanner::dwa_control(){
  State state = {current_pos_.x,current_pos_.y,current_pos_.theta,current_vel_,current_omega_};
  DW dw = calc_dynamic_window(state);
  calc_path_dwa(state,dw,goal_,o_map_double_,o_costmap_double_);
  //show_ob(o_map_double_);
  show_ob(o_costmap_double_);
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

double LPlanner::angle_correct(double theta){
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

std::vector<double> LPlanner::cost_normalize(double goal_cost,double speed_cost, double ob_cost){
  double max = std::max({goal_cost,speed_cost,ob_cost});
  double min = std::min({goal_cost,speed_cost,ob_cost});
  double d = max-min;
  std::vector<double> n = {(goal_cost-min)/d,(speed_cost-min)/d,(ob_cost-min)/d};
  return n;
}

void LPlanner::show_ob(std::vector<dNode> ob){
  std::cout<<"ob_size = "<<ob.size()<<std::endl;
    for(int i=0; i<ob.size();i++){
      double x = ob[i].x;
      double y = ob[i].y;
      std::cout<<"x,y="<<x<<","<<y<<std::endl;
    }   
}
