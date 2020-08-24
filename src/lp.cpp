/**
 * @file lp.cpp
 * @brief Local Planer for STELLA.
 * @author Kotaro HIHARA
 * @date 2020.04.28
*/
#include "stl_planner/lp.h"

LPlanner::LPlanner(){
  ;
}

void LPlanner::setMap(int width, int height, double resolution, Point lower_left, unsigned char* map) {
  width_ = width;
  height_ = height;
  max_x_ = width;
  max_y_ = height;
  resolution_ = resolution;

  o_map_ = rawmap_to_node(lower_left, map);
}

std::vector<Node> LPlanner::rawmap_to_node(Point lower_left, unsigned char* map) {
  std::vector<Node> g_map;
  Node buff;

  lower_left = ConvGridPoint(lower_left);

  for(int itr = 0; itr < width_*height_; itr++) {
    if(map[itr] != 0xFF && map[itr] != 0x00) {
      buff.cost = map[itr];
      buff.x = (int)(lower_left.x) + (int)(itr % width_);
      buff.y = (int)(lower_left.y) + (int)(itr / width_);
      g_map.push_back(buff);
    }
  }
  return g_map;
}

Point LPlanner::ConvGridPoint(Point point) {
  Point grid_point;
  grid_point.x = (int)(point.x / resolution_);
  grid_point.y = (int)(point.y / resolution_);
  return grid_point;
}

void LPlanner::SetParams(double v_max, double v_min, double max_acc, double w_max, double w_min, double v_reso, double w_reso, double dtime, double prediction_time) {
  max_vel = v_max;
  min_vel = v_min;
  max_w = w_max;
  min_w = w_min;
  max_accel = max_acc;
  dt = dtime;
  predict_time = prediction_time;
}

State LPlanner::motion(State x, double v, double w){
  x.yaw += w * dt;
  x.x += v * cos(x.yaw) * dt;
  x.y += v * sin(x.yaw) * dt;
  x.v = v;
  x.w = w;
  return x;
}

DW LPlanner::calc_dynamic_window(State state){
    //Dynamic window from robot specification
  std::vector<double> Vs ={min_vel, max_vel,
          min_w, max_w};
    //Dynamic window from motion model
  std::vector<double> Vd = {state.v - max_accel * dt,
        state.v + max_accel * dt,
        state.w - max_dw * dt,
        state.w + max_dw * dt};
    
    //[vmin,vmax, yawrate min, yawrate max]
   DW dw = {std::max(Vs[0], Vd[0]), std::min(Vs[1], Vd[1]),
         std::max(Vs[2], Vd[2]), std::min(Vs[3], Vd[3])};
  return dw;
}

std::vector<State> LPlanner::calc_trajectory(State x,double v,double w){   
  std::vector<State> traj ={};
  State x_ = x;
  double time = 0;
  while(time<=predict_time){
     x_ = motion(x_,v,w);
     traj.push_back(x_);
     time += dt;
  }
  return traj;
}

void LPlanner::calc_final_input(State x, VW u, DW dw, Point goal,std::vector<Point> ob){
  State xinit ={};
  double min_cost = 1000;
  VW min_u = u;
  min_u.v = 0;
  std::vector<State> best_traj ={x};

  for(double v=dw.v_min;v<dw.v_max;v+=v_resolution){
    for(double w = dw.w_min;w<dw.w_max;w+=w_resolution){
      std::vector<State> traj = calc_trajectory(v,w);
      double to_goal_cost = calc_to_goal_cost(traj,goal);
      double speed_cost = speed_cost_gain*(max_vel-traj[traj.size()-1].v);
      double ob_cost = calc_obstacle_cost(traj,ob);
      double final_cost = to_goal_cost+speed_cost+ob_cost;
      if (min_cost >= final_cost){
        min_cost = final_cost;
        min_u = {v,w};
        best_traj = traj;
      }else{}
    }
  }
  v_out = min_u.v;
  w_out = min_u.w;
  
}

double LPlanner::calc_obstacle_cost(std::vector<State> traj, std::vector<Point> ob){
  int skip_n = 2;
  float inf = std::numeric_limits<float>::infinity();
  double minr = inf;
  for(int ii=0;ii<traj.size();ii+=skip_n){
    for(int i=0;i<ob.size();i++){
      double dx = traj[ii].x-ob[i].x;
      double dy = traj[ii].y-ob[i].y;

      double r = sqrt(std::pow(dx,2)+std::pow(dy,2));
      if (r<=robot_radius){
        return inf;
      }else{}
      if(minr>=r){
        minr =r;
      } else{}
    }
  }
  return 1.0/minr;
}

double LPlanner::calc_to_goal_cost(std::vector<State> traj,Point goal){
   double goal_magnitude = std::sqrt(std::pow(goal.x,2)+ std::pow(goal.y,2));
   double traj_x = traj[traj.size()-1].x;
   double traj_y = traj[traj.size()-1].y;
   double traj_magnitude = std::sqrt(std::pow(traj_x,2) + std::pow(traj_y,2));
   double dot_product = (goal.x*traj_x) + (goal.y*traj_y);
   double error = dot_product / (goal_magnitude*traj_magnitude);
   double error_angle = std::acos(error);
   double cost = to_goal_cost_gain * error_angle;
   return cost;
}

void LPlanner::dwa_control(double x,double y,double theta, double v, double w,Point goal, std::vector<Point> ob){
  State state = {x,y,theta,v,w};
  VW u = {v,w};
  DW dw = calc_dynamic_window(state);
  calc_final_input(state,u,dw,goal,ob);
}
