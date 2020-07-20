/**
 * @file state_machine.cpp
 * @brief State Machine class.
 * @author Yosuke TASHIRO, Kotaro HIHARA
 * @date 2020.04.19
*/

#include "stl_planner/state_machine.h"

using namespace sm;

/** @fn 
 * @brief 
 * @param
 * @detail
*/
StateMachine::StateMachine() :
initialized_(false), current_state_(NO_DESTINATION)
{}


/** @fn 
 * @brief 
 * @param
 * @detail
*/
void StateMachine::Initialize(double robot_width, double robot_length){
  robot_width_ = robot_width;
  robot_length_ = robot_length;
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
void StateMachine::SetDWAParams(double v_max, double v_min, double max_acc, double w_max, double w_min, double v_reso, double w_reso, double dt, double prediction_time) {
  dwa.SetParams(v_max, v_min, max_acc, w_max, w_min, v_reso, w_reso, dt, prediction_time);
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
void StateMachine::setGoalPosition(double x, double y, double theta) {
  ;
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
void StateMachine::setCurrentPosition(double x, double y, double theta) {
  current_pos_.x = x;
  current_pos_.y = y;
  current_pos_.theta = theta;
  Point p;
  p.x = x;
  p.y = y;
  gp.setStartPoint(ConvGridPoint(p));
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
RobotStatus StateMachine::getCurrentState() {
  return current_state_;
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
void StateMachine::setMap(int width, int height, double map_resolution, Point lower_left,unsigned char* data) {
  map_ = data;
  m_width_ = width;
  m_height_ = height;
  resolution_ = map_resolution;
  std::cout << "state machine side: " << map_resolution << std::endl;
  lower_left.x = lower_left.x / map_resolution;
  lower_left.y = lower_left.y / map_resolution;
  
  gp.setMap(width, height,  map_resolution,lower_left, data);
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
void StateMachine::setCostmap(int width, int height, unsigned char* data) {
  costmap_= data;  
  c_width_ = width;
  c_height_ = height;

}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
std::vector<Point> StateMachine::CalcGlobalPlan(double x, double y, double theta) {
  Point goal;
  goal.x = x;
  goal.y = y;
  gp.setGoalPoint(ConvGridPoint(goal));
  //gp.calc_path();
  
  return gp.calc_path_astar();
  // 現在の自己位置を更新してGプラン
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
std::vector<Point> StateMachine::DebagCalcGlobalPlan(Point start, Point goal) {
  current_state_ = SET_DESTINATION; 
  gp.setStartGoal(ConvGridPoint(start), ConvGridPoint(goal));
  gp.calc_path();
  return gp.getPath();
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
Point StateMachine::ConvGridPoint(Point point) {
  Point grid_point;
  grid_point.x = (int)(point.x / resolution_);
  grid_point.y = (int)(point.y / resolution_);
  return grid_point;
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
void StateMachine::calc_paths() {
  switch(current_state_) {
    case NO_DESTINATION:
      break;
    case SET_DESTINATION:
      break;
    case STOP:
      break;
  }
}

