
#include "stl_planner/state_machine.h"

using namespace sm;

StateMachine::StateMachine() :
initialized_(false), current_state_(NO_DESTINATION)
{}


void StateMachine::Initialize(double robot_width, double robot_length){
  robot_width_ = robot_width;
  robot_length_ = robot_length;
}

void StateMachine::setGoalPosition(double x, double y, double theta) {
  ;
}

void StateMachine::setCurrentPosition(double x, double y, double theta) {
  current_pos_.x = x;
  current_pos_.y = y;
  current_pos_.theta = theta;
}

RobotStatus StateMachine::getCurrentState() {
  return current_state_;
}

void StateMachine::setMap(int width, int height, Point centre,unsigned char* data) {
  map_ = data;
  m_width_ = width;
  m_height_ = height;

  gp.setMap(width, height, centre, data);
}

void StateMachine::setCostmap(int width, int height, unsigned char* data) {
  costmap_= data;  
  c_width_ = width;
  c_height_ = height;
}

std::vector<Point> StateMachine::CalcGlobalPlan(double x, double y, double theta) {
  Point goal;
  goal.x = x;
  goal.y = y;
  gp.setGoalPoint(goal);
  gp.calc_path();
  return gp.getPath();
  // 現在の自己位置を更新してGプラン
}

std::vector<Point> StateMachine::DebagCalcGlobalPlan(Point start, Point goal) {
  gp.setStartGoal(start, goal);
  gp.calc_path();
  return gp.getPath();
}

