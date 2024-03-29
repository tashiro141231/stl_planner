
#include "stl_planner/line_planner.h"

LinePlanner::LinePlanner() {
  ;
}

void LinePlanner::Initialize(double max_vel, double min_vel, double acc, double max_w, double max_eta, double Ko, double Kp, double Ke, double loop_rate) {
  is_set_goal_ = false;
  max_vel_ = max_vel;
  min_vel_ = min_vel;
  max_omega_ = max_w;
  min_omega_ = -max_w;
  acc_ = acc;
  decel_ = 0.2;
  dt_ = 1 / loop_rate;
  Ko_ = Ko;
  Kp_ = Kp;
  Ke_ = Ke;
  max_eta_ = max_eta;
  current_vel_ = 0;
  current_omega_ = 0;
  prev_omega_ = 0;
  target_vel_ = 0;
  target_omega_ = 0;
  remain_distance_ = 0;
  deceleration_distance_ = 0.5;
  goal_distance_ = 0.2;
  
  //Stabele statement : Ko > 0 && Ko*Kp - Ke > 0 && Ko*Kp*Ke - Ke*Ke > 0
  if(Ko_ <=0 || Ko_ * Kp_ - Ke <=0 || Ko_ * Kp_ * Ke_ - Ke_*Ke_ <= 0)
    std::cout << "WARNING : Unstable PID params has been set." << std::endl;
}

bool LinePlanner::UpdateVW() {
  if(is_set_goal_) {
    if(remain_distance_ <= deceleration_distance_)  start_deceleration_ = true;
    //Update velocity
    if(!start_deceleration_) {
      if(target_vel_ < max_vel_) {
        target_vel_ += acc_ * dt_;
      }
      else {
        target_vel_ = max_vel_;
      }
    }
    //Deceleration section
    else {
      //target_vel_ = max_vel_ * (remain_distance_ - goal_distance_) / (deceleration_distance_ - goal_distance_);
      if(target_vel_ < 0.2) {
        target_vel_ = 0.2;
      }
      else {
        target_vel_ -= decel_ * dt_;
      }
    }

    //Update omega
    UpdateFeedbackParam();
    double add = dt_* (Ke_ * eta_ + Kp_ * phi_ + Ko_ * omega_diff_);
    //std::cout << "add: " << add << " dt: " << dt_ << std::endl;
    target_omega_ = prev_omega_ - add;
    if(std::abs(target_omega_) > std::abs(max_omega_)) {
      if(target_omega_ > 0) {
        target_omega_ = max_omega_;
      }
      else {
        target_omega_ = min_omega_;
      }
    }
    //Update previous omega
    prev_omega_ = target_omega_;
    //std::cout << "V: " << target_vel_ << " W: " << target_omega_ << " phi: " << phi_ /M_PI * 180 << "\n" << std::endl;
    return true;
  }
  return false;
}

void LinePlanner::UpdateFeedbackParam() {
  //Update ditance
  double a = sqrt(la_ * la_ + lb_ * lb_);
  eta_ = (la_ * current_pos_.x + lb_ * current_pos_.y + lc_) / a;
  if(eta_ > max_eta_ ) {
    eta_ = max_eta_;
  }
  else if(eta_ < -max_eta_) {
    eta_ = -max_eta_;
  }

  //Update angle diff
  double diff = -target_line_angle_ + ConvAngleRange(current_pos_.theta); 
  phi_ = std::atan2(std::sin(diff), std::cos(diff));
  //if( M_PI > phi_ && -M_PI < phi_) {
  //}
  //else if(phi_ >= M_PI) {
  //  phi_ = 2 * M_PI - phi_;
  //}
  //else if(phi_ <= -M_PI) {
  //  phi_ = -2 * M_PI - phi_;
  //}

  //Update omega diff
  omega_diff_ = -target_omega_ + current_omega_;
  //std::cout << "la_: " << la_ << " lb_: " << lb_ << " lc: " << lc_ << std::endl;
  std::cout << "eta_: " << eta_ << " phi_: " << phi_ << " omega_diff_: " << omega_diff_ << std::endl;
}

void LinePlanner::InitTargetLine() {
  target_line_angle_ = atan2((goal_.y - start_.y), (goal_.x - start_.x));
  if(goal_.x != start_.x) {
    double slope = (goal_.y - start_.y) / (goal_.x - start_.x);
    double seppen = goal_.y - slope * goal_.x;
    //Line of la* x + lb * y + lc = 0
    la_ = -slope;
    lb_ = 1;
    lc_ = -seppen;
  }
  else {
    la_ = 1;
    lb_ = 0;
    lc_ = goal_.x;
  }
}

void LinePlanner::InitTargetLineGoal() {
  target_line_angle_ = ConvAngleRange(goal_.theta);
  double seppen = goal_.y - target_line_angle_ * goal_.x;
  //Line of la* x + lb * y + lc = 0
  la_ = target_line_angle_;
  lb_ = 1;
  lc_ = -seppen;
  if(target_line_angle_ == M_PI /2 || target_line_angle_ == -M_PI/2) {
    la_ = 1;
    lb_ = 0;
    lc_ = goal_.x;
  }
}

bool LinePlanner::goalCheck() {
  if(is_set_goal_) {
    double dis = remain_distance_;
    if(dis < goal_distance_) {
      std::cout << "Has reached the goal." << std::endl;
      is_set_goal_ = false;
      //target_vel_ = 0;
      //target_omega_ = 0;
      prev_omega_ = 0;
      start_deceleration_ = false;
      return true;
    }
  }
  return false;
}

double LinePlanner::calc_distance(Point p1, Point p2) {
  double dis = hypot(p1.x - p2.x, p1.y - p2.y);
  return dis;
}

void LinePlanner::setCurrentPosition(Point point) {
  current_pos_ = point;
  remain_distance_ = calc_distance(current_pos_, goal_);
  //if(is_set_goal_) {
  //  std::cout << "Start x: " << start_.x << " y: " << start_.y << " theta: " << start_.theta * 180 / M_PI << std::endl;
  //  std::cout << "Goal x: " << goal_.x << " y: " << goal_.y << " theta: " << goal_.theta * 180 / M_PI << std::endl;
  //}
}

void LinePlanner::setCurrentVelOmega(double vel, double omega) {
  current_vel_ = vel;
  current_omega_ = omega;
}

void LinePlanner::setStartPoint(Point start) {
  start_ = start;
}

void LinePlanner::setGoalPoint(Point goal) {
  goal_ = goal;
}

void LinePlanner::setStartGoal(Point start, Point goal) {
  start_ = start;
  goal_ = goal;
  InitTargetLine();
  //InitTargetLineGoal();
  is_set_goal_ = true;
}

void LinePlanner::setMap(int width, int height, double resolution, Point lower_left, unsigned char *map) {
  map_width_ = width;
  map_height_ = height;
  map_resolution_ = resolution;
  map_lower_left_ = lower_left;

  static_map_ = map;
}

void LinePlanner::setCostMap(int width, int height, double reslution, Point lower_left, unsigned char *map) {
  ;
}

std::vector<Node> LinePlanner::RawMapToNode(Point lower_left, unsigned char *map) {
  std::vector<Node> g_map;
  Node buff;

  lower_left = ConvPointToGrid(lower_left);

  for(int itr = 0; itr < map_width_* map_height_; itr++) {
    if(map[itr] != 0xFF && map[itr] != 0x00) {
      buff.cost = map[itr];
      buff.x = (int)(lower_left.x) + (int)(itr % map_width_);
      buff.y = (int)(lower_left.y) + (int)(itr / map_width_);
      g_map.push_back(buff);
    }
  }
  return g_map;
}

bool LinePlanner::getCost(int x, int y) {
  if(x * y < map_width_ * map_height_ ) {
    //get cost from [m] or grid coordinate
    ;
  }
}

bool LinePlanner::isRobotPlannning() {
  return is_set_goal_;
}

double LinePlanner::getVelOut() {
  return target_vel_;
}

double LinePlanner::getOmgOut() {
  return target_omega_;
}

Point LinePlanner::ConvGridToPoint(Point p) {
  p.x = p.x * map_resolution_;
  p.y = p.y * map_resolution_;
  return p;
}

Point LinePlanner::ConvPointToGrid(Point p) {
  p.x = (int)(p.x / map_resolution_);
  p.y = (int)(p.y / map_resolution_);
  return p;
}

double LinePlanner::ConvAngleRange(double value) {
  if(value <= M_PI && value >= -M_PI) return value;
  else if(value < -M_PI) { 
    value = value + M_PI;
  }
  else {
    value = value - M_PI;
  }
  return value;
}

