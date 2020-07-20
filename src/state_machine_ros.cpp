/**
 * @file state_machine_ros.cpp
 * @brief State Machine ROS driver class.
 * @author Yosuke TASHIRO, Kotaro HIHARA
 * @date 2020.04.19
*/


#include "stl_planner/state_machine_ros.h"
#include "stl_planner/rosmsg_raw_conv.h"

using namespace smr;

/** @fn 
 * @brief 
 * @param
 * @detail
*/
StateMachineROS::StateMachineROS(tf2_ros::Buffer& tf) :
tf_(tf),
initialized_(false),
is_robot_moving_(false),
global_frame_("map"),
base_frame_("odom"),
vel_stop_thr_(0),
omega_stop_thr_(0),
v_resolution_(0.01),
w_resolution_(1*M_PI/180),
dt_(0.1),
max_acc_(0.2),
predict_time_(3.0)
{
  Initialize();
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
void StateMachineROS::Initialize()
{
  if(!initialized_) {
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;
    private_nh.getParam("global_frame", global_frame_);
    private_nh.getParam("base_frame", base_frame_);
    private_nh.getParam("vel_stop_threshold", vel_stop_thr_);
    private_nh.getParam("omega_stop_threshold", omega_stop_thr_);
    private_nh.getParam("robot_width", robot_width_);
    private_nh.getParam("robot_length", robot_length_);
    private_nh.getParam("goal_topic_name", goal_topic_);
    private_nh.getParam("loop_rate", loop_rate_);

    private_nh.getParam("dwa/max_vel", max_vel_);
    private_nh.getParam("dwa/min_vel", min_vel_);
    private_nh.getParam("dwa/max_w", max_w_);
    private_nh.getParam("dwa/min_w", min_w_);
    private_nh.getParam("dwa/max_acc", max_acc_);
    private_nh.getParam("dwa/v_resolution", v_resolution_);
    private_nh.getParam("dwa/w_resolution", w_resolution_);
    private_nh.getParam("dwa/dt", dt_);
    private_nh.getParam("dwa/predict_time", predict_time_);
    initialized_ = true;

    sm_.Initialize(robot_width_, robot_length_);
    sm_.SetDWAParams(max_vel_, min_vel_, max_acc_, max_w_, min_w_, v_resolution_, w_resolution_, dt_, predict_time_);

    //Subscriber
    map_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>("map", 1, &StateMachineROS::MapLoadCallback, this);
    costmap_sub_ =nh.subscribe<nav_msgs::OccupancyGrid>("move_base/local_cost_map/costmap", 1, &StateMachineROS::CostmapLoadCallback, this);
    goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 1, &StateMachineROS::WaypointCallback, this);

    //Publisher
    gp_path_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1000);
    lp_path_pub_ = nh.advertise<nav_msgs::Path>("local_plan", 1000);
    ROS_INFO("State_machine_ROS has been initialized.");
  }
}

//------------- Callback -------------------//
/** @fn 
 * @brief 
 * @param
 * @detail
*/
void StateMachineROS::MapLoadCallback(const nav_msgs::OccupancyGrid msg) {
  unsigned char* data = map_to_raw(msg); 
  Point lower_left;
  lower_left.x = msg.info.origin.position.x;
  lower_left.y = msg.info.origin.position.y;
  lower_left.theta = 0;
  std::cout << "ros side: " << msg.info.resolution << std::endl;

  sm_.setMap(msg.info.width,msg.info.height, msg.info.resolution, lower_left, data);
}

void StateMachineROS::CostmapLoadCallback(const nav_msgs::OccupancyGrid msg) {
  ROS_INFO("Costmap received");
  unsigned char* data = map_to_raw(msg);

  sm_.setCostmap(msg.info.width,msg.info.height, data);
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
void StateMachineROS::UpdateCurrentPosition() {
  try{
    ts_ = tf_.lookupTransform(global_frame_, base_frame_, ros::Time(0));
    double x = ts_.transform.translation.x;
    double y = ts_.transform.translation.y;
    double roll,pitch,yaw;
    tf2::Quaternion tfq;
    tf2::Quaternion q;
    tf2::convert(ts_.transform.rotation, q);
    tf2::Matrix3x3 (tfq).getEulerYPR(yaw,roll,pitch);
    current_pos_.x = x;
    current_pos_.y = y;
    current_pos_.theta = yaw;
    sm_.setCurrentPosition(x, y, yaw);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("Could not transform %s to %s :%s", global_frame_.c_str(), base_frame_.c_str(), ex.what());
  }
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
void StateMachineROS::WaypointCallback(geometry_msgs::PoseStamped msg) {
  nav_msgs::Path p = CalcGlobalPlan(msg);

  PubGlobalPath(p);
  ROS_INFO("outing");
}

//------------- End of Callback ----------------//

/** @fn 
 * @brief 
 * @param
 * @detail
*/
void StateMachineROS::PubLocalPath(nav_msgs::Path path) {
  lp_path_pub_.publish(path);
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
void StateMachineROS::PubGlobalPath(nav_msgs::Path path) {
  ROS_INFO("Pub global path now.");
  gp_path_pub_.publish(path);
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
void StateMachineROS::UpdateRobotStatus(double robot_v, double robot_w) {
  bool v_is_zero =false, omega_is_zero = false;

  if(fabs(robot_v) < vel_stop_thr_) {
    v_is_zero = true;
  }
  if(fabs(robot_w) < omega_stop_thr_) {
    omega_is_zero = true;
  }
  
  if(v_is_zero && omega_is_zero) {
    if(is_robot_moving_){
      stopped_time_ = ros::Time::now();
      is_robot_moving_ = false;
    }
  }
  else {
    is_robot_moving_ = true;
  }
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
nav_msgs::Path StateMachineROS::CalcGlobalPlan(geometry_msgs::PoseStamped goal) {
 double x,y,theta;
 x = goal.pose.position.x;
 y = goal.pose.position.y;
 theta = quaternion_to_theta(goal.pose.orientation);
 sm_.setCurrentPosition(current_pos_.x, current_pos_.y, current_pos_.theta);  //Update current pos
 nav_msgs::Path p =  path_to_rospath(sm_.CalcGlobalPlan(x, y, theta), global_frame_); //calc path to goal
 return p;
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
nav_msgs::Path StateMachineROS::DebagCalcGlobalPlan(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal) {
 Point s = pose_to_point(start.pose);
 Point g = pose_to_point(goal.pose);
 nav_msgs::Path p = path_to_rospath(sm_.DebagCalcGlobalPlan(s, g), global_frame_);

 return p;
}

/** @fn 
 * @brief 
 * @param
 * @detail
*/
void StateMachineROS::main_loop() {
  ros::Rate loop_rate(loop_rate_);
  while(ros::ok()) {
    UpdateCurrentPosition();
    ros::spinOnce();
    sm_.calc_paths();
    loop_rate.sleep();
  }
}

