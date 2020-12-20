
#include <stl_planner/planner_base_ros.h>

PlannerBaseROS::PlannerBaseROS(tf2_ros::Buffer& tf) :
tf_(tf),
initialized_(false),
is_robot_moving_(false),
global_frame_("map"),
base_frame_("base_link"),
vel_stop_thr_(0),
omega_stop_thr_(0),
v_resolution_(0.01),
w_resolution_(1*M_PI/180),
dt_(0.1),
max_acc_(0.2),
predict_time_(3.0),
goal_topic_("move_base_simple/goal")
{
  Initialize();
}

void PlannerBaseROS::Initialize() {
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
    private_nh.getParam("line_planner/loop_rate", loop_rate_);

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

    //Subscriber
    map_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>("map", 1, &PlannerBaseROS::MapLoadCallback, this);
    //costmap_sub_ =nh.subscribe<nav_msgs::OccupancyGrid>("move_base/local_cost_map/costmap", 1, &PlannerBaseROS::CostmapLoadCallback, this);
    costmap_sub_ =nh.subscribe<nav_msgs::OccupancyGrid>("costmap_test/local_cost_map/costmap", 1, &PlannerBaseROS::CostmapLoadCallback, this);
    goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(goal_topic_, 1, &PlannerBaseROS::WaypointCallback, this);
  }
}

void PlannerBaseROS::UpdateCurrentPosition() {
  try{
    ts_ = tf_.lookupTransform(global_frame_, base_frame_, ros::Time(0));
    double x = ts_.transform.translation.x;
    double y = ts_.transform.translation.y;
    double roll,pitch,yaw;
    tf2::Quaternion q;
    tf2::convert(ts_.transform.rotation, q);
    tf2::Matrix3x3 (q).getEulerYPR(yaw,roll,pitch);
    current_pos_.x = x;
    current_pos_.y = y;
    current_pos_.theta = yaw;
    std::cout << "PlannerBase: yaw pitch roll :" << yaw << " " << pitch << " " << roll << std::endl;
    setCurrentPositionToPlanner(current_pos_);
    // sm_.setCurrentPosition(x, y, yaw);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("Could not transform %s to %s :%s", global_frame_.c_str(), base_frame_.c_str(), ex.what());
  }
}



/*--------------------- Callback function  ---------------------*/
void PlannerBaseROS::MapLoadCallback(const nav_msgs::OccupancyGrid msg) {
  map_ = map_to_raw(msg);
  lower_left_.x = msg.info.origin.position.x;
  lower_left_.y = msg.info.origin.position.y;
  lower_left_.theta = 0;
  map_width_ = msg.info.width;
  map_height_ = msg.info.height;
  map_resolution_ = msg.info.resolution;

  setMap(map_width_, map_height_, map_resolution_, lower_left_, map_);
}

void PlannerBaseROS::CostmapLoadCallback(const nav_msgs::OccupancyGrid msg) {
  costmap_ = map_to_raw(msg);
  cost_lower_left_.x = msg.info.origin.position.x;
  cost_lower_left_.y = msg.info.origin.position.y;
  lower_left_.theta = 0;
  cost_width_ = msg.info.width;
  cost_height_ = msg.info.height;
  cost_resolution_ = msg.info.resolution;

  setCostMap(map_width_, map_height_, map_resolution_, lower_left_, costmap_);
}

void PlannerBaseROS::WaypointCallback(geometry_msgs::PoseStamped msg) {
  setGoal(msg); 
}
/*------------------ end of Callback function ------------------*/

