/**
 * @file state_machine_ros.h
 * @brief State Machine ROS driver class.
 * @author Yosuke TASHIRO, Kotaro HIHARA
 * @date 2020.04.19
*/

#include "ros/ros.h"

#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Path.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"

#include <string>
#include <vector>
#include <iostream>

#include <stl_planner/state_machine.h>

namespace smr {

class StateMachineROS {
  public:
    StateMachineROS(tf2_ros::Buffer& tf);
    void Initialize();
    void main_loop();
    void UpdateCurrentPosition();
    void UpdateRobotStatus(double robot_v, double robot_w);
    //Publish func
    void PubGlobalPath(nav_msgs::Path path);
    void PubLocalPath(nav_msgs::Path path);
    //Callback
    void MapLoadCallback(const nav_msgs::OccupancyGrid msg);
    void CostmapLoadCallback(const nav_msgs::OccupancyGrid msg);
    void WaypointCallback(const geometry_msgs::PoseStamped msg);
    //Planer call function
    nav_msgs::Path CalcGlobalPlan(geometry_msgs::PoseStamped goal);
    nav_msgs::Path DebagCalcGlobalPlan(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal);
  
  private:
    sm::StateMachine sm_;

    ros::Time stopped_time_;
    geometry_msgs::TransformStamped ts_;
    tf2_ros::Buffer& tf_;
    nav_msgs::OccupancyGrid map_;

    bool initialized_;
    bool is_robot_moving_;
    Position current_pos_;
   
    std::string global_frame_;
    std::string base_frame_;
    std::string goal_topic_;
    int loop_rate_;

    double vel_stop_thr_;
    double omega_stop_thr_;

    double robot_width_;
    double robot_length_;
    double robot_radius_;

    //dwa params
    double max_vel_, min_vel_, max_acc_;
    double max_w_, min_w_;
    double dt_, predict_time_;
    double goal_cost_gain_;
    double speed_cost_gain_;
    double v_resolution_, w_resolution_;
    
    ros::Publisher gp_path_pub_;
    ros::Publisher lp_path_pub_;
    ros::Subscriber map_sub_;
    ros::Subscriber costmap_sub_;
    ros::Subscriber goal_sub_;
};
};

