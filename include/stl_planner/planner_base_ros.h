#ifndef _PLANNER_BASE_H_
#define _PLANNER_BASE_H_

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

#include <stl_planner/nav_core.h>
#include <stl_planner/rosmsg_raw_conv.h>


class PlannerBaseROS {
  public:
    PlannerBaseROS(tf2_ros::Buffer& tf);
    virtual void main_loop();
    virtual void Initialize();
    void UpdateCurrentPosition();
    //void UpdateRobotStatus(double robot_v, double robot_w);
    // bool SetRobotInfo(std::strint path);
    //Callback
    void MapLoadCallback(const nav_msgs::OccupancyGrid msg);
    void CostmapLoadCallback(const nav_msgs::OccupancyGrid msg);
    virtual void setMap() = 0;

    //getter
    int getLoopRate() { return loop_rate_; }
    int getMapWidth() { return map_width_; }
    int getMapHeight() { return map_height_; }
    double getRobotWidth() { return robot_width_; }
    double getRobotLength() { return robot_length_; }
    double getMaxVel() { return max_vel_; }
    double getMinVel() { return min_vel_; }
    double getMaxOmega() { return max_w_; }
    double getMinOmega() { return min_w_; }
    double getMaxacc() { return max_acc_; }
    double getDt() { return dt_; }
    double getVelReso() { return v_resolution_; }
    double getOmegaReso() { return w_resolution_; }
    double getMapResolution() { return map_resolution_; }
    unsigned char* getMapRaw() { return map_; }
    double getPredictTime() { return predict_time_; }
    Point getCurrentPos() { return current_pos_; }
    bool isRobotMoving() { return is_robot_moving_; }
 
    //setter
    void setLoopRate(int val) { loop_rate_ = val; }
    void setMapWidth(int val) { map_width_ = val; }
    void setMapHeight(int val) { map_height_ = val; }
    void setRobotWidth(double val) { robot_width_ = val; }
    void setRobotLength(double val) { robot_length_ = val; }
    void setMaxVel(double val) { max_vel_ = val; }
    void setMinVel(double val) { min_vel_ = val; }
    void setMaxOmega(double val) { max_w_ = val; }
    void setMinOmega(double val) { min_w_ = val; }
    void setMaxAcc(double val) { max_acc_ = val; }
    void setGaolTopicName(std::string val) { goal_topic_ = val; }
    void setMapLowerLeft(Point val) { lower_left_ = val; }

  private:
    geometry_msgs::TransformStamped ts_;
    tf2_ros::Buffer& tf_;

    bool initialized_;
    bool is_robot_moving_;
    Point current_pos_;
    int loop_rate_;
    std::string goal_topic_;
   
    //ROS subscriber
    ros::Subscriber map_sub_;
    ros::Subscriber costmap_sub_;
    ros::Subscriber goal_sub_;

    //Robot control params
    double max_vel_, max_w_;
    double min_vel_, min_w_;
    double max_acc_;
    double predict_time_;
    double dt_;
    double vel_stop_thr_, omega_stop_thr_;
    double v_resolution_, w_resolution_;

    //robot param
    double robot_width_;
    double robot_length_;

    //map param
    std::string global_frame_;
    std::string base_frame_;
    Point lower_left_;
    int map_width_;
    int map_height_;
    int cost_width_;
    int cost_height_;
    double map_resolution_;
    double path_resolution_;
    unsigned char* map_;
    unsigned char* costmap_;
};

#endif //_PLANNER_BASE_H_

