/**
 * @file purepursuit.h
 * @brief Local Planer for STELLA.
 * @author Kotaro HIHARA
 * @date 2021.07.20
*/
#include <iostream>
#include <queue>
#include <vector>
#include <algorithm>
#include <cmath>

#include "stdio.h"

#include "stl_planner/nav_core.h"
#include "nav_msgs/Path.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

class PP_Planner{
    public: 
        PP_Planner();
        ~PP_Planner(){
        }

        void Initialize(double max_vel, double min_vel, double max_acc, double stop_dec,double max_w, double min_w,double set_vel,double wp_range,double goal_range,double max_dw,double dt,double stop_vel,double look_dist,double stop_dist,double predict_time) ;

        void SetParams(double v_max, double v_min, double max_acc,double stop_dec, double w_max, double w_min,double set_vel,double wp_range,double goal_range,double max_dw,double dt,double stop_vel,double look_dist,double stop_dist,double predict_time);
        bool UpdateVW();
        bool goalCheck();
        void pure_pursuit();
        double angle_correct(double theta);
        bool goalYawCheck();
        double diff_goalYaw();

        //setter
        void setCurrentPosition(Point point);
        void setStartPoint(Point start);
        void setGoalPoint(Point goal);
        void setStartGoal(Point start, Point goal);
        void updateSetVel(double current_set_vel);
        //getter
        bool isRobotPlannning();
        double getVelOut();
        double getOmgOut();
        nav_msgs::Path getPath();
        geometry_msgs::Point select_target();
        nav_msgs::Path global_path;
        bool stop_mode_;
        bool stop_navigation_;
        
    private:
        bool is_set_goal_;
        Point start_;
        Point goal_;
        Point centre_;
        int height_;
        int width_;
        double resolution_;

        double current_vel_;
        double current_omega_;
        double vel_out_;
        double omega_out_;
        Point current_pos_;
        
        //initialized
        double max_vel_ ;//[m/s]
        double min_vel_ ;
        double max_w_ ;//[rad/s]
        double min_w_ ;
        double max_dw_;//[rad/ss]
        double max_acc_;//[m/ss]
        double stop_dec_;
        double set_vel_;
        double wp_range_;
        double goal_range_;

        double alpha0;
        double alpha_;
        double look_ahead_distance_=3.0;
        double target_vel_;
        double target_omega_;
        double dist_;
        geometry_msgs::Point target_point_;
        double theta_;
        double stop_vel_;
        double look_dist_;
        double stop_dist_;
        double predict_time_;

        double r_;
        double dt_;
        bool dist_fix_;
        int predict_size_;
        double goalyaw_threshold=0.1;
        double yaw_adjust_omega=0.8;
};