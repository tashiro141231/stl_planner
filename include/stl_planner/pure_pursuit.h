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


class PP_Planner{
    public: 
        PP_Planner();
        ~PP_Planner(){
        }

        void Initialize(double max_vel, double min_vel, double max_acc, double max_w, double min_w) ;

        void SetParams(double v_max, double v_min, double max_acc, double w_max, double w_min);
        bool UpdateVW();
        bool goalCheck();
        void pure_pursuit();
        double angle_correct(double theta);

        //setter
        void setCurrentPosition(Point point);
        void setStartPoint(Point start);
        void setGoalPoint(Point goal);
        void setStartGoal(Point start, Point goal);
        //getter
        bool isRobotPlannning();
        double getVelOut();
        double getOmgOut();
        std::vector<Point> getPath();
        geometry_msgs::Point select_target();
        nav_msgs::Path global_path;
        bool stop_mode_;
        
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
        std::vector<Point> current_path_;
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

        double alpha0;
        double alpha_;
        double look_ahead_distance_=3.0;
        double target_vel_;
        double dist_;
        geometry_msgs::Point target_point_;
        double theta_;
        double stop_min_vel_;

        double r_;
};