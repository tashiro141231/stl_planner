/**
 * @file line_planner.h
 * @brief local planer for STELLA.
 * @author Yosuke TASHIRO, Kotaro HIHARA
 * @date 2020.08.01
*/

#include <iostream>
#include <queue>
#include <vector>
#include <algorithm>
#include <cmath>

#include "stdio.h"

#include "stl_planner/nav_core.h"

class LinePlanner {
    public:
        LinePlanner();
        ~LinePlanner(){}
        void Initialize(double max_vel, double min_vel, double acc, double max_w, double max_eta, double Ko, double Kp, double Ke, double loop_rate);
        bool UpdateVW();
        void UpdateFeedbackParam();
        bool getCost(int x, int y);
        bool goalCheck();
        double calc_distance(Point p1, Point p2);
        double ConvAngleRange(double value);
        void InitTargetLine();
        void InitTargetLineGoal();
        Point ConvGridToPoint(Point p);
        Point ConvPointToGrid(Point p);
        std::vector<Node> RawMapToNode(Point lower_left, unsigned char *map);
        
        //setter
        void setCurrentPosition(Point point);
        void setCurrentVelOmega(double vel, double omega);
        void setStartPoint(Point start);
        void setGoalPoint(Point goal);
        void setStartGoal(Point start, Point goal);
        void setMap(int width, int height, double resolution, Point lower_left, unsigned char *map);
        void setCostMap(int width, int height, double resolution, Point lower_left, unsigned char *map);

        //getter
        bool isRobotPlannning();
        double getVelOut();
        double getOmgOut();

    private:
        bool is_set_goal_;
        bool start_deceleration_;
        double deceleration_distance_;
        double goal_distance_;
        Point start_, goal_;
        double remain_distance_;
        double map_resolution_;
        int map_width_, map_height_;
        Point map_lower_left_;
        Point centre;
        Point current_pos_, prev_pos_;
        double robot_width_, robot_length_;
        unsigned char* static_map_;
        unsigned char* dynamic_map_;

        double max_vel_, min_vel_;
        double max_omega_, min_omega_;
        double current_vel_, prev_vel_;
        double current_omega_, prev_omega_;
        double target_vel_, target_omega_;
        double acc_;
        double decel_;
        double dt_;
        // For Feedback
        double Ke_, Kp_, Ko_;
        double eta_, phi_, omega_diff_;
        double max_eta_;
        //Target line
        double target_line_angle_;
        double la_, lb_,lc_;
       
};
