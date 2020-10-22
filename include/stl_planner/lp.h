/**
 * @file lp.h
 * @brief Local Planer for STELLA.
 * @author Kotaro HIHARA
 * @date 2020.05.10
*/
#include <iostream>
#include <queue>
#include <vector>
#include <algorithm>
#include <cmath>

#include "stdio.h"

#include "stl_planner/nav_core.h"

typedef struct State {
  double x;
  double y;
  double yaw;
  double v;
  double w;
} State;

typedef struct DW {
    double v_min;
    double v_max;
    double w_min;
    double w_max;
}DW;


class LPlanner{
    public:
        

        LPlanner();
        ~LPlanner(){
        }

        void Initialize(double max_vel, double min_vel, double max_acc, double max_w, double min_w, 
            double max_dw, double dt, double v_resolution, double w_resolution, double predict_time, 
            double goal_gain, double speed_gain, double ob_gain, double robot_radius) ;

        void SetParams(double v_max, double v_min, double max_acc, double w_max, double w_min, double v_reso, double w_reso, double dtime, double prediction_time);
        std::vector<Node> rawmap_to_point(Point lower_left, unsigned char* map);
        State motion(State x, double v, double w);
        DW calc_dynamic_window(State X);
        std::vector<State> calc_trajectory(State x,double v,double w);
        void calc_path_dwa(State x, DW dw, Point goal,std::vector<Node> ob);
        double calc_obstacle_cost(std::vector<State> traj, std::vector<Node> ob);
        double calc_to_goal_cost(std::vector<State> traj,Point goal);
        void dwa_control();
        bool UpdateVW();
        bool goalCheck();

        //setter
        void setCurrentPosition(Point point);
        void setStartPoint(Point start);
        void setGoalPoint(Point goal);
        void setStartGoal(Point start, Point goal);
        void setMap(int width, int height, double resolution, Point lower_left, unsigned char *map);
        void setCostMap(int width, int height, double resolution, Point lower_left, unsigned char *map);
        //getter
        bool isRobotPlannning();
        double getVelOut();
        double getOmgOut();
        std::vector<Point> getPath();

    private:
        bool is_set_goal_;
        Point start_;
        Point goal_;
        Point centre_;
        int height_;
        int width_;
        double resolution_;
        int max_x_;
        int max_y_;
        std::vector<Point> robot_model_;
        std::vector<Node> o_map_;
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
        double v_resolution_;
        double w_resolution_;
        double dt_;
        double predict_time_;
        double goal_gain_ ;
        double speed_gain_ ;
        double ob_gain_ ;
        double robot_radius_ ;
};

