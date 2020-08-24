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

typedef struct VW{
    double v;
    double w;
}VW;

class LPlanner{
    public:
        
        double max_vel = 1.0;//[m/s]
        double min_vel = 0;
        double max_w = 40*M_PI/180;//[rad/s]
        double min_w = -max_w;
        double max_dw = 40*M_PI/180;//[rad/ss]
        double max_accel = 0.2;//[m/ss]
        double v_resolution=0.01;
        double w_resolution=1*M_PI/180;
        double dt = 0.1;
        double predict_time =3.0;
        double to_goal_cost_gain =1.0;
        double speed_cost_gain = 1.0;
        double robot_radius = 0.3;
        double v_out;
        double w_out;
        LPlanner();
        void Initialize(double _sx,double _sy, double _gx, double _gy) {
            start_.x = _sx;
            start_.y = _sy;
            goal_.x = _gx;
            goal_.y = _gy;
        }
        ~LPlanner(){
        }

        void SetParams(double v_max, double v_min, double max_acc, double w_max, double w_min, double v_reso, double w_reso, double dtime, double prediction_time);
        std::vector<Node> rawmap_to_node(Point lower_left, unsigned char* map);
        void setMap(int width, int heihgt, double resolution, Point lower_left, unsigned char* map);
        State motion(State x, double v, double w);
        DW calc_dynamic_window(State X);
        std::vector<State> calc_trajectory(State x,double v,double w);
        void calc_final_input(State x, VW u, DW dw, Point goal,std::vector<Point> ob);
        double calc_obstacle_cost(std::vector<State> traj, std::vector<Point> ob);
        double calc_to_goal_cost(std::vector<State> traj,Point goal);
        void dwa_control(double x,double y,double theta, double v, double w, Point goal, std::vector<Point> ob);
        Point ConvGridScale(Point p);
        
    private:
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
};

