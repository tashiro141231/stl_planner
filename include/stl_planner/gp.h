/**
 * @file gp.h
 * @brief Global Planer for STELLA.
 * @author Yosuke TASHIRO, Kotaro HIHARA
 * @date 2020.04.19
*/

#include <iostream>
#include <queue>
#include <vector>
#include <algorithm>
#include <cmath>

#include "stdio.h"

#include "stl_planner/nav_core.h"

/*
struct Point{
	double x;
	double y;
} ;

struct Node{
    int x_ind;
    int y_ind;
    int grid_cost;
    int cost;
    Node *1arent;
};
*/

class GPlanner{
    public:
        double resolution = 0.1;
        double sx,sy,st; //start
        double gx,gy,gt; //goal
	      int height;
	      int width;
        double max_x,max_y,min_x,min_y;   //mapの
        int max_xind,max_yind;            //indexの最大
        std::vector<Point> path;
        GPlanner();
        void Initialize(double _sx,double _sy,double _gx,double _gy, 
        int _width, int _height , double _resolution, unsigned char *_cost){
            start_.x = _sx;
            start_.y = _sy;
            goal_.x = _gx;
            goal_.y = _gy;
            width_ = _width;
            height_ = _height;
            resolution_ = _resolution;
            Init_robotmodel();
            //calc_map(map_x,map_y,cost);
            //vector<Node> _grid(calc_ind(max_xind,max_yind));
            //grid.insert(grid.end(),_grid.begin(),_grid.end());
        }
        ~GPlanner(){
        }
        void Init_robotmodel();
        bool isSamePoint(Point start, Point goal);
        bool isObstacle(Point p);
        double calc_heuristic(Point p1, Point p2);
        int calc_ind(int x, int y);
        void calc_map(std::vector<double> map_x, std::vector<double> map_y, std::vector<int> cost);
        void map_init();
        std::vector<unsigned char> costmap_calc(Point lower_left, unsigned char* map);
        std::vector<unsigned char> costmap_calc_vis(Point lower_left, unsigned char* map);
        Point calc_position(int index,int min);  
        void calc_path();
        std::vector<Point> calc_path_astar();
        std::vector<Node> rawmap_to_node(Point lower_left, unsigned char* map);
        std::vector<Node> rawmap_to_node_vis(Point lower_left, unsigned char* map);
        void sortPointCost(std::vector<std::pair<Point, Point> >& open);
        void checkLists(std::pair<Point, Point> node, std::vector<std::pair<Point, Point> >& open, std::vector<std::pair<Point, Point> >& close);
        std::vector<Point> lookup_closednode(std::vector<std::pair<Point, Point> >& close, std::vector<std::pair<Point, Point> >& open);
        void visualize_result(std::vector<Point> path, std::vector<std::pair<Point, Point> > close);
        Point ConvGridScale(Point p);

        //setter
        void setStartPoint(Point start);
        void setGoalPoint(Point goal);
        void setStartGoal(Point start, Point goal);
        void setMap(int width, int height, double reslution, Point lower_left, unsigned char *map);
        //getter
        Node getCostOrigin(int x, int y);
        std::vector<Point> getPath();
        Point createpoint(double x, double y);
        Point createpoint_cost(double x, double y, double cost);

        Point ConvGridPoint(Point point);

    private:
        Point start_;
        Point goal_;
        Point centre_;
        int height_;
        int width_;
        double resolution_;
        int max_x_;
        int max_y_;
        Point lower_left_;
        std::vector<Point> robot_model_;
        std::vector<Node> o_map_;
        std::vector<unsigned char> total_cost_;
};

