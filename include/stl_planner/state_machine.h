#include <string>
#include <vector>
#include <iostream>

#include "stl_planner/nav_core.h"
#include "stl_planner/gp.h"

namespace sm {


class StateMachine {
  public:
    /**
    * @brief Constructor
    */
    StateMachine();
    
    /**
     * @brief  Destructor - Cleans up
     */
    //virtual ~StateMachine();
    void Initialize(double robot_width, double robot_length);
    //setter
    void setCurrentPosition(double x, double y, double theta);
    void setGoalPosition(double x, double y, double theta);
    void setMap(int width, int height, unsigned char* data);
    void setCostmap(int width, int height, unsigned char* data);
    //getter
    RobotStatus getCurrentState();

    //Planner call function
    std::vector<Point> CalcGlobalPlan(double x, double y, double theta);
    std::vector<Point> DebagCalcGlobalPlan(Point start, Point goal);


  private:
    GPlanner gp;
    RobotStatus current_state_;
    Position current_goal_;
    Position current_pos_;

    //robot param
    double robot_width_;
    double robot_length_;

    //map param
    int m_width_;
    int m_height_;
    int c_width_;
    int c_height_;
    double resolution_;
    bool initialized_;
    double path_resolution_;
    unsigned char* map_;
    unsigned char* costmap_;
};
};
