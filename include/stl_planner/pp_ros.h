
#include <stl_planner/nav_core.h>
#include <stl_planner/pure_pursuit.h>
#include <stl_planner/planner_base_ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <time.h>

class PurePursuit_ROS : PlannerBaseROS {
  public:
    PurePursuit_ROS(tf2_ros::Buffer& tf);
    PP_Planner pp;
    void main_loop();
    void PlannerInitialize();
    void GlobalPlanCallback(nav_msgs::Path path);
  
  private:
    void setMap(int width, int height, double resolution, Point lower_left, unsigned char* map);
    void setCostMap(int width, int height, double resolution, Point lower_left, unsigned char* map);
    void setGoal(geometry_msgs::PoseStamped msg);
    void setCurrentPositionToPlanner(Point point);
    void PubLocalPath(nav_msgs::Path path);
    void PubGlobalPath(nav_msgs::Path path);
    void PubVelOmgOutput(double v, double w);
    void StopModeCallback(std_msgs::Bool stop_mode);
    void StopNavigationCallback(std_msgs::Bool stop_navigation);
  
    bool planner_initialized_;

    ros::Publisher pub_pp_vw_;
    ros::Publisher pub_goal_;
    ros::Publisher pub_pp_path_;
    ros::Publisher pub_navigation_state_;

    //std::string astar_topic_;
    ros::Subscriber astar_sub_;
    ros::Subscriber sub_stop_mode_;
    ros::Subscriber sub_stop_navigation_;

    double rate_;
    std_msgs::String navigation_state_;
    bool stop_navigation_;
    bool stop_mode_;
    time_t time_goalset;
    time_t now;
    bool waiting;
};

