
#include <stl_planner/nav_core.h>
#include <stl_planner/lp.h>
#include <stl_planner/planner_base_ros.h>

class DWA_ROS : PlannerBaseROS {
  public:
    DWA_ROS(tf2_ros::Buffer& tf);
    LPlanner dwa;
    void main_loop();
    void PlannerInitialize();
  
  private:
    void setMap(int width, int height, double resolution, Point lower_left, unsigned char* map);
    void setCostMap(int width, int height, double resolution, Point lower_left, unsigned char* map);
    void setGoal(geometry_msgs::PoseStamped msg);
    void setCurrentPositionToPlanner(Point point);
    void PubLocalPath(nav_msgs::Path path);
    void PubGlobalPath(nav_msgs::Path path);
    void PubVelOmgOutput(double v, double w);
  
    bool planner_initialized_;

    ros::Publisher pub_dwa_vw_;
    ros::Publisher pub_gp_;
    ros::Publisher pub_goal_;
    ros::Publisher pub_dwa_path_;
};

