
#include <stl_planner/nav_core.h>
#include <stl_planner/lp.h>
#include <stl_planner/planner_base_ros.h>

class DWA_ROS : PlannerBaseROS {
  public:
    DWA_ROS(tf2_ros::Buffer& tf);
    LPlanner dwa;
    void main_loop();

  private:
    void setDWAParams();
    void setMap();

    ros::Publisher dwa_vw_;
    ros::Publisher dwa_path_;
};

