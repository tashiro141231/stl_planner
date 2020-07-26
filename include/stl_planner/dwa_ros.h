
#include <stl_planner/nav_core.h>
#include <stl_planner/lp.h>
#include <stl_planner/planner_base_ros.h>

class DWA_ROS : PlannerBaseROS {
  public:
    DWA_ROS(tf2_ros::Buffer& tf);
    LPlanner dwa;
    void main_loop();
    void setMap(int width, int height, double resolution, Point lower_left, unsigned char* map);
    void setCostMap(int width, int height, double resolution, Point lower_left, unsigned char* map);
    void setGoal(geometry_msgs::PoseStamped msg);

  private:
    void setDWAParams();

    ros::Publisher dwa_vw_;
    ros::Publisher dwa_path_;
};

