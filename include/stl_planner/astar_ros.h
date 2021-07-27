
#include <stl_planner/nav_core.h>
#include <stl_planner/gp.h>
#include <stl_planner/planner_base_ros.h>

class AStarROS : PlannerBaseROS {
  public:
    AStarROS(tf2_ros::Buffer& tf);
    GPlanner gp;
    void main_loop();

  private:
    void setDWAParams();
    void setMap(int width, int height, double resolution, Point lower_left, unsigned char* map);
    void setCostMap(int width, int height, double resolution, Point lower_left, unsigned char* map);
    void setCurrentPositionToPlanner(Point point);
    void setGoal(geometry_msgs::PoseStamped msg);
    void setCurrentPosition(Point pos);
    void PubGlobalPath(nav_msgs::Path path);

    ros::Publisher dwa_vw_;
    ros::Publisher pub_gp_;
};

