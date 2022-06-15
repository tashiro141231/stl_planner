
#include <stl_planner/nav_core.h>
#include <stl_planner/lp.h>
#include <stl_planner/planner_base_ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <time.h>

#include <sensor_msgs/PointCloud2.h>
//#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
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

    void StopModeCallback(std_msgs::Bool stop_mode);
    void StopNavigationCallback(std_msgs::Bool stop_navigation);
    void Obstacle2dCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  
  
    bool planner_initialized_;

    ros::Publisher pub_dwa_vw_;
    ros::Publisher pub_gp_;
    ros::Publisher pub_goal_;
    ros::Publisher pub_dwa_path_;
    ros::Publisher pub_navigation_state_;

    ros::Subscriber sub_stop_mode_;
    ros::Subscriber sub_stop_navigation_;
    ros::Subscriber sub_obstacle_2d_;

    double rate_;
    std_msgs::String navigation_state_;
    bool stop_navigation_;
    bool stop_mode_;
    time_t time_goalset;
    time_t now;
    bool waiting;
    pcl::PointCloud<pcl::PointXYZ> obstacle_2d_;

};

