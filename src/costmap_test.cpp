#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "costmap");
#if ROS_VERSION_MINIMUM(1,14,0)
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  costmap_2d::Costmap2DROS costmap("local_cost_map", tfBuffer);
#else
  tf::TransformListener tf(ros::Duration(10));
  costmap_2d::Costmap2DROS costmap("local_cost_map", tf);
#endif
  ros::NodeHandle n;
  costmap.start();
  ros::spin();
  return 0;
}
