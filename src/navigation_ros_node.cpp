#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"

using namespace sm;

void testCallback(const nav_msgs::OccupancyGrid msg){
  ;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stl_planner");
  ros::NodeHandle n;
  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);

  StateMachineROS smr();

  //ros::Subscriber sub = n.subscribe("/map", 1000, testCallback);

  ros::spin();

  return 0;
}
