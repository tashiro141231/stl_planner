#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include <vector>
#include <string>
#include <iostream>
#include <inttypes.h>

#include "tf/transform_listener.h"

#include "stl_planner/rosmsg_raw_conv.h"

unsigned char* map_to_raw(nav_msgs::OccupancyGrid map) {
  unsigned char *data = (unsigned char *)malloc(sizeof(unsigned char) * map.info.width*map.info.height);

  for(int i=0; i < map.info.width*map.info.height; i++){
    data[i] = map.data[i];
  }
  return data;
}

nav_msgs::Path path_to_rospath(std::vector<Point> path, std::string global_frame) {
  nav_msgs::Path ros_path;
  geometry_msgs::PoseStamped pose;
  geometry_msgs::PoseStamped pre_pose;
  pose.pose.position.z = 0;
  
  
  for(int itr = 0; itr < path.size()-1; itr++){
    pose.pose.position.x = path[itr].x;
    pose.pose.position.y = path[itr].y;
    std::cout << "path :" << path[itr].x << ", " << path[itr].y << std::endl;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(CalcDirection(path[itr], path[itr+1]));
    ros_path.poses.push_back(pose);
  }
  pose.pose.position.x = path[path.size()-1].x;
  pose.pose.position.y = path[path.size()-1].y;
  ros_path.poses.push_back(pose);


  ros_path.header.frame_id = global_frame;
  ros_path.header.stamp = ros::Time::now();

  return ros_path;
}

double CalcDirection(Point a, Point b) {
  return atan2(b.y-a.y, b.x-a.x);
}

Point pose_to_point(geometry_msgs::Pose pose) {
  Point p;
  p.x = pose.position.x;
  p.y = pose.position.y;
  p.theta = quaternion_to_theta(pose.orientation);

  return p;
}

double quaternion_to_theta(geometry_msgs::Quaternion orientation) {
  double yaw,roll,pitch;
  tf2::Quaternion tfq;
  tf2::convert(orientation, tfq);
  tf2::Matrix3x3(tfq).getRPY(roll, pitch, yaw);

  return yaw;
}
