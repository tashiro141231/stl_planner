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
  pose.pose.position.z = 0;
  
  for(auto itr = path.begin(); itr != path.end(); itr++){
    pose.pose.position.x = itr->x;
    pose.pose.position.y = itr->y;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(itr->theta);
    ros_path.poses.push_back(pose);
  }
  ros_path.header.frame_id = global_frame;
  ros_path.header.stamp = ros::Time::now();

  return ros_path;
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
