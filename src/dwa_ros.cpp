#include <stl_planner/dwa_ros.h>

DWA_ROS::DWA_ROS(tf2_ros::Buffer& tf) : PlannerBaseROS(tf)
 {
  Initialize();
}

void DWA_ROS::setDWAParams(){
  ;
}

void DWA_ROS::setMap() {
  dwa.SetMap(getMapWidth(), getMapHeight(), getMapResolution(), getMapLowerLeft(), getMapRaw());
}

void DWA_ROS::main_loop() {
  ros::Rate loop_rate(getLoopRate());
  while(ros::ok()) {
    UpdateCurrentPosition();
    ros::spinOnce();
    // lp.calc_path();
    loop_rate.sleep();
  }
}
