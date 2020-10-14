$ roslaunch turtlebot3_gazebo  turtlebot3_world.launch 
Simulator setup and turtlebot's tf start publishing(GUI off)

$ roslaunch stl_planner turtlebot_localization.launch
start localization (amcl) and move_base won't start.

Then you can debag your planner with v, omega.

---------------------------------------------------------------
$ roslaunch turtlebot3_gazebo  turtlebot3_world.launch 
turtlebot3 が起動します

$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
キーボードで動かせます。

roslaunch turtlebot3_fake turtlebot3_fake.launch
robot_state_publisherなど起動

