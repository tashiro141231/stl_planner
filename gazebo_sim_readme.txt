$ roslaunch turtlebot3_gazebo  turtlebot3_world.launch 
Simulator setup and the tf start publishing(GUI off)

$ roslaunch stl_planner turtlebot_localization.launch
start localization (amcl) and move_base won't start.
costmap_2d will not launch

Then you can debag your planner with v and omega.

---------------------------------------------------------------
$ roslaunch turtlebot3_gazebo  turtlebot3_world.launch 
turtlebot3 が起動します

$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
キーボードで動かせます。

roslaunch turtlebot3_fake turtlebot3_fake.launch
robot_state_publisherなど起動

