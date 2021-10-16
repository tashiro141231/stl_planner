//////////
gazebo上simulation用
roslaunch turtlebot3_gazebo  turtlebot3_world.launch 
roslaunch stl_planner costmap_test.launch
roslaunch stl_planner astar_sim.launch
roslaunch stl_planner purepursuit_sim.launch 

////////////////
参考情報
$ roslaunch turtlebot3_gazebo  turtlebot3_world.launch 
Simulator setup and turtlebot's tf start publishing(GUI off)

$ roslaunch stl_planner costmap_test.launch
start localization (amcl) and move_base won't start.
move_baseが動かないため代わりにcost_mapを発行するノードも動かしてる（はずだが？）
->amclとかもきどうしてる

$rosrun stl_planner costmap_test
->costmap発行デバッグだけするとき用


ここで一回Rvizで初期位置推定を修正（もしくはteleopで少し動かして自己位置を安定させる）

Then you can debag your planner with v, omega.

Rviz上でgoalを追加して動かす

$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
キーボードで動かせます。

roslaunch turtlebot3_fake turtlebot3_fake.launch
robot_state_publisherなど起動


