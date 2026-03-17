# 1902TargetSearch


# turtlebot3_maze_avoidance使用方法

终端1:启动 Gazebo 迷宫环境

  先按自己电脑路径进入TargetSearch文件夹

  source /opt/ros/noetic/setup.bash

  catkin_make

  source devel/setup.bash
  
  export TURTLEBOT3_MODEL=waffle

  roslaunch maze_runner start_maze.launch


终端2:启动 GMapping 建图

  先按自己电脑路径进入TargetSearch文件夹

  source /opt/ros/noetic/setup.bash

  source devel/setup.bash

  export TURTLEBOT3_MODEL=waffle

  roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
  

终端3:运行探索脚本

  先按自己电脑路径进入TargetSearch文件夹

  source /opt/ros/noetic/setup.bash

  source devel/setup.bash

  export TURTLEBOT3_MODEL=waffle

  rosrun maze_runner 文件名.py

测试时建议同时打开rviz和终端3，分屏观看


