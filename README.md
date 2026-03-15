# 1902TargetSearch

进度
3/4 可自动避障，从起点走到终点。
3/8 可遍历全图（地图2号），见image。未定义回到原点的条件。算法“贪心”陷阱。


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

<img width="415" height="408" alt="image" src="https://github.com/user-attachments/assets/5514cf63-8e90-4f9e-8ee0-9aca773187bb" />
<img width="416" height="383" alt="image" src="https://github.com/user-attachments/assets/77984596-9cd8-460d-949f-6a2b96039f7f" />
<img width="978" height="982" alt="6623545591e4c4dfb8595011ef9c2458" src="https://github.com/user-attachments/assets/4d3897b8-c670-431c-8808-a8d9b469cd7c" />

