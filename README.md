# 1902TargetSearch

# turtlebot3_maze_avoidance使用方法

每次打开新终端前执行这一个 source ~/maze_ws/devel/setup.bash

第一次打开终端执行 
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
source ~/.bashrc


终端一 启动带小车的 Gazebo 地图
roslaunch maze_runner start_maze.launch

终端二 启动 SLAM 建图和 RViz 可视化界面
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

终端三 运行避障导航算法
chmod +x ~/maze_ws/src/maze_runner/scripts/maze_solver.py
rosrun maze_runner maze_solver.py
