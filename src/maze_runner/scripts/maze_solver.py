#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf
import math

class MazeSolver:
    def __init__(self):
        rospy.init_node('maze_solver', anonymous=True)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.rate = rospy.Rate(10) # 10Hz
        self.laser_regions = {'front': 10.0, 'left': 10.0, 'right': 10.0}
        self.current_pos = {'x': 0.0, 'y': 0.0, 'theta': 0.0}

        # ！！请确认这里的坐标是你的终点坐标！！
        self.goal_x = 4.5
        self.goal_y = 4.5
        self.goal_reached = False
        
        # === 新增：打破死锁的状态变量 ===
        self.avoidance_timer = 0  # 强制避障计时器
        self.turn_dir = 1.0       # 1.0为左转，-1.0为右转

    def scan_callback(self, msg):
        ranges = msg.ranges
        self.laser_regions = {
            'right':  min(min(ranges[270:330]), 10.0),
            'front':  min(min(ranges[0:30] + ranges[330:359]), 10.0),
            'left':   min(min(ranges[30:90]), 10.0),
        }

    def odom_callback(self, msg):
        self.current_pos['x'] = msg.pose.pose.position.x
        self.current_pos['y'] = msg.pose.pose.position.y
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.current_pos['theta'] = euler[2]

    def run(self):
        rospy.loginfo("开始执行迷宫寻路与避障任务...")
        while not rospy.is_shutdown():
            if self.goal_reached:
                break

            msg = Twist()
            inc_x = self.goal_x - self.current_pos['x']
            inc_y = self.goal_y - self.current_pos['y']
            distance_to_goal = math.sqrt(inc_x**2 + inc_y**2)
            angle_to_goal = math.atan2(inc_y, inc_x)

            if distance_to_goal < 0.3:
                rospy.loginfo("成功到达终点！")
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.vel_pub.publish(msg)
                self.goal_reached = True
                break

            # 稍微缩小安全距离，让小车敢于穿过狭窄走廊
            safe_dist = 0.45 
            
            # === 核心改进版逻辑 ===
            if self.avoidance_timer > 0:
                # 【状态1：正在强制逃离死角】
                self.avoidance_timer -= 1
                if self.laser_regions['front'] < safe_dist:
                    # 如果前方还是有障碍，继续死磕之前决定的方向转弯
                    msg.linear.x = 0.0
                    msg.angular.z = 0.5 * self.turn_dir
                else:
                    # 前方终于空旷了！强制往前走一段时间，而不是马上回头看终点
                    msg.linear.x = 0.2
                    msg.angular.z = 0.0
            else:
                # 【状态2：正常的趋向终点】
                if self.laser_regions['front'] < safe_dist:
                    # 突然遇到障碍物，立刻开启强制逃离模式（持续 40 个循环 = 4秒）
                    self.avoidance_timer = 40 
                    
                    # 决定转弯方向（哪边宽敞往哪转），并在接下来的4秒内记住这个方向
                    if self.laser_regions['left'] > self.laser_regions['right']:
                        self.turn_dir = 1.0   # 左转
                    else:
                        self.turn_dir = -1.0  # 右转
                        
                    msg.linear.x = 0.0
                    msg.angular.z = 0.5 * self.turn_dir
                else:
                    # 前方安全，计算如何朝向终点移动
                    angle_diff = angle_to_goal - self.current_pos['theta']
                    while angle_diff > math.pi: angle_diff -= 2*math.pi
                    while angle_diff < -math.pi: angle_diff += 2*math.pi

                    if abs(angle_diff) > 0.2:
                        msg.linear.x = 0.0
                        msg.angular.z = 0.3 if angle_diff > 0 else -0.3
                    else:
                        msg.linear.x = 0.2
                        msg.angular.z = 0.0

            self.vel_pub.publish(msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        solver = MazeSolver()
        solver.run()
    except rospy.ROSInterruptException:
        pass