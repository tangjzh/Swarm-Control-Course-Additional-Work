#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import sys
import signal

def signal_handler(sig, frame):
    rospy.loginfo("Shutdown signal received. Exiting...")
    sys.exit(0)

def send_nav_goal(position_x):
    # 初始化节点
    rospy.init_node('send_nav_goals_node', anonymous=True)
    
    # 创建一个发布器，发布到 /move_base_simple/goal 话题fabu
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    # goal_pub.publish(goal)
    rospy.sleep(2.0)

    # 设置目标位置和方向
    for i in range(5):
        goal = PoseStamped()
        goal.header.frame_id = "map"  # 使用地图坐标系
        goal.header.stamp = rospy.Time.now()

        # 设置位置，每次向左移动一定距离，假设每次移动1米
        goal.pose.position.x = -23 + position_x * (i + 1)
        goal.pose.position.y = 0.0
        goal.pose.position.z = 0.5

        # 设置方向，假设不需要旋转
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        # 发布目标
        goal_pub.publish(goal)
        rospy.loginfo("Goal {} sent: x={}, y={}".format(i+1, goal.pose.position.x, goal.pose.position.y))
        
        sleep_time = [7, 11.8, 10, 9.5, 0]
        # 等待一段时间
        rospy.sleep(sleep_time[i])

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    try:
        # 设置向左移动的距离，假设每次移动1米
        move_distance = 10
        send_nav_goal(move_distance)
    except rospy.ROSInterruptException:
        pass
