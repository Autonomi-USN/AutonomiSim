#!/usr/bin/env python3
import rospy

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelState

def odom_cb_seadrone(data):
    global path_seadrone
    path_seadrone.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose.pose
    pose.pose.position.z = 0.2
    path_seadrone.poses.append(pose)
    path_pub.publish(path_seadrone)
    rospy.Rate(50).sleep()


rospy.init_node('path_node')
path_seadrone = Path()
odom_sub_seadrone = rospy.Subscriber('/usn_drone/robot_localization/odometry/filtered', Odometry, odom_cb_seadrone)
path_pub = rospy.Publisher('/usn_drone/path_seadrone', Path, queue_size=10)

if __name__ == '__main__':
    rospy.spin()