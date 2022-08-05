#!/usr/bin/env python3
import rospy

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelState


def odom_cb_boat(data):
    global path_boat
    path_boat.header.stamp = rospy.Time()
    path_boat.header.frame_id = "usn_drone/odom"
    
    pose = PoseStamped()
    pose.header.stamp = rospy.Time()
    pose.header.frame_id = "usn_drone/odom"
    pose.pose = data.pose
    
    path_boat.poses.append(pose)
    path_pub.publish(path_boat)
    rospy.Rate(50).sleep()


rospy.init_node('path_boat_node')
path_boat = Path()
odom_sub_boat = rospy.Subscriber('/gazebo/set_model_state/', ModelState, odom_cb_boat)
path_pub = rospy.Publisher('/usn_drone/path_boat', Path, queue_size=10)


if __name__ == '__main__':
    rospy.spin()