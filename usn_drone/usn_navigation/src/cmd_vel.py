#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
#from kingfisher_msgs.msg import Drive
from std_msgs.msg import Float32
import numpy as np
from nav_msgs.msg import Odometry


class Node():
    def __init__(self,linear_scaling,angular_scaling):
        self.linear_scaling = linear_scaling
        self.angular_scaling = angular_scaling

        self.cmd_sub = rospy.Subscriber('/cmd_vel', Twist, self.callback, queue_size=10)
        #self.odom_sub = rospy.Subscriber('/usn_drone/robot_localization/odometry/filtered', Odometry, self.odom_cb)
        self.right_pub = rospy.Publisher('/usn_drone/thrusters/left_thrust_cmd', Float32, queue_size=10)
        self.left_pub = rospy.Publisher('/usn_drone/thrusters/right_thrust_cmd', Float32, queue_size=10)

        self.driveMsgLeft = Float32()
        self.driveMsgRight = Float32()

        self.x = self.y = self.yaw = None #x position, y position, yaw angle
        self.yaw_rate = None #angular velocity
        self.linear_x = None #linear velocity, x (surge velocity), in the body frame


    # def odom_cb(self, msg):
    #     self.yaw_rate = msg.twist.twist.angular.z
    #     self.linear_x = msg.twist.twist.linear.x
        
    def callback(self,data):
        self.driveMsgRight.data = data.linear.x - data.angular.z
        self.driveMsgLeft.data = data.linear.x + data.angular.z

        self.left_pub.publish(self.driveMsgLeft)
        self.right_pub.publish(self.driveMsgRight)

if __name__ == '__main__':

    rospy.init_node('cmd_vel2thrust', anonymous=True)
    # Scaling from Twist.linear.x to (left+right)
    linear_scaling = rospy.get_param('~linear_scaling',1)
    # Scaling from Twist.angular.z to (right-left)
    angular_scaling = rospy.get_param('~angular_scaling', 1)

    node=Node(linear_scaling,angular_scaling)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
