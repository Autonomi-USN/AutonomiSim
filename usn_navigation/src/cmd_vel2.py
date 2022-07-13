#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
#from kingfisher_msgs.msg import Drive
from std_msgs.msg import Float32
import numpy as np



class Node():
    def __init__(self,linear_scaling,angular_scaling):
        self.linear_scaling = linear_scaling
        self.angular_scaling = angular_scaling

        self.cmd_sub = rospy.Subscriber('/cmd_vel', Twist, self.callback, queue_size=10)
        self.right_pub = rospy.Publisher('/usn_drone/thrusters/left_thrust_cmd', Float32, queue_size=10)
        self.left_pub = rospy.Publisher('/usn_drone/thrusters/right_thrust_cmd', Float32, queue_size=10)

        self.driveMsgLeft = Float32()
        self.driveMsgRight = Float32()
        
    def callback(self,data):
        linfac = self.linear_scaling
        angfac = self.angular_scaling
        
        self.driveMsgRight.data = linfac*data.linear.x - angfac*data.angular.z
        self.driveMsgLeft.data = linfac*data.linear.x + angfac*data.angular.z

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
