#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class CmdVelNode:
    def __init__(self):
        rospy.init_node("cmd_vel_node")
        rospy.loginfo("Starting ROSNode as cmd_vel_node.")

        self.left_thrust_pub = rospy.Publisher('/usn_drone/thrusters/left_thrust_cmd', Float32, queue_size=10)
        self.right_thrust_pub = rospy.Publisher('/usn_drone/thrusters/right_thrust_cmd', Float32, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback,queue_size=10)
        self.wheel_distance = 0.5

        self.left_thrust_msg = Float32()
        self.right_thrust_msg = Float32()
        
    def cmd_vel_callback(self, msg):
        speed_wish_right = (msg.angular.z*self.wheel_distance)/2 + msg.linear.x
        speed_wish_left = msg.linear.x*2-speed_wish_right

        print(speed_wish_left, speed_wish_right)
        self.left_thrust_msg.data = speed_wish_left
        self.right_thrust_msg.data = speed_wish_right



        self.left_thrust_pub.publish(self.left_thrust_msg)
        self.right_thrust_pub.publish(self.right_thrust_msg)

        #print(self.left_thrust_msg.data, self.right_thrust_msg.data)


if __name__ == "__main__":
    cmd_vel_node = CmdVelNode()
    rospy.spin()