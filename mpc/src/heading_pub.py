#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def imu_to_euler (msg):
    pub_msg = Float64()
    quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(quat)
    pub_msg.data = yaw
    pub.publish(pub_msg)



rospy.init_node('imu_quat_to_euler')
sub = rospy.Subscriber('/usn_drone/sensors/imu/imu/data', Imu, imu_to_euler)
pub = rospy.Publisher('/navigation/heading', Float64, queue_size = 10)

rospy.spin()
