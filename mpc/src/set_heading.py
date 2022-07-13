#!/usr/bin/env python3
from time import time

from sympy import euler
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import casadi as ca
import numpy as np
from casadi import sin, cos, pi
import matplotlib.pyplot as plt
from variables import *
import rospy
from sensor_msgs.msg import NavSatFix, Imu
from  geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32
import casadi as ca
import numpy as np
import queue
import rospy
import matplotlib.pyplot as plt
import pymap3d as pm
from casadi import sin, cos, pi
from variables import *
from std_msgs.msg import Float32
from time import time, ctime, sleep
from casadi import sin, cos, pi
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from tf.transformations import euler_from_quaternion
#from supporting_variables_GAZEBO import *
from geometry_msgs.msg import Pose
import casadi as ca
import numpy as np
import queue
import rospy
import matplotlib.pyplot as plt
import pymap3d as pm
from casadi import sin, cos, pi
from variables import *
from std_msgs.msg import Float32
from time import time, ctime
from casadi import sin, cos, pi
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from tf.transformations import euler_from_quaternion
from robot_localization.srv import SetPose, SetPoseRequest, SetPoseResponse
from nav_msgs.msg import Odometry

def get_object_line_angle(x, y):
    
    x_abs, y_abs = np.abs(x), np.abs(y)
    #theta_target = 0
    if (x > 0 and y > 0):
        theta_target = np.arctan(y_abs/x_abs)
    elif (x < 0 and y > 0):
        theta_target = np.pi/2+np.arctan(x_abs/y_abs)
    elif (x < 0 and y < 0):
        theta_target = -np.pi/2 - np.arctan(x_abs/y_abs)
    elif x > 0 and y < 0:
        theta_target = -np.arctan(y_abs/x_abs)
    elif x == 0 and y > 0:
        theta_target = np.pi/2
    elif x == 0 and y < 0:
        theta_target = -np.pi/2
    elif y == 0 and x > 0:
        theta_target = 0
    elif y == 0 and x < 0:
        theta_target = -np.pi

    rospy.loginfo('got target angle')
    return theta_target
        


def service_cb(request):
    rospy.loginfo('point received')
    left_pub = rospy.Publisher('/usn_drone/thrusters/left_thrust_cmd', Float32, queue_size=10)
    right_pub = rospy.Publisher('/usn_drone/thrusters/right_thrust_cmd', Float32, queue_size=10)
    left_pub_msg = Float32()
    right_pub_msg = Float32()


    orientation_target_rpy = request.pose.pose.pose.orientation
    theta_target = orientation_target_rpy.z
    x = request.pose.pose.pose.position.x
    y = request.pose.pose.pose.position.y
    
    theta_target = get_object_line_angle(x, y)

    odom_init = rospy.wait_for_message('/usn_drone/robot_localization/odometry/filtered', Odometry)
    orientation_init_qtr = odom_init.pose.pose.orientation
    (roll, pitch, theta_init) = euler_from_quaternion((orientation_init_qtr.x, orientation_init_qtr.y, orientation_init_qtr.z, orientation_init_qtr.w))
    theta_init = theta_init
    theta_err = theta_target - theta_init

    while np.abs(theta_err) > 0.1:
        rospy.loginfo('done setting heading')

        if theta_err < 0:
            left_pub_msg.data, right_pub_msg.data = 0.435, -1
        elif theta_err > 0:
            left_pub_msg.data, right_pub_msg.data = -1, 0.435

        left_pub.publish(left_pub_msg)
        right_pub.publish(right_pub_msg)

        odom_current = rospy.wait_for_message('/usn_drone/robot_localization/odometry/filtered', Odometry)
        orientation_current_qtr = odom_current.pose.pose.orientation
        
        (roll, pitch, theta_current) = euler_from_quaternion((orientation_current_qtr.x, orientation_current_qtr.y, orientation_current_qtr.z, orientation_current_qtr.w))
        theta_err = theta_target-theta_current
    return SetPoseResponse()

rospy.init_node('set_heading')
service = rospy.Service('set_heading', SetPose, service_cb)
rospy.spin()