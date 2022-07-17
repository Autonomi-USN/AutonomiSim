#!/usr/bin/env python3
import rospy
import numpy as np
from math import pi
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import actionlib
from usn_navigation.msg import LOSAction, LOSResult, LOSFeedback, LOSGoal
from geometry_msgs.msg import Pose
import time

rospy.init_node('client_pd_los')
client = actionlib.SimpleActionClient('LOS_PD_action', LOSAction)
client.wait_for_server()

goal = LOSGoal()    

pose = Pose()



pose.position.x = 20
pose.position.y = 20
goal.poses.poses.append(pose)
pose = Pose()



pose.position.x = -20
pose.position.y = 35
goal.poses.poses.append(pose)
pose = Pose()



pose.position.x = 0
pose.position.y = 0
goal.poses.poses.append(pose)
pose = Pose()



pose.position.x = -5
pose.position.y = 10
goal.poses.poses.append(pose)
pose = Pose()

pose.position.x = 20
pose.position.y = 10
goal.poses.poses.append(pose)
pose = Pose()


pose.position.x = 0
pose.position.y = 0
goal.poses.poses.append(pose)
pose = Pose()

client.send_goal(goal)
time.sleep(2)
#client.cancel_goal()
client.wait_for_result()
result = client.get_result()

print(result)
