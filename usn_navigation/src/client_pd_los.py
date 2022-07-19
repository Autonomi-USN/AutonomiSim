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


pose = Pose()
pose.position.x, pose.position.y = 0, 0
goal.poses.poses.append(pose)



pose = Pose()
pose.position.x, pose.position.y = 5, 20
goal.poses.poses.append(pose)



pose = Pose()
pose.position.x, pose.position.y = 10, 0
goal.poses.poses.append(pose)



pose = Pose()
pose.position.x, pose.position.y = 15, 20
goal.poses.poses.append(pose)



pose = Pose()
pose.position.x, pose.position.y = 20, 0
goal.poses.poses.append(pose)

pose = Pose()
pose.position.x, pose.position.y = 25, 20
goal.poses.poses.append(pose)


pose = Pose()
pose.position.x, pose.position.y = 30, 0
goal.poses.poses.append(pose)

pose = Pose()
pose.position.x, pose.position.y = 35, 20
goal.poses.poses.append(pose)

pose = Pose()
pose.position.x, pose.position.y = 5, 20
goal.poses.poses.append(pose)

pose = Pose()
pose.position.x, pose.position.y = 30, 0
goal.poses.poses.append(pose)

pose = Pose()
pose.position.x, pose.position.y = 10, 10
goal.poses.poses.append(pose)


client.send_goal(goal)
client.wait_for_result()
result = client.get_result()
print(result)