#!/usr/bin/env python3
import rospy
from tf.transformations import euler_from_quaternion
import actionlib
from usn_navigation.msg import LOSAction, LOSGoal
from geometry_msgs.msg import Pose
import tf
import numpy as np

def get_position(target1_string: str):
    """returns the position of the target frame, relative to usn_drone/odom"""
    listener = tf.TransformListener()
    rospy.loginfo("waiting for transforms")
    listener.waitForTransform(target1_string, "usn_drone/odom", rospy.Time(), rospy.Duration(100))
    rospy.loginfo("got transforms")
    (target1_pos,rot) = listener.lookupTransform('/usn_drone/odom', target1_string, rospy.Time(0))
    return target1_pos

def generate_pose_msg(point: list):
    """takes a point and returns a pose message with x and y values filled in"""
    pose = Pose()
    pose.position.x, pose.position.y = point[0], point[1]
    return pose

def equation_of_normal(point1: list, point2: list):
    """returns a and b for the equation of the normal line (y = ax + b) from point1 to point2, which intersects
       at the average of the two points, also returns this 'average point'"""
    intersect = (point1[0]+point2[0])/2, (point1[1]+point2[1])/2 
    a = dy_dx = (point2[1]- point1[1])/ (point2[0] - point1[0])
    a = -1/a

    x_1 = intersect[0]
    y_1 = intersect[1]
    b = a*(-x_1) + y_1
    return a, b, intersect

def get_waypoints(x_0: float, slope: float, b: float, radius: float):
    """takes in a, b of y = ax + b, which represents the equation of a line. The function returns two points that are at
       some distance along the line, defined by 'radius'"""
    x_1 = x_0 + radius*np.sqrt(1+slope**2)
    x_2 = x_0 - radius*np.sqrt(1+slope**2)

    y_1 = a*x_1 + b
    y_2 = a*x_2 + b
    return [x_1, y_1], [x_2, y_2]

def order_points_by_distance(position: list, point1: list, point2: list):
    """takes in 3 points: a base position and two comparison points. The function returns the two points, ordered
       by their distance to the base position, closest point first"""
    
    distance_point1 = np.sqrt((position[1]-point1[1])**2 + (position[0] - point1[0])**2)          #np.linalg.norm([position, point1])
    distance_point2 = np.sqrt((position[1]-point2[1])**2 + (position[0] - point2[0])**2)          #np.linalg.norm([position, point2])

    if distance_point1 > distance_point2:
        print('point2 closest')
        return point2, point1
    else:
        print('point1 closest')
        return point1, point2



rospy.init_node('client_pd_los')
goal = LOSGoal()    
client = actionlib.SimpleActionClient('LOS_PD_action', LOSAction)
client.wait_for_server()

boat_pos = get_position('usn_drone/base_link')

goal.poses.poses.append(generate_pose_msg([boat_pos[0], boat_pos[1]]))
goal.poses.poses.append(generate_pose_msg([0, 0]))
goal.poses.poses.append(generate_pose_msg([5, 20]))
goal.poses.poses.append(generate_pose_msg([10, 0]))
goal.poses.poses.append(generate_pose_msg([15, 20]))
goal.poses.poses.append(generate_pose_msg([20, 0]))
goal.poses.poses.append(generate_pose_msg([25, 20]))
goal.poses.poses.append(generate_pose_msg([30, 0]))
goal.poses.poses.append(generate_pose_msg([35, 20]))





client.send_goal(goal)
client.wait_for_result()
result = client.get_result()
print(result)