#!/usr/bin/env python3
import matplotlib.pyplot as plt
from gazebo_msgs.msg import ModelState
import matplotlib.pyplot as plt
import numpy as np
import rospy
from nav_msgs.msg import Odometry
import time
import csv


class ROSNode:
    def __init__(self):
        self.seadrone_data = []
        self.boat_data = []
        self.rate = rospy.Rate(10)
        rospy.wait_for_message('/gazebo/set_model_state', ModelState)
        self.start_time = time.time()
        self.seadrone_sub = rospy.Subscriber('/usn_drone/robot_localization/odometry/filtered', Odometry, self.seadrone_cb, queue_size = 1)
        self.boat_sub = rospy.Subscriber('/gazebo/set_model_state', ModelState, self.boat_cb, queue_size = 1)

    def boat_cb(self, msg):
        position = msg.pose.position
        self.boat_data.append([position.x, position.y, time.time() - self.start_time])
        self.rate.sleep()


    def seadrone_cb(self, msg):
        
        position = msg.pose.pose.position
        x, y = position.x, position.y
        self.seadrone_data.append([x, y, time.time() - self.start_time])
        self.rate.sleep()


def write_to_file(path, data):
    file = open(path, 'w')
    writer = csv.writer(file)
    writer.writerows(data)
    file.close()
    print('wrote to csv')





if __name__ == "__main__":
    rospy.init_node('data_gather')
    node = ROSNode()
    #path_boat = '/home/ros/vrx_ws/src/vrx/usn_colreg/src/data/LOS_0_0/boat.csv'
    path_seadrone = '/home/ros/vrx_ws/src/vrx/usn_drone/usn_colreg/src/data/heavy_waters/MPC_no_traj/seadrone.csv'

    while not rospy.is_shutdown():
        rospy.spin()

    #write_to_file(path_boat, node.boat_data)
    write_to_file(path_seadrone, node.seadrone_data)

    
    

    