#!/usr/bin/env python
import math
# import roslib
import rospy
import math
import numpy as np
from main.msg import VarColor
from geometry_msgs.msg import Vector3
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class BouyTf:
    def __init__(self, Color):
        rospy.loginfo("node started")
        self.color = Color
        self.br = tf.TransformBroadcaster()
        self.subColor = rospy.Subscriber('/Tracking/' + str(self.color), VarColor, self.detection_callback, queue_size=10)
        self.marked_x = [0]
        self.marked_y = [0]
        self.listener = tf.TransformListener()
        self.xcam, self.ycam, self.zcam = 0, 0, 0
        self.tf_index = 0
        self.discard = 0
        self.distance_tolerance = 0.15
        self.possible_bouys_x = []
        self.possible_bouys_y = []
        self.possible_bouys_recorded = 0
        self.matches_x = []
        self.matches_y = []
        self.average_x, self.average_y = 0, 0
        self.tolerance_match = 1


    def point_already_marked(self, x, y, z):

        trans = [x, y, z]
        for point in range(len(self.marked_x)):         #checks if the point is within the range of a previously marked point
            if ((x < (self.marked_x[point] + self.distance_tolerance) and \
                    x > (self.marked_x[point] - self.distance_tolerance)) and \
                   (y < (self.marked_y[point] + self.distance_tolerance) and \
                    y > (self.marked_y[point] - self.distance_tolerance))):
                    return True

    def get_matches(self, x, y, x_list, y_list, tolerance): #checks for matches of x, y in the lists
        matches_x = []
        matches_y = []
        for i in range(len(x_list)):
            if ((x > x_list[i] - tolerance and x < x_list[i] + tolerance) and \
                        (y > y_list[i] - tolerance and y < y_list[i] + tolerance)):
                        matches_x.append(x)
                        matches_y.append(y)
        return matches_x, matches_y

    def get_global_position(self, x, y, z):       #function that transforms from (x, y) in camera frame to (x, y) in global grame (odom)
        (trans, rot) = self.listener.lookupTransform('/odom','/zed2i_left_camera_frame', rospy.Time())
        (roll, pitch, yaw) = euler_from_quaternion(rot)
        boat_target_global = (x*math.cos(yaw)-y*math.sin(yaw), x*math.sin(yaw)+ y*math.cos(yaw))    #compensates for any rotation of the boat
        return (trans[0] + boat_target_global[0], trans[1] + boat_target_global[1], 0)
    
    def publish_transform(self, x, y):
        self.br.sendTransform((x, y, 0),
                                (0, 0, 0, 1),
                                rospy.Time(),
                                str(self.color) +"bouy" + str(self.tf_index),
                                "odom")
        rospy.loginfo("published transform: " + self.color + str(self.tf_index))
        self.tf_index += 1 #increment the transform namespace
        self.marked_x.append(x) #add the point to the list of marked points
        self.marked_y.append(y)

    def detection_callback(self, msg):    #main function, runs whenever the camera detects a bouy
        if msg.visible:
            if self.possible_bouys_recorded < 30: #gathers points and appends them to a list of possible bouys
                x, y, z = self.get_global_position(msg.z/100, -msg.xc/100, msg.yc/100)
                if not self.point_already_marked(x, y, z):
                    rospy.loginfo("new detection recorded")
                    self.possible_bouys_x.append(x)
                    self.possible_bouys_y.append(y) 
                    self.possible_bouys_recorded += 1
            else:
                for i in range(30):         
                    x, y = self.possible_bouys_x[i], self.possible_bouys_y[i] 
                    matches_x, matches_y = self.get_matches(x, y, self.possible_bouys_x, self.possible_bouys_y, self.tolerance_match)
                    print(len(matches_x))
                    if len(matches_x) > 10:
                        rospy.loginfo("publishing tf")
                        self.publish_transform(np.average(matches_x), np.average(matches_y))
                        break
                    rospy.loginfo("not enough matches found")

                self.possible_bouys_x, self.possible_bouys_y = [], []  #Clearing the lists and zeroing the index for the next detection list
                self.possible_bouys_recorded = 0


if __name__ == '__main__':
    rospy.init_node('bouy_tf_broadcaster')
    GreenBouyTf = BouyTf("green")
    RedBouyTf = BouyTf("red")
    rospy.spin()