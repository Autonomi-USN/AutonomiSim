#!/usr/bin/env python3

#########################################################################################################
#                                                                                                       #
#   This script detects buoys with the help of a tensorflow model. Draws the detections on a image      #
#   and publishes that image to /buoy_image. The different colored buoys gets differentiated by taking  #
#   the RGB value in five points inside the detection box. The distance to the buoyes is taken from     #
#   the ZED2i depth map and gets decomposed to x,y,z coordinated relative to the boat. Some code is     #
#   based on the former bachelors group code found in the color_tracker.py script.                      #
#   Produced by Henrik Wiik Roe, Endre Wold and Ola Indergord                                           #
#                                                                                                       #
#########################################################################################################

import numpy as np
import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings('ignore')   # Suppress Matplotlib warnings
import cv2

import rospy

from main.msg import VarColor
from sensor_msgs.msg import Image
import rospy
import sys
from main.msg import VarColor
import ros_numpy
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix

rospy.init_node('test_image')
img_detections  = rospy.Publisher('imagetest',Image,queue_size=1)
rate = rospy.Rate(10)

"load test image"
image = cv2.imread('/home/ros/vrx_ws/src/vrx/usn_vision/images/boats1.jpg')

"publish test image"
drawnimage = ros_numpy.msgify(Image, image,encoding='bgr8')

while not rospy.is_shutdown():
    img_detections.publish(drawnimage)
    rate.sleep()