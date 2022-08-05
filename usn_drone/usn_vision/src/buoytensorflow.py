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
import time
import tensorflow as tf
import os
import cv2
import colorsys
import rospy
import math
from main.msg import VarColor
from sensor_msgs.msg import Image
import rospy
import sys
from main.msg import VarColor
import ros_numpy
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix


publish_detection_image = True            # Variable to turn of the publishing of the detection image



def compass(msg):                         # Compass function.
    global DIR 
    DIR = msg.data

def gps(msg):                              # GPS-Subscriber-Function 
    global GPS_Lat, GPS_Long
    GPS_Lat = msg.latitude
    GPS_Long = msg.longitude

def calculateGeographicalPositionFromRangeBearing(latitude1, longitude1, alpha1To2, s ) :
        """
        taken from https://github.com/guardiangeomatics/pyall/blob/master/geodetic.py

        Returns the lat and long of projected point and reverse azimuth
        given a reference point and a distance and azimuth to project.
        lats, longs and azimuths are passed in decimal degrees
        Returns ( latitude2,  longitude2,  alpha2To1 ) as a tuple 
        """
        f = 1.0 / 298.257223563     # WGS84
        a = 6378137.0           # metres

        piD4 = math.atan( 1.0 )
        two_pi = piD4 * 8.0

        latitude1    = latitude1    * piD4 / 45.0
        longitude1 = longitude1 * piD4 / 45.0
        alpha1To2 = alpha1To2 * piD4 / 45.0
        if ( alpha1To2 < 0.0 ) : 
                alpha1To2 = alpha1To2 + two_pi
        if ( alpha1To2 > two_pi ) : 
                alpha1To2 = alpha1To2 - two_pi

        b = a * (1.0 - f)

        TanU1 = (1-f) * math.tan(latitude1)
        U1 = math.atan( TanU1 )
        sigma1 = math.atan2( TanU1, math.cos(alpha1To2) )
        Sinalpha = math.cos(U1) * math.sin(alpha1To2)
        cosalpha_sq = 1.0 - Sinalpha * Sinalpha

        u2 = cosalpha_sq * (a * a - b * b ) / (b * b)
        A = 1.0 + (u2 / 16384) * (4096 + u2 * (-768 + u2 * \
                (320 - 175 * u2) ) )
        B = (u2 / 1024) * (256 + u2 * (-128 + u2 * (74 - 47 * u2) ) )

        # Starting with the approximation
        sigma = (s / (b * A))

        last_sigma = 2.0 * sigma + 2.0  # something impossible

        # Iterate the following three equations 
        #  until there is no significant change in sigma 

        # two_sigma_m , delta_sigma
        while ( abs( (last_sigma - sigma) / sigma) > 1.0e-9 ) :
                two_sigma_m = 2 * sigma1 + sigma

                delta_sigma = B * math.sin(sigma) * ( math.cos(two_sigma_m) \
                        + (B/4) * (math.cos(sigma) * \
                        (-1 + 2 * math.pow( math.cos(two_sigma_m), 2 ) -  \
                        (B/6) * math.cos(two_sigma_m) * \
                        (-3 + 4 * math.pow(math.sin(sigma), 2 )) *  \
                        (-3 + 4 * math.pow( math.cos (two_sigma_m), 2 ))))) \

                last_sigma = sigma
                sigma = (s / (b * A)) + delta_sigma

        latitude2 = math.atan2 ( (math.sin(U1) * math.cos(sigma) + math.cos(U1) * math.sin(sigma) * math.cos(alpha1To2) ), \
                ((1-f) * math.sqrt( math.pow(Sinalpha, 2) +  \
                pow(math.sin(U1) * math.sin(sigma) - math.cos(U1) * math.cos(sigma) * math.cos(alpha1To2), 2))))

        lembda = math.atan2( (math.sin(sigma) * math.sin(alpha1To2 )), (math.cos(U1) * math.cos(sigma) -  \
                math.sin(U1) *  math.sin(sigma) * math.cos(alpha1To2)))

        C = (f/16) * cosalpha_sq * (4 + f * (4 - 3 * cosalpha_sq ))
        omega = lembda - (1-C) * f * Sinalpha *  \
                (sigma + C * math.sin(sigma) * (math.cos(two_sigma_m) + \
                C * math.cos(sigma) * (-1 + 2 * math.pow(math.cos(two_sigma_m),2) )))

        longitude2 = longitude1 + omega

        alpha21 = math.atan2 ( Sinalpha, (-math.sin(U1) * math.sin(sigma) +  \
                math.cos(U1) * math.cos(sigma) * math.cos(alpha1To2)))

        alpha21 = alpha21 + two_pi / 2.0
        if ( alpha21 < 0.0 ) :
                alpha21 = alpha21 + two_pi
        if ( alpha21 > two_pi ) :
                alpha21 = alpha21 - two_pi

        latitude2       = latitude2       * 45.0 / piD4
        longitude2    = longitude2    * 45.0 / piD4
        alpha21    = alpha21    * 45.0 / piD4

        return longitude2, latitude2, alpha21

def rgb_to_hsv(r,g,b):
#This function takes the rgb values from the center point of detection
#and returns it in text to the buoy_publisher function
  r             = float(r/255.0)                                                                    #Dividing by 255.0 and converting to float for the colorsys.rgb_to_hsv function
  g             = float(g/255.0)                                                                    #Dividing by 255.0 and converting to float for the colorsys.rgb_to_hsv function
  b             = float(b/255.0)                                                                    #Dividing by 255.0 and converting to float for the colorsys.rgb_to_hsv function
  hsv           = colorsys.rgb_to_hsv(r,g,b)                                                        #Converting the rgb values for the center point of the detected buoy to hsv values
  Hue           = hsv[0]*360                                                                        #Converting the hue value to degrees so its easier to interpret for us humans
  Saturation    = hsv[1]                                                                            #Saturation if needed
  Value         = hsv[2]                                                                            #Value if needed

  if ((Hue<=35 and Hue > 0)  or Hue > 270):                                                                          # If the hue is within the red "spectre" return red
    return "red"
  if (Hue<75 and Hue > 35):                                                                         # If the hue is within the yellow "spectre" return yellow
    return "yellow"
  if (Hue<270 and Hue >= 75):                                                                        # If the hue is within the green "spectre" return green
    return "green"



def centerpointcolor(boxcoordinates,image):
    #This function returns the color of the detected buoy as well ass the coordinates.
    global height, width, centerx, centery
    left            = boxcoordinates[1]*width                                                        #Left pixel in the bounding box
    right           = boxcoordinates[3]*width                                                        #Right pixel in the bounding box
    top             = boxcoordinates[0]*height                                                       #Top pixel in the bounding box
    bottom          = boxcoordinates[2]*height                                                       #Bottom pixel in the bounding box
    centerwidth     = (right-left)/2.0
    centerx         = int(centerwidth+left)
    centerheight    = (bottom-top)/2.0
    centery         = int(centerheight+top)
    #Average RGB color of 5 points
    rgb             = image[centery+int(centerheight*0.2)][centerx]
    rgb1            = image[centery-int(centerheight*0.2)][centerx]
    rgb2            = image[centery][centerx]
    rgb3            = image[centery][centerx+int(centerwidth*0.2)]
    rgb4            = image[centery][centerx-int(centerwidth*0.2)]
    R               = int(((int(rgb[2])+int(rgb1[2])+int(rgb2[2])+int(rgb3[2])+int(rgb4[2]))/5))
    G               = int(((int(rgb[1])+int(rgb1[1])+int(rgb2[1])+int(rgb3[1])+int(rgb4[1]))/5))
    B               = int(((int(rgb[0])+int(rgb1[0])+int(rgb2[0])+int(rgb3[0])+int(rgb4[0]))/5))
    color           = rgb_to_hsv(R,G,B)
    return (color,int(left),int(right),int(top),int(bottom),centerx,centery)

def depth_zed2i(data):
#This function is a callback function for the depth subscriber.
#It updates the depth image and makes i public for other functions to use 
  global cv_depth
  cv_depth = np.frombuffer(data.data,dtype=np.float32).reshape(data.height,data.width,-1)
 

def zed_camera(msg):
#This funtion takes raw image data drom the zed2i camera and detects buoys with the
#tensorflow model.
    global cv_depth, depth, detections, height, width, centerx,centery,DIR,GPS_Lat,GPS_Long,Earth_Radius,f
    encoding        = msg.encoding
    image_np        = np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.width,-1)
    input_tensor    = tf.convert_to_tensor(image_np, dtype=tf.float32)
    input_tensor    = input_tensor[tf.newaxis, ...]
    input_tensor    = input_tensor[:, :, :, :3]
    detections      = detect_fn(input_tensor)
    num_detections  = int(detections.pop('num_detections'))
    detections      = {key: value[0, :num_detections].numpy()
                   for key, value in detections.items()}

    detections['num_detections'] = num_detections
    detections['detection_classes'] = detections['detection_classes'].astype(np.int64)                             # detection_classes should be ints.
    boxes               = detections['detection_boxes']
    max_boxes_to_draw   = boxes.shape[0]
    scores              = detections['detection_scores']
    min_score_thresh    =.95

    if(publish_detection_image==True):
        rgbimg = ros_numpy.numpify(msg)
    
    teller=0  
    Green                    =  False                                                                                    #Variable to keep track of what buoyes are visible
    Yellow                   =  False                                                                                    #Variable to keep track of what buoyes are visible
    Red                      =  False                                                                                    #Variable to keep track of what buoyes are visible
    bearing = 0.0
    
    for i in range(min(max_boxes_to_draw, boxes.shape[0])):                                                              #Loop through the detections and extract the needed information
        if scores is None or scores[i] > min_score_thresh:
            colormsg                 =  VarColor()                                                                       #Creates a message of type VarColor
            
            colormsg.visible         =  True
            colormsg.rad             =  25.0  
            iteminfo                 =  centerpointcolor(boxes[i],image_np)
            colormsg.x               =  iteminfo[5]                                                                          #Fill the message with information
            colormsg.y               =  iteminfo[6]                                                                          #Fill the message with information
            topleft                  =  (int(iteminfo[1]),int(iteminfo[3]))
            bottomright              =  (int(iteminfo[2]),int(iteminfo[4]))
            depth                    =  float(cv_depth[iteminfo[6],iteminfo[5]]*100.0)
            n = 0 
            
            if(math.isnan(depth) or math.isinf(depth)):                                                                 #If the depth is not a number or infinite, check nearby pixels for depth data
                while((math.isnan(depth) and n < 10) or (math.isinf(depth) and n < 10)):
                    n = n + 1 
                    depth = float(cv_depth[iteminfo[6]+n,iteminfo[5]+n]*100.0)
                    if(math.isnan(depth) or math.isinf(depth)):
                       depth = float(cv_depth[iteminfo[6]-n,iteminfo[5]-n]*100.0) 
                
            if(math.isnan(depth) or math.isinf(depth)):                                                                 #If the algorithm above did not find any real values, set the detection as not visible
                depth = 0 
                colormsg.visible = False
            colormsg.length          =  depth + 0.55/2
            #print(colormsg.length)
            ang_x                    =  round(np.degrees(math.atan(((iteminfo[5]-(1920/2))*2*10**-4)/focallength)),6)             #Calculating the angle in x direction
            colormsg.angle           =  ang_x
            ang_y                    =  round((np.degrees(math.atan(((iteminfo[6]-(1080/2))*2*10**-4)/focallength)))*-1,6)       #Calculating the angle in y direction
            colormsg.xc              =  (depth*(math.sin(np.radians(ang_x))))+6                                             #Distance from camera in x direction
            colormsg.yc              =  depth*(math.sin(np.radians(ang_y)))                                             #Distance from camera in y direction
            colormsg.z               =  depth*(math.cos(np.radians(abs(ang_x))))                                        #Distance from camera in z direction                                                                        #Radius of the buoy
            Length_m                 =  round((depth/100) +0.55/2,3)                                                              #Length in meters
            print("detected:",iteminfo[0],"Score:",round(scores[i]*100,2),"%")

            Tot_Angle                =  round(ang_x+DIR,3)                              #Total angle used for vinc function

            if(depth!=0.0):
                LONGITUDE,LATITUDE, bearing       =  calculateGeographicalPositionFromRangeBearing(GPS_Lat, GPS_Long, Tot_Angle, Length_m)
                colormsg.lat             =  LATITUDE
                colormsg.lon             =  LONGITUDE
            if (iteminfo[0]=="green" or iteminfo[0]=="yellow" or iteminfo[0]=="red"):
              color_publisher        =  rospy.Publisher("/Tracking/"+iteminfo[0], VarColor , queue_size = 1)            #Publisher to publish the detection data
              color_publisher.publish(colormsg)
            teller=teller+1
            if(publish_detection_image==True):
                cv2.putText(rgbimg, str("Distance: %.2f cm" %depth), (iteminfo[1]+70,iteminfo[3]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 205, 50), 2, cv2.LINE_AA)
                cv2.putText(rgbimg, iteminfo[0], (iteminfo[1],iteminfo[3]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 205, 50), 2, cv2.LINE_AA)
                cv2.rectangle(rgbimg, topleft, bottomright,(50, 205, 50), 4)
            if (iteminfo[0] == "red"):  
              Red = True
            if (iteminfo[0] == "yellow"):
              Yellow = True
            if (iteminfo[0] == "green"):
              Green = True   


    
    #These if statements will "undetect" the bouys. 
    if (Green == False or teller==0):
      color_publisher = rospy.Publisher("/Tracking/green", VarColor , queue_size = 1)
      msg = VarColor()
      msg.visible = False
      color_publisher.publish(msg)
    if (Red == False or teller==0):
      color_publisher = rospy.Publisher("/Tracking/red", VarColor , queue_size = 1)
      msg = VarColor()
      msg.visible = False
      color_publisher.publish(msg)
    if (Yellow == False or teller==0):
      color_publisher = rospy.Publisher("/Tracking/yellow", VarColor , queue_size = 1)
      msg = VarColor()
      msg.visible = False
      color_publisher.publish(msg)
    
    #Only publishes the detection image if wanted/needed.(to save cpu and memory)
    if(publish_detection_image==True):
        
        drawnimage = ros_numpy.msgify(Image, rgbimg,encoding='bgr8')
        img_detections.publish(drawnimage)


# LOADING MODEL
gpus = tf.config.experimental.list_physical_devices('GPU')
for gpu in gpus:
    tf.config.experimental.set_memory_growth(gpu, True)
# Load saved tensorflow model and build the detection function
#detect_fn = tf.saved_model.load('/home/autodrone/TF-TRT/Models/TensorflowModels/50000steps/converted') #Directory of the converted model.
detect_fn = tf.saved_model.load('/home/ros/TF-TRT/Models/TensorflowModels/45000steps/saved_model') #Directory of the converted model

#Subscribers and publishers
zed2i               = rospy.Subscriber('/camera/color/image_raw', Image, zed_camera)
zed2i_depth         = rospy.Subscriber('camera/depth/image_raw',Image, depth_zed2i)
if(publish_detection_image==True):
    img_detections  = rospy.Publisher('buoy_image',Image,queue_size=1)
Sub_Compass         = rospy.Subscriber("/navigation/heading", Float64, compass)
Sub_GPS             = rospy.Subscriber("/usn_drone/sensors/gps/gps/fix", NavSatFix, gps)

GPS_Lat      = 59.36933000                                                         #Global variable for GPS lat
GPS_Long     = 10.44036000                                                         #Global variable for GPS long
detections   = 0                                                                   #Detection array from the model
cv_depth     = 0                                                                   #Depth array from the zed camera
width        = 1920                                                                #Resolution of the zed image
height       = 1080                                                               #Resolution of the zed image
centerx      = 0                                                                   #Centerpoint of detection
centery      = 0                                                                   #Centerpoint of detection
focallength  = 0.212                                                               #Zed2i focal length
DIR          = 0                                                                   #Heading variable



def main(args):
    
  try:
    rospy.init_node('buoy_detections', anonymous = False)
    rospy.loginfo("Buoy detection node started")
    rospy.spin()
    

  except KeyboardInterrupt:
          print("Shutting down")
          cv2.destroyAllWindows()
      
if __name__ == '__main__':
  main(sys.argv)
  
    
