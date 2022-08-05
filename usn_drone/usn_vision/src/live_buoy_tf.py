#!/usr/bin/env python3
#-----------------------------------------------------------------------#
#       Live publication of bouy transforms                             #
#       Produced by: Edvart Gruer Bjerke and Torben Falleth Olsen       #
#-----------------------------------------------------------------------#
import math
import roslib
import rospy
import math
import numpy as np
from main.msg import VarColor
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class BouyTf:
    def __init__(self, Color):
        self.color = Color
        self.br = tf.TransformBroadcaster()
        self.subColor = rospy.Subscriber('/Tracking/' + str(self.color), VarColor, self.camera_detection_callback, queue_size=10)
        self.subRP = rospy.Subscriber('/scan', LaserScan, self.camera_rplidar_merge, queue_size=10)
        self.marked_x = [0]
        self.marked_y = [0]
        self.detected = False
        self.listener = tf.TransformListener()
        self.xcam, self.ycam, self.zcam = 0, 0, 0
        self.x_cam,self.y_cam = 0, 0
        self.tf_index = 0
        self.lidar_dist = 0.0
        self.lidar_x = 0.0
        self.lidar_y = 0.0
        self.lidar_detect = 0
        self.ang = 0.0
        self.discard = 0  
        self.RP_dist_x = 0
        self.RP_dist_y = 0 
        self.x, self.y = 0, 0
        self.distance_tolerance = 0.5
        self.possible_bouys_x = []
        self.possible_bouys_y = []
        self.possible_bouys_recorded = 0
        self.tolerance_match = 0.5
        

#------ Checking to see if bouy is already marked (Function) ------#
    def point_already_marked(self, x, y, z):

        trans = [x, y, z]
        for point in range(len(self.marked_x)):                                     # Checks if the point is within the range of a previously marked point
            if ((x < (self.marked_x[point] + self.distance_tolerance) and \
                    x > (self.marked_x[point] - self.distance_tolerance)) and \
                   (y < (self.marked_y[point] + self.distance_tolerance) and \
                    y > (self.marked_y[point] - self.distance_tolerance))):
                    return True

                    
#------ Publishing the bouy frame (Function) ------#                 
    def publish_transform(self, x, y):                                               # Uses the x and y values to set a frame
        self.br.sendTransform((x, y, 0),
                                (0, 0, 0, 1),
                                rospy.Time(),
                                str(self.color) +"bouy" + str(self.tf_index),
                                "usn_drone/odom")
        rospy.loginfo("Published transform: " + self.color + str(self.tf_index))
        self.tf_index += 1                                                           # Increment the transform namespace
        self.marked_x.append(x)                                                      # Add the point to the list of marked points
        self.marked_y.append(y)
   
        
#------ Looks for matches in value within a (x,y) list (Function) ------#
    def get_matches(self, x, y, x_list, y_list, tolerance):                          # Checks for matches of x, y in the lists
        matches_x = []
        matches_y = []
        for i in range(len(x_list)):
            if ((x > x_list[i] - tolerance and x < x_list[i] + tolerance) and \
                        (y > y_list[i] - tolerance and y < y_list[i] + tolerance)):  # Then adds the matches to a new list
                        matches_x.append(x)
                        matches_y.append(y)
        return matches_x, matches_y

             
#------ Calculates the bouy frame position from camera data (Callback function) ------#
    def camera_detection_callback(self, msg):    
        if msg.visible:                                                              # Checks to see is any objects are visible
            self.detected = msg.visible
            self.xcam, self.ycam = msg.z/100, -msg.xc/100
            self.cam_dist = msg.length/100
            self.cam_ang = msg.angle
            if self.possible_bouys_recorded < 30:                                    # Gathers points and appends them to a list of possible bouys
                self.x, self.y, self.z = self.get_odom_position(msg.z/100, -msg.xc/100, msg.yc/100,'usn_drone/front_camera_link',0)
                if not self.point_already_marked(self.x, self.y, self.z):                           # Checks that the point is not already marked 
                    #rospy.loginfo("new detection recorded")
                    self.possible_bouys_x.append(self.x)
                    self.possible_bouys_y.append(self.y)
                    self.possible_bouys_recorded += 1
            else:
                for i in range(30):         
                    x, y = self.possible_bouys_x[i], self.possible_bouys_y[i] 
                    matches_x, matches_y = self.get_matches(x, y, self.possible_bouys_x, self.possible_bouys_y, self.tolerance_match)
                    if len(matches_x) > 10:                                          # Filers out the values that dont match the position and puts the average value in a variable
                        self.x_cam,self.y_cam = np.average(matches_x), np.average(matches_y)
                        
                        break
                    rospy.loginfo("Not enough matches found")

                self.possible_bouys_x, self.possible_bouys_y = [], []                # Clearing the lists and zeroing the index for the next detection list
                self.possible_bouys_recorded = 0
                print('sending tf')
                print(self.x, self.y, self.z)
                self.br.sendTransform((self.x, self.y, 0),(0, 0, 0, 1),rospy.Time.now(),str(self.color) +"bouy_camera","usn_drone/odom")

        #else:
            #rospy.logerr("No object detected (Camera)") 
            
                   
#------ Calculates and publishes bouy frame from camera and rplidar data (Main function) ------#
    def camera_rplidar_merge(self,msg):

        self.ang = math.degrees(self.LidarAngleFromCamera(self.xcam,self.ycam))      # Getting the lidar to object angle from camera angle
        span = 10      # Degrees
        self.ang_start = int(3.56944*(self.ang))
        self.ang_from = int(3.56944*(self.ang-span))                                 # Defining the span area in degrees where the lidar are to search for an object.
        self.ang_too = int(3.56944*(self.ang+span))                                  # Since lidar has 1285 elements we devide (1285/360)*angle
        
        if self.detected:
            for i in range (self.ang_from,self.ang_too,+1):                          # Iteraring through the span area
              self.lidar_dist = msg.ranges[i]                                        # Extracting the distance of the certain angle  
              
              if self.lidar_dist == float('inf'):                                    # If value equals to inf then return empty
                self.lidar_dist = ""
              else:
                lr_x = self.lidar_dist*math.cos(math.radians(i/3.56944))             # Converting distance to x and y
                lr_y = self.lidar_dist*math.sin(math.radians(i/3.56944))
                self.lidar_x, self.lidar_y, z = self.get_odom_position(lr_x, lr_y, 0,'usn_drone/lidar_usn_drone_link',math.pi)

                if (self.lidar_x <= self.x_cam+self.distance_tolerance) and \
                    (self.lidar_x >= self.x_cam-self.distance_tolerance) and \
                    (self.lidar_y <= self.y_cam+self.distance_tolerance) and \
                    (self.lidar_y >= self.y_cam-self.distance_tolerance):               # Checks that the lidar object is close to the camera object
                    
                    self.RP_dist_x += self.lidar_x
                    self.RP_dist_y += self.lidar_y
                    self.lidar_detect+=1         
            
            if self.lidar_detect > 0:                                               # If the lidar has spotted an object within the conditions
                self.dist_x = self.RP_dist_x/self.lidar_detect                      # Calculating the average of the points assembled
                self.dist_y = self.RP_dist_y/self.lidar_detect
                print(self.x,self.y)
                self.br.sendTransform((self.x, self.y, 0),(0, 0, 0, 1),rospy.Time.now(),str(self.color) +"bouy_camera","odom")
                self.br.sendTransform((self.dist_x, self.dist_y, 0),(0, 0, 0, 1),rospy.Time.now(),str(self.color) +"bouy","odom")  
             

        self.RP_dist_x = 0
        self.RP_dist_y = 0
        self.lidar_detect = 0 
     
        
#------ Calculates the lidar angle from camera data (Function) ------#
    def LidarAngleFromCamera(self, cam_x, cam_y):
        cam_rp_x = 0.515                                                             # Distance between Camera and Rplidar in meter
        cam_rp_y = 0.075
        lidar_to_bouy_x = cam_x - cam_rp_x                                           # Distance from the lidar to the object in meter
        lidar_to_bouy_y = cam_y + cam_rp_y
        angle = math.atan(lidar_to_bouy_y/lidar_to_bouy_x)                           # Calculates the the angle from the lidar to the object
        return angle  
        
        
#------ Transforms from (x, y) in camera frame to (x, y) in odom frame (Function) ------#
    def get_odom_position(self, x, y, z,frame,theta): 
        (trans, rot) = self.listener.lookupTransform('usn_drone/odom',frame, rospy.Time())     # Listens to get the transformation and rotation between the two frames
        (roll, pitch, yaw) = euler_from_quaternion(rot)                              # Convert from quaternion to euler
        boat_target_global = (x*math.cos(yaw+theta)-y*math.sin(yaw+theta), x*math.sin(yaw+theta)+y*math.cos(yaw+theta))    # Compensates for any rotation of the boat
        
        return (trans[0] + boat_target_global[0], trans[1] + boat_target_global[1], trans[2] + z)


            
if __name__ == '__main__':
    rospy.init_node('bouy_live_tf_broadcaster')
    GreenBouyTf = BouyTf("green")
    RedBouyTf = BouyTf("red")
    rospy.spin()