#!/usr/bin/env python
import math
import roslib
import rospy
import math
import numpy as np
from main.msg import VarColor
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3, PoseStamped
from nav_msgs.msg import Odometry, Path

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class VisualizePath:

    def __init__(self):
        self.broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.wp_pub = rospy.Publisher('/waypoint/path', Path, queue_size=1)             # Publishing to Servo and thursters
        self.wp_vel = Path()
 #       self.pos_sub = rospy.Subscriber('/robot_localization/odometry/filtered', Odometry, self.position_callback, queue_size=10)
 #       self.wp = [[2.0,2.0],[2.0,-2.0],[-2.0,-2.0],[-2.0,2.0],[1.0,2.0]]
        self.wayp = [0.0, 0.0]
        self.segments = [0.0, 0.0]
        self.way = [0.0, 0.0]
        self.seg = [0.0, 0.0]
 #       self.position = [0.0,0.0]


#    def position_callback(self,msg):
#        x = msg.pose.pose.position.x
#        y = msg.pose.pose.position.y
#        self.position = [x,y]
#        if not self.wp_vel.poses:
#          self.publish_path_wp0(self.wp[0],self.position)
#        elif len(self.wp_vel.poses) <= len(self.wp)*10:
#          self.publish_path_next(self.wp)
#        self.wp_pub.publish(self.wp_vel)
        
    def set_start_and_stop(self,start_pos,stop_wp): 
        for i in range(len(stop_wp)):
            if not self.wp_vel.poses:
              self.publish_path_wp0(stop_wp[0],start_pos)
            elif len(self.wp_vel.poses) <= len(stop_wp)*10:
              self.publish_path_next(stop_wp)
            self.wp_pub.publish(self.wp_vel)        
        
    def publish_path_wp0(self,waypoint,position):
       self.wayp[0] = waypoint[0]-position[0]
       self.wayp[1] = waypoint[1]-position[1]
       for i in range (0,100):
         self.segments[0] = (self.wayp[0]/10)*i
         self.segments[1] = (self.wayp[1]/10)*i
         if np.linalg.norm(self.segments) <= np.linalg.norm(self.wayp):
           self.create_path_segments_wp0(self.segments,position)
         else:
           break

          
    def create_path_segments_wp0(self,p,pos):
       path = PoseStamped()
       self.wp_vel.header.frame_id = 'usn_drone/odom' 
       path.header.frame_id = 'usn_drone/odom'      
       path.pose.position.x = p[0]+pos[0]
       path.pose.position.y = p[1]+pos[1]
       self.wp_vel.poses.append(path)
       
    def publish_path_next(self,waypoint):
       print('hei')
       for j in range(1,len(waypoint)):
         self.way[0] = waypoint[j][0]-waypoint[j-1][0]
         self.way[1] = waypoint[j][1]-waypoint[j-1][1]
         for i in range (0,10):
           self.seg[0] = (self.way[0]/10)*i
           self.seg[1] = (self.way[1]/10)*i

           if np.linalg.norm(self.seg) <= np.linalg.norm(self.way):
             self.create_path_segments_next(self.seg,waypoint[j-1])
           else:
             break

          
    def create_path_segments_next(self,p,wp):
       path = PoseStamped()
       self.wp_vel.header.frame_id = 'usn_drone/odom' 
       path.header.frame_id = 'usn_drone/odom'      
       path.pose.position.x = p[0]+wp[0]
       path.pose.position.y = p[1]+wp[1]
       self.wp_vel.poses.append(path)
            
if __name__ == '__main__':
    rospy.init_node('visualize_path_node')
    VP = VisualizePath()
    #wp = [[2.0,-2.0],[-2.0,-2.0],[-2.0,2.0],[1.0,2.0]]
    #position = [2.0,2.0]
    # VP.set_start_and_stop(position,wp)
    rospy.spin()