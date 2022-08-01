#!/usr/bin/env python3
import rospy
import numpy as np
from math import pi
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import actionlib
from actionlib_msgs.msg import GoalID
from usn_navigation.msg import LOSAction, LOSResult, LOSFeedback
from gazebo_msgs.msg import ModelState
import math
#! action server for Line Of Sight Guidance between two points at a time, takes an array of points and iterates through them
#! The current Linear Thrust control is very primitive and should probably be looked at for improvement.
#! With current implementation there is a chance that the boat will surpass the target if the target is too close to the starting position.
#! Written by Edvart Bjerke, USN-Autonomi


class PDLineOfSight:
    # messages used to publish feedback/result
    _feedback = LOSFeedback
    _result = LOSResult

    def __init__(self):
        #PD Gains and Filter Constants 
        self.Kp = rospy.get_param('LOS_Kp', 10)
        self.Kd = rospy.get_param('LOS_Kd', 1)
        rate    = rospy.get_param('LOS_rate', 50)
        self.T = 0.02
        self.Tf = 0.02

        #constants for setting the lookahead distance
        self.delta_max = 5
        self.delta_min = 0.1
        #self.delta = 3
        self.gamma = 0.15
        self.goal_proximity = 1

        #keep track of previous time-tick error and derivative part
        self.error_prev = 0
        self.derivative_tminus1 = 0

        self.rate = rospy.Rate(rate)
        self.twist_msg = Twist()
        self.x, self.y = 0, 0
        self.psi = 0
        
        #spin up subscribers and start the Action Server
        rospy.Subscriber('/usn_drone/robot_localization/odometry/filtered', Odometry, self.odom_cb, queue_size = 1)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.pub_error = rospy.Publisher('/LOS_heading_error', Float32, queue_size = 10)
        self.foreign_boat_sub = rospy.Subscriber("/gazebo/set_model_state", ModelState, self.foreign_boat_cb)


    def calculate_y_ep(self): #calculates distance from point (x, y) to line from (x_k, y_k) to (x_k1, y_k1)
        p1=np.array([self.x_k, self.y_k])
        p2=np.array([self.x_k1, self.y_k1])
        p3=np.array([self.x, self.y])
        distance=np.cross(p2-p1,p3-p1)/np.linalg.norm(p2-p1)
        return distance

    def PD_controller(self): #PD controller for differential thrust (heading of the boat)
        e_t, e_tminus1 = self.error, self.error_prev
        Kp, Kd = self.Kp, self.Kd
        
        derivative = Kd*(e_t-e_tminus1)
        proportional = Kp*e_t
        
        #Standard PD control
        #delta_n = proportional + derivative
        
        #PD Control with Low Pass Filter
        delta_n = proportional + (self.Tf/(self.Tf + self.T)) * self.derivative_tminus1 + (Kd/(self.Tf+self.T)) * (e_t - e_tminus1)
        self.derivative_tminus1 = derivative
        return delta_n

    def calculate_pi_p(self): #calculates theta_p, path-tangential angle
        return -(np.arctan2((self.x_k1 - self.x_k),(self.y_k1-self.y_k)))

    def get_lookahead(self): #calculates the line-of-sigh lookahead distance
        lookahead = (self.delta_max-self.delta_min)*np.exp(-self.gamma*np.abs(self.y_ep))
        return lookahead

    def distance_from_goal(self): #gets the absolute distance from current position to current target
        return np.sqrt((self.x_k1-self.x)**2+(self.y_k1-self.y)**2)

    def get_desired_heading(self): #calculates the desired heading for following the line of sight
        self.x_d = self.pi_p - np.arctan(self.y_ep/self.delta) #sets the desired heading
        if self.x_d > pi:
            self.x_d = self.x_d - 2*pi
        elif self.x_d < -pi:
            self.x_d = self.x_d + 2*pi
    
        self.error = self.psi-self.x_d
        if self.error >= pi:
            self.error = self.error -2*pi
        elif self.error <= -pi:
            self.error = self.error +2*pi
        
    def odom_cb(self, msg): #keeps track of current USV heading and position (x, y)
        quat = msg.pose.pose.orientation
        position = msg.pose.pose.position
        self.psi = euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))[2] - pi/2
        if self.psi < (-pi):
            self.psi = self.psi+2*pi
        self.x, self.y = position.x, position.y
        self.rate.sleep()

    "isRight and calculate_bearing are imported functions"
    def isright(self, sourceAngle, otherAngle):
        # sourceAngle and otherAngle should be in the range -180 to 180
        sourceAngle = sourceAngle - 180
        otherAngle = otherAngle - 180
        difference = otherAngle - sourceAngle
        if (difference < -180.0):
            difference += 360.0
        if (difference > 180.0):
            difference -= 360.0
        if (difference > 0.0):
            return 1
        if (difference < 0.0):
            return
        return

    def calculate_bearing(self, pointA, pointB):
        diff_y = (pointB[1]-pointA[1])
        diff_x = (pointB[0]-pointA[0])
        bearing = math.atan2(diff_x, diff_y)
        initial_bearing = math.degrees(bearing)
        final_bearing = (initial_bearing + 360) % 360
        return final_bearing


    def foreign_boat_cb(self, data):
        self.foreign_boat_pos = [data.pose.position.x, data.pose.position.y]

    def execute(self): #runs on each new action goal
        rospy.wait_for_message('/gazebo/set_model_state', ModelState)
        x_k1_ultimate, y_k1_ultimate = 50, 0

        "set initial position and ultimate target"
        self.x_k, self.y_k = 0, 0
        self.x_k1, self.y_k1 = x_k1_ultimate, y_k1_ultimate 
        rospy.loginfo('pursuing line from:[' + str(self.x_k) + ', ' + str(self.y_k) + '] to: [' + str(self.x_k1) + ', ' + str(self.y_k1) + ']' )
    
        while True and not rospy.is_shutdown():
            "check if boat is on the right or left. if boat on right, set x_k1,y_k1 to trail boat"
            drone_to_dest = self.calculate_bearing([self.x, self.y], [x_k1_ultimate, y_k1_ultimate]) #angle between seadrone and target
            drone_to_foreign_boat = self.calculate_bearing([self.x, self.y], self.foreign_boat_pos) #angle between seadrone and detected boat
            
            if self.isright(drone_to_dest, drone_to_foreign_boat):
                self.x_k, self.y_k = self.x, self.y
                self.x_k1, self.y_k1 = self.foreign_boat_pos[0]-1.225, self.foreign_boat_pos[1]-1.225 #trail boat
            else:
                self.x_k1, self.y_k1 = x_k1_ultimate, y_k1_ultimate  #proceed to ultimate target    
            
            print("position: ", self.x, self.y)
            print("k_position: ", self.x_k, self.y_k)
            print("k1_position: ", self.x_k1, self.y_k1)



            if self.distance_from_goal() > self.goal_proximity:
                #gets angles and distances
                self.y_ep = self.calculate_y_ep()
                self.delta = self.get_lookahead()
                self.pi_p = self.calculate_pi_p()
                #gets desired angle x_d
                self.get_desired_heading()
                self.delta_n = self.PD_controller()
                self.error_prev = self.error
                
                self.twist_msg.angular.z = -self.delta_n
                self.twist_msg.linear.x = np.min([self.distance_from_goal() * 0.1, 1])
                self.pub.publish(self.twist_msg)
                #print(self.twist_msg)
                self.rate.sleep()
            else:
                break
        return

if __name__ == "__main__":
    rospy.init_node('LOS_colreg')
    LOS_colreg = PDLineOfSight()
    LOS_colreg.execute()
