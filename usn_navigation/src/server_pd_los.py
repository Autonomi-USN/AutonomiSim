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
from visualize_path import VisualizePath
#! action server for Line Of Sight Guidance between two points at a time, takes an array of points and iterates through them
#! The current Linear Thrust control is very primitive and should probably be looked at for improvement.
#! With current implementation there is a chance that the boat will surpass the target if the target is too close to the starting position.
#! Written by Edvart Bjerke, USN-Autonomi


class PDLineOfSight:
    # messages used to publish feedback/result
    _feedback = LOSFeedback
    _result = LOSResult

    def __init__(self):
        self.VP = VisualizePath()
        #PD Gains and Filter Constants 
        self.Kp = rospy.get_param('LOS_Kp', 10)
        self.Kd = rospy.get_param('LOS_Kd', 1)
        rate    = rospy.get_param('LOS_rate', 50)
        self.T = 0.02
        self.Tf = 0.02

        #constants for setting the lookahead distance
        self.delta_max = 5
        self.delta_min = 0.1
        self.delta = 3
        self.gamma = 0.15
        self.goal_proximity = 1

        #keep track of previous time-tick error and derivative part
        self.error_prev = 0
        self.derivative_tminus1 = 0

        self.rate = rospy.Rate(rate)
        self.twist_msg = Twist()
        
        #spin up subscribers and start the Action Server
        rospy.Subscriber('/usn_drone/robot_localization/odometry/filtered', Odometry, self.odom_cb, queue_size = 1)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.pub_error = rospy.Publisher('/LOS_heading_error', Float32, queue_size = 10)
        self._as = actionlib.SimpleActionServer('LOS_PD_action', LOSAction, execute_cb=self.execute_cb, auto_start = False)
        self._as_cancel_sub = rospy.Subscriber('/LOS_PD_action/cancel', GoalID, self.set_cancelled, queue_size = 1)
        self._result = LOSResult()
        self._as.start()

    def set_cancelled(self, msg): #cancels the action goal
        self.goal_active = False
        self._result.success = False

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

    def execute_cb(self, goal): #runs on each new action goal
        wp = []
        poses = goal.poses.poses

        for i in range(1, len(poses)):
            wp.append([poses[i].position.x, poses[i].position.y])

        self.VP.set_start_and_stop([poses[0].position.x, poses[0].position.y],wp) #Visualize waypoints in rviz

        self.goal_active = True
        self._result.success = False

        feedback = LOSFeedback()

        for i in range(len(poses)-1):
            self.x_k, self.y_k = poses[i].position.x, poses[i].position.y
            self.x_k1, self.y_k1 = poses[i+1].position.x, poses[i+1].position.y
            rospy.loginfo('pursuing line from:[' + str(self.x_k) + ', ' + str(self.y_k) + '] to: [' + str(self.x_k1) + ', ' + str(self.y_k1) + ']' )
    
            while True:
                if (self.distance_from_goal() > self.goal_proximity and self.goal_active):
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

                    feedback.distance = self.distance_from_goal()
                    self._as.publish_feedback(feedback)
                else:
                    break
        
        if self.goal_active:
            self._result.success = True
            self.goal_active = False
            self._as.set_succeeded(self._result)
        else:
            self._result = False
            self._as.set_preempted(self._result)
        return

if __name__ == "__main__":
    rospy.init_node('PD_line_of_sight')
    name_node = PDLineOfSight()
    rospy.spin()