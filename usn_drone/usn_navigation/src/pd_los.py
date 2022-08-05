#!/usr/bin/env python3
import rospy
import numpy as np
from math import pi
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist

class PDLineOfSight:
    def __init__(self):
        #PD Gains and Filter Constants 
        self.Kp = rospy.get_param('LOS_Kp', 10)
        self.Kd = rospy.get_param('LOS_Kd', 1)
        rate    = rospy.get_param('LOS_rate', 50)

        self.T = 0.02
        self.Tf = 0.02

        self.delta_max = 5
        self.delta_min = 0.1
        self.delta = 3
        self.gamma = 0.15

        self.error_prev = 0
        self.derivative_tminus1 = 0

        #! hard coded waypoints, will make subscriber later
        self.x_k, self.y_k = -20, 35
        self.x_k1, self.y_k1 = 20, 20

        self.rate = rospy.Rate(rate)
        self.twist_msg = Twist()
        
        rospy.Subscriber('/usn_drone/robot_localization/odometry/filtered', Odometry, self.odom_cb, queue_size = 1)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.pub_error = rospy.Publisher('/LOS_heading_error', Float32, queue_size = 10)

    def calculate_y_ep(self): #calculates distance from point (x, y) to line from (x_k, y_k) to (x_k1, y_k1)
        p1=np.array([self.x_k, self.y_k])
        p2=np.array([self.x_k1, self.y_k1])
        p3=np.array([self.x, self.y])
        distance=np.cross(p2-p1,p3-p1)/np.linalg.norm(p2-p1)
        return distance

    def PD_controller(self):
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

    def get_lookahead(self):
        lookahead = (self.delta_max-self.delta_min)*np.exp(-self.gamma*np.abs(self.y_ep))
        return lookahead

    def distance_from_goal(self):
        return np.sqrt((self.x_k1-self.x)**2+(self.y_k1-self.y)**2)
        
    def odom_cb(self, msg): #keeps track of current USV heading and position (x, y)
        quat = msg.pose.pose.orientation
        position = msg.pose.pose.position
        self.psi = euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))[2] - pi/2
        if self.psi < (-pi):
            self.psi = self.psi+2*pi

        
        self.x, self.y = position.x, position.y

        if self.distance_from_goal() > 0.5:
            #gets angles and distances
            self.y_ep = self.calculate_y_ep()
            self.delta = self.get_lookahead()
            self.pi_p = self.calculate_pi_p()
            #gets desired angle x_d
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

            self.delta_n = self.PD_controller()
            self.error_prev = self.error
            

            self.twist_msg.angular.z = -self.delta_n
            self.twist_msg.linear.x = np.min([self.distance_from_goal() * 0.1, 1])

            self.pub.publish(self.twist_msg)
            
            
            #publish error for live plotting 
            #msg = Float32()
            #msg.data = self.error        
            #self.pub_error.publish(msg)
            
            print('---------------------')
            print('| x         |',np.round(self.x, 3))
            print('| y         |',np.round(self.y, 3))
            print('| distance  |',np.round(self.y_ep, 3))
            print('| pi_p      |',np.round(self.pi_p, 3))
            print('| heading   |', np.round(self.psi, 3))
            print('| heading_d |', np.round(self.x_d, 3))
            print('| error     |', np.round(self.error, 3))
            print('| delta_n   |', np.round(self.delta_n, 3))
            print('| lookahead |', np.round(self.delta, 5))
            print('---------------------')
        self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node('PD_line_of_sight')
    name_node = PDLineOfSight()
    rospy.spin()