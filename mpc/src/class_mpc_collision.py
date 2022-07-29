#!/usr/bin/env python3

from pyrr import Vector3
from tf.transformations import euler_from_quaternion
import casadi as ca
import numpy as np
from casadi import sin, cos, pi
import matplotlib.pyplot as plt
from variables import *
import rospy
from sensor_msgs.msg import NavSatFix, Imu
from  geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32
import casadi as ca
import numpy as np
import rospy
import matplotlib.pyplot as plt
import pymap3d as pm
from casadi import sin, cos, pi
from variables import *
from std_msgs.msg import Float32
from time import time
from casadi import sin, cos, pi
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from tf.transformations import euler_from_quaternion
import casadi as ca
import numpy as np
import rospy
import matplotlib.pyplot as plt
import pymap3d as pm
from casadi import sin, cos, pi
from variables import *
from std_msgs.msg import Float32
from time import time
from casadi import sin, cos, pi
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from tf.transformations import euler_from_quaternion
import math
from nav_msgs.msg import Odometry


class mpc:
    def __init__(self):
        self.gps_sub = rospy.Subscriber("/usn_drone/sensors/gps/gps/fix", NavSatFix, self.gpsCallback)
        self.imu_sub = rospy.Subscriber("/usn_drone/sensors/imu/imu/data", Imu, self.imuCallback)
        self.velocity_sub = rospy.Subscriber("/usn_drone/robot_localization/odometry/filtered", Odometry, self.fixVelCallback)        
        self.pub_right_thrust = rospy.Publisher('/usn_drone/thrusters/right_thrust_cmd', Float32, queue_size = 10)
        self.pub_left_thrust  = rospy.Publisher('/usn_drone/thrusters/left_thrust_cmd', Float32, queue_size = 10)

        self.foreign_boat_sub = rospy.Subscriber("/gazebo/set_model_state", ModelState, self.foreignBoatPosCb)

        self.init_counter = True

        self.step_horizon = 0.5  # time between steps in seconds?? 0.1 Try diefferent step size.
        self.N = 40               # number of look ahead steps?? May increase to 2 sec horizon
        self.sim_time = 4000      # simulation time
        self.max_u = 2.8
        self.max_fwd_F = 28 #max forward thrust
        self.max_bwd_F = -20 #max backward thrust
        
        self.latitude_ori = 0
        self.longitude_ori = 0
        self.altitude_ori = 0

        self.target_list_x = [50]   #!delete later
        self.target_list_y = [10]   #!----------!#

        self.ultimate_target_x = 50
        self.ultimate_target_y = 0

        self.x_init, self.y_init, self.yaw = 0, 0, 0
        self.u, self.v, self.r = 0, 0, 0
        self.tic = 0

    def foreignBoatPosCb(self, data):
        if data.model_name == "collision_boat":
            self.foreign_boat_pos = [data.pose.position.x, data.pose.position.y]


    def gpsCallback(self, data):
        latitude = data.latitude
        longitude = data.longitude
        altitude = data.altitude
        #To capture initial lat long for ENU reference
        #Executes only once at beganing
        if self.init_counter:
            self.latitude_ori = latitude
            self.longitude_ori = longitude
            self.altitude_ori = altitude
            self.init_counter = False

        x_init_temp, y_init_temp, z_init_temp = pm.geodetic2enu(latitude, longitude,\
                                altitude, self.latitude_ori, self.longitude_ori, self.altitude_ori)
        self.x_init = x_init_temp
        self.y_init = y_init_temp
        self.z_init = z_init_temp

    def imuCallback(self, data):
        (roll,pitch,yaw) = euler_from_quaternion([data.orientation.x,\
            data.orientation.y,data.orientation.z,data.orientation.w])
        
        self.yaw = yaw
        r_init_temp = data.angular_velocity.z
        self.r = r_init_temp

    def fixVelCallback(self, data):
        self.u = data.twist.twist.linear.x
        self.v = -data.twist.twist.linear.y
        
    
    def get_solver(self):
        T = self.step_horizon
        N = self.N

        Q_x = 500
        Q_y = 500
        Q_theta = 100
        Q_u = 1
        Q_v = 0
        Q_r = 0

        R1 = 1
        R2 = 1

        m = 18      #Mass of USV
        Iz = 50      #Value did not find in code gussed and set value
        Xu = 20.3     #Value from code
        Yv = 20
        Nr = 20
        Xdotu = 0#-0.08  #Value from code
        Ydotv = 0#50   #Value from code
        Ndotr = 0   #Value from code
        dp = 0.28      #Value from code

        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        u = ca.SX.sym('u')
        v = ca.SX.sym('v')
        r = ca.SX.sym('r')

        states = ca.vertcat(
            x,
            y,
            theta,
            u,
            v,
            r
        )
        n_states = states.numel()

        # control symbolic variables
        XP1 = ca.SX.sym('XP1')
        XP2 = ca.SX.sym('XP2')

        controls = ca.vertcat(
            XP1,
            XP2
        )
        n_controls = controls.numel()

        # matrix containing all states over all time steps +1 (each column is a state vector)
        X = ca.SX.sym('X', n_states, N + 1)

        # matrix containing all control actions over all time steps (each column is an action vector)
        U = ca.SX.sym('U', n_controls, N)

        # coloumn vector for storing initial state and target state
        P = ca.SX.sym('P', n_states + N * (n_states + n_controls))

        # state weights matrix (Q_X, Q_Y, Q_THETA)
        Q = ca.diagcat(Q_x, Q_y, Q_theta, Q_u, Q_v, Q_r)

        # controls weights matrix
        R = ca.diagcat(R1, R2)


        # RHS = ca.vertcat(u*cos(theta),\
        #                 u*sin(theta),\
        #                 r,\
        #                 ((XP1+XP2)/20),\
        #                 0,\
        #                 ((XP1-XP2)/177))
        #!proper model
        RHS = ca.vertcat(u*cos(theta) -v*sin(theta),\
                        u*sin(theta)+ v*cos(theta),\
                        r,\
                        (-Xu*u + (XP1+XP2) + (m-Ydotv)*v*r)/(m-Xdotu),\
                        (-Yv*v + (m-Xdotu)*u*r)/(m-Ydotv),\
                        ((XP1-XP2)*dp - Nr*r + u*v*((m-Xdotu)-(m-Ydotv)))/(Iz-Ndotr))

        # maps controls from [va, vb, vc, vd].T to [vx, vy, omega].T
        f = ca.Function('f', [states, controls], [RHS])


        obj = 0  # cost function
        g = []
        st = X[:, 0]




        
        g = ca.vertcat(g,st-P[0:n_states]) #initial condition constraints
        for k in range(0,N):
            st = X[:,k]
            con = U[:,k]
            
            mtimes1 = ca.mtimes((st - P[(n_states+n_controls)*(k+1):(n_states+n_controls)*(k+1)+n_states]).T,Q)
            mtimes2 = ca.mtimes(mtimes1,(st - P[(n_states+n_controls)*(k+1):(n_states+n_controls)*(k+1)+n_states]))
            obj = obj + mtimes2
            mtimes3 = ca.mtimes((con - P[(n_states+n_controls)*(0+1)-n_controls:(n_states+n_controls)*(0+1)]).T,R)
            mtimes4 = ca.mtimes(mtimes3,(con - P[(n_states+n_controls)*(0+1)-n_controls:(n_states+n_controls)*(0+1)]))
            obj = obj + mtimes4

            #####
            st_next = X[:,k+1] #next state vector
            f_value = f(st,con) #calculates the state change
            st_next_euler = st + (T*f_value) #forward euler method for iteration
            g = ca.vertcat(g,st_next-st_next_euler) #compute constraints




        OPT_variables = ca.vertcat(
            X.reshape((n_states*(N+1), 1)),   # Example: 3x11 ---> 33x1 where 3=states, 11=N+1
            U.reshape((n_controls*N, 1))
        )

        nlp_prob = {
            'f': obj,
            'x': OPT_variables,
            'g': g,
            'p': P
        }

        opts = {
            'ipopt': {
                'max_iter': 500, #Correction Try out first
                'print_level': 0,
                'acceptable_tol': 1e-4,
                'acceptable_obj_change_tol': 1e-2
            },
            'print_time': 0
        }

        solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)
        return solver


    def optimize(self, x_init, y_init, yaw_init, u_init, v_init, r_init, x_target, y_target, x_target_prev, y_target_prev):
        
        #convert to model frame, see paper
        y_init_temp = y_init
        y_init = x_init
        x_init = y_init_temp
        yaw_init = pi/2 - yaw_init
        r_init = -r_init

        y_target_temp = y_target
        y_target = x_target
        x_target = y_target_temp
        y_target_prev_temp = y_target_prev
        y_target_prev = x_target_prev
        x_target_prev = y_target_prev_temp

        print('target: ', x_target, y_target)
        print('position: ', x_init, y_init)

        n_states = 6
        n_controls = 2
        N = self.N
        T = self.step_horizon

        lbx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))
        ubx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))
        ##########
        lbx[0: n_states*(N+1): n_states] = -500     # X lower bound
        lbx[1: n_states*(N+1): n_states] = -500     # Y lower bound
        lbx[2: n_states*(N+1): n_states] = -7     # theta lower bound
        lbx[3: n_states*(N+1): n_states] = -6        # u lower bound
        lbx[4: n_states*(N+1): n_states] = -6        # v lower bound
        lbx[5: n_states*(N+1): n_states] = -50     # r lower bound
        ubx[0: n_states*(N+1): n_states] = 500      # X upper bound
        ubx[1: n_states*(N+1): n_states] = 500      # Y upper bound
        ubx[2: n_states*(N+1): n_states] = 7      # theta upper bound
        ubx[3: n_states*(N+1): n_states] = 6         # u upper bound
        ubx[4: n_states*(N+1): n_states] = 6        # v upper bound
        ubx[5: n_states*(N+1): n_states] = 50      # r upper bound
        lbx[n_states*(N+1):] = self.max_bwd_F                   # tau lower bound
        ubx[n_states*(N+1):] = self.max_fwd_F                   # tau upper bound

        args = {
            'lbg': ca.DM.zeros((n_states*(N+1), 1)),  # constraints lower bound
            'ubg': ca.DM.zeros((n_states*(N+1), 1)),  # constraints upper bound
            'lbx': lbx,
            'ubx': ubx
        }
        
        p = np.zeros((N+1)*n_states + N*n_controls)
        

        #!new coordinates
        psi_ref = ca.mod(math.atan2( (y_target-y_target_prev),(x_target-x_target_prev) ) + 2*pi, 2*pi)
        v_x_ref = self.max_u*cos(psi_ref)
        v_y_ref = self.max_u*sin(psi_ref)


        current_trajectory_state_x, current_trajectory_state_y = self.p4([x_target_prev, y_target_prev], [x_target, y_target], [x_init, y_init])
        x_ref0 = current_trajectory_state_x #cross-track
        y_ref0 = current_trajectory_state_y #cross-track
        

        for k in range(0,N+1): #new - set the reference to track
            t_predict = (k)*T # predicted time instant
            x_ref = x_ref0 + v_x_ref*t_predict
            y_ref = y_ref0 + v_y_ref*t_predict            
            p[(n_states+n_controls)*(k):(n_states+n_controls)*k + n_states] = [x_ref, y_ref, psi_ref, self.max_u, 0, 0] #check if 28*2 because two motors
            if k != N:
                p[(n_states+n_controls)*(k)+n_states:(n_states+n_controls)*(k) + n_states + n_controls] = [self.max_fwd_F, self.max_bwd_F]

        p[0], p[1], p[2], p[3], p[4], p[5] = x_init, y_init, yaw_init, self.u, self.v, r_init
        args['p'] = p
        args['x0'] = np.zeros((N+1)*n_states + N * n_controls)

        sol = self.solver(
            x0=args['x0'],
            lbx=args['lbx'],
            ubx=args['ubx'],
            lbg=args['lbg'],
            ubg=args['ubg'],
            p=args['p']
        )

        #Extract and seperate control and signal data from solver output
        u = ca.reshape(sol['x'][n_states * (N + 1):], n_controls, N)
        X0 = ca.reshape(sol['x'][: n_states * (N+1)], n_states, N+1)
        cont_XP1 = self.DM2Arr(u[0, 0])
        cont_XP2 = self.DM2Arr(u[1, 0])
        
        print(psi_ref)
        return cont_XP1, cont_XP2

    def DM2Arr(self,dm):
        return np.array(dm.full())

    def p4(self, p1, p2, p3):
        x1, y1 = p1
        x2, y2 = p2
        if p1 == p2:
            return 0, 0
        x3, y3 = p3
        dx, dy = x2-x1, y2-y1
        det = dx*dx + dy*dy
        a = (dy*(y3-y1)+dx*(x3-x1))/det
        return x1+a*dx, y1+a*dy

    def isTargetReached(self, x_target, y_target):
        state_init = np.array([self.x_init, self.y_init, self.yaw])
        state_target = np.array([x_target, y_target, 0])

        if (ca.norm_2(state_init[0:2] - state_target[0:2]) < 1):

            if (ca.norm_2(state_init[2] - state_target[2]) < 1):
                return True

        return False


    def run(self):
        x_target, y_target = 0, 0
        for x, y in zip(self.target_list_x, self.target_list_y):
            x_target_prev, y_target_prev = x_target, y_target
            x_target, y_target = x, y

            
            while True:
                self.tic = time()

                if rospy.is_shutdown():
                    break
                if self.isTargetReached(x_target, y_target):
                    print('reached goal')
                    break

                cont_XP1, cont_XP2 = self.optimize(self.x_init, self.y_init, self.yaw, self.u, self.v, self.r, x_target, y_target, x_target_prev, y_target_prev)
                
                #Scaling control to 1 to -1
                if cont_XP1 < 0 :
                    cont_XP1 = cont_XP1/np.abs(self.max_bwd_F)
                else :
                    cont_XP1 = cont_XP1/self.max_fwd_F

                if cont_XP2 < 0 :
                    cont_XP2 = cont_XP2/np.abs(self.max_bwd_F)
                else :
                    cont_XP2 = cont_XP2/self.max_fwd_F
                
                self.pub_left_thrust.publish(cont_XP1)
                self.pub_right_thrust.publish(cont_XP2)

        
if __name__ == "__main__":
    rospy.init_node('mpc')
    mpc = mpc()
    mpc.solver = mpc.get_solver()
    if not rospy.is_shutdown():
        mpc.run()