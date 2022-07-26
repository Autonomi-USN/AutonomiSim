#!/usr/bin/env python3

from time import time

from sympy import euler
from tf.transformations import euler_from_quaternion, quaternion_from_euler
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
import queue
import rospy
import matplotlib.pyplot as plt
import pymap3d as pm
from casadi import sin, cos, pi
from variables import *
from std_msgs.msg import Float32
from time import time, ctime, sleep
from casadi import sin, cos, pi
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from tf.transformations import euler_from_quaternion
#from supporting_variables_GAZEBO import *
from geometry_msgs.msg import Pose
import casadi as ca
import numpy as np
import queue
import rospy
import matplotlib.pyplot as plt
import pymap3d as pm
from casadi import sin, cos, pi
from variables import *
from std_msgs.msg import Float32
from time import time, ctime
from casadi import sin, cos, pi
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from tf.transformations import euler_from_quaternion
from robot_localization.srv import SetPose, SetPoseRequest, SetPoseResponse
from nav_msgs.msg import Odometry
import math



def p4(p1, p2, p3):
    x1, y1 = p1
    x2, y2 = p2
    if p1 == p2:
        return 0, 0
    x3, y3 = p3
    dx, dy = x2-x1, y2-y1
    det = dx*dx + dy*dy
    a = (dy*(y3-y1)+dx*(x3-x1))/det
    return x1+a*dx, y1+a*dy


x_target, x_target_prev = 0, 0
y_target, y_target_prev = 0, 0
v_max, v_min = 28.449, -20
#yaw = 0

def service_cb(request):
    global theta_target, DM2Arr, pub_r, pub_l, odom_init, odom, x_init, y_init, z_init, init_counter,latitude_ori, longitude_ori, altitude_ori, r_init, theta_init, u_init, v_init, mpc_iter, step_horizon, sim_time, state_init, state_target, obstacleFlag, args, state_init, state_target, n_states, n_controls, N, f, solver, nlp_prob, mpc_iter, u0, X0, t0, t, cat_states, cat_controls, cont_XP1, cont_XP2, times, x_init, y_init, theta_init, u_init, v_init, r_init, obstacleFlag, state_init, state_target, mpc_iter, movingTargetCounter, mpc_iter, ax, p, obstacle_axis, obstacle_clearance

    target_list_x = [5, 10, 15, 20, 25, 30, 35]
    target_list_y = [20,  0, 20,  0, 20, 0, 20]

    Q_x = 100
    Q_y = 100
    Q_theta = 100
    Q_u = 1
    Q_v = 0
    Q_r = 0

    R1 = 0.01
    R2 = 0.01

    step_horizon = 0.5  # time between steps in seconds?? 0.1 Try diefferent step size.
    N = 40              # number of look ahead steps?? May increase to 2 sec horizon
    sim_time = 4000      # simulation time
    
    #getting the initial pose of the boat
    odometry = rospy.wait_for_message('/usn_drone/robot_localization/odometry/filtered', Odometry)
    orientation_quats = odometry.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion((orientation_quats.x, orientation_quats.y, orientation_quats.z, orientation_quats.w))
    
    # specs
    x_init = 0
    y_init = 0
    theta_init = yaw #yaw
    u_init = 0
    v_init = 0
    r_init = 0

    theta_target = 0
    u_target = 0
    v_target = 0
    r_target = 0

    v_max = 28.449
    v_min = -20

    state_init = ca.DM([x_init, y_init, theta_init, u_init, v_init, r_init])        # initial state
    state_target = ca.DM([x_target, y_target, theta_target, u_target, v_target, r_target])  # target state

    #Obstacle
    obstacle_axis = [5, 20]
    obstacle_clearance = 5
    obstacle_stationary_trajectory = [[13, 5]]
    obstacle_moving_trajectory = [[30,10],[20,20],[10,30],[0,40]]
    obstacleFlag = True
    movingTargetCounter = -1

    #Plot
    graphLimitPos = 70
    graphLimitNeg = -30
    ax = plt.gca()
    ax.set_xlim((graphLimitNeg,graphLimitPos))
    ax.set_ylim((graphLimitNeg,graphLimitPos))
    ax.set_aspect(1)

    #ROS
    latitude_ori = 0
    longitude_ori = 0
    altitude_ori = 0

    init_counter = True

    #Loop variables
    mpc_iter = 0

    m = 18      #Mass of USV
    Iz = 50      #Value did not find in code gussed and set value
    Xu = 20.3     #Value from code
    Yv = 20
    Nr = 20
    Xdotu = 5#-0.08  #Value from code
    Ydotv = 5#50   #Value from code
    Ndotr = 1   #Value from code
    dp = 0.28      #Value from code

    # state symbolic variables
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

    lbx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))
    ubx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))
    ##########
    lbx[0: n_states*(N+1): n_states] = -1000     # X lower bound
    lbx[1: n_states*(N+1): n_states] = -1000     # Y lower bound
    lbx[2: n_states*(N+1): n_states] = -500     # theta lower bound
    lbx[3: n_states*(N+1): n_states] = -30        # u lower bound
    lbx[4: n_states*(N+1): n_states] = -30        # v lower bound
    lbx[5: n_states*(N+1): n_states] = -5     # r lower bound

    ubx[0: n_states*(N+1): n_states] = 1000      # X upper bound
    ubx[1: n_states*(N+1): n_states] = 1000      # Y upper bound
    ubx[2: n_states*(N+1): n_states] = 500      # theta upper bound
    ubx[3: n_states*(N+1): n_states] = 30         # u upper bound
    ubx[4: n_states*(N+1): n_states] = 30        # v upper bound
    ubx[5: n_states*(N+1): n_states] = 5      # r upper bound

    lbx[n_states*(N+1):] = v_min                   # tau lower bound
    ubx[n_states*(N+1):] = v_max                   # tau upper bound


    args = {
        'lbg': ca.DM.zeros((n_states*(N+1), 1)),  # constraints lower bound
        'ubg': ca.DM.zeros((n_states*(N+1), 1)),  # constraints upper bound
        'lbx': lbx,
        'ubx': ubx
    }

    RHS = ca.vertcat(u*cos(theta) -v*sin(theta),\
                    u*sin(theta)+ v*cos(theta),\
                    r,\
                    (-Xu*u + (XP1+XP2) + (m-Ydotv)*v*r)/(m-Xdotu),\
                    (-Yv*v - (m-Xdotu)*u*r)/(m-Ydotv),\
                    ((-Nr*r) + ((XP1-XP2)*dp) + (-Xdotu+Ydotv)*u*v)/(Iz-Ndotr))
        
    # maps controls from [va, vb, vc, vd].T to [vx, vy, omega].T
    f = ca.Function('f', [states, controls], [RHS])


    obj = 0  # cost function
    g = []
    st = X[:, 0]
    
    g = ca.vertcat(g,st-P[0:n_states]) #initial condition constraints
    for k in range(0,N):
        st = X[:,k]
        con = U[:,k]
        ##### objective function
        mtimes1 = ca.mtimes((st - P[(n_states+n_controls)*(k+1)-n_states:(n_states+n_controls)*(k+1)+0]).T,Q)
        mtimes2 = ca.mtimes(mtimes1,(st - P[(n_states+n_controls)*(k+1)-n_states:(n_states+n_controls)*(k+1)+0]))
        obj = obj + mtimes2
        mtimes3 = ca.mtimes((con - P[(n_states+n_controls)*(k+1)+0:(n_states+n_controls)*(k+1)+n_controls]).T,R)
        mtimes4 = ca.mtimes(mtimes3,(con - P[(n_states+n_controls)*(k+1)+0:(n_states+n_controls)*(k+1)+n_controls]))
        obj = obj + mtimes4
        #####
        st_next = X[:,k+1] #next state vector
        f_value = f(st,con) #calculates the state change
        st_next_euler = st + (step_horizon*f_value) #forward euler method for iteration
        g = ca.vertcat(g,st_next-st_next_euler) #compute constraints




    OPT_variables = ca.vertcat(
        X.reshape((-1, 1)),   # Example: 3x11 ---> 33x1 where 3=states, 11=N+1
        U.reshape((-1, 1))
    )

    nlp_prob = {
        'f': obj,
        'x': OPT_variables,
        'g': g,
        'p': P
    }

    opts = {
        'ipopt': {
            'max_iter': 10000, #Correction Try out first
            'print_level': 1,
            'acceptable_tol': 1e-4,
            'acceptable_obj_change_tol': 1e-2
        },
        'print_time': 1
    }

    solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

    mpc_iter = 0
    u0 = ca.DM.zeros((n_controls, N))          # initial control
    X0 = ca.repmat(state_init, 1, N+1)         # initial state full
    t0 = 0
    t = ca.DM(t0)

    def DM2Arr(dm):
        return np.array(dm.full())

    cat_states = DM2Arr(X0)
    cat_controls = DM2Arr(u0[:, 0])
    cont_XP1 = DM2Arr(u0[0, 0])
    cont_XP2 = DM2Arr(u0[1, 0])
    times = np.array([[0]])

    rospy.Subscriber("/usn_drone/sensors/gps/gps/fix", NavSatFix, gpsCallback)
    rospy.Subscriber("/usn_drone/sensors/imu/imu/data", Imu, imuCallback)
    rospy.Subscriber("/usn_drone/sensors/gps/gps/fix_velocity", Vector3Stamped, fixVelCallback)
    pub_r = rospy.Publisher('/usn_drone/thrusters/right_thrust_cmd', Float32, queue_size = 10)
    pub_l = rospy.Publisher('/usn_drone/thrusters/left_thrust_cmd', Float32, queue_size = 10)

    
    i = 0
    while True:
        #Check if target reached

        if isTargetReached():
            print('target reached')
            x_target_prev, y_target_prev = x_target, y_target
            x_target, y_target = target_list_x[i], target_list_y[i]
            state_target = ca.DM([x_target, y_target, theta_target, u_target, v_target, r_target])


            i += 1
            if i == (len(target_list_x) - 1):
                break

        chaseTarget()
    
    ################################### Plotting ##################################
    ax.plot(cat_states[0, 0], cat_states[1, 0])

    #Print Obstacle
    if isObstacleStationary():
        drawObstacle(obstacle_stationary_trajectory)
    else :
        drawObstacle(obstacle_moving_trajectory)

    #Plot
    ax.plot(0,0, 'o', color='k')
    ax.plot(x_target, y_target, 'o', color='r')
    plt.show()
    #plt.cla()
    return SetPoseResponse()


def gpsCallback(data):
    global x_init, y_init, z_init,\
         init_counter,latitude_ori, longitude_ori, altitude_ori

    latitude = data.latitude
    longitude = data.longitude
    altitude = data.altitude
    #To capture initial lat long for ENU reference
    #Executes only once at beganing
    if init_counter:
        latitude_ori = latitude
        longitude_ori = longitude
        altitude_ori = altitude
        init_counter = False
        #(str(latitude_ori) + ", " + str(longitude_ori) + ", "+str(altitude_ori))

    x_init_temp, y_init_temp, z_init_temp = pm.geodetic2enu(latitude, longitude,\
                            altitude, latitude_ori, longitude_ori, altitude_ori)
    x_init = x_init_temp
    y_init = y_init_temp
    z_init = z_init_temp


def imuCallback(data):
    global r_init, theta_init
    (roll,pitch,yaw) = euler_from_quaternion([data.orientation.x,\
        data.orientation.y,data.orientation.z,data.orientation.w])
    theta_init = yaw
    r_init_temp = data.angular_velocity.z
    r_init = r_init_temp

def fixVelCallback(data):

    global u_init, v_init
    u_init_temp = data.vector.x
    v_init_temp = data.vector.y
    u_init = u_init_temp
    v_init = v_init_temp



def isTargetReached():
    global mpc_iter, step_horizon, sim_time, state_init,\
        state_target, obstacleFlag, theta_target

    if (mpc_iter * step_horizon > sim_time):
        return True

    print(ca.norm_2(state_init[0:3] - ca.DM([x_target, \
        y_target, theta_target])))
    print(state_target[0], state_target[1])

    if (ca.norm_2(state_init[0:2] - state_target[0:2]) < 1):

        if (ca.norm_2(state_init[2] - state_target[2]) < 0.2):
            return True

    return False

def chaseTarget():
    global args, state_init, state_target, n_states,\
        n_controls, N, f, solver, nlp_prob, mpc_iter, u0,\
        X0, t0, t, cat_states, cat_controls, cont_XP1,\
        cont_XP2, times, x_init, y_init, theta_init,\
        u_init, v_init, r_init, x_target_prev, y_target_prev, x_target, y_target, v_max, v_min, yaw
            

    t1 = time()

    state_init[0,0] = x_init
    state_init[1,0] = y_init
    state_init[2,0] = theta_init
    state_init[3,0] = u_init
    state_init[4,0] = v_init
    state_init[5,0] = r_init
    #print(state_init)

    p = np.zeros(N*n_states + (N+1)*n_controls)
    
    psi_ref = ca.mod(math.atan2( (x_target-x_target_prev),(y_target-y_target_prev) ) + 2*pi, 2*pi)
    v_x_ref = v_max*sin(psi_ref)
    v_y_ref = v_max*cos(psi_ref)

    current_trajectory_state_x, current_trajectory_state_y = p4([x_target_prev, y_target_prev], [x_target, y_target], [x_init, y_init])
    x_ref0 = current_trajectory_state_x #cross-track
    y_ref0 = current_trajectory_state_y #cross-track
    x_ref_previous = x_ref0
    y_ref_previous = y_ref0
    r_ref = 0
    

    for k in range(0,N): #new - set the reference to track
        t_predict = (k)*step_horizon # predicted time instant
        x_ref = x_ref0 + v_x_ref*t_predict
        y_ref = y_ref0 + v_y_ref*t_predict
        x_ref_previous = x_ref
        y_ref_previous = y_ref
        p[(n_states+n_controls)*(k+1)-n_states:(n_states+n_controls)*(k+1)+0] = [x_ref, y_ref, psi_ref, 28, 0, r_ref] #check if 28*2 because two motors
        p[(n_states+n_controls)*(k+1)+0:(n_states+n_controls)*(k+1)+n_controls] = [28, 28]

    p[0], p[1], p[2] = x_init, y_init, yaw
    args['p'] = p

    print(state_target)
    # optimization variable current state
    # args['x0'] = ca.vertcat(
    #     ca.reshape(X0, n_states*(N+1), 1),
    #     ca.reshape(u0, n_controls*N, 1)
    # )

    #args['x0'] = np.zeros(n_states*(N+1) + n_controls*N) #!original
    
    args['x0'] = np.zeros(N*n_states*(N+1) * n_controls)
    print(args['p'])
    
    sol = solver(
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

    # print(u)
    #Store state and control for ploting
    cat_states = np.dstack((
        cat_states,
        DM2Arr(X0)
    ))
    cat_controls = np.vstack((
        cat_controls,
        DM2Arr(u[:, 0])
    ))

    cont_XP1 = DM2Arr(u[0, 0])
    cont_XP2 = DM2Arr(u[1, 0])

    #Scaling control to 1 to -1
    if cont_XP1 < 0 :
        cont_XP1 = DM2Arr(u[0, 0])/20
    else :
        cont_XP1 = DM2Arr(u[0, 0])/28.449

    if cont_XP2 < 0 :
        cont_XP2 = DM2Arr(u[1, 0])/20
    else :
        cont_XP2 = DM2Arr(u[1, 0])/28.449

    print(cont_XP1, cont_XP2)

    #Publish to ROS
    if not rospy.is_shutdown():
        pub_r.publish(cont_XP1)
        pub_l.publish(cont_XP2)


    t = np.vstack((
        t,
        t0
    ))

    #Mathematical iteration muted for GAZEBO
    # t0, state_init, u0 = shift_timestep(step_horizon, t0, state_init, u, f)

    # print(X0)
    X0 = ca.horzcat(
        X0[:, 1:],
        ca.reshape(X0[:, -1], -1, 1)
    )

    # xx ...
    t2 = time()
    #print(mpc_iter)
    # print(t2-t1)
    times = np.vstack((
        times,
        t2-t1
    ))

    mpc_iter = mpc_iter + 1


#Draws obstacle is grap
#Input : trajectory in array
def drawObstacle(trajectory):
    global ax, p, obstacle_axis, obstacle_clearance

    for axis in trajectory:
        ax.plot((axis[0]), (axis[1]), 'o', alpha=1, color='y')
        p = plt.Circle(( axis[0] , axis[1] ), obstacle_clearance
                       ,color='r', alpha=1, fill=False)
        ax.add_artist(p)

    ax.add_artist(p)




# rospy.Subscriber("/usn_drone/sensors/gps/gps/fix", NavSatFix, gpsCallback)
# rospy.Subscriber("/usn_drone/sensors/imu/imu/data", Imu, imuCallback)
# rospy.Subscriber("/usn_drone/sensors/gps/gps/fix_velocity", Vector3Stamped, fixVelCallback)
# pub_r = rospy.Publisher('/usn_drone/thrusters/right_thrust_cmd', Float32, queue_size = 10)
# pub_l = rospy.Publisher('/usn_drone/thrusters/left_thrust_cmd', Float32, queue_size = 10)
service = rospy.Service('run_mpc', SetPose, service_cb)


rospy.init_node('publisher_thruster_right', anonymous=True)
rate = rospy.Rate(10)
rospy.spin()