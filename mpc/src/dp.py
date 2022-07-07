#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64, Float32
from sensor_msgs.msg import NavSatFix
from simple_pid import PID
from numpy import array
from numpy.linalg import norm



class Dynamic_pos:
  def __init__(self):
    self.heading_sub = rospy.Subscriber("/navigation/heading",Float64, self.heading_callback)
    self.position_sub = rospy.Subscriber("/usn_drone/sensors/gps/gps/fix",NavSatFix, self.position_callback)

    self.left_thruster_cmd_pub = rospy.Publisher('/usn_drone/thrusters/left_thrust_cmd', Float32, queue_size=10)
    self.right_thruster_cmd_pub = rospy.Publisher('/usn_drone/thrusters/right_thrust_cmd', Float32, queue_size=10)
    self.left_thruster_angle_pub = rospy.Publisher('/usn_drone/thrusters/left_thrust_angle', Float32, queue_size=10)
    self.right_thruster_angle_pub = rospy.Publisher('/usn_drone/thrusters/right_thrust_angle', Float32, queue_size=10)

    self.right_thruster_cmd_msg = Float32()
    self.left_thruster_cmd_msg = Float32()

    self.pid = PID(20, 0.0, 0, setpoint=0.0)
    self.PIDhold = PID(2, 0, 0 ,sample_time=0.01,output_limits=(-35,35))
    self.dynamic_point = 0.0
    self.count = 0
    self.delta_pos = 0.0
    self.position = 0.0
    self.heading = 0.0
    self.dynamic_heading = 0.0
    self.thrust_dir1 = 0.0

    self.direc1 = 0.0
    self.direc2 = 0.0
    self.thu1 = 0.0
    self.thu2 = 0.0

  def thruster(self,thu1,thu2,direc1,direc2):
    self.direc1 = direc1      # Decides the thruster direction
    self.direc2 = direc2
    self.thu1 = thu1          # motor thrust message
    self.thu2 = thu2

  def servo_publish(self, servo_angle_l, servo_angle_r):
    self.move.right_thruster.angular.z= servo_angle_r
    self.move.left_thruster.angular.z= servo_angle_l

  def heading_callback(self,msg):
    self.heading = msg.data


  def position_callback(self,data):
    self.lat = data.latitude
    self.lon = data.longitude
    self.position = [self.lat, self.lon]
    self.dynamic_wp()
    self.regulation()

  def P_reg(self, error, Kp):
      return Kp*error

  def dynamic_wp(self):
      if self.count == 0:
        self.dynamic_point = self.position
        self.dynamic_heading = self.heading
        self.count +=1




  def regulation(self):
    #--------------- Position --------------------------
    self.delta_pos = [self.dynamic_point[0] - self.position[0],self.dynamic_point[1] - self.position[1]]
    self.norm_pos = norm(self.delta_pos)
    self.delta_heading = self.dynamic_heading - self.heading

    """
    self.pid_reg_heading = self.pid(self.delta_heading)
    self.force_thruster1 = self.pid(self.delta_pos)
    """
    self.force_thruster1 = self.P_reg(self.norm_pos, 2)
    self.force_thurster1 = self.force_thruster1*10
    self.force_thruster2 = self.force_thruster1

    self.force_servo_angle = 50
    if self.force_thruster1 < 0.0 :
      self.thrust_dir1 = 1
    elif self.force_thruster1 > 0.0 :
      self.thrust_dir1 = 2
    #--------------- Heading ---------------------------
    #self.delta_heading = self.dynamic_heading - self.heading
    #self.pid_reg_heading = self.PIDhold(self.delta_heading)

    #--------------- Thruster allocator ----------------

    self.thrust_dir2 = self.thrust_dir1
    self.thruster(self.force_thruster1,self.force_thruster2,self.thrust_dir1,self.thrust_dir2)                                 # publishes the thruster force

    print(self.force_thruster1)



    if self.thrust_dir1 == 1:
        self.right_thruster_cmd_msg.data = -(self.force_thruster1*1.5)
        self.left_thruster_cmd_msg.data = -(self.force_thruster2*1.5)

    else:
        self.right_thruster_cmd_msg.data = (self.force_thruster1*1.5)
        self.left_thruster_cmd_msg.data = (self.force_thruster2*1.5)

    self.right_thruster_cmd_pub.publish(self.right_thruster_cmd_msg)
    self.left_thruster_cmd_pub.publish(self.left_thruster_cmd_msg)



if __name__ == '__main__':
    rospy.init_node('dp_node')
    """while not rospy.is_shutdown():
      dp = Dynamic_pos()
      dp.dynamic_wp()
      dp.regulation()"""

    dp = Dynamic_pos()

    rospy.spin()
