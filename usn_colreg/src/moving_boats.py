#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelState
import numpy as np

class MovingBoat:
    def __init__(self, model_name, start_pos, end_pos, orientation, steps):
        self.pub_state = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 10)
        self.model_state = ModelState()
        self.model_state.model_name = model_name
        self.start_pos = np.array(start_pos)
        self.end_pos = np.array(end_pos)
        self.steps = steps
        self.orientation = orientation
        self.rate = rospy.Rate(50)
        self.i = 0
        self.position = self.model_state.pose.position
        self.position.x, self.position.y, self.position.z = self.start_pos[0], self.start_pos[1], self.start_pos[2]
    
    def move_boat(self):

        orientation = self.model_state.pose.orientation
        orientation.x, orientation.y, orientation.z, orientation.w = self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3]

        delta_pos = self.end_pos-self.start_pos
        increment = delta_pos/self.steps
        
        if self.i < self.steps:
            self.position.x = self.position.x + increment[0]
            self.position.y = self.position.y + increment[1]
            self.position.z = self.position.z + increment[2]
            self.model_state.pose.position = self.position 
            self.model_state.pose.orientation = orientation
            self.pub_state.publish(self.model_state)
            self.i += 1
            self.rate.sleep()
        else:
            self.model_state.pose.position = self.position 
            self.model_state.pose.orientation = orientation
            self.pub_state.publish(self.model_state)



if __name__ == '__main__':
    rospy.init_node("moving_boats")
    boat1 = MovingBoat('boat1', [40, -15, 0.25], [40, 15, 0.25], [0, 0, 0.707, 0.707], 500)
    boat2 = MovingBoat('boat2', [50, 15, 0.25], [50, -15, 0.25], [0, 0, -0.707, 0.707], 1000)

    while not rospy.is_shutdown():
        for i in range(2):
            boat1.move_boat()
            boat2.move_boat()

