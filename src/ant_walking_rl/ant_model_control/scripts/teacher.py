#!/usr/bin/env python3

import math
import random

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

from gazebo_msgs.msg import ModelState, ModelStates

class BaseLeariningAlgorithm(object):
    
    def __init__(self):
        rospy.init_node('learning_node')
        
        self.state_variables = [
        0.002, # self.leg_y = msg.data[0]
        0.0028, # self.body_height_down = msg.data[1]
        0.0022, # self.body_height_up = msg.data[2]
        0.0003, # self.step_dist = msg.data[3]
        1., # self.cycle_time = msg.data[4]
        0., # self.step_diff = msg.data[5]
        0.0011, # self.dx_zero['forward'] = msg.data[6]
        0., # self.dx_zero['middle'] = msg.data[7]
        -0.0017, # self.dx_zero['backward'] = msg.data[8]
        0., # self.dy_zero['forward'] = msg.data[9]
        0.0002, # self.dy_zero['middle'] = msg.data[10]
        0.0001# self.dy_zero['backward'] = msg.data[11]
        ]
        
        self.pos = (0, 0)
        self.start_pos = (0, 0)

        # subscribe on model position
        self.pos_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.cb_model_state)
        
        # connect with gazebo reset function
        self.reset_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
        # parameter publisher
        self.param_pub = rospy.Publisher('params', Float64MultiArray, queue_size=1)
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
    def cb_model_state(self, msg):
        if 'ant_model' in msg.name:
            ind = msg.name.index('ant_model')
            self.pos = (msg.pose[ind].position.x, msg.pose[ind].position.y)
        
    def reset_sim(self):
        ms = ModelState()
        ms.model_name = 'ant_model'
        ms.pose.position.z = 0.001
        ms.pose.orientation.w = 1
        self.reset_pub.publish(ms)
       
    def send_params(self):
        out_msg = Float64MultiArray()
        out_msg.data = self.state_variables
        self.param_pub.publish(out_msg)
        
    def set_velocity(self, velocity):
        out_msg = Twist()
        out_msg.linear.x = velocity
        self.vel_pub.publish(out_msg)
        
    def start_pos_measurement(self):
        self.start_pos = self.pos

    def end_pos_measurement(self):
        return math.hypot(self.pos[0] - self.start_pos[0],
                          self.pos[1] - self.start_pos[1])

       
    def gather_data(self):
        rospy.sleep(2.)
        cycles = [1., 0.8, 0.6, 0.4, 0.2, 0.1]
        dists = [0.0008, 0.0016, 0.0024]
        qtable = [0.] * (len(cycles) * len(dists))

        num_iter = 0
        alpha = 0.3
        while num_iter < 1000:
            if random.random() < alpha:
                variant = random.randrange(len(cycles) * len(dists))
            else:
                mv = max(qtable)
                max_inds = [i for i,x in enumerate(qtable) if x==mv]
                variant = random.choice(max_inds)
            # calculate params
            nc = variant % len(cycles)
            nd = variant // len(cycles)
            
            #self.state_variables[0] = 0.0025 if self.state_variables[0] < 0.002 else 0.0015
            #self.state_variables[4] = cycles[ind]
            self.state_variables[4] = cycles[nc]
            self.state_variables[3] = dists[nd]
            num_iter += 1
            self.send_params()
            self.reset_sim()
            rospy.sleep(0.1)
            self.set_velocity(1)
            rospy.sleep(0.1)
            self.start_pos_measurement()
            rospy.sleep(20.)
            dist = self.end_pos_measurement()
            # save results
            qtable[variant] = dist
            print('{} -> {}'.format(variant, dist))
            if num_iter % 10 == 0:
                print(qtable)
        
if __name__ == '__main__':
    try:
        bla = BaseLeariningAlgorithm()
        bla.gather_data()
    except rospy.ROSInterruptException:
        pass
