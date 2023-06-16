# class for motion control of ant robot. Uses two triangles to process ends of legs  

import rospy

class BitriangularController(object):
    def __init__(self, lf_pos, lm_pos, lb_pos, height, step_distance):
        """ lX_pos --- 2D-coordinates of legs endpoints in body frame """
        self.lf_zero_pos = lf_pos
        self.lm_zero_pos = lm_pos
        self.lb_zero_pos = lb_pos
        self.zero_height = height
        self.step_distance = step_distance
        
        self.leg_raising_part = 0.05
        self.leg_lowering_part = 0.05
        
        self.vel_x = 0
        self.vel_y = 0
        self.rot_z = 0
        self.last_command_time = rospy.Time.now()

    def set_velocity(self, vx, vy, rz):
        self.vel_x = vx
        self.vel_y = vy
        self.rot_z = rz
        
    def get_commands(self, current_time):
        """ send commands to all executive devices (legs) """
        # recalculate cycle parameters
        # recalculate triangles
        pass
        
