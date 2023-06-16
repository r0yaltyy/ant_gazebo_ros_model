import math

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

from std_msgs.msg import Float64MultiArray

from ant_model_control.limb_description import LimbDescr
from ant_model_control.geometry_primitives import angle_diff
from ant_model_control.robot_geometry_loader import load_robot_geometry

def gen_walking_template(dx, dy, height_down, height_up,
                         step_dist, step_dy):
    p1 = (dx, dy, -height_down)
    p2 = (dx - step_dist*0.5, dy - step_dy*0.5, -height_down)
    p3 = (dx - step_dist*0.5, dy - step_dy*0.5, -height_up)
    p4 = (dx + step_dist*0.5, dy + step_dy*0.5, -height_up)
    p5 = (dx + step_dist*0.5, dy + step_dy*0.5, -height_down)
    return [p1, p2, p2, p3, p4, p5, p5, p1]

class AntDescr(object):
    def __init__(self, descr_str, step_dist, cycle_time):
        
        # load data from description
        descr = load_robot_geometry(descr_str, True)
        self._generate_limbs(*descr)
        
        # initialize limbs
        #possible_pos = ['forward', 'middle', 'backward']
        #possible_sides = ['left', 'right']
        #self.limbs = {(p,s) : LimbDescr(p,s,4)
        #    for p in possible_pos
        #    for s in possible_sides}
        
        # constants
        self.leg_y = 0.007
        self.body_height_down = 0.003
        self.body_height_up = 0.001
        self.step_dist = step_dist # 0.0003
        self.cycle_time = cycle_time
        self.step_diff = 0.
        
        self.dx_zero = {'forward': 0.003,
                        'middle': 0.,
                        'backward': -0.004}
        self.dy_zero = {'forward': 0.,
                        'middle': 0.00,
                        'backward': 0.0}
        
        # state variables
        self.anim_plan = {}
        self.animation_start_time = None
        
        # subscriber for parameters storage
        self.param_sub = rospy.Subscriber('params', Float64MultiArray, self.cb_params)
        
        # subscriber to update joint states
        self.j_st_sub = rospy.Subscriber('joint_states', JointState, self.cb_j_st)
        
        # subscribe to velocity command
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cb_cmd_vel)
        
        # timer
        self.anim_timer = rospy.Timer(rospy.Duration(0.01), self.cb_anim_timer)

    def cb_params(self, msg):
        #rospy.logerr('Data received: {}'.format(msg.data))
        self.leg_y = msg.data[0]
        self.body_height_down = msg.data[1]
        self.body_height_up = msg.data[2]
        self.step_dist = msg.data[3]
        self.cycle_time = msg.data[4]
        self.step_diff = msg.data[5]
        self.dx_zero['forward'] = msg.data[6]
        self.dx_zero['middle'] = msg.data[7]
        self.dx_zero['backward'] = msg.data[8]
        self.dy_zero['forward'] = msg.data[9]
        self.dy_zero['middle'] = msg.data[10]
        self.dy_zero['backward'] = msg.data[11]
        

    def cb_cmd_vel(self, msg):
        """ change animation according to velocities """
        for (pos, side), limb in self.limbs.items():
            # calculate disps
            dx = self.dx_zero[pos]
            dy = self.dy_zero[pos] + self.leg_y
            if side == 'right':
                dy = -dy
            # shifts
            sh_x = msg.linear.x * self.step_dist
            sh_y1 = msg.linear.y * self.step_dist
            sh_y2 = msg.angular.z * self.step_dist
            if pos == 'middle':
                sh_y2 = 0.
            elif pos == 'backward':
                sh_y2 = -sh_y2
            # rotation of legs
            sh_x2 = msg.angular.z * self.step_dist if side == 'right' else -msg.angular.z * self.step_dist
            # get positions
            poses = gen_walking_template(dx, dy,
                                         self.body_height_down, self.body_height_up,
                                         sh_x + sh_x2, sh_y1 + sh_y2)
            #rospy.logwarn('{},{}: {} {}'.format(pos, side, sh_x + sh_x2, sh_y1 + sh_y2))
            get_up_time = self.cycle_time * 0.25
            move_time = self.cycle_time * 0.5
            get_down_time = self.cycle_time * 0.25
            time_moments = [0.,
                        move_time/2, 
                        move_time/2 + get_down_time, 
                        move_time/2 + get_down_time + get_up_time,
                        3*move_time/2 + get_down_time + get_up_time,
                        3*move_time/2 + get_up_time + 2*get_down_time,
                        3*move_time/2 + 2*get_up_time + 2*get_down_time,
                        2*(move_time + get_up_time + get_down_time)]

            
            # add animation
            self.add_animation_description(pos, side, 'walk',
                                           time_moments,
                                           poses)
        self.walk()

    def _generate_limbs(self, first_parts_pos, limb_parts_poses):
        self.limbs = {}
        # for all known limbs
        for (side, pos), (x, y, z, yaw) in first_parts_pos.items():
            # generate limb description:
            # [dx, dy, origin_dir, *lengths]
            limb_descr = [x, y, yaw]
            # add length of each limb to description
            for p_len in limb_parts_poses[(side, pos)]:
                limb_descr.append(p_len)
            self.limbs[(pos, side)] = LimbDescr(pos, side, 
                                                len(limb_parts_poses[(side, pos)]),
                                                limb_descr)
    
    def add_animation_description(self, pos, side, name, moments, poses):
        self.limbs[(pos,side)].add_animation_description(name, moments, poses)

    def plan_animation_execution_loop(self, pos, side, name, start_moment):
        self.anim_plan[(pos, side)] = (name, start_moment, None)

    def plan_animation_execution_num(self, pos, side, name, start_moment, ntimes):
        self.anim_plan[(pos, side)] = (name, start_moment, ntimes)

    def prepare_for_animation(self):
        self.generate_get_up_animation()
        self.generate_walking_animation()

    def start_animations(self):
        self.animation_start_time = rospy.Time.now()
   
    def finish_animations(self):
        while self.anim_plan:
            rospy.sleep(0.1)

    def execute_animations(self, curr_time):
        if self.animation_start_time is not None:
            time_from_start = (curr_time - self.animation_start_time).to_sec()
            anims_to_stop = []
            for limb in self.limbs.keys():
                if limb in self.anim_plan.keys():
                    name, start_moment, number_of_times = self.anim_plan[limb]
                    # if number_of_times is None or animation is not yet finished, execute it
                    duration = self.limbs[limb].get_animation_duration(name)
                    if number_of_times is None or time_from_start < number_of_times*duration:
                        self.limbs[limb].execute_animation(name, start_moment, time_from_start)
                    else:
                        # stop animation
                        anims_to_stop.append(limb)
            # drop old animations
            for ats in anims_to_stop:
                del self.anim_plan[ats]

    def cb_j_st(self, msg):
        # update all joints (i.e. all limbs)
        for k in self.limbs.keys():
            self.limbs[k].update_current_pos(msg)
        
    def cb_anim_timer(self, e):
        curr_time = rospy.Time.now()
        # update animation commands
        self.execute_animations(curr_time)
        # execute commands
        for k in self.limbs.keys():
            self.limbs[k].process(curr_time)

    def generate_get_up_animation(self):
        """ getting up """
        time_moments = [0., 1., 2.]
        # for each limb generate motions
        for (pos,side), limb in self.limbs.items():
            # calculate disps
            dx = self.dx_zero[pos]
            dy = self.dy_zero[pos] + self.leg_y
            if side == 'right':
                dy = -dy
            poses = [limb.fk_local(0.,0.,0.)]
            # move legs to body
            #p1 = limb.ik_local(dx, dy, 0)
            p1 = (dx, dy, 0)
            #rospy.logerr('{},{}: {}'.format(pos, side, p1))
            #exit(-1)
            poses.append(p1)
            # move body up
            #p2 = limb.ik_local(dx, dy, -self.body_height_down)
            p2 = (dx, dy, -self.body_height_down)
            poses.append(p2)
            # add animation
            self.add_animation_description(pos, side, 'get_up',
                                           time_moments,
                                           poses)
    
    def generate_walking_animation(self):
        """ step movements """
        get_up_time = self.cycle_time * 0.25
        move_time = self.cycle_time * 0.5
        get_down_time = self.cycle_time * 0.25
        time_moments = [0.,
                        move_time/2, 
                        move_time/2 + get_down_time, 
                        move_time/2 + get_down_time + get_up_time,
                        3*move_time/2 + get_down_time + get_up_time,
                        3*move_time/2 + get_up_time + 2*get_down_time,
                        3*move_time/2 + 2*get_up_time + 2*get_down_time,
                        2*(move_time + get_up_time + get_down_time)]
        for (pos, side), limb in self.limbs.items():
            # calculate disps
            dx = self.dx_zero[pos]
            dy = self.dy_zero[pos] + self.leg_y
            if side == 'right':
                dy = -dy
            # get positions
            poses = gen_walking_template(dx, dy,
                                         self.body_height_down, self.body_height_up,
                                         0., 0.)
            # add animation
            self.add_animation_description(pos, side, 'walk',
                                           time_moments,
                                           poses)
            
    
    def get_up(self):
        self.plan_animation_execution_num('forward', 'left', 'get_up', 0., 1)
        self.plan_animation_execution_num('middle', 'left', 'get_up', 0., 1)
        self.plan_animation_execution_num('backward', 'left', 'get_up', 0., 1)
        self.plan_animation_execution_num('forward', 'right', 'get_up', 0., 1)
        self.plan_animation_execution_num('middle', 'right', 'get_up', 0., 1)
        self.plan_animation_execution_num('backward', 'right', 'get_up', 0., 1)
        # process movements
        self.start_animations()
        self.finish_animations()
        
    def walk(self):
        self.plan_animation_execution_loop('forward', 'left', 'walk', 0.)
        self.plan_animation_execution_loop('middle', 'left', 'walk', self.cycle_time)
        self.plan_animation_execution_loop('backward', 'left', 'walk', 0.)
        self.plan_animation_execution_loop('forward', 'right', 'walk', self.cycle_time)
        self.plan_animation_execution_loop('middle', 'right', 'walk', 0.)
        self.plan_animation_execution_loop('backward', 'right', 'walk', self.cycle_time)
        self.start_animations()
 
