import math

import rospy

from ant_model_control.joint_description import JointDescr
from ant_model_control.geometry_primitives import angle_diff

class LimbAnimation(object):
    
    def __init__(self, name, moments, poses):
        assert(len(moments) == len(poses))
        assert(len(moments) > 2)
        # TODO add extra checks
        self.name = name
        self.moments = moments
        self.source_poses = poses
        self.actual_poses = poses[:]
    
    def get_pose(self, phase):
        if phase <= self.moments[0]:
            return self.actual_poses[0]
        if phase >= self.get_duration():
            return self.actual_poses[-1]
        
        for i in range(len(self.moments) - 1):
            if self.moments[i+1] > phase:
                # interpolate position from moments i and i+1
                coeff = (phase - self.moments[i]) / (self.moments[i+1] - self.moments[i])
                return [p1 * (1 - coeff) + p2*coeff 
                        for p1,p2 in zip(self.actual_poses[i], self.actual_poses[i+1])]
        return self.actual_poses[-1]
    
    def get_duration(self):
        return self.moments[-1]
    
class LimbDescr(object):
    def __init__(self, pos, side, j_num, geom_params):
        self.pos = pos
        self.side = side
        self.geom_params = geom_params
        # create joints
        self.joints = [JointDescr(pos, side, i)
                       for i in range(1, j_num + 1)]
        self.animations = {}
    
    def set_anim_velocity(self, anim_name, vx, vy, wz):
        if anim_name not in self.animations.keys():
            rospy.logerr('ant_motion_controller: animation {} not found in limb {}_{}'
                         .format(anim_name, self.side, self.pos))
            return
        self.animations[name].set_velocity(vx, vy, wz)
    
    def set_limb_pos(pos_list, duration, curr_time):
        for jnt,pos in zip(self.joints,pos_list):
            jnt.set_goal_pos(pos, duration, curr_time)
        
    def process(self, curr_time):
        for jnt in self.joints:
            jnt.process(curr_time)
            
    def add_animation_description(self, name, moments, poses):
        self.animations[name] = LimbAnimation(name, moments, poses)
        
    def update_current_pos(self, j_st_msg):
        for jnt in self.joints:
            jnt.update_current_pos(j_st_msg)

    def execute_animation(self, name, start_moment, time_from_start):
        # get animation description
        if name not in self.animations.keys():
            rospy.logerr('ant_motion_controller: data abount animation {} is absent in limb {}_{}'
                         .format(name, self.side, self.pos))
            return
        animation = self.animations[name]
        phase = (time_from_start - start_moment) % animation.get_duration()
        dx, dy, z = animation.get_pose(phase)
        goal_pose = self.ik_local(dx, dy, z)
        # send goals to joints
        for i,jd in enumerate(self.joints):
            jd.set_goal_pos(goal_pose[i],0,None)
    
    def ik_local(self, dx, dy, z, debug_print = False):
        """ calculate angles for inverse kinematics
        with positions relative to base of the limb """
        _,_,ori,l1,l2,*l3s = self.geom_params
        l3 = sum(l3s)
        ori_target = math.atan2(dy,dx)
        # angle for 1st joint
        j1 = angle_diff(ori_target, ori)
        rad = math.hypot(dx, dy) - l1
        # solve ik task: side1 = l2, side2 = l3, side3:
        side3 = math.hypot(rad, z)
        if debug_print:
            rospy.logerr('ori: {:.3f}, j1: {:.3f}, rad: {:.3f}, l2: {:.3f}, l3: {:.3f}'
                         .format(ori_target, j1, rad, l2, l3))
        try:
            j2 = math.acos((side3**2 + l2**2 - l3**2) / (2*side3*l2)) + math.atan2(z, rad)
        except ValueError:
            rospy.logerr('ant_motion_controller: too big angle in j2 for limb ({},{}) in position {:.5f}, {:.5f}, {:.5f}'
                         .format(self.pos, self.side, dx, dy, z))
            j2 = 0.
        try:
            j3 = math.acos((l2**2 + l3**2 - side3**2) / (2*l2*l3))
        except ValueError:
            rospy.logerr('ant_motion_controller: too big angle in j3 for limb ({},{}) in position {:.5f}, {:.5f}, {:.5f}'
                         .format(self.pos, self.side, dx, dy, z))
            j3 = 0
        return (j1,-j2,math.pi - j3)

    def fk_local(self, a1, a2, a3):
        """ forward kinematics """
        _,_,ori,l1,l2,*l3s = self.geom_params
        l3 = sum(l3s)
        print('{} {}: {} {} {} {}'.format(self.pos, self.side, ori, l1, l2, l3))
        # vector of first part
        v1 = (math.cos(ori + a1), math.sin(ori+a1), 0.)
        p1 = (v1[0]*l1, v1[1]*l1, v1[2]*l1)
        # vector of second part
        c2 = math.cos(a2)
        s2 = math.sin(a2)
        v2 = (v1[0], v1[1]*c2 - v1[2]*s2, v1[1]*s2 + v1[2]*c2)
        p2 = (p1[0] + v2[0]*l2, p1[1] + v2[1]*l2, p1[2] + v2[2]*l2)
        # vector of third part
        c3 = math.cos(a3)
        s3 = math.sin(a3)
        v3 = (v2[0], v2[1]*c3 - v2[2]*s3, v2[1]*s3 + v2[2]*c3)
        p3 = (p2[0] + v3[0]*l3, p2[1] + v3[1]*l3, p2[2] + v3[2]*l3)
        # vector of fourth part
        return p3

    def get_animation_duration(self, name):
        if name in self.animations.keys():
            return self.animations[name].get_duration()
        else:
            return 0.
