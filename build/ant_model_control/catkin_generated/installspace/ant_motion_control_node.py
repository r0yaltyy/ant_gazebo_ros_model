#!/usr/bin/env python3

# license removed for brevity

import rospy

from ant_model_control.ant_description import AntDescr

if __name__ == '__main__':
    try:
        rospy.init_node('ant_motion_controller', anonymous=True)
        # ant description
        ant_descr = rospy.get_param('robot_description')
        
        step_dist = rospy.get_param('~step_dist')
        cycle_time = rospy.get_param('~cycle_time')
        ad = AntDescr(ant_descr, step_dist, cycle_time)
        
        ad.prepare_for_animation() # generate poses
        ad.get_up()
        ad.walk()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        

