#!/usr/bin/env python3

""" special teleop node to control ant movement """

import rospy 

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class TeleopControl:
    def __init__(self):
        rospy.init_node('ant_joy_teleop')
        # numbers of channels to use for control tasks
        self.channel_lr = rospy.get_param('~channel_lr', 0)
        self.channel_fb = rospy.get_param('~channel_fb', 1)
        self.channel_rotation = rospy.get_param('~channel_rotation', 2)
        
        # coeffitients for each channel
        self.coeff_lr = rospy.get_param('~coeff_lr', 0.4)
        self.coeff_fb = rospy.get_param('~coeff_fb', 0.5)
        self.coeff_rotation = rospy.get_param('~coeff_rotation', 0.5)
        
        self.button_enable = rospy.get_param('~button_enable', 0)
        
        # create ROS infrastructure
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.out_msg = Twist()
        self.sub = rospy.Subscriber('joy', Joy, self.cb_joy)
    
    def cb_joy(self, msg):
        # allow processing if button is absent; skip processing if button is not pressed
        if len(msg.buttons) > self.button_enable and not msg.buttons[self.button_enable]:
            self.out_msg.linear.x = 0.
            self.out_msg.linear.y = 0.
            self.out_msg.angular.z = 0.
        else:
            # calculate parameters of output
            self.out_msg.linear.x = (0. 
                                     if len(msg.axes) <= self.channel_fb 
                                     else msg.axes[self.channel_fb] * self.coeff_fb)
            self.out_msg.linear.y = (0. 
                                     if len(msg.axes) <= self.channel_lr 
                                     else msg.axes[self.channel_lr] * self.coeff_lr)
            self.out_msg.angular.z = (0. 
                                      if len(msg.axes) <= self.channel_rotation 
                                      else msg.axes[self.channel_rotation] * self.coeff_rotation)
        #rospy.logerr('Values: {:.3f} {:.3f} {:.3f}'.format(self.out_msg.linear.x, 
        #                                                   self.out_msg.linear.y,
        #                                                   self.out_msg.angular.z))
        self.pub.publish(self.out_msg)
    
def main():
    try:
        tc = TeleopControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    


if __name__ == '__main__':
    main()
