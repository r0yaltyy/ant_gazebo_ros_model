#!/usr/bin/env python3

import threading
import math
import rospy
from sensor_msgs.msg import Image
from gazebo_msgs.msg import LinkStates, ModelState
from cv_bridge import CvBridge

import cv2

class CSVCreator:
    
    def __init__(self):
        
        # constants
        self.camera_link = 'camera::camera_link'
        self.body_parts = ['ant_model::base_link',
                           'ant_model::head_end_link',
                           'ant_model::abdomen_end_link',
                           'ant_model::left_backward4_link',
                           'ant_model::left_forward4_link',
                           'ant_model::left_middle4_link',
                           'ant_model::right_backward4_link',
                           'ant_model::right_forward4_link',
                           'ant_model::right_middle4_link',
                           'ant_model::left_backward_end_link',
                           'ant_model::left_forward_end_link',
                           'ant_model::left_middle_end_link',
                           'ant_model::right_backward_end_link',
                           'ant_model::right_forward_end_link',
                           'ant_model::right_middle_end_link']
        self.body_name_parts = ['base',                           
                                'head',
                                'abdomen',
                                'left_bckwrd4', #1
                                'left_frwd4',  #2
                                'left_mid4',   #3
                                'right_bckwrd4',#4
                                'right_frwd4', #5
                                'right_mid4',
                                'left_bckwrd_end',
                                'left_frwd_end',
                                'left_mid_end',
                                'right_bckwrd_end',
                                'right_frwrd_end',
                                'right_mid_end']  
        
        # state variables
        self.image = None
        self.poses = None
        self.bridge = CvBridge()
        self.data_index = 0
        self.flag_read_pose = False
        
        self.pose_time = rospy.Time.now()
        self.image_time = rospy.Time.now()
        
        # mutex
        self.mutex = threading.Lock()
        self.cam_hfov = 1.047
        # create ROS infrastructure
        self.camera_pos_msg = ModelState()
        self.camera_pos_msg.model_name = 'camera'
        self.camera_pos_msg.pose.position.x = 0.015
        self.camera_pos_msg.pose.position.y = 0.015
        self.camera_pos_msg.pose.position.z = self.camera_pos_msg.pose.position.x / math.tan(self.cam_hfov / 2)
        self.camera_pos_msg.pose.orientation.x = 0.5
        self.camera_pos_msg.pose.orientation.y = -0.5
        self.camera_pos_msg.pose.orientation.z = -0.5
        self.camera_pos_msg.pose.orientation.w = -0.5
        self.camera_pos_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
        
        self.img_sub = rospy.Subscriber('/camera/camera_sensor/camera/image_raw', Image, self.cb_image)
        self.pos_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.cb_link_states)
        
        self.timer = rospy.Timer(rospy.Duration(1.), self.cb_timer)
    
    def cb_image(self, msg):
        ''' update input image '''
        self.mutex.acquire()
        self.image = msg
        self.flag_read_pose = True
        self.image_time = msg.header.stamp
        #rospy.logerr('Time difference: {}'.format((self.image_time - self.pose_time).to_sec()))
        self.mutex.release()
        
    def cb_link_states(self, msg):
        ''' update position info  '''
        if self.flag_read_pose:
            self.mutex.acquire()
            self.poses = msg
            self.flag_read_pose = False
            self.pose_time = rospy.Time.now()
            self.mutex.release()
        
    def cb_timer(self, _):
        ''' process data, save results '''
        # 1. move camera, wait
        self.camera_pos_pub.publish(self.camera_pos_msg)
        rospy.sleep(0.2)
        
        self.mutex.acquire()
        # 2. process position data, save position
        # find camera description
        if self.poses is None or self.camera_link not in self.poses.name:
            self.mutex.release()
            return
        cam_index = self.poses.name.index(self.camera_link)
        cam_pose = self.poses.pose[cam_index]
        cam_data = [cam_pose.position.x, cam_pose.position.y, cam_pose.position.z,
                    cam_pose.orientation.x, cam_pose.orientation.y,
                    cam_pose.orientation.z, cam_pose.orientation.w]
        #rospy.logerr(cam_data)
        points_index = []
        points_pose = []
        points_data = []
        tmp = 0
        x, y = [],[]
        for i in self.body_parts:
            points_index.append(self.poses.name.index(i))
            points_pose.append(self.poses.pose[points_index[tmp]])
            x_real = points_pose[tmp].position.x
            y_real = points_pose[tmp].position.y
            H = self.camera_pos_msg.pose.position.z
            halfW = self.camera_pos_msg.pose.position.x
            scale = 500/(2*halfW)
            if points_pose[tmp].position.x > halfW:
                u = ((H * (points_pose[tmp].position.x - halfW))/(H - points_pose[tmp].position.z))
                x_real = (u + halfW)
            if points_pose[tmp].position.x < halfW and points_pose[tmp].position.x > 0: 
                u = ((H * (halfW - points_pose[tmp].position.x))/(H - points_pose[tmp].position.z))
                x_real = (halfW - u)
            
            if points_pose[tmp].position.y > halfW:
                u = ((H * (points_pose[tmp].position.y - halfW))/(H - points_pose[tmp].position.z))
                y_real = (u + halfW)
            if points_pose[tmp].position.y < halfW and points_pose[tmp].position.y > 0: 
                u = ((H * (halfW - points_pose[tmp].position.y))/(H - points_pose[tmp].position.z))
                y_real = (halfW - u)
            
            
            points_data.append([self.body_name_parts[tmp], int(scale * x_real), int(scale * y_real)])
            x.append(int(scale * x_real))
            y.append(int(scale * y_real))
            tmp = 1 + tmp                
            
        # gather all other data
        # TODO
        with open('/home/alex/data_ant/gathered_data.csv', 'at') as f:
            data_str = ''
            for i in range(len(self.body_name_parts)):
                data_str += ';'.join([str(v) for v in points_data[i]])+ ';'
            data_str += '\n'
            f.write(data_str)
       
        # 3. save image
        cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding='passthrough')
        cv2.imwrite(f'/home/alex/data_ant/data{self.data_index}.png', cv_image)
        for i in range(len(self.body_name_parts)):
            if x[i] > 0 and x[i] < 500 and y[i] > 0 and y[i] < 500:
                cv2.circle(cv_image, (x[i], 500 - y[i]), 1, (0,0,255), 1)
        cv2.imwrite(f'/home/alex/data_ant/draw/data{self.data_index}.png', cv_image)
        self.data_index+= 1
        self.mutex.release()

if __name__ == '__main__':
    try:
        rospy.init_node('csv_creator')
        cc = CSVCreator()
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        pass
