import rospy
from std_msgs.msg import Float64

# class for a single joint
class JointDescr(object):
    def __init__(self, pos, side, num):
        # consts
        self.pos = pos
        self.side = side
        self.num = num
        
        self.joint_name = self.get_joint_name()
        
        # state
        self.goal = None
        self.curr_pos = 0.
        
        self.pub = rospy.Publisher(self.get_cmd_topic_name(), Float64, queue_size=10)
        
    def get_cmd_topic_name(self):
        return '{}_{}{}_position_controller/command'.format(self.side, self.pos, self.num)
    
    def get_joint_name(self):
        return '{}_{}{}_joint'.format(self.side, self.pos, self.num)
    
    def set_goal_pos(self, pos, duration, curr_time):
        self.goal = (self.curr_pos, pos, duration, curr_time)

    def update_current_pos(self, j_st_msg):
        try:
            ind = j_st_msg.name.index(self.joint_name)
        except:
            rospy.logerr('ant_model_control.joint_desrciption: data about joint {} not found in joint_states'
                         .format(self.joint_name))
            return
        self.curr_pos = j_st_msg.position[ind]

    def process(self, curr_time):
        """ move to the goal stated """
        if self.goal is None:
            return
        # get goal
        start_pos, goal_pos, duration, start_time = self.goal
        # process instant command separately
        if abs(duration) < 1e-2:
            pos = goal_pos
        else:
            time_from_start = (curr_time - start_time).to_sec()
            if time_from_start < duration - 1e-2: # process separately movements
                                                  # that are almost finished
                pos = time_from_start / duration * (goal_pos - start_pos) + start_pos
            else:
                pos = goal_pos
                
        # send message
        msg = Float64()
        msg.data = pos
        self.pub.publish(msg)
        
