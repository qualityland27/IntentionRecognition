from geometry_msgs.msg import Pose
from panda_intention_reaction.msg import joint_msgs


class PandaPositions:
    '''
    This class stores positins.
    '''
    def __init__(self):
        self.curr_wrist = Pose()
        self.curr_wrist.position.x = 0
        self.curr_wrist.position.y = 0
        self.curr_wrist.position.z = 0

        self.time_of_curr_wrist = None

        self.std_ori_x = 0.924550513753
        self.std_ori_y = -0.380570191172
        self.std_ori_z = 0.0190297249452
        self.std_ori_w = 0.00324756497727

        self.gripping_ori_x = -0.620654050925
        self.gripping_ori_y = 0.347337538545
        self.gripping_ori_z = 0.193769022416
        self.gripping_ori_w = 0.675720910835      

        self.curr_pos = None

    def high_pose(self):
        pose = Pose()
        pose.position.x = 0.5
        pose.position.y = 0.0
        pose.position.z = 0.8

        pose.orientation.x = self.std_ori_x
        pose.orientation.y = self.std_ori_y
        pose.orientation.z = self.std_ori_z
        pose.orientation.w = self.std_ori_w

        return pose
    
    def mid_pose(self):
        pose = Pose()
        pose.position.x = 0.5
        pose.position.y = 0.0
        pose.position.z = 0.6

        pose.orientation.x = self.std_ori_x
        pose.orientation.y = self.std_ori_y
        pose.orientation.z = self.std_ori_z
        pose.orientation.w = self.std_ori_w

        return pose

    def lower_pose(self):
        pose = Pose()
        pose.position.x = 0.5
        pose.position.y = 0.0
        pose.position.z = 0.3

        pose.orientation.x = self.std_ori_x
        pose.orientation.y = self.std_ori_y
        pose.orientation.z = self.std_ori_z
        pose.orientation.w = self.std_ori_w

        return pose

    def unload_pose(self):
        pose = Pose()
        pose.position.x = 0.5
        pose.position.y = -0.25
        pose.position.z = 0.4

        pose.orientation.x = self.std_ori_x
        pose.orientation.y = self.std_ori_y
        pose.orientation.z = self.std_ori_z
        pose.orientation.w = self.std_ori_w

        return pose

    def wait_for_interaction_pose(self):
        pose = Pose()
        pose.position.x = 0.2584
        pose.position.y = -0.02
        pose.position.z = 0.3716

        pose.orientation.x = self.std_ori_x
        pose.orientation.y = self.std_ori_y
        pose.orientation.z = self.std_ori_z
        pose.orientation.w = self.std_ori_w

        return pose
    
    def wait_for_interaction_joint_pose(self):
        joint_pose = joint_msgs()
        joint_pose.panda_joint1 = -0.00284461351785
        joint_pose.panda_joint2 = -0.949317806779
        joint_pose.panda_joint3 = -0.0816787352638
        joint_pose.panda_joint4 = -3.00613674935
        joint_pose.panda_joint5 = -0.085287011077
        joint_pose.panda_joint6 = 2.07448632481
        joint_pose.panda_joint7 = 0.831856744984

        return joint_pose
