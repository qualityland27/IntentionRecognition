import argparse
import rospy
from std_msgs.msg import String
import actionlib
import time

from franka_msgs.msg import ErrorRecoveryActionGoal, ErrorRecoveryAction

class Helper:
    def __init__(self):
        topic = 'franka_control/error_recovery/goal'
        self.error_recovery_publisher = rospy.Publisher(topic, ErrorRecoveryActionGoal, queue_size=5)
        

    def get_commandline_args(self):
        parser = argparse.ArgumentParser(description='Process some integers.')
        parser.add_argument('-object_detection', type=bool, default=False,
                            help='an boolean to determine to use object angle in path planning')

        args = parser.parse_args()

        return args

    def recover_from_error(self):
        '''
        Send Action to Recover from Error.
        '''
        client = actionlib.SimpleActionClient("franka_control/error_recovery", ErrorRecoveryAction)
        client.wait_for_server()
        goal = ErrorRecoveryActionGoal()
        goal.header.stamp.secs = 0
        goal.header.stamp.nsecs = 0
        goal.header.frame_id = ''
        goal.goal_id.stamp.secs = 0
        goal.goal_id.stamp.nsecs = 0
        goal.goal_id.id = ''
        goal.goal = {}

        time.sleep(2)
        client.send_goal(goal)
