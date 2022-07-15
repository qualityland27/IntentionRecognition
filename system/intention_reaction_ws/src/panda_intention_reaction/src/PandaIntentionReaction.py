#!/usr/bin/env python

from threading import current_thread
import rospy
import actionlib
import time
import sys
import moveit_commander
import numpy

from panda_intention_reaction.msg import joint_msgs, object_msgs, class_and_position_msgs
from panda_intention_reaction.srv import  poseMovementResponse
from geometry_msgs.msg import Pose, PoseStamped
from franka_gripper.msg import GraspAction, GraspGoal, MoveGoal, MoveAction
from std_msgs.msg import Bool

import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from positions import PandaPositions
from security import PandaMovementSecurity
from PublishArray import PublishArrayClass
from objects import detectedObjects
from helper import Helper
import copy

# this node has one topic which sends the current position and pose orientation of the robot
# and one service to send the robot to a position
group = ""
publishArrayInstance = PublishArrayClass()

def intentionRecognition():
    # Get Commandline Args
    global helper
    helper = Helper()
    global args
    args = helper.get_commandline_args()

    # Recover from Error 
    helper.recover_from_error()
    rospy.loginfo("============ Recovered from Error ")

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()

    robot_is_busy = False

    scene = moveit_commander.PlanningSceneInterface()
    group_name = "panda_arm"
    global group
    group = moveit_commander.MoveGroupCommander(group_name)
    
    gripper_group_name = "hand"
    global gripper_group
    gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name)

    group.set_max_acceleration_scaling_factor(0.8)    
    group.set_max_velocity_scaling_factor(0.9)
    
    #velocity = group.get_known_constraints()
    #rospy.loginfo("============ Velocity: %s" % velocity)
    
    group.set_num_planning_attempts(3)
    group.set_planning_time(3)
    
    #group.set_goal_position_tolerance(0.005)

    #group.set_goal_orientation_tolerance(0.01)

    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()

    # **Assuming /tf2 topic is being broadcasted
    global tf_buffer
    tf_buffer = tf2_ros.Buffer()
    global listener
    listener = tf2_ros.TransformListener(tf_buffer)

    global positions 
    positions = PandaPositions()
    global movement_security
    movement_security = PandaMovementSecurity()
    global detected_object
    detected_object = detectedObjects()

    rospy.loginfo("============ Reference frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    rospy.loginfo("============ End effector: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    rospy.loginfo("============ Robot Groups: %s", robot.get_group_names())

    # Initialize PublishArray Class
    publishArrayInstance = PublishArrayClass()
    rospy.loginfo("============ PublishArrayClass Instance Created")
    #print(type(publishArrayInstance))

    # Add collision Objects to Planning Scene
    scene, planning_frame = movement_security.add_collision_objects(scene, planning_frame, table=True, camera=True, desktops=True)
    #add_collision_objects(scene, planning_frame, table=True, camera=True, desktops=True)
    rospy.loginfo("============ Added Collision Objects")
    rospy.sleep(0.5)

    #### Waiting Position
    rospy.loginfo("============ Initializing Robot Position")
    moveRobotcontinuous_withinWS(positions.mid_pose())
    move_to_joint_pos(positions.wait_for_interaction_joint_pose())
    time.sleep(1)

    ### Publisher ### 
    pub = rospy.Publisher('intention_recognition/PosePublisher', Pose, queue_size=10)
    pubJoint = rospy.Publisher('intention_recognition/JointPosePublisher', joint_msgs, queue_size=10)
    #move_service = rospy.Service('intention_recognition/move_panda_robot', poseMovement, moveRobot)

    ###Topics###
    rospy.Subscriber('intention_recognition/intentionPublisher', class_and_position_msgs, intention_reaction, callback_args=robot_is_busy, queue_size=1)
    rospy.Subscriber('intention_recognition/wristPublisher', class_and_position_msgs, log_current_position, queue_size=1)
    rospy.Subscriber('intention_recognition/objectPublisher', object_msgs, log_object, queue_size=1)
    rospy.Subscriber('intention_recognition/PoseConverterFromPose', Pose, move_to_position_from_camera)
    rospy.Subscriber('intention_recognition/PoseListener', Pose, moveRobotcontinuous_withinWS)
    rospy.Subscriber('intention_recognition/recovery_gesture_publisher', Bool, recover_from_error, queue_size=1)
    
    rospy.loginfo("InteractionReaction initialized")
    
    while not rospy.is_shutdown():
        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        # rospy.loginfo("============ Printing robot state")
        # rospy.loginfo(robot.get_current_state())

        current_pose = group.get_current_pose().pose
        joint_pose = joint_msgs()

        joint_values = group.get_current_joint_values()
        joint_pose.panda_joint1 = joint_values[0]
        joint_pose.panda_joint2 = joint_values[1]
        joint_pose.panda_joint3 = joint_values[2]
        joint_pose.panda_joint4 = joint_values[3]
        joint_pose.panda_joint5 = joint_values[4]
        joint_pose.panda_joint6 = joint_values[5]
        joint_pose.panda_joint7 = joint_values[6]

        pub.publish(current_pose)
        pubJoint.publish(joint_pose)

        rate.sleep()

def stopRobot(stopSignal):
    '''
    When the stop signal is received, the robot stops.
    '''
    rospy.loginfo("============ Robot should stop: ")
    group.stop()
    rospy.loginfo("Robot stopped successfully")

def recover_from_error(recovery_gesture):
    '''
    Recover from error and moving back to the wait position.
    
    :param recovery_gesture: Boolean, if True, the robot will recover from an error
    '''
    if recovery_gesture:
        # Recover from Error 
        group.clear_pose_targets()

        helper.recover_from_error()
        rospy.loginfo("============ Recovered from Error ")
        recovery_gesture = False

        group.set_max_acceleration_scaling_factor(0.1)    
        group.set_max_velocity_scaling_factor(0.1)

        rospy.loginfo("============ highFive: Return to wait pos")
        move_to_joint_pos(positions.wait_for_interaction_joint_pose())

        group.set_max_acceleration_scaling_factor(0.8)    
        group.set_max_velocity_scaling_factor(0.9)

def transform_pose(input_pose, from_frame, to_frame):
    '''
    Transform a pose from one frame to another.
    
    :param input_pose: The pose to be transformed
    :param from_frame: The frame of the pose you are transforming from
    :param to_frame: The frame to transform the pose to
    :return: A pose
    '''
    # **Assuming /tf2 topic is being broadcasted
    # tf_buffer = tf2_ros.Buffer()
    # listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    #print(pose_stamped.header.frame_id)
    pose_stamped.header.stamp = rospy.Time.now()

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise

def assign_to_pose(x, y, z):
    '''
    Assigns the x, y, and z values to a Pose object.
    
    :param x: the x coordinate of the camera
    :param y: the y-coordinate of the camera
    :param z: distance from the camera to the object
    :return: The pose of the camera
    '''
    cameraPose = Pose()
    cameraPose.position.x = float(x)
    cameraPose.position.y = float(y)
    cameraPose.position.z = float(z)

    return cameraPose


def log_object(center_width_height_angle):
    '''
    This function takes info about detected object and saves this info into local object.
    
    :param center_width_height_angle: The center of the object, its width, its height, and its angle
    '''
    object = object_msgs()
    object = center_width_height_angle

    detected_object.update(object)
   

def log_current_position(x_y_z):
    '''
    This function takes info about current wrist and saves this info into local object.
    '''

    ################################
    #         Split Input         #
    ################################
    class_and_position = class_and_position_msgs()
    class_and_position = x_y_z
    x, y, z, rospyTime = class_and_position.x, class_and_position.y, class_and_position.z, class_and_position.time 

    ################################
    #     Assign to Pose objekt    #
    ################################
    cameraPose = assign_to_pose(x, y, z)
    
    ################################
    #    Transform to world fram   #
    ################################
    worldPose = transform_pose(cameraPose, 'camera_color_optical_frame', 'panda_link0')

    # Assign positions to instance ob Positions Objekt
    positions.curr_wrist.position.x = worldPose.position.x
    positions.curr_wrist.position.y = worldPose.position.y
    positions.curr_wrist.position.z = worldPose.position.z
    positions.time_of_curr_wrist = rospyTime


def intention_reaction(x_y_z, robot_is_busy):
    '''
    This is the main function for defining the reaction based on the intention from intention_recognition.

    This function takes info about intention classification and current wrist 
    as well as info about whether robot is available.
    '''
    if not robot_is_busy:
        robot_is_busy = True

        #t_start = time.time()

        ################################
        #         Split String         #
        ################################
        #classification, x, y, z, rospyTime = split_incoming_string(x_y_z)
        class_and_position = class_and_position_msgs()
        class_and_position = x_y_z
        classification, x, y, z, rospyTime = class_and_position.classification, class_and_position.x, class_and_position.y, class_and_position.z, class_and_position.time 

        ################################
        #     Assign to Pose objekt    #
        ################################
        cameraPose = assign_to_pose(x, y, z)

        ################################
        #    Transform to world fram   #
        ################################
        worldPose = transform_pose(cameraPose, 'camera_color_optical_frame', 'panda_link0')

        # get Orientation from positions
        worldPose.orientation.x = positions.std_ori_x
        worldPose.orientation.y = positions.std_ori_y
        worldPose.orientation.z = positions.std_ori_z
        worldPose.orientation.w = positions.std_ori_w

        ################################
        #        Publish marker        #
        ################################
        publishArrayInstance.publishOneMarker(worldPose)

        # storage is used to make security check while robot is moving
        storage = Pose()
        storage = worldPose

        right_hand_pose = worldPose
        right_hand_pose.position.x = worldPose.position.x
        right_hand_pose.position.y = worldPose.position.y
        right_hand_pose.position.z = worldPose.position.z

        rospy.loginfo("============ Classification: " + str(classification))

        init_pos_in_tol = movement_security.positions_are_the_same(storage, positions.curr_wrist, tollerance=0.2)
        currentTime = rospy.get_time()
        
        if not init_pos_in_tol and currentTime - positions.time_of_curr_wrist < 0.5:
            rospy.loginfo("============ before movement: Failed Security Check ")
        else:
            pass    

        ################################
        #             Move             #
        ################################
        if right_hand_pose.position.x < 1.0 and init_pos_in_tol and currentTime - rospyTime < 1.5:
            if classification == "highFive":
                ################################
                #           highFive           # 
                ################################
                # First, bring Robot Hand up to the same height 
                right_hand_pose.position.x = right_hand_pose.position.x - 0.3
                right_hand_pose.position.z = right_hand_pose.position.z - 0.1

                rospy.loginfo("============ highFive: First Movement (-30 cm)")
                #t_movement = time.time()

                #print("Zeitstempel Versenden: {}".format(rospyTime))
                #print("Netzwerk: {} ".format(t_start - rospyTime))
                #print("Senden bis Movement: {}".format(t_movement - t_start))

                move_success = moveRobotcontinuous_withinWS(right_hand_pose)

                # Das ist komischerweise wichtig, da sonst der Vergleich nicht stimmt
                right_hand_pose.position.x = right_hand_pose.position.x + 0.3
                right_hand_pose.position.z = right_hand_pose.position.z + 0.1

                poses_within_tolerance = movement_security.positions_are_the_same(storage, positions.curr_wrist, tollerance=0.2)
                if move_success and poses_within_tolerance:
                    #### Second, Check In
                    rospy.loginfo("============ highFive: Passed Security Check")
                    rospy.loginfo("============ highFive: Second Movement (-15 cm)")
                    waypoints = []
                    right_hand_pose.position.x = positions.curr_wrist.position.x - 0.2
                    right_hand_pose.position.y = positions.curr_wrist.position.y
                    right_hand_pose.position.z = positions.curr_wrist.position.z - 0.1
                    waypoints.append(copy.deepcopy(right_hand_pose))

                    rospy.loginfo("============ highFive: Third Movement (-30 cm)")
                    right_hand_pose.position.x = positions.curr_wrist.position.x - 0.3
                    waypoints.append(copy.deepcopy(right_hand_pose))

                    #rospy.loginfo("============ highFive: Return to wait pos")
                    #waypoints.append(copy.deepcopy(positions.wait_for_interaction_pose()))

                    (plan, fraction) = group.compute_cartesian_path(
                                                    waypoints,   # waypoints to follow
                                                    0.01,        # eef_step
                                                    0.0)         # jump_threshold
                    
                    group.execute(plan, wait=True)
                    time.sleep(0.2)

                    #group.set_max_acceleration_scaling_factor(0.1)    
                    #group.set_max_velocity_scaling_factor(0.1)

                    rospy.loginfo("============ highFive: Return to wait pos")
                    move_to_joint_pos(positions.wait_for_interaction_joint_pose())

                    group.set_max_acceleration_scaling_factor(0.8)    
                    group.set_max_velocity_scaling_factor(0.9)
                    time.sleep(2.0)
                else:
                    rospy.loginfo("============ highFive: Failed Security Check")
                    rospy.loginfo("============ highFive: Return to wait pos")
                    move_to_joint_pos(positions.wait_for_interaction_joint_pose())
                    time.sleep(2.0)

            elif classification == "handOver":
                ################################
                #           handOver           #
                ################################
                
                objectTime = float(detected_object.time)
                if args.object_detection and currentTime - objectTime < 2.0:
                    angle = detected_object.angle - 90
                    #print("angle")
                    #print(angle)

                    if angle > - 90:
                        # von rechts
                        #angle = angle - 90
                        quaternion = quaternion_from_euler(numpy.deg2rad(-90.0), numpy.deg2rad(45.0), numpy.deg2rad(angle-90))

                        right_hand_pose.orientation.x = quaternion[0]
                        right_hand_pose.orientation.y = quaternion[1]
                        right_hand_pose.orientation.z = quaternion[2]
                        right_hand_pose.orientation.w = quaternion[3]

                        right_hand_pose.position.x = right_hand_pose.position.x - 0.15
                        right_hand_pose.position.y = right_hand_pose.position.y + 0.0
                        right_hand_pose.position.z = right_hand_pose.position.z
                    else:
                        # von links
                        #angle = angle + 90
                        quaternion = quaternion_from_euler(numpy.deg2rad(-90.0), numpy.deg2rad(45.0), numpy.deg2rad(angle+90))

                        right_hand_pose.orientation.x = quaternion[0]
                        right_hand_pose.orientation.y = quaternion[1]
                        right_hand_pose.orientation.z = quaternion[2]
                        right_hand_pose.orientation.w = quaternion[3]

                        right_hand_pose.position.x = right_hand_pose.position.x - 0.15
                        right_hand_pose.position.y = right_hand_pose.position.y - 0.3
                        right_hand_pose.position.z = right_hand_pose.position.z
                else:
                    # Immer gleich von links
                    right_hand_pose.orientation.x = positions.gripping_ori_x
                    right_hand_pose.orientation.y = positions.gripping_ori_y
                    right_hand_pose.orientation.z = positions.gripping_ori_z
                    right_hand_pose.orientation.w = positions.gripping_ori_w

                    right_hand_pose.position.x = right_hand_pose.position.x - 0.1
                    right_hand_pose.position.y = right_hand_pose.position.y - 0.3
                    right_hand_pose.position.z = right_hand_pose.position.z

                t_movement = time.time()

                rospy.loginfo("============ handOver: Arm Movement 1")
                move_success = moveRobotcontinuous_withinWS(right_hand_pose)
                time.sleep(0.1)

                # Das ist komischerweise wichtig, da sonst der Vergleich nicht stimmt
                if args.object_detection and currentTime - objectTime < 2.0:
                    if angle > - 90:
                        right_hand_pose.position.x = right_hand_pose.position.x + 0.15
                        right_hand_pose.position.y = right_hand_pose.position.y - 0.0
                    else:
                        right_hand_pose.position.x = right_hand_pose.position.x + 0.15
                        right_hand_pose.position.y = right_hand_pose.position.y + 0.3
                else:
                    right_hand_pose.position.x = right_hand_pose.position.x + 0.1
                    right_hand_pose.position.y = right_hand_pose.position.y + 0.3

                poses_within_tolerance = movement_security.positions_are_the_same(storage, positions.curr_wrist, tollerance=0.2)
                if move_success and poses_within_tolerance:
                    rospy.loginfo("============ handOver: Passed Security Check")
                    rospy.loginfo("============ handOver: Gripper Movement")
                    goal = MoveGoal(width = 0.02, speed = 1.0)
                    moveGripper(goal)
                    time.sleep(0.5)

                    rospy.loginfo("============ handOver: Arm Movement 2")
                    moveRobotcontinuous_withinWS(positions.unload_pose())

                    rospy.loginfo("============ handOver: Gripper Movement 2")
                    goal = MoveGoal(width = 0.1, speed = 1.0)
                    moveGripper(goal)
                    time.sleep(0.5)

                    rospy.loginfo("============ handOver: Return to wait pos")
                    move_to_joint_pos(positions.wait_for_interaction_joint_pose())
                    time.sleep(2.0)
                else:
                    rospy.loginfo("============ handOver: Failed Security Check")
                    rospy.loginfo("============ handOver: Return to wait pos")
                    move_to_joint_pos(positions.wait_for_interaction_joint_pose())
                    time.sleep(2.0)

            else:
                ################################
                #           negative           #
                ################################
                #rospy.loginfo("============ negative: Arm Movement")
                move_to_joint_pos(positions.wait_for_interaction_joint_pose())
                goal = MoveGoal(width = 0.1, speed = 1.0)
                moveGripper(goal)

        positions.curr_pos = classification
        robot_is_busy = False

def move_to_joint_pos(joint_pose):
    '''
    Move the robot to a specified joint position.
    
    :param joint_pose: The pose of the end-effector
    '''
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = joint_pose.panda_joint1
    joint_goal[1] = joint_pose.panda_joint2
    joint_goal[2] = joint_pose.panda_joint3
    joint_goal[3] = joint_pose.panda_joint4
    joint_goal[4] = joint_pose.panda_joint5
    joint_goal[5] = joint_pose.panda_joint6
    joint_goal[6] = joint_pose.panda_joint7

    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

def move_to_position_from_camera(cameraPose):
    '''
    This function takes in a pose from the camera frame and transforms it into the world frame.
    
    :param cameraPose: The pose of the camera in the world frame
    '''
    rospy.loginfo("============ Calculating Pose from camera coords into world coords!")
    rospy.loginfo(cameraPose)

    # Initialize PublishArray Class
    publishArrayInstance = PublishArrayClass()
    rospy.loginfo("============ PublishArrayClass Instance Created")

    worldPose = transform_pose(cameraPose, 'camera_color_optical_frame', 'panda_link0')
    currPose = group.get_current_pose().pose

    worldPose.orientation.x = currPose.orientation.x
    worldPose.orientation.y = currPose.orientation.y
    worldPose.orientation.z = currPose.orientation.z
    worldPose.orientation.w = currPose.orientation.w

    rospy.loginfo(worldPose)

    publishArrayInstance.publishArray(worldPose)

    moveRobotcontinuous_withinWS(worldPose)

def moveRobotcontinuous_withinWS(goalpose):
    '''
    This functions performs movement within secure cube.
    '''
    #rospy.loginfo("============ Robot should move to safe position within Workspace.")
    #rospy.loginfo(goalpose)

    pose_goal = goalpose
    group.set_start_state_to_current_state()
    group.set_pose_target(pose_goal)

    if movement_security.pose_within_ws(pose_goal):
        rospy.loginfo("============ Movement Security: Within cube")
        group.go(pose_goal, wait=True)
        #group.execute(path, wait=True)

        group.stop()
        group.clear_pose_targets()

        resp = poseMovementResponse("Done")

        return True

    else:
        rospy.logwarn("Movement Security: Robot will not move! Target is not within security cube.")
        #rospy.logwarn(msg, *args, **kwargs)

        return False

def moveGripper(gripper_command):
    #rospy.loginfo("============ Gripper should move: ")
    #rospy.loginfo(gripper_command)

    client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    client.wait_for_server()
    width = gripper_command.width
    speed = gripper_command.speed

    goal = MoveGoal(width, speed)
    client.send_goal(goal)
    
def graspGripper(gripper_command):
    rospy.loginfo("============ Gripper should move: ")
    rospy.loginfo(gripper_command)
    
    client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
    client.wait_for_server()
    width = gripper_command.width
    epsilon = gripper_command.epsilon
    speed = gripper_command.speed
    force = gripper_command.force
    goal = GraspGoal(width, epsilon, speed, force)
    client.send_goal(goal)

    rospy.loginfo("============ Gripper moved successfully")


if __name__ == '__main__':
    rospy.init_node('intention_interaction', anonymous=False)
    rate = rospy.Rate(1)

    try:
        intentionRecognition()
    except rospy.ROSInterruptException:
        pass
