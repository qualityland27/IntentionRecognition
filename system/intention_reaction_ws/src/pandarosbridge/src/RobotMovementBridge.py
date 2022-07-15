#!/usr/bin/env python

from threading import current_thread
import rospy
import roslib
import actionlib
from std_msgs.msg import String, Bool, Float64
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from pandarosbridge.srv import gripperMovement, gripperMovementResponse, poseMovement, poseMovementResponse, jntMovement, jntMovementResponse, movementSpeed, movementSpeedResponse
from geometry_msgs.msg import Pose
from franka_gripper.msg import GraspAction, GraspGoal, MoveGoal, MoveAction

import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
from PublishArray import PublishArrayClass

# this node has one topic which sends the current position and pose orientation of the robot
# and one service to send the robot to a position
group = ""
publishArrayInstance = PublishArrayClass()

def movementBridge():
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()
    group_name = "panda_arm"
    global group
    group = moveit_commander.MoveGroupCommander(group_name)
    
    gripper_group_name = "hand"
    global gripper_group
    gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name)

    group.set_max_acceleration_scaling_factor(0.1)    
    group.set_max_velocity_scaling_factor(0.1)
    
    #velocity = group.get_known_constraints()
    #rospy.loginfo("============ Velocity: %s" % velocity)
    
    group.set_num_planning_attempts(3)
    group.set_planning_time(3)
    
    #group.set_goal_position_tolerance(0.005)

    #group.set_goal_orientation_tolerance(0.01)

    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    #group.set_pose_reference_frame('camera_color_optical_frame')
    #planning_frame = group.get_planning_frame() 
    #print("hi")	

    rospy.loginfo("============ Reference frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    rospy.loginfo("============ End effector: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    rospy.loginfo("============ Robot Groups: %s", robot.get_group_names())

    pub = rospy.Publisher('panda_movement_bridge/PosePublisher', Pose, queue_size=10)
    move_service = rospy.Service('panda_movement_bridge/move_panda_robot', poseMovement, moveRobot)

    # Initialize PublishArray Class
    #publishArrayInstance = PublishArrayClass()
    rospy.loginfo("============ PublishArrayClass Instance Created")
    print(type(publishArrayInstance))
    
    ###Services###
    #gripper_service = rospy.Service('panda_movement_bridge/move_panda_gripper', gripperMovement, moveGripper)
    joint_service = rospy.Service('panda_movement_bridge/JointService', jntMovement, moveJoints)
    movementSpeed_service = rospy.Service('panda_movement_bridge/moveSpeedService', movementSpeed, robotSpeed)


    
    ###Topics###
    rospy.Subscriber('panda_movement_bridge/PoseConverterFromString', String, publishArrayContiniously)
    rospy.Subscriber('panda_movement_bridge/PoseConverterFromPose', Pose, convertFromCameraToWorld)
    rospy.Subscriber('panda_movement_bridge/PoseListener', Pose, moveRobotcontinuous)
    rospy.Subscriber('panda_movement_bridge/StopListener', Bool, stopRobot)
    rospy.Subscriber('panda_movement_bridge/GripperListenerGrasp', GraspGoal, graspGripper)
    rospy.Subscriber('panda_movement_bridge/GripperListenerMove', MoveGoal, moveGripper)
    #rospy.Subscriber('panda_movement_bridge/SpeedListener', Float64, robotSpeed)


    rospy.loginfo("RobotMovementBridge initialized")
    

    while not rospy.is_shutdown():
        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        # rospy.loginfo("============ Printing robot state")
        # rospy.loginfo(robot.get_current_state())

        current_pose = group.get_current_pose().pose

        # rospy.loginfo(current_pose)

        # rospy.loginfo(outstring)
        pub.publish(current_pose)

        rate.sleep()

def stopRobot(stopSignal):
    rospy.loginfo("============ Robot should stop: ")
    group.stop()
    rospy.loginfo("Robot stopped successfully")

def transform_pose(input_pose, from_frame, to_frame):
    # **Assuming /tf2 topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

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

def publishArrayContiniously(x_y_z):
    rospy.loginfo("============ Calculating Pose from camera coords into world coords!")
    rospy.loginfo(x_y_z)

    print("das kommt rein: ")
    print(x_y_z)
    x_y_zString = str(x_y_z)[7:]

    splitted_classification_x_y_z = x_y_zString.split("_")
    if len(splitted_classification_x_y_z) > 3:
        x = splitted_classification_x_y_z[0]
        y = splitted_classification_x_y_z[1]
        z = splitted_classification_x_y_z[2]
        rospyTime = splitted_classification_x_y_z[3]

        cameraPose = Pose()
        print(type(x))
        print(x)
        print(type(float(x)))
        cameraPose.position.x = float(x)
        cameraPose.position.y = float(y)
        cameraPose.position.z = float(z)

        rospy.loginfo(cameraPose)

        worldPose = transform_pose(cameraPose, 'camera_color_optical_frame', 'panda_link0')
        currPose = group.get_current_pose().pose

        worldPose.orientation.x = currPose.orientation.x
        worldPose.orientation.y = currPose.orientation.y
        worldPose.orientation.z = currPose.orientation.z
        worldPose.orientation.w = currPose.orientation.w

        rospy.loginfo(worldPose)

        publishArrayInstance.publishOneMarker(worldPose)
        print("geht doch weiter")

        #moveRobotcontinuous(worldPose)


def convertFromCameraToWorldFromString(classification_x_y_z):
    rospy.loginfo("============ Calculating Pose from camera coords into world coords!")
    rospy.loginfo(classification_x_y_z)

    print(type(classification_x_y_z))

    classification_x_y_zString = str(classification_x_y_z)
    print(type(classification_x_y_zString))

    splitted_classification_x_y_z = classification_x_y_zString.split("_")
    if len(splitted_classification_x_y_z) > 4:
        classification = splitted_classification_x_y_z[0]
        x = splitted_classification_x_y_z[1]
        y = splitted_classification_x_y_z[2]
        z = splitted_classification_x_y_z[3]
        rospyTime = splitted_classification_x_y_z[4]

        if x != 0 and y != 0 and z != 0:
            # Initialize PublishArray Class
            publishArrayInstance = PublishArrayClass()
            rospy.loginfo("============ PublishArrayClass Instance Created")

            cameraPose = Pose()
            cameraPose.position.x = float(x)
            cameraPose.position.y = float(y)
            cameraPose.position.z = float(z)

            rospy.loginfo(cameraPose)

            worldPose = transform_pose(cameraPose, 'camera_color_optical_frame', 'panda_link0')
            currPose = group.get_current_pose().pose

            worldPose.orientation.x = currPose.orientation.x
            worldPose.orientation.y = currPose.orientation.y
            worldPose.orientation.z = currPose.orientation.z
            worldPose.orientation.w = currPose.orientation.w

            rospy.loginfo(worldPose)

            publishArrayInstance.publishArray(worldPose)
            print("geht doch weiter")

            #moveRobotcontinuous(worldPose)






def convertFromCameraToWorld(cameraPose):
    rospy.loginfo("============ Calculating Pose from camera coords into world coords!")
    rospy.loginfo(cameraPose)

    # Initialize PublishArray Class
    publishArrayInstance = PublishArrayClass()
    rospy.loginfo("============ PublishArrayClass Instance Created")

    # def transform_pose(input_pose, from_frame, to_frame):
    #     # **Assuming /tf2 topic is being broadcasted
    #     tf_buffer = tf2_ros.Buffer()
    #     listener = tf2_ros.TransformListener(tf_buffer)

    #     pose_stamped = tf2_geometry_msgs.PoseStamped()
    #     pose_stamped.pose = input_pose
    #     pose_stamped.header.frame_id = from_frame
    #     #print(pose_stamped.header.frame_id)
    #     pose_stamped.header.stamp = rospy.Time.now()

    #     try:
    #         # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
    #         output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
    #         return output_pose_stamped.pose

    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #         raise
    

    worldPose = transform_pose(cameraPose, 'camera_color_optical_frame', 'panda_link0')
    #worldPose.orientation.x = cameraPose.orientation.x
    #worldPose.orientation.y = cameraPose.orientation.y
    #worldPose.orientation.z = cameraPose.orientation.z
    #worldPose.orientation.w = cameraPose.orientation.w
    currPose = group.get_current_pose().pose

    worldPose.orientation.x = currPose.orientation.x
    worldPose.orientation.y = currPose.orientation.y
    worldPose.orientation.z = currPose.orientation.z
    worldPose.orientation.w = currPose.orientation.w

    rospy.loginfo(worldPose)

    publishArrayInstance.publishArray(worldPose)
    print("geht doch weiter")

    moveRobotcontinuous(worldPose)



def moveRobotcontinuous(goalpose):
    rospy.loginfo("============ Robot should move continuous to Pose: Move Bitch!")
    rospy.loginfo(goalpose)

    pose_goal = goalpose
    group.set_start_state_to_current_state()
    group.set_pose_target(pose_goal)

    group.go(pose_goal, wait=False)

    #group.execute(path, wait=True)

    group.stop()
    group.clear_pose_targets()

    resp = poseMovementResponse("Done")
    # resp.answer_str = "Done"

    rospy.loginfo("Robot moved to position successfully. Really?")


def moveRobot(thispose):
    rospy.loginfo("============ Robot should move to Pose:")
    rospy.loginfo(thispose)

    pose_goal = thispose.pose
    group.set_start_state_to_current_state()
    group.set_pose_target(pose_goal)
    group.go(pose_goal, wait=True)

    # group.execute(plan, wait=True)

    group.stop()
    group.clear_pose_targets()

    resp = poseMovementResponse("Done")
    # resp.answer_str = "Done"

    rospy.loginfo("Robot moved to position successfully.")

    return resp


def moveGripper(gripper_command):
    rospy.loginfo("============ Gripper should move: ")
    rospy.loginfo(gripper_command)
    
    client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    client.wait_for_server()
    width = gripper_command.width
    speed = gripper_command.speed
    #rospy.loginfo(widthNum)
    #goal = MoveGoal(width = 0.02, speed = 0.1)
    goal = MoveGoal(width, speed)
    client.send_goal(goal)
    
    
    #resp = gripperMovementResponse("Done")
    rospy.loginfo("Gripper moved successfully")
    
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
    #goal = GraspGoal(width = 0.08, speed =0.1, force=1)
    client.send_goal(goal)
    
    #resp = gripperMovementResponse("Done")
    rospy.loginfo("Gripper moved successfully")
    
def robotSpeed(speedRequest):
    rospy.loginfo("============ Speed of robot movement set to: ")
    speed = float(format(speedRequest.moveSpeed, '.2f'))
    
    rospy.loginfo(speed)
    group.set_max_velocity_scaling_factor(speed)
    return speed
    
def moveJoints(jointTarget):
    joint_goal = group.get_current_joint_values()
    rospy.loginfo(joint_goal)
    joint_goal[0] = joint_goal[0] + jointTarget.panda_joint1
    joint_goal[1] = joint_goal[1] + jointTarget.panda_joint2
    joint_goal[2] = joint_goal[2] + jointTarget.panda_joint3
    joint_goal[3] = joint_goal[3] + jointTarget.panda_joint4
    joint_goal[4] = joint_goal[4] + jointTarget.panda_joint5
    joint_goal[5] = joint_goal[5] + jointTarget.panda_joint6
    joint_goal[6] = joint_goal[6] + jointTarget.panda_joint7
    
    group.go(joint_goal, wait=True)
    group.stop()
    joint_goal_answer = group.get_current_joint_values()
    return joint_goal_answer[0], joint_goal_answer[1], joint_goal_answer[2], joint_goal_answer[3], joint_goal_answer[4], joint_goal_answer[5], joint_goal_answer[6]
    


if __name__ == '__main__':
    rospy.init_node('panda_movement_bridge', anonymous=False)
    rate = rospy.Rate(1)

    try:
        movementBridge()
    except rospy.ROSInterruptException:
        pass
