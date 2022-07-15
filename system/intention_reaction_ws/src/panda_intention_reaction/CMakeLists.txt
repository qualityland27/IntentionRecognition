cmake_minimum_required(VERSION 2.8.3)
project(panda_intention_reaction)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  franka_msgs
#  Eigen3 REQUIRED
)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS

        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        pluginlib
        
        message_generation
        )

find_package(Franka 0.5.0 REQUIRED)

add_service_files(
        FILES
        poseMovement.srv
        gripperMovement.srv
        jntMovement.srv
        movementSpeed.srv
        gripperJoint.srv
)

add_message_files(
  FILES
  object_msgs.msg
  joint_msgs.msg
  class_and_position_msgs.msg
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
)


catkin_package(
        LIBRARIES
        interactivity_utils
        INCLUDE_DIRS
        ${THIS_PACKAGE_INCLUDE_DIRS}
        CATKIN_DEPENDS message_runtime
        franka_msgs
        moveit_core
        moveit_ros_planning_interface
        DEPENDS Franka
       # EIGEN3
)



include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
