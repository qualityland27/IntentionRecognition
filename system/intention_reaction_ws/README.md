# Intention_Reaction

## Preliminaries
- Install and Build packages in your workspace with `catkin build` (you find more details at the end of the file)
- Make shure your terminals are sourced
- If you run the Modules on different machines you have to take care on Network nettings in ROS.

## Run Packages

##### MoveiIt Controller
`roslaunch panda_moveit_config panda_control_moveit_rviz.launch launch_rviz:=true robot_ip:=172.16.0.2 load_gripper:=true`

##### Interaktion_Reaction
`rosrun intention_recognition PandaReaction.py` Option: -object_detection=True Deafult: False

_At -object_detection=True angual of object will be considerred on grasping_

##### Kamerakoordinatensystem
`cd /camera_calibration`
`roslaunch camera_pose.launch`


## For Developers

##### Camera starten für Kalibrierung

`roslaunch realsense2_camera rs_camera.launch`

Hinweis: Tutorial von MoveIt! Calibraiton beachten (Link ist weiter unten zu finden)




### Packages im Workspace:

moveit calibraiton 
- install from source: 
- https://github.com/ros-planning/moveit_calibration
- Tutorial:
- https://ros-planning.github.io/moveit_tutorials/doc/hand_eye_calibration/hand_eye_calibration_tutorial.html
    
rviz_visualization_tools 
- install from soure:
- https://github.com/PickNikRobotics/rviz_visual_tools

franka-panda-description
- https://github.com/justagist/franka_panda_description

realsense 
- https://github.com/IntelRealSense/realsense-ros#installation-instructions
