#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
import geometry_msgs.msg

class PublishArrayClass:
    def __init__(self):
        topic = 'calibration_visualization_marker_array_converted_in_panda_link0'
        self.publisher = rospy.Publisher(topic, MarkerArray, queue_size=5)
        topic1 = 'calibration_visualization_marker_converted_in_panda_link0'
        self.publisher1 = rospy.Publisher(topic1, Marker, queue_size=5)

        #rospy.init_node('register_calibration')

        self.count = 0
        self.MARKERS_MAX = 10
        self.markerArray = MarkerArray()


    def publishArray(self, tPose):
        """
        Publish Marker Array in rviz on target pose.
        """
        targetPose = geometry_msgs.msg._Pose.Pose()
        targetPose = tPose
        markerArray = MarkerArray()

        count = 0
        MARKERS_MAX = 10

        while not rospy.is_shutdown():

            marker = Marker()
            marker.header.frame_id = "/panda_link0"
            marker.type = marker.ARROW
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0  # if 0 --> invisible
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.pose.orientation.x = targetPose.orientation.x
            marker.pose.orientation.y = targetPose.orientation.y
            marker.pose.orientation.z = targetPose.orientation.z
            marker.pose.orientation.w = targetPose.orientation.w
            marker.pose.position.x = targetPose.position.x
            marker.pose.position.y = targetPose.position.y 
            marker.pose.position.z = targetPose.position.z
           

            # We add the new marker to the MarkerArray, removing the oldest
            # marker from it when necessary
            if(self.count > self.MARKERS_MAX):
                self.markerArray.markers.pop(0)

            # Renumber the marker IDs
            id = 0
            for m in markerArray.markers:
                m.id = id
                id += 1

            # Publish the MarkerArray
            self.publisher.publish(markerArray)

            count += 1

            rospy.sleep(0.01)

    def publishOneMarker(self, tPose):
        """
        Publish One Marker in rviz on target pose.
        """
        targetPose = geometry_msgs.msg._Pose.Pose()
        targetPose = tPose

        marker = Marker()
        marker.header.frame_id = "/panda_link0"
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.ns = "my_namespace"
        marker.id = self.count
        marker.scale.x = 0.1
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0  # if 0 --> invisible
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.x = targetPose.orientation.x
        marker.pose.orientation.y = targetPose.orientation.y
        marker.pose.orientation.z = targetPose.orientation.z
        marker.pose.orientation.w = targetPose.orientation.w
        marker.pose.position.x = targetPose.position.x
        marker.pose.position.y = targetPose.position.y 
        marker.pose.position.z = targetPose.position.z

        # We add the new marker to the MarkerArray, removing the oldest
        # marker from it when necessary
        if(self.count > self.MARKERS_MAX):
            self.markerArray.markers.pop(0)
            #break

        self.markerArray.markers.append(marker)
        #self.markerArray.markers.append(textMarker)

        # Renumber the marker IDs
        id = 0
        for m in self.markerArray.markers:
            m.id = id
            id += 1

        # Publish the MarkerArray
        self.publisher.publish(self.markerArray)
        #self.publisher.publish(textMarker)

        self.count += 1

        rospy.sleep(0.01)


