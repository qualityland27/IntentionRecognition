#!/usr/bin/env python  
import rospy

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math

# Because of transformations
import tf_conversions
import tf2_ros
import geometry_msgs.msg


topic = 'transformed_marker_array'
publisher = rospy.Publisher(topic, MarkerArray)

rospy.init_node('transform_calibration')

markerArray = MarkerArray()

count = 0
MARKERS_MAX = 100





#rospy.init_node('tf2_turtle_broadcaster')



while not rospy.is_shutdown():
   br = tf2_ros.TransformBroadcaster()
   t = geometry_msgs.msg.TransformStamped()

   t.header.stamp = rospy.Time.now()
   t.header.frame_id = "panda_link0"
   t.child_frame_id = "camera_color_optical_frame"
   #t.transform.translation.x = msg.x
   #t.transform.translation.y = msg.y
   t.transform.translation.z = 1.0
   #q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
   #t.transform.rotation.x = q[0]
   #t.transform.rotation.y = q[1]
   #t.transform.rotation.z = q[2]
   t.transform.rotation.w = 1.0

   br.sendTransform(t)

   print(t)

   

   rospy.sleep(0.01)


