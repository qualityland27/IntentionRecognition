#!/usr/bin/env python  
import rospy
import math
import tf2_ros
import geometry_msgs.msg
import tf



rospy.init_node('tf_test')
rospy.loginfo("Node for testing actionlib server")

#from_link = '/head_color_camera_l_link'
#to_link = '/base_link'

from_link = '/panda_link0'
to_link = '/camera_color_optical_frame'

tfBuffer = tf2_ros.Buffer()
tfl = tf.Transformer()

rospy.sleep(5)


t = rospy.Time(0)

mpose = geometry_msgs.msg.PoseStamped()

mpose.pose.position.x = 0
mpose.pose.position.y = 0
mpose.pose.position.z = 1

mpose.pose.orientation.x = 0
mpose.pose.orientation.y = 0
mpose.pose.orientation.z = 0
mpose.pose.orientation.w = 1

mpose.header.frame_id = from_link
mpose.header.stamp = rospy.Time.now()

rospy.sleep(5)

mpose_transf = None

rospy.loginfo('Waiting for transform for some time...')

tfl.waitForTransform(to_link,from_link,t,rospy.Duration(5))

if tfl.canTransform(to_link,from_link,t):
  
  mpose_transf = tfl.transformPose(to_link,mpose)
  
  print mpose_transf
  
else:
  
  rospy.logerr('Transformation is not possible!')
  sys.exit(0)


rospy.init_node('listener_benny')

topic = 'transformed_marker_array_2'
turtle_vel = rospy.Publisher(topic, geometry_msgs.msg.Twist, queue_size=1)

t = tf.Transformer(True, rospy.Duration(10.0))
exist = t.frameExists("/world")
print(exist)


rost = tf.TransformerROS()
print("done")

target_frame = "\camera_color_optical_frame"
ps = geometry_msgs.msg.PoseStamped()
ps.header.frame_id = "versuch_mers"
ps.header.stamp = rospy.Time.now()
ps.pose.position.z = 1.0
ps.pose.orientation.w = 1.0

newMsg = rost.transformPose(target_frame, ps)
print("done")



tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)



rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    try:
        trans = tfBuffer.lookup_transform('panda_link0', 'camera_color_optical_frame', rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
       rate.sleep()
       continue

    msg = geometry_msgs.msg.Twist()

    turtle_vel.publish(msg)

    rate.sleep()
