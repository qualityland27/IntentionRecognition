#!/usr/bin/env python
# license removed for brevity
import time
import rosgraph

import threading
import logging
import rospy
from std_msgs.msg import Bool
from intention_recognition.msg import object_msgs, class_and_position_msgs


class ROSInterface:
    def __init__(self, master_ip, node_name, topic_name, limit_until_publishing):
        self.log = logging.getLogger('intention_recognition')

        self.master_available = False
        self.node_initialized = False
        self.master_ip = master_ip
        self.topic_name = topic_name
        self.node_name = node_name
        self.limit_until_publishing = limit_until_publishing
        self.count_until_publish = 0
        self.published = False

        self.classification_publisher = None
        self.wrist_publisher = None
        self.object_pose_publisher = None
        self.recovery_gesture_publisher = None

        self.zeit_des_letzten_sendens = time.time()

        self.recovery_stack = []

        self.stop_ros_threat = False

        try:
            x = threading.Thread(target=self.checkROSmasterAvailability, args=())
            x.start()
        except:
            self.log.warning("unable to start thread to Check if ros-Master is available")

    #####################################
    #######      Organisation      ######
    #####################################
    def checkROSmasterAvailability(self):
        """
        This function check conitniously if rosmaster is available.
        Result will be logged.
        """
        online_status_logged = False
        while not self.stop_ros_threat:
            if rosgraph.is_master_online():  # Checks the master uri and results boolean (True or False)
                self.master_available = True
                if not online_status_logged:
                    self.log.info("'ros MASTER is Online'")
                    online_status_logged = True
            else:
                self.master_available = False
                self.log.warning("'ros MASTER is Offline'")
                if online_status_logged:
                    online_status_logged = False
                time.sleep(5)

    def initializeNodeByName(self, ENV, queue_size=1):
        '''
        This functions initializes node and topics
        '''
        self.classification_publisher = rospy.Publisher(ENV.TOPIC_INTENTION_PUBLISHER, class_and_position_msgs,
                                                        queue_size=1)
        self.wrist_publisher = rospy.Publisher(ENV.TOPIC_WRIST_PUBLISHER, class_and_position_msgs,
                                               queue_size=1)
        self.object_pose_publisher = rospy.Publisher(ENV.TOPIC_OBJECT_PUBLISHER, object_msgs,
                                                     queue_size=1)
        self.recovery_gesture_publisher = rospy.Publisher(ENV.TOPIC_RECOVERY_GESTURE_PUBLISHER, Bool,
                                                          queue_size=1)

        rospy.init_node(self.node_name, anonymous=True)
        # rospy.loginfo("============ Node Chatter Initialized")
        self.log.info("Node '{}' initialized.".format(self.node_name))
        self.node_initialized = True

    #####################################
    #######   Prepare Publishing   ######
    #####################################
    def check_timing(self, seconds=1.0):
        """
        Check time difference since last message.
        """
        if (time.time() - self.zeit_des_letzten_sendens) > seconds:
            return True
        else:
            return False

    def publish_with_filters(self, prediction, coords, prediction_same_in_two_frames):
        """
        Prepare Publishing with time filter. Only send every 0.5 Seconds and check
        if prediction is within two fra,es the same.
        """
        if self.master_available and self.node_initialized and prediction_same_in_two_frames:
            # Check timing to limit frequency
            if self.check_timing(seconds=0.5):
                try:
                    self.publishClassification(prediction, coords)
                    self.published = True
                    self.zeit_des_letzten_sendens = time.time()
                except Exception as e:
                    logging.error("ROS Publishing Error")

    def publish_without_filters(self, prediction, coords):
        """
        Preapre Publishing Continously withou any filters.
        """
        if self.master_available and self.node_initialized:
            try:
                self.publishCoordinates(prediction, coords)
            except Exception as e:
                logging.error("ROS Publishing Error")

    def publish_object_detection(self, detected_objects):
        """
        This function handles publishing of object detection. It will only send if detected_objects object is not
        empty.
        """
        if detected_objects[3] is not 0:
            object_detection_string = str(detected_objects)
            if self.master_available and self.node_initialized:
                try:
                    self.publishObjectDetection(detected_objects)
                except Exception as e:
                    logging.error("ROS Publishing Error")

    #####################################
    #######     Call Publisher     ######
    #####################################
    def publishClassification(self, prediction, coords):
        """
        Create Message and send Classification.
        """
        to_publish = class_and_position_msgs()
        to_publish.classification = prediction
        to_publish.x = coords[0]
        to_publish.y = coords[1]
        to_publish.z = coords[2]
        to_publish.time = rospy.get_time()

        self.classification_publisher.publish(to_publish)

    def add_time(self, message):
        time_string = str(rospy.get_time())
        to_publish = message + "_" + time_string
        # rospy.loginfo(to_publish)
        return to_publish

    def publishCoordinates(self, prediction, coords):
        """
        Create Message and Coords of Wrist.

        Note: Prediction is Empty here!
        """
        to_publish = class_and_position_msgs()
        to_publish.classification = prediction
        to_publish.x = coords[0]
        to_publish.y = coords[1]
        to_publish.z = coords[2]
        to_publish.time = rospy.get_time()

        self.wrist_publisher.publish(to_publish)

    def publishObjectDetection(self, detected_objects):
        """
        Create Message and send Detected Object.
        """
        center = detected_objects[0]
        width = detected_objects[1]
        height = detected_objects[2]
        angle = detected_objects[3]
        time = rospy.get_time()

        to_publish = object_msgs()
        to_publish.x = center[0]
        to_publish.y = center[1]
        to_publish.width = width
        to_publish.height = height
        to_publish.angle = angle
        to_publish.time = time

        self.object_pose_publisher.publish(to_publish)

    def published_recovery_gesture(self, recover_gesture_done):
        if recover_gesture_done:
            if len(self.recovery_stack) == 0:
                self.recovery_stack.append(time.time())
            else:
                now = time.time()
                if (self.recovery_stack[0] - now) < 3.0:
                    self.recovery_stack.append(recover_gesture_done)
                    if len(self.recovery_stack) > 8:
                        try:
                            self.recovery_gesture_publisher.publish(recover_gesture_done)
                            self.recovery_stack.clear()
                        except Exception as e:
                            logging.error("ROS Publishing Error")
        else:
            self.recovery_stack.clear()
