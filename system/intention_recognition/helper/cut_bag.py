import numpy as np
import cv2
import rosbag




import os
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    startCut = False
    bag = rosbag.Bag("/home/benny/Documents/cutTest.bag", "r")
    bridge = CvBridge()
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    count = 0
    cv_img = 0
    startTS = 0
    endTS = 0
    for topic, msg, t in bag.read_messages():
        try:
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            if key == 32:
                if not startCut:
                    startCut = True
                    startTS = t
                    print(startTS)
                else:
                    startCut = False
                    endTS = t
                    print(endTS)

        except:
            pass
        cv2.imshow('RealSense', cv_img)
        cv2.waitKey(1)

    bag.close()

    with rosbag.Bag('/home/benny/Documents/cutTestOut.bag', 'w') as outbag:
        for topic, msg, t in rosbag.Bag('/home/benny/Documents/cutTest.bag').read_messages():
            if t > startTS and t < endTS:
                outbag.write(topic, msg, t)


if __name__ == "__main__":
    main()