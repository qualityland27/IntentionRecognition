import argparse
import time

import pyrealsense2 as rs
import numpy as np
import cv2

# Create object for parsing command-line options
parser = argparse.ArgumentParser(description="Read recorded bag file and display depth stream in jet colormap.\
                                Remember to change the stream fps and format to match the recorded.")
# Add argument which takes path to a bag file as an input
parser.add_argument("-n", "--number", type=int, help="no of video")

args = parser.parse_args()

print("")
print("Count!")
print("")

print("5")
time.sleep(1)
print("4")
time.sleep(1)
print("3")
time.sleep(1)
print("2")
time.sleep(1)
print("1")
print("")

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

videNo = args.number
path = '/home/benny/Documents/evaluationData/highFive' + str(videNo).zfill(4) + '.bag'
config.enable_record_to_file(path)

# Start streaming
pipeline.start(config)

e1 = cv2.getTickCount()

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        e2 = cv2.getTickCount()
        t = (e2 - e1) / cv2.getTickFrequency()
        if t < 4:
            print("UP AND WAIT !!")

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Stack both images horizontally
        images = np.hstack((color_image, depth_colormap))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)


        if t > 4:
            print("CHECK IN!!")

        if t>5: # change it to record what length of video you are interested in
            print("")
            print("Done!")
            print("")
            print(videNo)
            break

finally:
    # Stop streaming
    pipeline.stop()
