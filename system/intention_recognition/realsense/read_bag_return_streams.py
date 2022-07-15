#####################################################
##               Read bag from file                ##
#####################################################


# First import library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
# Import argparse for command-line options
import argparse
# Import os.path for file path manipulation
import os.path

def read_bag(self, pathToBagFile):

    # Safety if no parameter have been given
    if not pathToBagFile:
        print("No input paramater have been given.")
        print("For help type --help")
        exit()
    # Check if the given file have bag extension
    if ".bag" not in pathToBagFile:
        print("The given file is not of correct file format.")
        print("Only .bag files are accepted")
        exit()
    try:
        # Create pipeline
        pipeline = rs.pipeline()

        # Create a config object
        config = rs.config()

        # Tell config that we will use a recorded device from file to be used by the pipeline through playback.
        rs.config.enable_device_from_file(config, pathToBagFile)

        # Configure the pipeline to stream the depth stream
        # Change this parameters according to the recorded bag file resolution
        config.enable_stream(rs.stream.depth, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, rs.format.bgr8, 30)

        # Start streaming from file
        pipeline.start(config)

        ## Create opencv window to render image in
        #cv2.namedWindow("Depth Stream", cv2.WINDOW_AUTOSIZE)

        # create first timestamp variable
        first_ts = 0

        # Streaming loop
        while True:
            # Get frameset of depth
            frames = pipeline.wait_for_frames()

            # make shure we leave the loop when video is finished
            ts = frames.get_timestamp()
            if (first_ts - ts) == 0:
                break
            if first_ts == 0:
                first_ts = ts

            # Get depth frame
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            key = cv2.waitKey(1)
            # if pressed escape exit program
            if key == 27:
                cv2.destroyAllWindows()
                break

            return depth_frame, color_frame

    finally:
        pass

if __name__ == "__main__":
    read_bag()