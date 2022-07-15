###############################################
##                return distance            ##
###############################################
import pyrealsense2 as rs
import numpy as np
import cv2
import logging


class Realsense:
    """Realsense.

      This Class is responsible for interaction with realsense sdk.
      Most important function are configuration of camera for live stream or from .bag files.

      E.G. Extract Landmarks from MediaPipe to the dataframe.

      """

    def __init__(self):
        self.profile = None
        self.depth_scale = None
        self.log = logging.getLogger('intention_recognition')

        align_to = rs.stream.color
        self.align = rs.align(align_to)

        self.device = None

        self.depth_scale = None

        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.threshold_filter = None

        self.depth_frame = 0

    def configure_camera(self, width, height):
        '''
        This function configures camera to get data from pipeline. We just have to feed the function with needed
        width and height of the image.

        :param width: width of image
        :param height: height of image
        :return: success bool
        '''
        try:
            ########## Configure depth stream ##########
            # pipeline = rs.pipeline()
            # config = rs.config()

            # Get device product line for setting a supporting resolution
            pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
            pipeline_profile = self.config.resolve(pipeline_wrapper)
            self.device = pipeline_profile.get_device()
            device_product_line = str(self.device.get_info(rs.camera_info.product_line))

            self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
            self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)

            # Start streaming
            self.profile = self.pipeline.start(self.config)

            # Getting the depth sensor's depth scale (see rs-align example for explanation)
            depth_sensor = self.profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            return True

        except Exception as e:
            self.log.error("Error during configuration of camera. Trying again! ")
            return False

    def configure_realsense_from_bag(self, path, file, frames_until_start=3):
        '''
        This function configures camera from .bag files. If its not possible to configure the camera
        from file. The functions wait some frames before continue

        :param path: path of file
        :param file: name of the file
        :param frames_until_start: # frames to wait
        :return: success bool
        '''
        self.pipeline, self.config = self.read_bag_return_pipeline(path + file)

        for i in range(frames_until_start):
            frames = self.pipeline.wait_for_frames()

    def stop_pipeline(self):
        """
        This function stops the pipeline
        """
        self.pipeline.stop()

    def get_color_depth_stream(self):
        """
        This function waits for Frames from Pipeline. Alignes color to depth frame
        and converts to numpy array.

        :param isflipped if True the numpy array gets flipped
        :retrun color & depth frames and color & depth arrays
        """
        frames = self.pipeline.wait_for_frames()

        # Aligning color frame to depth frame
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        self.depth_frame = depth_frame

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        return color_frame, depth_frame, color_image, depth_image

    def cut_background(self, depth_image, color_image, color_for_bg_removal, clipping_distance_in_meters=1):
        """
        Helper Function for Object Detection.
        Puts all Pixels more far away from sensor than threshold to specific color.
        """
        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = color_for_bg_removal
        clipping_distance = clipping_distance_in_meters / self.depth_scale
        depth_image_3d = np.dstack(
            (depth_image, depth_image, depth_image))  # depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

        return bg_removed

    def return_distance(self, x, y, depth_frame, color_frame):
        """
        This function need the rgb_frame as well as x and y coordinates of target point. The function also needs the
        pipeline to open the depth_frame.
        The Function fits size of frames together if needed.
        The function calculates the distance of the target point to the camera.

        :return: z, resized_color_image
        """

        z = 0
        try:
            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape

            resized_color_image = color_frame
            # If depth and color resolutions are different, resize color image to match depth image for display
            if depth_colormap_dim != color_colormap_dim:
                resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                                                 interpolation=cv2.INTER_AREA)

            z = depth_frame.get_distance(int(x), int(y))

        finally:
            return z, resized_color_image

    def return_distance_from_pipeline(self, x, y, pipeline):
        """
        This function need the rgb_frame as well as x and y coordinates of target point. The function also needs the
        pipeline to open the depth_frame.
        The Function fits size of frames together if needed.
        The function calculates the distance of the target point to the camera.
        Return values are the distance and the resized_color_image.
        """

        z = 0
        try:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame)

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape

            resized_color_image = color_frame
            # If depth and color resolutions are different, resize color image to match depth image for display
            if depth_colormap_dim != color_colormap_dim:
                resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                                                 interpolation=cv2.INTER_AREA)

            z = depth_frame.get_distance(int(x), int(y))

        finally:
            return z, resized_color_image

    def read_bag_return_streams(self, pathToBagFile):
        """
        This function needs a path to a bag file. The function will read the file and return the rgb and depth streams.
        Return values are rgb and depth streams of the video.
        """

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

            # Create opencv window to render image in
            # cv2.namedWindow("Depth Stream", cv2.WINDOW_AUTOSIZE)

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

    def read_bag_return_pipeline(self, pathToBagFile, repeat_playback=True):
        """
        This function needs a path to a bag file. The function will read the file and return the pipeline.

        :return: pipeline, config
        """

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
            rs.config.enable_device_from_file(config, pathToBagFile, repeat_playback=repeat_playback)

            # Get device product line for setting a supporting resolution
            pipeline_wrapper = rs.pipeline_wrapper(pipeline)
            pipeline_profile = config.resolve(pipeline_wrapper)
            device = pipeline_profile.get_device()
            device_product_line = str(device.get_info(rs.camera_info.product_line))

            # Configure the pipeline to stream the depth stream
            # Change this parameters according to the recorded bag file resolution
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

            # Start streaming from file
            pipeline.start(config)

            return pipeline, config

        finally:
            pass


if __name__ == "__main__":
    Realsense.return_distance(300, 300)
