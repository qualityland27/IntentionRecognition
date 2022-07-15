import cv2  # Import opencv
import numpy as np


class Visualizer:
    def __init__(self, width, height):
        # before image flip
        self.mp_width = width
        self.mp_height = height

        # after image flip
        self.image_width = height
        self.image_height = width

        # Dimensions of Visualizer
        self.status_section_height = 120
        self.classification_section_height = 80
        self.coordinate_section_width = 300

        # Composite Window
        self.composite_width = self.image_width + self.coordinate_section_width
        self.composite_height = self.image_height + self.status_section_height + self.classification_section_height

        # colors
        # self.textColor = (255, 255, 255) # white
        self.textColor = (201, 201, 201)
        self.black = (0, 0, 0)
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.gray = (105, 105, 105)
        self.yellow = (0, 255, 255)

        self.kit_green = (0, 150, 130)
        self.kit_blue = (70, 100, 170)
        self.kit_march_green = (70, 182, 60)
        self.kit_yellow = (252, 229, 0)
        self.kit_red = (162, 34, 35)

        # fonts
        # self.text_font = cv2.FONT_HERSHEY_SIMPLEX
        self.text_normal = cv2.FONT_HERSHEY_SIMPLEX
        self.text_bold = cv2.FONT_HERSHEY_DUPLEX

    def draw_within_picture(self, image, results, data_factory):
        """
        This function draws features before flipping the image.
        """
        ##### Visualize nose
        # nose_x, nose_y = data_factory.nose_abs[0], data_factory.nose_abs[1]
        # cv2.circle(image, (nose_x, nose_y), 5, self.kit_blue, -1)

        ##### Visualize head_posture
        head_posture_x = int(data_factory.head_post[0] * self.mp_width)
        head_posture_y = int(data_factory.head_post[1] * self.mp_height)

        cv2.circle(image, data_factory.nose_from_head_posture, 5, self.kit_blue, -1)
        cv2.line(image, data_factory.nose_from_head_posture, (head_posture_x, head_posture_y), self.kit_yellow, 2)
        # cv2.putText(image,
        #             str((head_posture_x, head_posture_y)), (head_posture_x, head_posture_y),
        #             self.text_normal,
        #             1,
        #             self.kit_yellow)

        # ##### Visualize Object Detection
        data_factory.draw_objects = True

        def draw_circle_from_xyz(xyz, color, size=5):
            x, y = xyz[0], xyz[1]
            cv2.circle(image, (x, y), size, color, -1)

        # # Wrist
        # draw_circle_from_xyz(data_factory.wrist_l_abs, self.kit_yellow)
        # draw_circle_from_xyz(data_factory.wrist_r_abs, self.kit_yellow)

        # # Eyes
        # draw_circle_from_xyz(data_factory.eye_l_abs, self.kit_yellow)
        # draw_circle_from_xyz(data_factory.eye_r_abs, self.kit_yellow)

        # # Shoulder
        # draw_circle_from_xyz(data_factory.shoulder_l_abs, self.kit_yellow)
        # draw_circle_from_xyz(data_factory.shoulder_r_abs, self.kit_yellow)

        # # Elbow
        # draw_circle_from_xyz(data_factory.elbow_r_abs, self.kit_yellow)

    def draw_image_section(self, image, data_factory):
        """
        Draw within picture after flipping the image
        """
        self.draw_face_direction_borders(image, data_factory)
        self.draw_mp_coordinate_system(image)

        return image

    def draw_classification_section(self, image, classification, published):
        """
        Draw Information regarding classification-result.
        Draws Prediction and whether prediction was published to ROS.
        """
        ##### Extend image ####
        arr = np.zeros((self.classification_section_height, self.composite_width, 3), dtype=np.uint8)
        image = np.concatenate((arr, image))

        x_start = 40
        y_start = 25

        cv2.putText(image, ("Prediction: " + classification),
                    (x_start, y_start), self.text_normal, 0.8, self.kit_green, 2, cv2.LINE_AA)
        cv2.putText(image, ("Published: " + str(published)),
                    (x_start, y_start + 30), self.text_normal, 0.8, self.textColor, 1, cv2.LINE_AA)

        return image

    def draw_coords_section(self, image, results, data_factory, position="left"):
        """
        This function does
        1. Extend the Image by Black Box left or right;
        2. Draw Landmarks from MediaPipe.results
        3. Draw Absolute values from Class feature_factory
        4. Draw Additional Features
        5. Draw Results from Object Detection
        """

        ##### Extend image ####
        container_height = self.image_height

        arr = np.zeros((container_height, self.coordinate_section_width, 3), dtype=np.uint8)
        if position == "left":
            image = np.concatenate((arr, image), axis=1)
            x_start_xyz = 150
            y_start = 10
        else:
            image = np.concatenate((image, arr), axis=1)
            x_start_xyz = self.image_width + 150
            y_start = 10

        font_size = 0.6

        # Display xyz header
        cv2.putText(image, "x"
                    , (x_start_xyz, y_start), self.text_bold, font_size, self.textColor, 1, cv2.LINE_AA)
        cv2.putText(image, "y"
                    , ((x_start_xyz + 50), y_start), self.text_bold, font_size, self.textColor, 1, cv2.LINE_AA)
        cv2.putText(image, "z"
                    , ((x_start_xyz + 100), y_start), self.text_bold, font_size, self.textColor, 1, cv2.LINE_AA)

        # Calculate horizontal start value
        if position == "left":
            x_start = 5
        else:
            x_start = (self.image_width + 10)

        ############################################
        #         Display Mediapipe Values         #
        ############################################

        # Display mediapipie header
        y_step = 20
        y_start = y_start + y_step
        cv2.putText(image, "MediaPipe"
                    , ((x_start - 3), y_start), self.text_bold, font_size, self.textColor, 1, cv2.LINE_AA)

        # Display left wrist
        text, x_start, y_start, landmark = 'wrist_l', x_start, (y_start + y_step), 15
        self.draw_landmark_text(text, image, x_start, y_start, landmark, mediapipe_results=results.pose_landmarks)

        # Display right wrist
        text, x_start, y_start, landmark = 'wrist_r', x_start, (y_start + y_step), 16
        self.draw_landmark_text(text, image, x_start, y_start, landmark, mediapipe_results=results.pose_landmarks)

        # Display thump tip
        text, x_start, y_start, landmark = 'thump_r', x_start, (y_start + y_step), 4
        self.draw_landmark_text(text, image, x_start, y_start, landmark, mediapipe_results=results.right_hand_landmarks)

        # Display pinky finger tip
        text, x_start, y_start, landmark = 'pinky_r', x_start, (y_start + y_step), 20
        self.draw_landmark_text(text, image, x_start, y_start, landmark, mediapipe_results=results.right_hand_landmarks)

        # Display nose
        text, x_start, y_start, landmark = 'nose', x_start, (y_start + y_step), 0
        self.draw_landmark_text(text, image, x_start, y_start, landmark, mediapipe_results=results.pose_landmarks)

        ############################################
        #     Display Absolute Distance Values     #
        ############################################

        # Display z header
        y_start = (y_start + 2 * y_step)
        cv2.putText(image, "z [m]"
                    , ((x_start_xyz + 80), y_start), self.text_normal, font_size, self.textColor, 1, cv2.LINE_AA)

        # Display realsense header
        y_start = (y_start + y_step)
        cv2.putText(image, "RealSense"
                    , ((x_start - 3), y_start), self.text_bold, font_size, self.textColor, 1, cv2.LINE_AA)

        # Display Nose Absolute
        text, x_start, y_start = "nose_abs", x_start, (y_start + y_step)
        val = str(data_factory.absolute_z_nose_mean_10 * data_factory.normalize_factor_for_distance)[:5]
        self.draw_absolute_value(text, image, x_start, y_start, val)

        # Display Left Wirst Absolute
        text, x_start, y_start = "wrist_abs_l", x_start, (y_start + y_step)
        val = str(data_factory.absolute_z_left_wrist_mean_10 * data_factory.normalize_factor_for_distance)[:5]
        self.draw_absolute_value(text, image, x_start, y_start, val)

        # Display Right Wrist Absolute
        text, x_start, y_start = "wrist_abs_r", x_start, (y_start + y_step)
        val = str(data_factory.absolute_z_right_wrist_mean_10 * data_factory.normalize_factor_for_distance)[:5]
        self.draw_absolute_value(text, image, x_start, y_start, val)

        # Display Left Eye Absolute
        text, x_start, y_start = "eye_abs_l", x_start, (y_start + y_step)
        val = str(data_factory.absolute_z_leftEye_mean_10 * data_factory.normalize_factor_for_distance)[:5]
        self.draw_absolute_value(text, image, x_start, y_start, val)

        # Display Right Eye Aboslute
        text, x_start, y_start = "eye_abs_r", x_start, (y_start + y_step)
        val = str(data_factory.absolute_z_rightEye_mean_10 * data_factory.normalize_factor_for_distance)[:5]
        self.draw_absolute_value(text, image, x_start, y_start, val)

        # Display Left Shoulder Absolute
        text, x_start, y_start = "shoulder_abs_l", x_start, (y_start + y_step)
        val = str(data_factory.absolute_z_leftShoulder_m10 * data_factory.normalize_factor_for_distance)[:5]
        self.draw_absolute_value(text, image, x_start, y_start, val)

        ############################################
        #        Display Additional Values        #
        ############################################

        # Display unit
        y_start = (y_start + 2 * y_step)
        cv2.putText(image, "various"
                    , ((x_start_xyz + 80), y_start), self.text_normal, font_size, self.textColor, 1, cv2.LINE_AA)

        # Display name
        y_start = y_start + y_step
        cv2.putText(image, "Other"
                    , ((x_start - 3), y_start), self.text_bold, font_size, self.textColor, 1, cv2.LINE_AA)

        # Display face2robot
        text, x_start, y_start = "face2robot", x_start, (y_start + y_step)
        val = "Yes" if data_factory.watchToRobot == 1 else "No"
        self.draw_absolute_value(text, image, x_start, y_start, val)

        # Display hand_high
        text, x_start, y_start = "hand_high", x_start, (y_start + y_step)
        val = "High" if data_factory.handHigh == 1 else "Low"
        self.draw_absolute_value(text, image, x_start, y_start, val)

        # Display hand_inner_side
        text, x_start, y_start = "hand_inner_side", x_start, (y_start + y_step)
        val = "Inner" if data_factory.handInnerSide == 1 else "Outter"
        self.draw_absolute_value(text, image, x_start, y_start, val)

        # Display wrist_shoulder_ratio
        text, x_start, y_start = "wrist_shoulder_ratio", x_start, (y_start + y_step)
        self.draw_absolute_value(text, image, x_start, y_start, str(data_factory.wrist_shoulder_ratio))

        # Display wrist_height_center_m
        text, x_start, y_start = "wrist_height_center_m", x_start, (y_start + y_step)
        self.draw_absolute_value(text, image, x_start, y_start, str(data_factory.wrist_dist_height_from_center_m))

        # Display wrist_width_center_m
        text, x_start, y_start = "wrist_width_center_m", x_start, (y_start + y_step)
        self.draw_absolute_value(text, image, x_start, y_start, str(data_factory.wrist_dist_width_from_center_m))

        # Display wrist_width_center_m
        text, x_start, y_start = "visibility_wrist_r", x_start, (y_start + y_step)
        self.draw_absolute_value(text, image, x_start, y_start, str(data_factory.wrist_r_abs[3]))

        ############################################
        #   Display Results from Object Detection  #
        ############################################
        # Display unit
        y_start = (y_start + 2 * y_step)
        cv2.putText(image, ""
                    , ((x_start_xyz + 80), y_start), self.text_normal, font_size, self.textColor, 1, cv2.LINE_AA)

        # Display name
        y_start = y_start + y_step
        cv2.putText(image, "Objects"
                    , ((x_start - 3), y_start), self.text_bold, font_size, self.textColor, 1, cv2.LINE_AA)

        # Display bool object detected
        text, x_start, y_start = "Object detected", x_start, (y_start + y_step)
        object_detected = True if data_factory.object_detection.detected_objects[1] != 0 else False
        self.draw_absolute_value(text, image, (x_start), y_start, str(object_detected), indent_for_val=-40)

        # Display center
        text, x_start, y_start = "center", x_start, (y_start + y_step)
        self.draw_absolute_value(text, image, x_start, y_start, str(data_factory.object_detection.detected_objects[0]),
                                 indent_for_val=-40)

        # Display angle
        text, x_start, y_start = "angle", x_start, (y_start + y_step)
        # print(data_factory.object_detection.detected_objects)
        self.draw_absolute_value(text, image, x_start, y_start, str(data_factory.object_detection.detected_objects[3]),
                                 indent_for_val=-40)

        return image

    def draw_ros_status(self, image, title, name, status, status_text, x_start, y_start, x_step1, y_step1):
        """
        Draw Information regarding ROS status.
        Draws ROS-Ip, whether ROS-Master is online and whether ROS-Node is initialized.
        """
        # name
        cv2.putText(image, title, (x_start, y_start), self.text_bold, 0.6, self.textColor, 1, cv2.LINE_AA)

        # name of node
        cv2.putText(image, name, (x_step1, y_start), self.text_normal, 0.6, self.textColor, 1, cv2.LINE_AA)

        # status
        status_color = self.kit_march_green if status is True else self.kit_red
        cv2.putText(image, status_text, (x_step1, y_step1), self.text_normal, 0.6, status_color, 1, cv2.LINE_AA)

    def draw_mediapipe_status_text_color(self, image, name, sel, avail, x_start, y_start, x_step1, x_step2):
        """
        Helper Function to draw MediaPipe Status in section.
        """
        # name
        cv2.putText(image, name, (x_start, y_start), self.text_bold, 0.6, self.textColor, 1, cv2.LINE_AA)

        # selected
        sel_color = self.kit_march_green if sel is True else self.kit_red
        cv2.putText(image, "selected", (x_step1, y_start), self.text_normal, 0.6, sel_color, 1, cv2.LINE_AA)

        # available
        avail_color = self.kit_march_green if avail is True else self.kit_red
        cv2.putText(image, "visible", (x_step2, y_start), self.text_normal, 0.6, avail_color, 1, cv2.LINE_AA)

    def draw_landmark_text(self, text, image, x_start, y_start, landmark, mediapipe_results):
        """
        Helper Function to draw landmarks in section.
        """
        font_size = 0.6

        try:
            x = str(int(mediapipe_results.landmark[landmark].x * self.mp_width))[:3]
            y = str(int(mediapipe_results.landmark[landmark].y * self.mp_height))[:3]
            z = str(mediapipe_results.landmark[landmark].z)[:4]
        except:
            x = "0"
            y = "0"
            z = "0"

        cv2.putText(image, text, (x_start, y_start), self.text_normal, font_size, self.textColor, 1, cv2.LINE_AA)

        cv2.putText(image, x, ((x_start + 120), y_start), self.text_normal, font_size, self.textColor, 1, cv2.LINE_AA)
        cv2.putText(image, y, ((x_start + 170), y_start), self.text_normal, font_size, self.textColor, 1, cv2.LINE_AA)
        cv2.putText(image, z, ((x_start + 220), y_start), self.text_normal, font_size, self.textColor, 1, cv2.LINE_AA)

    def draw_absolute_value(self, text, image, x_start, y_start, valueString, indent_for_val=0):
        """
        Helper Function to draw absolute values in section.
        """
        font_size = 0.6

        cv2.putText(image, text
                    , (x_start, y_start), self.text_normal, font_size, self.textColor, 1, cv2.LINE_AA)
        try:
            cv2.putText(image, valueString
                        , ((x_start + (230 - abs(indent_for_val))), y_start), self.text_normal, font_size,
                        self.textColor, 1, cv2.LINE_AA)
        except:
            cv2.putText(image, "0"
                        , ((x_start + (230)), y_start), self.text_normal, font_size, self.textColor, 1, cv2.LINE_AA)

    def draw_status_section(self, image, mp_sel, mp_avail, ros_interface=None, file=None, class_name=None):
        """
        Extend Image.
        Draw MediaPipe Status.
        Draw ROS-Status.
        """
        ##### Extend image #####
        container_width = self.image_width + self.coordinate_section_width

        arr = np.zeros((self.status_section_height, self.composite_width, 3), dtype=np.uint8)
        image = np.concatenate((arr, image))

        self.draw_mediapipe_status_text_color(image, "pose", mp_sel[0], mp_avail[0], 20, 40, 150, 250)
        self.draw_mediapipe_status_text_color(image, "face", mp_sel[1], mp_avail[1], 20, 60, 150, 250)
        self.draw_mediapipe_status_text_color(image, "left_hand", mp_sel[2], mp_avail[2], 20, 80, 150, 250)
        self.draw_mediapipe_status_text_color(image, "right_hand", mp_sel[3], mp_avail[3], 20, 100, 150, 250)

        ##### Draw ROS_Status #####
        half_screen_width = int((self.coordinate_section_width + self.image_width) / 2)

        if ros_interface is not None:
            master_ip = ros_interface.master_ip
            master_is_avail = ros_interface.master_available
            self.draw_ros_status(image, "ros_master", master_ip, master_is_avail, "online",
                                 half_screen_width, 40, (half_screen_width + 120), 60)

            node_name = ros_interface.node_name
            node_initialized = ros_interface.node_initialized
            self.draw_ros_status(image, "ros_node", node_name, node_initialized, "initialized",
                                 half_screen_width, 80, (half_screen_width + 120), 100)
            # self.draw_ros_status(image, "ros_topic", rosSelf.topic_name, True, "", (half_screen_width + 10), 120,
            #                      (half_screen_width + 150), 140)

        if file is not None:
            self.draw_ros_status(image, "class:", class_name, True, "",
                                 half_screen_width, 40, (half_screen_width + 120), 60)
            self.draw_ros_status(image, "file:", file, True, "",
                                 half_screen_width, 60, (half_screen_width + 120), 100)

        return image

    def draw_face_direction_borders(self, image, data_factory):
        """
        Helper Function to draw Borders for HeadPosture within picture.
        """

        # Draw Lines left and right
        cv2.line(image, (data_factory.watchToRobot_indent_y, 0),
                 (data_factory.watchToRobot_indent_y, self.image_height), self.gray)
        cv2.line(image, ((self.image_width - data_factory.watchToRobot_indent_y), 0),
                 ((self.image_width - data_factory.watchToRobot_indent_y), self.image_height), self.gray)

    def draw_mp_coordinate_system(self, image):
        """
        Helper Function to draw MediaPipe's Coordinate System within picture.
        """
        start_point = (20, 20)
        end_point = (60, 20)
        thickness = 2
        cv2.arrowedLine(image, start_point, end_point,
                        self.kit_green, thickness)

        origin = (end_point[0] - 15, end_point[1] - 6)
        cv2.putText(image, "y", origin, self.text_bold, 0.5, self.kit_green)

        start_point = (20, 20)
        end_point = (20, 60)
        thickness = 2
        cv2.arrowedLine(image, start_point, end_point,
                        self.kit_green, thickness)

        origin = (end_point[0] - 12, end_point[1] - 12)
        cv2.putText(image, "x", origin, self.text_bold, 0.5, self.kit_green)
