import math
from .distance import Distance
from .head_posture import HeadPosture
from .object_detection import ObjectDetection


class FeatureFactory:
    def __init__(self, WIDTH, HEIGHT, detect_objects=False):

        self.WIDTH = WIDTH
        self.HEIGHT = HEIGHT

        self.normalize_factor_for_distance = 10

        # Nose
        self.absolute_z_nose = 0  # Absoluter Abstand zur Nase, normalized
        self.absolute_z_nose_mean_10 = 0  # Durchschnittlicher absoluter Abstand von 10 Punkten zu Nase, normalized
        self.nose_direction_x = 0  # Richtung (Endpunkt der Linie) der Nase in x, normalized
        self.nose_direction_y = 0  # Richtung (Endpunkt der Linie) der Nase in y, normalized
        self.nose_from_head_posture = [0, 0]
        self.head_post = [0, 0]
        self.nose_abs = [0, 0, 0, 0]

        # Wrist
        self.z_left_wrist = 0  # Absoluter Abstand zum Handgelenk, normalized
        self.z_right_wrist = 0  # Absoluter Abstand zum Handgelenk, normalized
        self.absolute_z_left_wrist_mean_10 = 0  # Durchschnittlicher absoluter Abstand von 10 Pkt., normalized
        self.absolute_z_right_wrist_mean_10 = 0  # Durchschnittlicher absoluter Abstand von 10 Pkt., normalized
        self.wrist_l_abs = [0, 0, 0, 0]
        self.wrist_r_abs = [0, 0, 0, 0]

        # Wrist from pose
        self.wrist_pose_z_mean_10 = 0
        self.wrist_pose_r_abs = [0, 0, 0, 0]

        # Eyes
        self.absolute_z_leftEye_mean_10 = 0  # Durchschnittlicher absoluter Abstand von 10 Pkt., normalized
        self.absolute_z_rightEye_mean_10 = 0  # Durchschnittlicher absoluter Abstand von 10 Pkt., normalized
        self.eye_l_abs = [0, 0, 0, 0]
        self.eye_r_abs = [0, 0, 0, 0]

        # Shoulder
        self.absolute_z_leftShoulder_m10 = 0  # Durchschnittlicher absoluter Abstand von 10 Pkt., normalized
        self.absolute_z_rightShoulder_m10 = 0  # Durchschnittlicher absoluter Abstand von 10 Pkt., normalized
        self.shoulder_l_abs = [0, 0, 0, 0]
        self.shoulder_r_abs = [0, 0, 0, 0]

        # Elbow
        self.abs_z_elbow_r_m10 = 0  # Durchschnittlicher absoluter Abstand von 10 Pkt., normalized
        self.elbow_r_abs = [0, 0, 0, 0]

        # Watch to Robot
        self.watchToRobot = 0  # Blickt die Person in Richtung Roboter, bool
        self.watchToRobot_indent_x = 50  # Border rechts links für Entscheidung, px
        self.watchToRobot_indent_y = 80  # Border oben unten für Entscheidung, px

        # Other
        self.wrist_shoulder_ratio = 0  # Handgelenk oberhalb, oder unterhalb der Schulter, oben = 1
        self.handHigh = 0  # Fingerspitzen zeigen nach oben oder unten, oben = 1
        self.handInnerSide = 0  # Innen- oder Außenfläche der Hand sichtbar, innen = 1
        self.hiGesture = 0  # Hand offen, offen = 1

        # Wrist distance in meter, für ros-Koordinatensystem
        self.wrist_dist_height_from_center_m = 0
        self.wrist_dist_width_from_center_m = 0
        self.right_wrist_z_m = 0
        self.wrist_coords = [0, 0, 0]

        # Gesture to Recover from Error
        self.recover_from_error = False

        # Classes for feature calculation
        self.distance = Distance(self.WIDTH, self.HEIGHT)
        self.head_posture = HeadPosture()

        self.detect_objects = detect_objects
        self.object_detection = ObjectDetection()
        self.draw_objects = None

    def calculate_features(self, results, depth_frame, image, depth_image, color_image, rs_interface):
        '''
        This function is responsible to calculate all features except MediaPipe landmarks..

        :param results: from MediaPipe
        :param image: image to draw on
        :param depth_frame: for directly request distance
        :return: -
        '''
        # Get Nose coordinates
        self.absolute_z_nose_mean_10 = 0
        self.absolute_z_nose = 0
        self.nose_direction_x = 0
        self.nose_direction_y = 0
        self.watchToRobot = 0
        self.nose_abs = [0, 0, 0, 0]
        if results.face_landmarks is not None:

            ############################################
            #         Get Nose Distance First          #
            ############################################

            self.absolute_z_nose_mean_10, center = \
                self.distance.get_distance_from_two_lm(
                    results.face_landmarks,
                    5,
                    95,
                    depth_frame,
                    mean=True,
                    image=None,
                    normalize_factor=self.normalize_factor_for_distance
                )

            self.nose_abs[0] = center[0]
            self.nose_abs[1] = center[1]
            self.nose_abs[2] = self.absolute_z_nose_mean_10
            self.nose_abs[3] = results.pose_landmarks.landmark[0].visibility

            ############################################
            #         Calculate Nose Direction         #
            ############################################

            self.head_post = [0, 0]
            self.nose_direction_x, self.nose_direction_y, self.nose_from_head_posture = \
                self.head_posture.get_head_posture(results,
                                                   (self.absolute_z_nose_mean_10 * self.normalize_factor_for_distance),
                                                   self.WIDTH,
                                                   self.HEIGHT,
                                                   image=None)

            self.head_post[0] = self.nose_direction_x
            self.head_post[1] = self.nose_direction_y

            ############################################
            #          Calculate Facing Robot          #
            ############################################

            if self.watchToRobot_indent_y < (self.nose_direction_y * self.HEIGHT) < (
                    self.HEIGHT - self.watchToRobot_indent_y):
                # print("schaut Richtung robot")
                self.watchToRobot = 1

        ############################################
        #        Get Other Absolute Distances      #
        ############################################

        # Get Left Wrist Coordinate
        self.absolute_z_left_wrist_mean_10 = 0
        self.z_left_wrist = 0
        self.wrist_l_abs = [0, 0, 0, 0]
        if results.left_hand_landmarks is not None:
            self.absolute_z_left_wrist_mean_10, center = \
                self.distance.get_distance_from_two_lm(
                    results.left_hand_landmarks,
                    0,
                    9,
                    depth_frame,
                    mean=True,
                    image=None,
                    normalize_factor=self.normalize_factor_for_distance)

            self.wrist_l_abs[0] = center[0]
            self.wrist_l_abs[1] = center[1]
            self.wrist_l_abs[2] = self.absolute_z_left_wrist_mean_10
            self.wrist_l_abs[3] = results.pose_landmarks.landmark[15].visibility

        # Get Right Wrist Coordinate
        self.absolute_z_right_wrist_mean_10 = 0.0
        self.z_right_wrist = 0
        self.wrist_r_abs = [0, 0, 0, 0]
        self.wrist_pose_r_abs = [0, 0, 0, 0]
        if results.right_hand_landmarks is not None:
            self.absolute_z_right_wrist_mean_10, center = \
                self.distance.get_distance_from_two_lm(
                    results.right_hand_landmarks,
                    0,
                    9,
                    depth_frame,
                    mean=True,
                    image=None,
                    normalize_factor=self.normalize_factor_for_distance)

            self.wrist_r_abs[0] = center[0]
            self.wrist_r_abs[1] = center[1]
            self.wrist_r_abs[2] = self.absolute_z_right_wrist_mean_10
            self.wrist_r_abs[3] = results.pose_landmarks.landmark[16].visibility

            self.wrist_pose_z_mean_10, center = \
                self.distance.get_distance_from_two_lm(
                    results.pose_landmarks,
                    16,
                    16,
                    depth_frame,
                    mean=True,
                    image=None,
                    normalize_factor=self.normalize_factor_for_distance)

            self.wrist_pose_r_abs[0] = center[0]
            self.wrist_pose_r_abs[1] = center[1]
            self.wrist_pose_r_abs[2] = self.wrist_pose_z_mean_10
            self.wrist_pose_r_abs[3] = results.pose_landmarks.landmark[16].visibility

        self.absolute_z_leftEye_mean_10 = 0
        self.absolute_z_rightEye_mean_10 = 0
        self.eye_l_abs = [0, 0, 0, 0]
        self.eye_r_abs = [0, 0, 0, 0]
        if results.face_landmarks is not None:
            self.absolute_z_leftEye_mean_10, center = \
                self.distance.get_distance_from_two_lm(
                    results.face_landmarks,
                    105,
                    104,
                    depth_frame,
                    mean=True,
                    image=None,
                    normalize_factor=self.normalize_factor_for_distance)

            self.eye_l_abs[0] = center[0]
            self.eye_l_abs[1] = center[1]
            self.eye_l_abs[2] = self.absolute_z_leftEye_mean_10
            self.eye_l_abs[3] = results.pose_landmarks.landmark[0].visibility

            self.absolute_z_rightEye_mean_10, center = \
                self.distance.get_distance_from_two_lm(
                    results.face_landmarks,
                    334,
                    333,
                    depth_frame,
                    mean=True,
                    image=None,
                    normalize_factor=self.normalize_factor_for_distance)

            self.eye_r_abs[0] = center[0]
            self.eye_r_abs[1] = center[1]
            self.eye_r_abs[2] = self.absolute_z_rightEye_mean_10
            self.eye_r_abs[3] = results.pose_landmarks.landmark[0].visibility

        self.absolute_z_leftShoulder_m10 = 0
        self.absolute_z_rightShoulder_m10 = 0
        self.shoulder_r_abs = [0, 0, 0, 0]
        if results.pose_landmarks is not None:
            self.absolute_z_rightShoulder_m10, center = \
                self.distance.get_distance_from_two_lm(
                    results.pose_landmarks,
                    12,
                    12,
                    depth_frame,
                    mean=True,
                    image=None,
                    normalize_factor=self.normalize_factor_for_distance)

            self.shoulder_r_abs[0] = center[0]
            self.shoulder_r_abs[1] = center[1]
            self.shoulder_r_abs[2] = self.absolute_z_rightShoulder_m10
            self.shoulder_r_abs[3] = results.pose_landmarks.landmark[12].visibility

            self.shoulder_l_abs = [0, 0, 0, 0]
            self.absolute_z_leftShoulder_m10, center = \
                self.distance.get_distance_from_two_lm(
                    results.pose_landmarks,
                    11,
                    11,
                    depth_frame,
                    mean=True,
                    image=None,
                    normalize_factor=self.normalize_factor_for_distance)

            self.shoulder_l_abs[0] = center[0]
            self.shoulder_l_abs[1] = center[1]
            self.shoulder_l_abs[2] = self.absolute_z_leftShoulder_m10
            self.shoulder_l_abs[3] = results.pose_landmarks.landmark[11].visibility

        self.abs_z_elbow_r_m10 = 0
        self.elbow_r_abs = [0, 0, 0, 0]
        if results.pose_landmarks is not None:
            self.abs_z_elbow_r_m10, center = \
                self.distance.get_distance_from_two_lm(
                    results.pose_landmarks,
                    14,
                    14,
                    depth_frame,
                    mean=True,
                    image=None,
                    normalize_factor=self.normalize_factor_for_distance)

            self.elbow_r_abs[0] = center[0]
            self.elbow_r_abs[1] = center[1]
            self.elbow_r_abs[2] = self.abs_z_elbow_r_m10
            self.elbow_r_abs[3] = results.pose_landmarks.landmark[14].visibility

        ############################################
        #            Calculate Features            #
        ############################################
        self.wrist_shoulder_ratio = None
        if results.pose_landmarks is not None:
            right_shoulder_x = results.pose_landmarks.landmark[12].x
            right_wrist_x = results.pose_landmarks.landmark[16].x

            if (right_shoulder_x - right_wrist_x) <= 0:
                self.wrist_shoulder_ratio = 1
            else:
                self.wrist_shoulder_ratio = 0

        if results.right_hand_landmarks is not None:
            wrist_x = results.right_hand_landmarks.landmark[0].x
            wrist_y = results.right_hand_landmarks.landmark[0].y

            middle_finger_x = results.right_hand_landmarks.landmark[12].x
            middle_finger_y = results.right_hand_landmarks.landmark[12].y

            thump_x = results.right_hand_landmarks.landmark[4].x
            thump_y = results.right_hand_landmarks.landmark[4].y

            self.handHigh, self.handInnerSide = 0, 0
            if (wrist_x - middle_finger_x) > 0:
                self.handHigh = 0
                # print("Hand zeigt nach UNTEN")

                if (thump_y - wrist_y) < 0:
                    self.handInnerSide = 1
                    # print("HAND INNENFLÄCHE")
                else:
                    self.handInnerSide = 0
                    # print("HANDRÜCKEN")

            else:
                self.handHigh = 1
                # print("Hand zeigt nach OBEN")

                if (thump_y - wrist_y) > 0:
                    self.handInnerSide = 1
                    # print("HAND INNENFLÄCHE")
                else:
                    self.handInnerSide = 0
                    # print("HANDRÜCKEN")

        ############################################
        #        Calculate wrist z in meter        #
        ############################################
        if results.right_hand_landmarks is not None:
            right_wrist_z = self.absolute_z_right_wrist_mean_10
            self.right_wrist_z_m = round((right_wrist_z * self.normalize_factor_for_distance), 5)

            # Get Coordinates
            right_wrist_x = int(((results.right_hand_landmarks.landmark[0].x * self.WIDTH) + (
                    results.right_hand_landmarks.landmark[9].x * self.WIDTH)) / 2)

            right_wrist_y = int(((results.right_hand_landmarks.landmark[0].y * self.HEIGHT) + (
                    results.right_hand_landmarks.landmark[9].y * self.HEIGHT)) / 2)

            # width/2 because picture is flipped
            fov_width = 22  # Sichtfeld laut datasheet 55° in der Breite, Hälfte = 22,5 ; musste korrigieren weil die Werte sonst nicht gestimmt haben
            flippedPicture_width_m = (math.tan(math.radians(fov_width)) * self.right_wrist_z_m) * 2
            flippedPicture_pixelWidth_m = flippedPicture_width_m / self.HEIGHT  # durch height, weil Bild gedreht
            wrist_distance_from_right_m = flippedPicture_pixelWidth_m * right_wrist_y
            self.wrist_dist_width_from_center_m = round(((flippedPicture_width_m / 2) - wrist_distance_from_right_m),
                                                        3) * -1

            # Calculate x in meter (das Bild ist gedreht, x ist die Höhe)
            # width/2 because picture is flipped
            fov_height = 28  # Sichtfeld laut datasheet 70° in der Höhe, Hälfte = 35
            flippedPicture_height_m = (math.tan(math.radians(fov_height)) * self.right_wrist_z_m) * 2
            flippedPicture_pixelHeight_m = flippedPicture_height_m / self.WIDTH  # durch width, weil Bild gedreht
            wrist_distance_from_bottom_m = flippedPicture_pixelHeight_m * right_wrist_x
            self.wrist_dist_height_from_center_m = round(((flippedPicture_height_m / 2) - wrist_distance_from_bottom_m),
                                                         3) * -1

            self.wrist_coords = [self.wrist_dist_height_from_center_m,
                            self.wrist_dist_width_from_center_m,
                            self.right_wrist_z_m]

        ############################################
        #          Apply Object Detection          #
        ############################################
        if self.detect_objects:
            self.object_detection.detect_objects(image, color_image, depth_image, rs_interface, self)
        else:
            pass

        ############################################
        #  Calculate gesture to recover from error #
        ############################################
        self.recover_from_error = False
        if results.right_hand_landmarks is not None:
            tip_finger_x = results.right_hand_landmarks.landmark[8].x
            middle_finger_x = results.right_hand_landmarks.landmark[12].x
            ring_finger_x = results.right_hand_landmarks.landmark[16].x
            pinky_finger_x = results.right_hand_landmarks.landmark[20].x

            tip_finger_y = results.right_hand_landmarks.landmark[8].y
            thump_y = results.right_hand_landmarks.landmark[4].y

            if tip_finger_x > middle_finger_x and \
                    tip_finger_x > ring_finger_x and \
                    tip_finger_x > pinky_finger_x and \
                    thump_y < tip_finger_y and \
                    self.handInnerSide == 1 and \
                    self.handHigh == 1:
                self.recover_from_error = True

        ##### Calculate hi gesture
        self.hiGesture = False
        if results.right_hand_landmarks is not None:
            thump_finger_x_dip = results.right_hand_landmarks.landmark[4].x
            index_finger_x_dip = results.right_hand_landmarks.landmark[8].x
            middle_finger_x_dip = results.right_hand_landmarks.landmark[12].x
            ring_finger_x_dip = results.right_hand_landmarks.landmark[16].x
            pinky_finger_x_dip = results.right_hand_landmarks.landmark[20].x

            thump_finger_x_mcp = results.right_hand_landmarks.landmark[2].x
            index_finger_x_mcp = results.right_hand_landmarks.landmark[5].x
            middle_finger_x_mcp = results.right_hand_landmarks.landmark[9].x
            ring_finger_x_mcp = results.right_hand_landmarks.landmark[13].x
            pinky_finger_x_mcp = results.right_hand_landmarks.landmark[17].x

            tip_finger_y = results.right_hand_landmarks.landmark[8].y
            thump_y = results.right_hand_landmarks.landmark[4].y

            if thump_y > tip_finger_y and \
                    thump_finger_x_dip > thump_finger_x_mcp > wrist_x and \
                    index_finger_x_dip > index_finger_x_mcp > wrist_x and \
                    middle_finger_x_dip > middle_finger_x_mcp > wrist_x and \
                    ring_finger_x_dip > ring_finger_x_mcp > wrist_x and \
                    pinky_finger_x_dip > pinky_finger_x_mcp > wrist_x and \
                    self.handInnerSide == 1 and \
                    self.handHigh == 1:
                self.hiGesture = True
