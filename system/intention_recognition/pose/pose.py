import mediapipe as mp  # Import mediapipe
import cv2  # Import opencv
import pickle
import pandas as pd
import numpy as np


class Pose:
    """
    Pose. Interface with MediaPipe. Contains everything with MediaPipe.
    """

    def __init__(self, width, height):
        self.mp_drawing = mp.solutions.drawing_utils  # Drawing helpers
        self.mp_holistic = mp.solutions.holistic  # Mediapipe Solutionsnegative

        self.width = width
        self.height = height

    def calculate_coordinates(self, holistic, image):
        '''
        This function calls calculation of MediaPipe landmarks

        :param holistic: model from MediaPipe
        :param image: image for processing
        :return: results, avail - avail tells which coords are available
        '''
        # Flag: Information about the memory layout of the array.
        image.flags.writeable = False

        # Make Detections
        results = holistic.process(image)

        # Flag: Information about the memory layout of the array.
        image.flags.writeable = True

        # Check which Landmarks are avalable
        pose_avail = True if results.pose_landmarks is not None else False
        face_avail = True if results.face_landmarks is not None else False
        left_hand_avail = True if results.left_hand_landmarks is not None else False
        right_hand_avail = True if results.right_hand_landmarks is not None else False

        avail = [pose_avail, face_avail, left_hand_avail, right_hand_avail]

        return results, avail

    def draw_landmarks_from_list(self,
                                 listWithResults,
                                 image,
                                 listWithLandmarksToDraw,
                                 radius=5,
                                 color=(255, 0, 0),
                                 thickness=-1):
        if listWithResults is not None:
            for lm in listWithLandmarksToDraw:
                x = int(listWithResults.landmark[lm].x * self.width)
                y = int(listWithResults.landmark[lm].y * self.height)

                image = cv2.circle(image, (x, y), radius, color, thickness)

            return image

    def draw_landmarks_from_holistic(self, results, image, pose=True, left_hand=False, right_hand=False, face=False):
        '''
        This function draws landmarks from MediaPipe with MediaPipe's drawing tool.

        :param results: from MediaPipe
        :param image: image to draw on
        :param pose: bool - if pose landmarks should be drawn
        :param left_hand: bool - if left_hand landmarks should be drawn
        :param right_hand: bool - if right_hand landmarks should be drawn
        :param face: bool - if face landmarks should be drawn
        :return: -
        '''

        if face:
            # 1. Draw face landmarks
            self.mp_drawing.draw_landmarks(image, results.face_landmarks, self.mp_holistic.FACE_CONNECTIONS,
                                           self.mp_drawing.DrawingSpec(color=(136, 136, 136), thickness=1,
                                                                       circle_radius=1),
                                           self.mp_drawing.DrawingSpec(color=(162, 34, 35), thickness=1,
                                                                       circle_radius=1)
                                           )

        if right_hand:
            # 2. Right hand
            self.mp_drawing.draw_landmarks(image, results.right_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS,
                                           self.mp_drawing.DrawingSpec(color=(136, 136, 136), thickness=2,
                                                                       circle_radius=4),
                                           self.mp_drawing.DrawingSpec(color=(70, 100, 170), thickness=2,
                                                                       circle_radius=2)
                                           )

        if left_hand:
            # 3. Left Hand
            self.mp_drawing.draw_landmarks(image, results.left_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS,
                                           self.mp_drawing.DrawingSpec(color=(136, 136, 136), thickness=2,
                                                                       circle_radius=4),
                                           self.mp_drawing.DrawingSpec(color=(70, 100, 170), thickness=2,
                                                                       circle_radius=2)
                                           )

        if pose:
            # 4. pose Detections
            self.mp_drawing.draw_landmarks(image, results.pose_landmarks, self.mp_holistic.POSE_CONNECTIONS,
                                           self.mp_drawing.DrawingSpec(color=(136, 136, 136), thickness=2,
                                                                       circle_radius=4),
                                           self.mp_drawing.DrawingSpec(color=(0, 150, 130), thickness=2))
