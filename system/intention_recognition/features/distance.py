import mediapipe as mp  # Import mediapipe
import cv2  # Import opencv
import pickle
import pandas as pd
import numpy as np
from .head_posture import HeadPosture


class Distance:
    def __init__(self, width, height):
        self.width = width
        self.height = height

    def get_distance_from_two_lm(self, landmark_list, no_of_first_lm, no_of_second_lm, depth_frame, mean=False,
                                 image=None, normalize_factor=None):
        '''
        Calculation of distance based on two landmarks from MediaPipe.

        :param landmark_list:
        :param no_of_first_lm:
        :param no_of_second_lm:
        :param depth_frame:
        :param mean:
        :param image:
        :return: normalized distance
        :return middle of the two points as center
        '''
        lm1 = no_of_first_lm
        lm2 = no_of_second_lm
        absolute_z = 0
        list_10px_z = []
        if landmark_list is not None:
            x1 = landmark_list.landmark[lm1].x * self.width
            y1 = landmark_list.landmark[lm1].y * self.height

            x2 = landmark_list.landmark[lm2].x * self.width
            y2 = landmark_list.landmark[lm2].y * self.height

            x12 = int((x1 + x2) / 2)
            y12 = int((y1 + y2) / 2)

            if mean:
                start = 0
                end = 1
            else:
                start = -1
                end = 2

            for x in range(start, end):
                for y in range(start, end):
                    try:
                        absolute_z = depth_frame.get_distance(x + x12, y + y12)
                        #absolute_z = depth_frame.get_distance((y + y12), (x + x12))
                        #absolute_z = realsenseSelf.get_distance(x + x12, y + y12)
                        if absolute_z == 0:
                            continue
                        list_10px_z.append(absolute_z)

                        if (x == 1) and (y == 1):
                            absolute_z = sum(list_10px_z) / len(list_10px_z)

                        # Normalize Data
                        if normalize_factor is not None:
                            absolute_z /= normalize_factor

                        if image is not None:
                            cv2.circle(image, (x + x12, y + y12), 5, (0, 255, 0), -1)
                    except:
                        pass

            return absolute_z, (x12, y12)

    def get_absolute_z_nose_and_nose_direction(self, results, realsenseSelf, width, height, image=None,
                                               normalize_factor=None):
        if results.face_landmarks is not None:
            nose_x = int(results.face_landmarks.landmark[5].x * width)
            nose_y = int(results.face_landmarks.landmark[5].y * height)
            absolute_z_nose = 0.0
            try:
                # old
                # absolute_z_nose = depth_frame.get_distance(nose_x, nose_y)
                # absolute_z_nose = depth_frame.get_distance(nose_y, nose_x)

                # new
                absolute_z_nose = realsenseSelf.get_distance(nose_x, nose_y)
                # cv2.putText(image, str(absolute_z_nose), (10,20), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 0))
            except:
                pass

            # TODO: Instanz ist nicht richtig
            p1, p2 = HeadPosture.calculate_headPosture("", results, absolute_z_nose, width, height)

            if image is not None:
                image = cv2.circle(image, (nose_x, nose_y), 5, (252, 229, 0), -1)
                image = cv2.line(image, p1, p2, (252, 229, 0), 2)
                # image = cv2.circle(image, (p2[0], p2[1]), 5, (252, 229, 0), -1)
                cv2.putText(image, str(p2), p2, cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 0))

            # normalize x- and y-Coordinate
            nose_direction_x, nose_direction_y = (p2[0] / width), (p2[1] / height)

            if normalize_factor is not None:
                absolute_z_nose /= normalize_factor

            return absolute_z_nose, nose_direction_x, nose_direction_y, image
