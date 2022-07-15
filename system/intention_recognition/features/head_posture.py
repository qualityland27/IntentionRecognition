import cv2
import numpy as np
from .custom.face_geometry import (
    PCF,
    get_metric_landmarks,
    procrustes_landmark_basis,
)


class HeadPosture:
    def __init__(self):
        pass

    def get_head_posture(self, results, absolute_z_nose, width, height, image=None):

        # p1 = nose; p2 ende der Geraden
        p1, p2 = self.calculate_headPosture(results, absolute_z_nose, width, height)

        if image is not None:
            image = cv2.line(image, p1, p2, (252, 229, 0), 2)
            # image = cv2.circle(image, (p2[0], p2[1]), 5, (252, 229, 0), -1)
            # cv2.putText(image, str(p2), p2, cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 0))

        # normalize x- and y-Coordinate
        nose_direction_x, nose_direction_y = (p2[0] / width), (p2[1] / height)

        return nose_direction_x, nose_direction_y, p1

    def calculate_headPosture(self, results, absolute_z_nose, width, height):

        points_idx = [33, 263, 61, 291, 199]
        points_idx = points_idx + [key for (key, val) in procrustes_landmark_basis]
        points_idx = list(set(points_idx))
        points_idx.sort()

        # uncomment next line to use all points for PnP algorithm
        # points_idx = list(range(0,468)); points_idx[0:2] = points_idx[0:2:-1];

        frame_width, frame_height, channels = (width, height, 3)

        # pseudo camera internals
        focal_length = frame_width
        center = (frame_width / 2, frame_height / 2)
        camera_matrix = np.array(
            [[focal_length, 0, center[0]], [0, focal_length, center[1]], [0, 0, 1]],
            dtype="double",
        )

        dist_coeff = np.zeros((4, 1))

        pcf = PCF(
            near=1,
            far=10000,
            frame_height=frame_height,
            frame_width=frame_width,
            fy=camera_matrix[1, 1],
        )

        face_landmarks = results.face_landmarks
        landmarks = np.array(
            [(lm.x, lm.y, lm.z) for lm in face_landmarks.landmark]
        )
        # print(landmarks.shape)
        landmarks = landmarks.T

        metric_landmarks, pose_transform_mat = get_metric_landmarks(
            landmarks.copy(), pcf
        )
        model_points = metric_landmarks[0:3, points_idx].T
        image_points = (
            landmarks[0:2, points_idx].T
            * np.array([frame_width, frame_height])[None, :]
        )


        # Finds an object pose from 3D-2D point correspondences. This function returns the rotation and the translation vectors
        # that transform a 3D point expressed in the object coordinate frame to the camera coordinate frame, using different methods.
        success, rotation_vector, translation_vector = cv2.solvePnP(
            model_points,
            image_points,
            camera_matrix,
            dist_coeff,
            flags=cv2.cv2.SOLVEPNP_ITERATIVE,
        )

        if absolute_z_nose == 0.0:
            length_of_line = 50.0
        else:
            length_of_line = absolute_z_nose * 50

        if focal_length <= 50.0:
            length_of_line = 50.0

        # Projects 3D points to an image plane.
        (nose_end_point2D, jacobian) = cv2.projectPoints(
            np.array([(0.0, 0.0, length_of_line)]),
            rotation_vector,
            translation_vector,
            camera_matrix,
            dist_coeff,
        )

        p1 = (int(image_points[0][0]), int(image_points[0][1]))
        p2 = (int(nose_end_point2D[0][0][0]), int(nose_end_point2D[0][0][1]))

        return p1, p2
