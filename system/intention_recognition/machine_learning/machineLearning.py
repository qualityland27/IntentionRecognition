import csv

import pandas as pd
import numpy as np
import logging


class MachineLearning:
    """Machine Learning.

      This Class holds lot of functions for handling dataframes and how to put
      features within this frame.

      E.G. Extract Landmarks from MediaPipe to the dataframe.
      """
    def __init__(self):
        self.log = logging.getLogger('intention_recognition')

        self.pose = None
        self.face = None
        self.right_hand = None
        self.left_hand = None

        self.sel_arr = None

    def createDataframe(self,
                        additional_features,
                        pose=True,
                        face=False,
                        left_hand=False,
                        right_hand=False,
                        write_to_csv=False,
                        csv_filename=None):
        '''
        This function creates an dataframe (df) which becomes values out of MediaPipe (all landmarks)
        and all additionally calculated features.
        The df will contain multiple lines of landmarks and features.
        Later, all the unnecessary features will be dropped.
        The df will then be used for calculating rolling window.
        The df will then be used for prediction.


        :param face: Boolean Value, if face landmarks are going to be calculated;
        :param left_hand: Boolean Value, if left_hand landmarks are going to be calculated;
        :param right_hand: Boolean Value, if right_hand landmarks are going to be calculated;
        :param additional_features:  Array with features additional to landmarks from MediaPipe
        :return: DataFrame containing ALL landmarks and additional calculated features and
        Boolean Value for success
        '''
        self.pose = pose
        self.face = face
        self.right_hand = right_hand
        self.left_hand = left_hand

        self.sel_arr = [self.pose,
                        self.face,
                        self.left_hand,
                        self.right_hand]

        # Prepare Dataframe. Dataframe is needed to put all values in and to make detection.
        success = False
        try:
            num_coords = 33
            self.log.info('Dataframe includes landmarks from pose')
            if face:
                num_coords += 468
                self.log.info('Dataframe includes landmarks from face')
            if left_hand:
                num_coords += 21
                self.log.info('Dataframe includes landmarks from left_hand')
            if right_hand:
                num_coords += 21
                self.log.info('Dataframe includes landmarks from right_hand')

            # add x,y,z,v for each landmark
            for val in range(1, num_coords + 1):
                additional_features += ['x{}'.format(val), 'y{}'.format(val), 'z{}'.format(val), 'v{}'.format(val)]

            if write_to_csv:
                with open(csv_filename + '.csv', mode='w', newline='') as f:
                    csv_writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                    csv_writer.writerow(additional_features)

            df = pd.DataFrame(columns=additional_features, dtype='float32')
            n = df.shape[1]

            self.log.info('Dataframe successfully created. Number of Columns: ' + str(n))

            return df

        except Exception as e:
            self.log.error("Dataframe could not be created", str(e))

    def export_coordinates(self, results):
        '''
        This function export coordinates from MediaPipe to a single row and returns this row.
        Its depending on which landmarks are wanted to be tracked in the row.
        Pose Landmarks are always getting tracked and exported.

        Note: This function does not work for get-data.py and vice verca.
        '''
        # try:
        #     pose_row = MachineLearning.extractPoseLandmarks(self, results.pose_landmarks.landmark)
        # except:
        #     self.log.debug("no pose data available")

        pose_row = MachineLearning.extractLandmarksIfAvailable(self, results.pose_landmarks.landmark, 32)

        if self.face:
            face_row = MachineLearning.extractLandmarksIfAvailable(self, results.face_landmarks.landmark, 1872)

        if self.left_hand:
            left_hand_row = MachineLearning.extractLandmarksIfAvailable(self, results.left_hand_landmarks.landmark,
                                                                        84)
        if self.right_hand:
            right_hand_row = MachineLearning.extractLandmarksIfAvailable(self, results.right_hand_landmarks.landmark,
                                                                         84)
        # Concate rows
        # Funktioniert nicht
        # row = machine_learning.concate_rows(pose_row, face_row, left_hand_row, right_hand_row)

        # Concate rows
        row = pose_row  # 33 * 4 = 132
        if self.face:
            row = row + face_row  # 468 * 4 = 1872
        if self.left_hand:
            row += left_hand_row  # 21 * 4 = 84
        if self.right_hand:
            row += right_hand_row  # 21 * 4 = 84

        return row

    def export_coordinates_for_get_data(self, results):
        '''
        This function export coordinates from MediaPipe to a single row and returns this row.
        Its depending on which landmarks are wanted to be tracked in the row.
        Pose Landmarks are always getting tracked and exported.

        Note: This function does not work for make-detection.py and vice verca.
        '''
                # Extract Pose landmarks
        pose = results.pose_landmarks.landmark
        pose_row = list(np.array(
            [[landmark.x, landmark.y, landmark.z, landmark.visibility] for landmark in
             pose]).flatten())

        if self.face:
            try:
                # Extract Face landmarks
                face = results.face_landmarks.landmark
                face_row = list(np.array(
                    [[landmark.x, landmark.y, landmark.z, landmark.visibility] for landmark in
                     face]).flatten())
            except:
                face_row = []
                for i in range(1872):
                    face_row.append(0)

        if self.left_hand:
            try:
                # Extract right hand landmarks
                right_hand = results.right_hand_landmarks.landmark
                right_hand_row = list(np.array(
                    [[landmark.x, landmark.y, landmark.z, landmark.visibility] for landmark in
                     right_hand]).flatten())
            except:
                right_hand_row = []
                for i in range(84):
                    right_hand_row.append(0)

        if self.right_hand:
            try:
                # Extract left hand landmarks
                left_hand = results.right_hand_landmarks.landmark
                left_hand_row = list(np.array(
                    [[landmark.x, landmark.y, landmark.z, landmark.visibility] for landmark in
                     left_hand]).flatten())
            except:
                left_hand_row = []
                for i in range(84):
                    left_hand_row.append(0)

        # Concate rows
        row = pose_row  # 33 * 4 = 132
        if self.face:
            row = row + face_row  # 468 * 4 = 1872
        if self.left_hand:
            row += left_hand_row  # 21 * 4 = 84
        if self.right_hand:
            row += right_hand_row  # 21 * 4 = 84

        return row

    def extractPoseLandmarks(self, results):
        '''
        This function extracts Landmarks from MediaPipe pose to a row.
        These results are necessary for the application to run.
        The row can be inserted into a dataframe later.

        :param results: Mediapipe results. Example: results.pose_landmarks.landmark
        :return:
        '''

        pose_results = results
        pose_row = list(np.array(
            [[landmark.x, landmark.y, landmark.z, landmark.visibility] for landmark in
             pose_results]).flatten())
        return pose_row

    def extractLandmarksIfAvailable(self, results, length_of_result_list):
        '''
        This function extracts Landmarks from MediaPipe to a row.
        If the results are not available to function fills the ros with 0

        :param results: Mediapipe results. Example: results.right_hand_landmarks.landmark
        :param length_of_result_list: length of the results list, needed to fill with zeros
        :return: row
        '''

        try:
            # Extract landmarks
            row = list(np.array(
                [[landmark.x, landmark.y, landmark.z, landmark.visibility] for landmark in
                 results]).flatten())
        except:
            row = []
            for i in range(length_of_result_list):
                row.append(0)
            # self.log.debug("mediapipe {} not evailable".format(str(results)))
            # print("WARNING: mediapipe FACE not evailable")
            print("des lag am LOGEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEENN")

        return row

    def apply_rolling(self,
                      rolling_window,
                      no_rolling_features_arr,
                      size4=False,
                      size8=False,
                      size12=False,
                      size16=False):
        '''
        This function calculates rolling over features in features array from .env.
        '''

        for col_name in rolling_window:
            if col_name not in no_rolling_features_arr:
                if size16:
                    rolling_window[col_name + '_moving_avg_16'] = rolling_window[col_name].rolling(16).mean()
                    rolling_window[col_name + '_moving_sum_16'] = rolling_window[col_name].rolling(16).sum()
                    rolling_window[col_name + '_moving_var_16'] = rolling_window[col_name].rolling(16).var()
                    rolling_window[col_name + '_moving_std_16'] = rolling_window[col_name].rolling(16).std()

                if size12:
                    rolling_window[col_name + '_moving_avg_12'] = rolling_window[col_name].rolling(12).mean()
                    rolling_window[col_name + '_moving_sum_12'] = rolling_window[col_name].rolling(12).sum()
                    rolling_window[col_name + '_moving_var_12'] = rolling_window[col_name].rolling(12).var()
                    rolling_window[col_name + '_moving_std_12'] = rolling_window[col_name].rolling(12).std()

                if size8:
                    rolling_window[col_name + '_moving_avg_8'] = rolling_window[col_name].rolling(8).mean()
                    rolling_window[col_name + '_moving_sum_8'] = rolling_window[col_name].rolling(8).sum()
                    rolling_window[col_name + '_moving_var_8'] = rolling_window[col_name].rolling(8).var()
                    rolling_window[col_name + '_moving_std_8'] = rolling_window[col_name].rolling(8).std()

                if size4:
                    rolling_window[col_name + '_moving_avg_4'] = rolling_window[col_name].rolling(4).mean()
                    rolling_window[col_name + '_moving_sum_4'] = rolling_window[col_name].rolling(4).sum()
                    rolling_window[col_name + '_moving_var_4'] = rolling_window[col_name].rolling(4).var()
                    rolling_window[col_name + '_moving_std_4'] = rolling_window[col_name].rolling(4).std()

        return rolling_window

    def concate_rows(self, pose_row, face_row, left_hand_row, right_hand_row):
        '''
        This function brings all rows together. For this it takes the single row with landmarks and puts them
        together.
        '''
        row = pose_row  # 33 * 4 = 132
        if self.face:
            row = row + face_row  # 468 * 4 = 1872
        if self.left_hand:
            row += left_hand_row  # 21 * 4 = 84
        if self.right_hand:
            row += right_hand_row  # 21 * 4 = 84

        return row

    def append_additionals_to_row(self, row, featureSelf, class_name):
        '''
        This function put additional calculated features to the row. It returns the row.
        '''
        row.insert(0, class_name)  # Inhaltlich nicht relevant, nur für #Spalten
        row.insert(1, featureSelf.absolute_z_nose)
        row.insert(2, featureSelf.absolute_z_nose_mean_10)
        row.insert(3, featureSelf.nose_direction_x)
        row.insert(4, featureSelf.nose_direction_y)
        row.insert(5, featureSelf.z_left_wrist)
        row.insert(6, featureSelf.z_right_wrist)
        row.insert(7, featureSelf.absolute_z_left_wrist_mean_10)
        row.insert(8, featureSelf.absolute_z_right_wrist_mean_10)
        row.insert(9, featureSelf.absolute_z_leftShoulder_m10)
        row.insert(10, featureSelf.absolute_z_rightShoulder_m10)
        row.insert(11, featureSelf.abs_z_elbow_r_m10)
        row.insert(12, featureSelf.watchToRobot)
        row.insert(13, featureSelf.wrist_shoulder_ratio)
        row.insert(14, featureSelf.handHigh)
        row.insert(15, featureSelf.handInnerSide)
        row.insert(16, featureSelf.hiGesture)

        return row
