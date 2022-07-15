#!/usr/bin/env python
import pickle
import cv2  # Import opencv
import pandas as pd
from pose.pose import Pose
from visualizer.visualizer import Visualizer
from realsense.realsense import Realsense
from features.feature_factory import FeatureFactory
from machine_learning.machineLearning import MachineLearning
from ros.ROS_interface import ROSInterface
from helper.formatter import CustomFormatter
import helper.environment_data_parser as env_parser
import sys


def make_detection():
    log = CustomFormatter.customFormatter(CustomFormatter())

    ENV = env_parser.EnvironmentDataParser()

    pd.options.mode.chained_assignment = None

    # Loading the Model for Prediction with ML
    with open(ENV.ML_MODEL_NAME + '.pkl', 'rb') as f:
        model = pickle.load(f)

    machine_learning = MachineLearning()
    all_features_df = machine_learning.createDataframe(ENV.ADDITIONAL_FEATURES,
                                                       pose=True,
                                                       face=False,
                                                       left_hand=False,
                                                       right_hand=True, )

    # rolling_window = pd.DataFrame()
    rolling_window_initialized = False

    rs_interface = Realsense()
    success = False
    while not success:
        success = rs_interface.configure_camera(ENV.WIDTH, ENV.HEIGHT)

    # Wird im Moment nicht unterstützt. Muss in Visualize umgesetzt werden. Aber ist nicht funktionell.
    if ENV.IS_FLIPPED == "True":
        buffer = ENV.WIDTH
        ENV.WIDTH = ENV.HEIGHT
        ENV.HEIGHT = buffer

    # pose Objekt enthält Funktionen mit mediapipe
    pose = Pose(ENV.WIDTH, ENV.HEIGHT)

    # Methods to check availability of master, initialize the node and to publish messages in ros
    limit_until_publishing = 6
    ros_interface = ROSInterface(ENV.MASTER_IP, ENV.NODE_NAME, ENV.TOPIC_NAME, limit_until_publishing)

    # Calculation of all features. They are available in the instance 'featureSelf
    data_factory = FeatureFactory(ENV.WIDTH, ENV.HEIGHT, detect_objects=ENV.DETECT_OBJECTS)

    # Methods with cv2
    visualizer = Visualizer(ENV.WIDTH, ENV.HEIGHT)

    # classification_counter = 0
    previous_prediction = str(ENV.CLASS_NAMES_ARR[0])

    # Initiate holistic model
    with pose.mp_holistic.Holistic(min_detection_confidence=0.8, min_tracking_confidence=0.8) as holistic:
        while True:
            # Initialize ROS-Node
            if ros_interface.master_available and not ros_interface.node_initialized:
                ros_interface.initializeNodeByName(ENV)

            # Get frame data from realsense
            color_frame, depth_frame, color_image, depth_image = rs_interface.get_color_depth_stream()

            # to RGB
            image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

            # Calculate Results
            results, avail = pose.calculate_coordinates(holistic, image)

            # Draw Landmarks
            pose.draw_landmarks_from_holistic(results, image, pose=False, right_hand=False, face=False)

            # Calculate all features. They are stored in object of Class 'features'
            data_factory.calculate_features(results, depth_frame, image, depth_image, color_image, rs_interface)

            # Draw Stuff within Picture before flipping (those stuff from MediaPipe)
            visualizer.draw_within_picture(image, results, data_factory)

            # Export coordinates
            try:
                # First export Coords from MediaPipe
                row = machine_learning.export_coordinates(results)

                # Second export additionals
                row = machine_learning.append_additionals_to_row(row, data_factory, class_name="class_name")

                # Third, put them together
                # Aus der Row wird ne Serie. Die Series wird dann in ein dataframe mit
                # dem Namen all_features_df umgewandelt.
                a_series = pd.Series(row, index=all_features_df.columns)
                all_features_df = all_features_df.append(a_series, ignore_index=True)
                all_features_df = all_features_df[-ENV.ROLLING_WINDOW_SIZE:]

            except Exception as e:
                # log.debug("Calculation of coordinates was not successful. At least pose coordinates necessary.")
                pass

            # make shure dataframe contains enough values for prediction
            if len(all_features_df) < ENV.ROLLING_WINDOW_SIZE:
                message = "rolling window has to be initialized. Please go to camera. Then system needs " + \
                          str(ENV.ROLLING_WINDOW_SIZE - len(all_features_df))
                log.debug(message)
            else:
                if not rolling_window_initialized:
                    log.info("Rolling window initialized.")
                    rolling_window_initialized = True

            prediction = "Prediction not possible"
            ros_interface.published = False
            if rolling_window_initialized:
                # Auswahl aus dem Dataframe 'all_features_df'.
                # Die Auswahl sind die features, die im Classifier verwendet werden. Muss mit dem Training uebereinstimmen.
                rolling_window = all_features_df[ENV.SELECTED_FEATURES]

                # # Fuer diese features wird kein Rolling angewendet
                no_rolling_features_arr = ENV.NO_ROLLING_FEATURES

                # Apply rolling
                rolling_window = machine_learning.apply_rolling(rolling_window,
                                                                no_rolling_features_arr,
                                                                size4=True,
                                                                size8=True,
                                                                size12=False,
                                                                size16=False)

                X = rolling_window.dropna()

                try:
                    # Make Detections
                    body_language_class = model.predict(X)[0]
                    prediction = str(ENV.CLASS_NAMES_ARR[body_language_class])
                except:
                    log.debug("Prediction was not successful")
                    pass

                # Check whether prediction is same in two frames
                prediction_same_in_two_frames = False
                if prediction == previous_prediction:
                    prediction_same_in_two_frames = True
                previous_prediction = prediction

                # Export Data to ROS
                pose_available = avail[0]
                right_hand_available = avail[3]
                visibility_right_wrist = data_factory.wrist_r_abs[3]
                if pose_available and right_hand_available and visibility_right_wrist > 0.95:
                    ros_interface.publish_with_filters(prediction, data_factory.wrist_coords,
                                                       prediction_same_in_two_frames)
                    ros_interface.publish_without_filters("", data_factory.wrist_coords)
                    ros_interface.publish_object_detection(data_factory.object_detection.detected_objects)
                    ros_interface.published_recovery_gesture(data_factory.recover_from_error)

            if ENV.VISUALIZER:
                # Flip image
                image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)

                # Sequence of drawing selections is important - don't change
                image = visualizer.draw_image_section(image, data_factory)
                image = visualizer.draw_coords_section(image, results, data_factory, position='right')
                image = visualizer.draw_classification_section(image, prediction, ros_interface.published)
                image = visualizer.draw_status_section(image, machine_learning.sel_arr, avail, ros_interface)

                # to BGR
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                cv2.imshow('Panda Intention Recognition', image)

            if cv2.waitKey(10) & 0xFF == ord('q'):
                break

    Realsense.stop_pipeline(rs_interface)
    cv2.destroyAllWindows()
    ros_interface.stop_ros_threat = True
    sys.exit(0)


if __name__ == '__main__':
    make_detection()
