#!/usr/bin/env python
import csv
import os
import time
import cv2  # Import opencv
import pandas as pd
from pose.pose import Pose
from visualizer.visualizer import Visualizer
from realsense.realsense import Realsense
from features.feature_factory import FeatureFactory
from machine_learning.machineLearning import MachineLearning
from helper.formatter import CustomFormatter
import helper.environment_data_parser as env_parser


def get_csv_from_bag():
    log = CustomFormatter.customFormatter(CustomFormatter())

    ENV = env_parser.EnvironmentDataParser()

    pd.options.mode.chained_assignment = None
    #
    # with open(ENV.ML_MODEL_NAME + '.pkl', 'rb') as f:
    #     model = pickle.load(f)

    machine_learning = MachineLearning()
    all_features_df = machine_learning.createDataframe(ENV.ADDITIONAL_FEATURES,
                                                       pose=True,
                                                       face=True,
                                                       left_hand=True,
                                                       right_hand=True,
                                                       write_to_csv=True,
                                                       csv_filename=ENV.CSV_FILE_NAME
                                                       )

    # rolling_window = pd.DataFrame()
    rolling_window_initialized = False

    rs_interface = Realsense()
    # success = False
    # while not success:
    #     success = rs_interface.configure_camera(ENV.WIDTH, ENV.HEIGHT)

    # Wird im Moment nicht unterstützt. Muss in Visualize umgesetzt werden. Aber ist nicht funktionell.
    if ENV.IS_FLIPPED == "True":
        buffer = ENV.WIDTH
        ENV.WIDTH = ENV.HEIGHT
        ENV.HEIGHT = buffer

    # pose Objekt enthält Funktionen mit mediapipe
    pose = Pose(ENV.WIDTH, ENV.HEIGHT)

    # Calculation of all features. They are available in the instance 'featureSelf
    data_factory = FeatureFactory(ENV.WIDTH, ENV.HEIGHT)

    # Methods with cv2
    visualizer = Visualizer(ENV.WIDTH, ENV.HEIGHT)

    failure_list = []

    start = time.time()

    for class_name in ENV.CLASS_NAMES_ARR:
        for root, dirs, files in os.walk(ENV.PATH_TO_BAG_FILES):
            for file in files:
                if class_name in file:
                    log.debug(ENV.PATH_TO_BAG_FILES + file)
                    first_ts = 0

                    try:
                        rs_interface.configure_realsense_from_bag(ENV.PATH_TO_BAG_FILES,
                                                                  file,
                                                                  frames_until_start=3)
                    except Exception as e:
                        log.error("Error during configuration of camera", str(e))
                        failure_list.append(file)
                        log.warn("Video failure with: " + ENV.PATH_TO_BAG_FILES + file)
                        continue

                    # Initiate holistic model
                    with pose.mp_holistic.Holistic(min_detection_confidence=0.8,
                                                   min_tracking_confidence=0.8) as holistic:
                        while True:
                            # Get frame data from realsense
                            color_frame, depth_frame, color_image, depth_image = rs_interface.get_color_depth_stream()

                            # make shure we leave the loop when video is finished
                            ts = color_frame.get_timestamp()
                            if ts >= first_ts:
                                first_ts = ts
                            else:
                                break

                            # to RGB
                            image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

                            # Calculate Results
                            results, avail = pose.calculate_coordinates(holistic, image)

                            # Draw Landmarks
                            pose.draw_landmarks_from_holistic(results, image, pose=True, right_hand=True, face=True)

                            # Calculate all features. They are stored in object of Class 'features'
                            data_factory.calculate_features(results, depth_frame, image, depth_image, color_image,
                                                            rs_interface)

                            # Draw Stuff within Picture before flipping (those stuff from MediaPipe)
                            visualizer.draw_within_picture(image, results, data_factory)

                            try:
                                # First export Coords from MediaPipe
                                row = machine_learning.export_coordinates_for_get_data(results)

                                # Second export additionals
                                row = machine_learning.append_additionals_to_row(row, data_factory, class_name)

                                # Export to CSV
                                with open(ENV.CSV_FILE_NAME + '.csv', mode='a', newline='') as f:
                                    csv_writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                                    csv_writer.writerow(row)

                            except Exception as e:
                                log.error("Error during export of coordinates to .csv @ " + file + str(e))

                            if ENV.VISUALIZER:
                                image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)

                                # Sequence of drawing selection is important - don't change
                                image = visualizer.draw_image_section(image, data_factory)
                                image = visualizer.draw_coords_section(image, results, data_factory, position='right')
                                image = visualizer.draw_status_section(image, machine_learning.sel_arr, avail,
                                                                       ros_interface=None, file=file,
                                                                       class_name=class_name)

                                # to BGR
                                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                                cv2.imshow('Panda Intention Recognition', image)

                            if cv2.waitKey(10) & 0xFF == ord('q'):
                                break

                    log.debug("Next video ... ")
                    Realsense.stop_pipeline(rs_interface)
                    cv2.destroyAllWindows()

    print("The following .bag files are unusable:")
    print("")
    print(failure_list)

    # end time, total time taken
    end = time.time()
    log.debug(f"Runtime of the program is {end - start}")


if __name__ == '__main__':
    get_csv_from_bag()
