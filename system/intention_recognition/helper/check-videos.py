import mediapipe as mp # Import mediapipe
import cv2 # Import opencv
import os
from realsense.realsense import Realsense
import pyrealsense2 as rs
import numpy as np

mp_drawing = mp.solutions.drawing_utils # Drawing helpers
mp_holistic = mp.solutions.holistic # Mediapipe Solutionsnegative

first_ts = 0

# handShake, kiss, highFive, hug, negative
#class_name = "handShake"
class_name_arr = ["handShake", "highFive", "negative"]
path="/home/benny/Documents/trainingData/"

align_to = rs.stream.depth
align = rs.align(align_to)
failure_list = []


for class_name in class_name_arr:
    for root, dirs, files in os.walk (path):
        for file in files:
            if (class_name in file):
                print(path+file)
                first_ts = 0
                try:
                    pipeline, config = Realsense.read_bag_return_pipeline("x", path+file)
                except:
                    failure_list.append(file)
                    print("Fehler bei: " + path + file)
                    continue

                # Initiate holistic model
                with mp_holistic.Holistic(min_detection_confidence=0.8, min_tracking_confidence=0.8) as holistic:
                    while True:

                        frames = pipeline.wait_for_frames()

                        # Aligning color frame to depth frame
                        aligned_frames = align.process(frames)
                        depth_frame = aligned_frames.get_depth_frame()
                        color_frame = aligned_frames.get_color_frame()

                        depth_image = np.asanyarray(depth_frame.get_data())
                        color_image = np.asanyarray(color_frame.get_data())

                        # recolor Feed
                        image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

                        # make shure we leave the loop when video is finished
                        ts = color_frame.get_timestamp()
                        if ts >= first_ts:
                            first_ts = ts
                        else:
                            print("break")
                            break

                        # Recolor Feed
                        image.flags.writeable = False

                        # Make Detections
                        results = holistic.process(image)

                        # Recolor image back to BGR for rendering
                        image.flags.writeable = True

                        # 1. Draw face landmarks
                        mp_drawing.draw_landmarks(image, results.face_landmarks, mp_holistic.FACE_CONNECTIONS,
                                                  mp_drawing.DrawingSpec(color=(136, 136, 136), thickness=1,
                                                                         circle_radius=1),
                                                  mp_drawing.DrawingSpec(color=(162, 34, 35), thickness=1,
                                                                         circle_radius=1)
                                                  )

                        # 2. Right hand
                        mp_drawing.draw_landmarks(image, results.right_hand_landmarks, mp_holistic.HAND_CONNECTIONS,
                                                  mp_drawing.DrawingSpec(color=(136, 136, 136), thickness=2,
                                                                         circle_radius=4),
                                                  mp_drawing.DrawingSpec(color=(70, 100, 170), thickness=2,
                                                                         circle_radius=2)
                                                  )

                        # 3. Left Hand
                        mp_drawing.draw_landmarks(image, results.left_hand_landmarks, mp_holistic.HAND_CONNECTIONS,
                                                  mp_drawing.DrawingSpec(color=(136, 136, 136), thickness=2,
                                                                         circle_radius=4),
                                                  mp_drawing.DrawingSpec(color=(70, 100, 170), thickness=2,
                                                                         circle_radius=2)
                                                  )

                        # 4. pose Detections
                        mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_holistic.POSE_CONNECTIONS,
                                                  mp_drawing.DrawingSpec(color=(136, 136, 136), thickness=2,
                                                                         circle_radius=4),
                                                  mp_drawing.DrawingSpec(color=(0, 150, 130), thickness=2,
                                                                         circle_radius=2)
                                                  )
                        # # Export coordinates
                        # try:
                        #     # Extract pose landmarks
                        #     pose = results.pose_landmarks.landmark
                        #     pose_row = list(np.array([[landmark.x, landmark.y, landmark.z, landmark.visibility] for landmark in pose]).flatten())
                        #
                        #     # Extract Face landmarks
                        #     face = results.face_landmarks.landmark
                        #     face_row = list(np.array([[landmark.x, landmark.y, landmark.z, landmark.visibility] for landmark in face]).flatten())
                        #
                        #     # Extract right hand landmarks
                        #     right_hand = results.right_hand_landmarks.landmark
                        #     right_hand_row = list(np.array([[landmark.x, landmark.y, landmark.z, landmark.visibility] for landmark in right_hand]).flatten())
                        #
                        #     # Extract left hand landmarks
                        #     left_hand = results.right_hand_landmarks.landmark
                        #     left_hand_row = list(np.array([[landmark.x, landmark.y, landmark.z, landmark.visibility] for landmark in left_hand]).flatten())
                        #
                        #
                        #     # Concate rows
                        #     row = pose_row  #
                        #     #row = row + face_row
                        #     row = row + right_hand_row
                        #     row = row + left_hand_row
                        #
                        #
                        #     # Append class name
                        #     row.insert(0, class_name)
                        #
                        #     # Export to CSV
                        #     with open(name+'.csv', mode='a', newline='') as f:
                        #         csv_writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                        #         csv_writer.writerow(row)
                        #
                        # except:
                        #     pass

                        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                        image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
                        cv2.imshow('Raw Webcam Feed', image)

                        if cv2.waitKey(10) & 0xFF == ord('q'):
                            break

                print("finish")
                pipeline.stop()
                cv2.destroyAllWindows()

print("")
print("Done")
print("The following .bag files are unusable:")
print("")
print(failure_list)
