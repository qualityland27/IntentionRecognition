import cv2
import mediapipe as mp


mp_drawing = mp.solutions.drawing_utils
mp_holistic = mp.solutions.holistic

def run():
    # # For static images:
    # IMAGE_FILES = ['../Bilder/benny_demopose.jpg']
    # with mp_holistic.Holistic(static_image_mode=True) as holistic:
    #   for idx, file in enumerate(IMAGE_FILES):
    #     image = cv2.imread(file)
    #     image_height, image_width, _ = image.shape
    #     # Convert the BGR image to RGB before processing.
    #     results = holistic.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    #
    #     if results.pose_landmarks:
    #       print(
    #           f'Nose coordinates: ('
    #           f'{results.pose_landmarks.landmark[mp_holistic.PoseLandmark.NOSE].x * image_width}, '
    #           f'{results.pose_landmarks.landmark[mp_holistic.PoseLandmark.NOSE].y * image_height})'
    #       )
    #     # Draw pose, left and right hands, and face landmarks on the image.
    #     annotated_image = image.copy()
    #     mp_drawing.draw_landmarks(
    #         annotated_image, results.face_landmarks, mp_holistic.FACE_CONNECTIONS)
    #     mp_drawing.draw_landmarks(
    #         annotated_image, results.left_hand_landmarks, mp_holistic.HAND_CONNECTIONS)
    #     mp_drawing.draw_landmarks(
    #         annotated_image, results.right_hand_landmarks, mp_holistic.HAND_CONNECTIONS)
    #     mp_drawing.draw_landmarks(
    #         annotated_image, results.pose_landmarks, mp_holistic.POSE_CONNECTIONS)
    #     cv2.imwrite('/tmp/annotated_image' + str(idx) + '.png', annotated_image)

    # For webcam input:
    writer = None
    cap = cv2.VideoCapture('../Bilder/clap.mp4')
    with mp_holistic.Holistic(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as holistic:
        while cap.isOpened():
            success, image = cap.read()
            if not success:
                print("Ignoring empty camera frame.")
                # If loading a video, use 'break' instead of 'continue'.
                continue
            if (writer == None):
                fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
                img_h, img_w, _ = image.shape
                videoWriter = cv2.VideoWriter('../Bilder/clap_solution.mp4', fourcc, 30, (img_w, img_h))
                videoWriter.open('../../Bilder/clap_solution.mp4', fourcc, 30, (img_w, img_h))
                writer = 1

            # Flip the image horizontally for a later selfie-view display, and convert
            # the BGR image to RGB.
            image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
            # To improve performance, optionally mark the image as not writeable to
            # pass by reference.
            image.flags.writeable = False
            results = holistic.process(image)

            # Draw landmark annotation on the image.
            image.flags.writeable = True

            image.fill(255)

            # height, width, _ = image.shape
            #
            # for i in range(height):
            #     for j in range(width):
            #         # img[i,j] is the RGB pixel at position (i, j)
            #         # check if it's [0, 0, 0] and replace with [255, 255, 255] if so
            #         image[i, j] = [0, 0, 0]

            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            mp_drawing.draw_landmarks(
                image, results.face_landmarks, mp_holistic.FACE_CONNECTIONS)
            mp_drawing.draw_landmarks(
                image, results.left_hand_landmarks, mp_holistic.HAND_CONNECTIONS)
            mp_drawing.draw_landmarks(
                image, results.right_hand_landmarks, mp_holistic.HAND_CONNECTIONS)
            mp_drawing.draw_landmarks(
                image, results.pose_landmarks, mp_holistic.POSE_CONNECTIONS)
            cv2.imshow('MediaPipe Holistic', image)


            videoWriter.write(image)


            if cv2.waitKey(5) & 0xFF == 27:
                break
    cap.release()


if __name__ == '__main__':
    run()

