import cv2
import numpy as np


class ObjectDetection:
    def __init__(self):
        self.detected_objects = [(0, 0), 0, 0, 0]  # [[(201, 197), 73, 58, 16]] center, width, height, angle
        self.thresh_distance = 50  # Max Distance between object and hand

    def inverte(self, imagem):
        imagem = (255 - imagem)
        # cv2.imwrite(name, imagem)
        return imagem

    def cut_skin(self, image, target_color):
        # define the upper and lower boundaries of the HSV pixel
        # intensities to be considered 'skin'
        lower = np.array([0, 48, 80], dtype="uint8")
        upper = np.array([20, 255, 255], dtype="uint8")

        # resize the frame, convert it to the HSV color space,
        # and determine the HSV pixel intensities that fall into
        # the speicifed upper and lower boundaries
        converted = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Pixels that are white (255) in the mask represent areas of the frame that are skin.
        # Pixels that are black (0) in the mask represent areas that are not skin.
        skin_mask = cv2.inRange(converted, lower, upper)

        # blur the mask to help remove noise, then apply the
        # mask to the frame
        skin_mask = cv2.GaussianBlur(skin_mask, (1, 1), 0)

        if target_color == 0:
            # Pixels that are white (0) in the mask represent areas of the frame that are skin.
            # Pixels that are black (255) in the mask represent areas that are not skin.
            skin_mask = self.inverte(skin_mask)

            # skin = cv2.bitwise_not(image, image, mask=skin_mask)
            image = cv2.bitwise_and(image, image, mask=skin_mask)
        else:
            # find indices of the mask
            # fill pixels at these indices with white color
            indices = np.where(skin_mask == 255)
            image[indices[0], indices[1], :] = [255, 255, 255]

        return image

    def detect_objects(self, image, color_image, depth_image, rs_interface, data_factory):
        """
        Hier wird zweimal die Funktion detect_contours() aufgerufen. ...
        """
        self.detected_objects = [(0, 0), 0, 0, 0]

        # Hintergrund ist schwarz
        self.detect_contours(image, color_image, depth_image, "window_1", rs_interface, data_factory,
                             color_for_bg_removal=0)
        # Hintergrund ist weiß
        self.detect_contours(image, color_image, depth_image, "window_2", rs_interface, data_factory,
                             color_for_bg_removal=255)

    def detect_contours(self, image, img, depth_image, window_name, rs_interface, data_factory, color_for_bg_removal=0):
        """
        This programs calculates the orientation of an object.
        The input is an image, and the output is an annotated image
        with the angle of otientation for each object (0 to 180 degrees)
        """
        ######################################
        #        Remove Background           #
        ######################################
        clip_dist_m = (data_factory.wrist_pose_r_abs[2] * 10) + 0.1
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = rs_interface.cut_background(depth_image, img, color_for_bg_removal,
                                          clipping_distance_in_meters=clip_dist_m)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        img = self.cut_skin(img, color_for_bg_removal)

        # ######################################
        # #          Cut out Circle            #
        # ######################################
        # define circles
        radius = 100
        xc = data_factory.wrist_r_abs[0]
        yc = data_factory.wrist_r_abs[1]

        # draw filled circle in white on black background as mask
        mask = np.zeros_like(img)
        mask = cv2.circle(mask, (xc, yc), radius, (255, 255, 255), -1)

        # apply mask to image
        img = cv2.bitwise_and(img, mask)

        ######################################
        #           find contours            #
        ######################################
        # Convert image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Convert image to binary
        _, bw = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

        # Find all the contours in the thresholded image
        contours, _ = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        for i, c in enumerate(contours):

            # Calculate the area of each contour
            area = cv2.contourArea(c)

            # Ignore contours that are too small or too large
            if area < 1500 or 5000 < area:
                continue

            # cv.minAreaRect returns:
            # (center(x, y), (width, height), angle of rotation) = cv2.minAreaRect(c)
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            # Retrieve the key parameters of the rotated bounding box
            center = (int(rect[0][0]), int(rect[0][1]))
            width = int(rect[1][0])
            height = int(rect[1][1])
            angle = int(rect[2])

            # Get coordinates of wrist
            center_wrist = (data_factory.wrist_pose_r_abs[0], data_factory.wrist_pose_r_abs[1])

            # only use object if it is close to hand
            distance_x = abs(center_wrist[0] - center[0])
            distance_y = abs(center_wrist[1] - center[1])
            if distance_x < self.thresh_distance and distance_y < self.thresh_distance:
                if width < height:
                    angle = 90 - angle
                else:
                    angle = -angle

                # self.detected_objects.append([center, width, height, angle])
                self.detected_objects[0] = center
                self.detected_objects[1] = width
                self.detected_objects[2] = height
                self.detected_objects[3] = angle

                # Draw detected objects
                if data_factory.draw_objects:
                    # label = "  Rotation Angle: " + str(angle) + " degrees"
                    # textbox = cv2.rectangle(image, (center[0] - 35, center[1] - 25),
                    # (center[0] + 295, center[1] + 10), (255, 255, 255), -1)
                    # cv2.putText(image, label, (center[0] - 50, center[1]),
                    # cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 1, cv2.LINE_AA)
                    cv2.drawContours(image, [box], 0, (0, 0, 255), 2)

        return self.detected_objects
