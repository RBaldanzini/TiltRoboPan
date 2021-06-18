#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from rgb_hsv import BGR_HSV

black = [(83, 83, 83)]


def line_detect(image):
    """
    Detect the centre of the line
    """
    image = cv2.resize(image, (320, 320))

    rgb_hsv = BGR_HSV()
    rgb_to_track = [83, 83, 83]
    hsv, hsv_numpy_percentage = rgb_hsv.rgb_hsv(rgb=rgb_to_track)
    min_hsv = hsv * (1.0 - (10 / 100.0))
    max_hsv = hsv * (1.0 + (10 / 100.0))
    lower_yellow = np.array(min_hsv)
    upper_yellow = np.array(max_hsv)

    (height, width, channels) = image.shape
    crop_img = image
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    #res = cv2.bitwise_and(crop_img, crop_img, mask=mask)
    (__, contours_blk, __) = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    x_last = height
    y_last = width

    cx = []
    cy = []

    contours_blk_len = len(contours_blk)
    if contours_blk:
        if contours_blk_len > 0:
            canditates = []
            off_bottom = 0
            for con_num in range(contours_blk_len):
                blackbox = cv2.minAreaRect(contours_blk[con_num])
                (x_min, y_min), (w_min, h_min), ang = blackbox
                box = cv2.boxPoints(blackbox)
                (x_box, y_box) = box[0]
                # if y_box > 358:
                #     off_bottom += 1
                canditates.append((y_box, con_num, x_min, y_min))
            canditates = sorted(canditates)
            if off_bottom > 1:
                canditates_off_bottom = []
                for con_num in range((contours_blk_len - off_bottom), contours_blk_len):
                    (y_highest, con_highest, x_min, y_min) = canditates[con_num]
                    total_distance = (abs(x_min - x_last) ** 2 + abs(y_min - y_last) ** 2) ** 0.5
                    canditates_off_bottom.append((total_distance, con_highest))
                canditates_off_bottom = sorted(canditates_off_bottom)
                (total_distance, con_highest) = canditates_off_bottom[0]
                blackbox = cv2.minAreaRect(contours_blk[con_highest])

            box = cv2.boxPoints(blackbox)
            box = np.int0(box)
            cv2.line(image, (int(x_min), 200), (int(x_min), 250), (255, 0, 0), 3)
            cx = (x_min - 160) / 160
            cy = (y_min - 160) / 160

    else:
        rospy.loginfo("Unable to find a line")
        cx = 0
        cy = 0

    cv2.imshow("original", image)

    return cx, cy
