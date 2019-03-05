#!/usr/bin/env python

import cv2
import cv_bridge
import numpy as np

import rospy
from sensor_msgs.msg import Image


class Bound:
    def __init__(self):
        self.h, self.s, self.v = (0, 0, 0)

    def set_h(self, h):
        self.h = h

    def set_s(self, s):
        self.s = s

    def set_v(self, v):
        self.v = v

    @property
    def hsv(self):
        return self.h, self.s, self.v

    @property
    def bgr(self):
        x = np.uint8(self.hsv).reshape([1, 1, 3])
        x = cv2.cvtColor(x, cv2.COLOR_HSV2BGR)
        x = x.ravel()
        return x[0], x[1], x[2]


def set_img_color(lower_hsv, h, s, v):
    lower_hsv[:, :, 0] = h
    lower_hsv[:, :, 1] = s
    lower_hsv[:, :, 2] = v
    return lower_hsv


def set_bounds_in_hsv(img, window="Searching Bounds"):
    """
    :param img: in HSV
    :param window: rather arbitrary, may provide info which color to find
    :return: two sets of HSV values
    """
    cv2.imshow(winname="Original Image",
               mat=cv2.cvtColor(img, cv2.COLOR_HSV2BGR))
    cv2.namedWindow(window)

    # Init bounds
    lower = Bound()
    upper = Bound()

    # Create track bars
    cv2.createTrackbar('H (Low)', window, 0, 180, lower.set_h)
    cv2.createTrackbar('H (High)', window, 0, 180, upper.set_h)
    cv2.setTrackbarPos('H (High)', window, 180)
    cv2.createTrackbar('S (Low)', window, 0, 255, lower.set_s)
    cv2.createTrackbar('S (High)', window, 0, 255, upper.set_s)
    cv2.setTrackbarPos('S (High)', window, 255)
    cv2.createTrackbar('V (Low)', window, 0, 255, lower.set_v)
    cv2.createTrackbar('V (High)', window, 0, 255, upper.set_v)
    cv2.setTrackbarPos('V (High)', window, 255)

    while True:
        img_threshold = cv2.inRange(img, lower.hsv, upper.hsv)
        cv2.imshow(winname=window, mat=img_threshold)

        # Show bounds as colors
        empty = np.ones([100, 100, 3], dtype=np.uint8)
        lower_hsv = set_img_color(empty, *lower.hsv)
        lower_hsv = cv2.cvtColor(lower_hsv, cv2.COLOR_HSV2BGR)
        upper_hsv = set_img_color(empty, *upper.hsv)
        upper_hsv = cv2.cvtColor(upper_hsv, cv2.COLOR_HSV2BGR)
        cv2.imshow(winname="Visualize Bounds",
                   mat=np.hstack([lower_hsv, upper_hsv]))

        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            return lower.hsv, upper.hsv


def get_image(node_name_, topic_):
    rospy.init_node(node_name_, anonymous=True)
    msg = rospy.wait_for_message(topic_, Image)
    cv_image = cv_bridge.CvBridge().imgmsg_to_cv2(msg, encoding)
    return cv_image


if __name__ == '__main__':
    node_name = "color_bounds"
    topic = "/camera/color/image_raw"
    encoding = "bgr8"
    print "Party started"
    img = get_image(node_name, topic)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    print "Best image ever"
    print img
    bounds = {}
    for ball in ["ball_center", "ball_edge"]:
        bounds[ball] = set_bounds_in_hsv(img)

    # Write parameters
    rospy.set_param("color_bounds", bounds)
    print "color_bounds/ball_center"
    print rospy.get_param("color_bounds/ball_center")
    print "color_bounds/ball_edge"
    print rospy.get_param("color_bounds/ball_edge")
