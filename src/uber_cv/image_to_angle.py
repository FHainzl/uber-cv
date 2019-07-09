#!/usr/bin/env python

from math import atan2

import numpy as np
import rospy
import cv2
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge, CvBridgeError

from cv_tools import get_contour_center, extract_contours, draw_rects
from config import config as c


class ImageToAngle:
    """


    Note: Don't mix up 'core' and 'center':
    Core refers to the ball that is positioned closer to the pendulum axis
    Center refers to middle of enclosing rectangle of contour
    """

    def __init__(self):

        # Read from config, which doesn't change at runtime
        self.min_area = c["minimal_area"]
        self.max_area = c["maximal_area"]

    def image_to_angle(self, img_cv, bounds_core, bounds_edge):
        results = {}
        results["mask"] = {}
        results["area"] = {}
        results["rect"] = {}
        results["center"] = {}
        results["rect_img"] = None

        bounds = {"core": bounds_core,
                  "edge": bounds_edge}

        for ball in ("core", "edge"):
            lower, higher = bounds[ball]
            mask = extract_contours(img_cv, lower, higher)

            results["mask"][ball] = mask

            rect, area = get_contour_center(mask, self.min_area, self.max_area)
            if area is not None:
                results["area"][ball] = area
            try:
                x, y, w, h = rect
                center = (x + 0.5 * w, y + 0.5 * h)
                center = np.int16(np.around(center))
            except TypeError:
                rospy.loginfo("No {} contour found!".format(ball))
                center = None

            results["rect"][ball] = rect
            results["center"][ball] = center

        rect_img = draw_rects(img_cv,
                              (results["rect"]["core"],
                               results["rect"]["edge"]),
                              [(255, 0, 0), (0, 255, 0)])

        results["rect_img"] = rect_img

        if None not in results["center"].values():
            center = results["center"]
            y_minus, x = center["edge"] - center["core"]
            y = -y_minus
            a = atan2(y, x)
            results["angle"] = a
        else:
            results["angle"] = None

        return results
