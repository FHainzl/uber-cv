#!/usr/bin/env python
from math import atan2

import cv2
import numpy as np
from config import config as c


def image_to_angle(img_cv, bounds_core, bounds_edge):
    """
    Detect two colored balls in BGR image

    Return angle between balls and vertical, as well as processing steps
    """

    results = {
        "angle": None,  # float with angle
        "mask": {},  # bitmaps with contours in HSV range
        "area": {},  # floats with areas of selected contour
        "rect": {},  # (x,y,w,h) of rects
        "center": {},  # Tuples with x,y of center of balls
        "rect_img": None  # Image with bounding rectangles around selected areas
    }

    bounds = {"core": bounds_core,
              "edge": bounds_edge}

    for ball in ("core", "edge"):
        lower_bound, higher_bound = bounds[ball]
        mask = contours_in_range(img_cv, lower_bound, higher_bound)
        rect, area = contour_center(mask, c["min_area"], c["max_area"])

        try:
            x, y, w, h = rect
            center = (x + 0.5 * w, y + 0.5 * h)
            center = np.int16(np.around(center))
        except TypeError:
            center = None

        results["mask"][ball] = mask
        results["area"][ball] = area
        results["rect"][ball] = rect
        results["center"][ball] = center

    rect_img = draw_rects(img_cv,
                          (results["rect"]["core"],
                           results["rect"]["edge"]),
                          [(255, 0, 0), (0, 255, 255)])

    results["rect_img"] = rect_img

    if None not in results["center"].values():
        center = results["center"]
        y_minus, x = center["edge"] - center["core"]
        y = -y_minus
        angle = atan2(y, x)
        results["angle"] = angle
    else:
        results["angle"] = None

    return results


def contours_in_range(img, lower_bound, upper_bound):
    """
    Apply range operation on HSV image

    :param img: BGR image
    :param lower_bound: lower bound HSV values, tuple with three entries
    :param upper_bound: upper bound HSV values, tuple with three entries
    :return: Mask with contours within range
    """
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(img_hsv, lower_bound, upper_bound)
    return mask


def contour_center(mask, min_area, max_area):
    """
    Select largest contour from mask that is between min_area and max_area
    Return its bounding rect and area

    :param mask: binary mask with contours
    :param min_area: minimal area of valid contour
    :param max_area: maximal area of valid contour
    :return: (x,y,w,h), area of bounding rect,      if area within range
             None, None                             else
    """

    im2, contours, hierarchy = cv2.findContours(mask,
                                                cv2.RETR_LIST,
                                                cv2.CHAIN_APPROX_NONE)

    try:
        c_i, c = max(enumerate(contours), key=lambda e: cv2.contourArea(e[1]))
    except ValueError:
        return None, None
    while cv2.contourArea(c) > max_area:
        contours.pop(c_i)
        c_i, c = max(enumerate(contours), key=lambda e: cv2.contourArea(e[1]))
    area = cv2.contourArea(c)

    if area < min_area:
        return None, None
    else:
        rect = cv2.boundingRect(c)
        rect = np.uint16(
            np.around(rect))  # Round to integer, pixel values wanted
        return rect, area


def draw_rects(img, rects, colors):
    _img = img.copy()
    for rect, color in zip(rects, colors):
        if rect is None:
            continue

        # Rectangle
        x, y, w, h = rect
        cv2.rectangle(_img, (x, y), (x + w, y + h), color, 2)
        # Center
        c1, c2 = (x + w / 2, y + h / 2)
        cv2.circle(_img, (c1, c2), 2, color, 5)
    return _img
