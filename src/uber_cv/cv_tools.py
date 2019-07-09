#!/usr/bin/env python
from math import atan2

import cv2
import numpy as np
from config import config as c


def image_to_angle(img_cv, bounds_core, bounds_edge):
    results = {
        "mask": {},
        "area": {},
        "rect": {},
        "center": {},
        "rect_img": None
    }

    bounds = {"core": bounds_core,
              "edge": bounds_edge}

    for ball in ("core", "edge"):
        lower, higher = bounds[ball]
        mask = extract_contours(img_cv, lower, higher)
        rect, area = get_contour_center(mask, c["min_area"], c["max_area"])

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
                          [(255, 0, 0), (0, 255, 0)])

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


def extract_contours(img, lower_bound, upper_bound):
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(img_hsv, lower_bound, upper_bound)
    return mask


def get_contour_center(mask, min_area, max_area):
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
    rect = cv2.boundingRect(c)
    return np.uint16(np.around(rect)), area


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
