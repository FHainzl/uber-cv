#!/usr/bin/env python

import cv2
import numpy as np


def range_mask(img, lower_bound, upper_bound):
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(img_hsv, lower_bound, upper_bound)
    return mask


def contour_center(mask, min_area, max_area):
    im2, contours, hierarchy = cv2.findContours(mask,
                                                cv2.RETR_LIST,
                                                cv2.CHAIN_APPROX_NONE)

    try:
        c_i, c = max(enumerate(contours), key=lambda e: cv2.contourArea(e[1]))
    except ValueError:
        return None
    max_area = 1000
    while cv2.contourArea(c) > max_area:
        contours.pop(c_i)
        c_i, c = max(enumerate(contours), key=lambda e: cv2.contourArea(e[1]))
    print cv2.contourArea(c)
    if cv2.contourArea(c) < min_area:
        return None
    rect = cv2.boundingRect(c)
    return np.uint16(np.around(rect))


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
