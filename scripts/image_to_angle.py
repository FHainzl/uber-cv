#!/usr/bin/env python
from math import atan2

import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge, CvBridgeError

from cv_tools import contour_center, range_mask, draw_rects
from publisher import Publisher
from img_publisher import ImgPublisher
from setup_ddrec_server import setup_ddrec_server
from utils import human_time, get_dy

from config import config as c


class ImageConverter:
    def __init__(self):
        self.bridge = CvBridge()

        # Publisher nodes
        self.pub_masks = {"core": ImgPublisher("mask_core"),
                          "edge": ImgPublisher("mask_edge")}

        self.pub_circles = ImgPublisher("circles_image")
        self.pub_angle = Publisher("angle", JointState)

        # Subscriber node
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)

    def callback(self, data):
        header = data.header

        # Realsense time stamps were off
        header.stamp = rospy.Time.now()

        try:
            img_cv = self.bridge.imgmsg_to_cv2(data, encoding)
        except CvBridgeError as e:
            print(e)
            return

        if c["print_img_received"]:
            time = human_time(data.header.stamp)
            rospy.loginfo("Message received from {}".format(time))

        bounds = {"core": bounds_core_ball(),
                  "edge": bounds_edge_ball()}

        rects = {}
        centers = {}
        for ball in ("core", "edge"):
            lower, higher = bounds[ball]
            mask = range_mask(img_cv, lower, higher)

            self.pub_masks[ball].publish(mask)
            rect = contour_center(mask, c["minimal_area"], c["maximal_area"])
            try:
                x, y, w, h = rect
                center = (x + 0.5 * w, y + 0.5 * h)
                center = np.int16(np.around(center))
            except TypeError:
                rospy.loginfo("No {} contour found!".format(ball))
                center = None
            rects[ball] = rect
            centers[ball] = center

        circle_img = draw_rects(img_cv, (rects["core"], rects["edge"]),
                                [(255, 0, 0), (0, 255, 0)])
        self.pub_circles.publish(circle_img)

        try:
            y_minus, x = centers["edge"] - centers["core"]
            y = -y_minus
            a = atan2(y, x)
            # a -= 0.5 * pi  # Hanging is zero angle
            a += get_dy("angle_offset")  # Center

            angle_msg = JointState()
            angle_msg.header = header
            angle_msg.position = [a]
            self.pub_angle.publish(angle_msg)
        except TypeError:
            pass


def bounds_core_ball():
    return _bounds("Core")


def bounds_edge_ball():
    return _bounds("Edge")


def _bounds(ball):
    low_keys = ["{}_Low_{}".format(t, ball) for t in "HSV"]
    high_keys = ["{}_High_{}".format(t, ball) for t in "HSV"]
    low = [get_dy(key) for key in low_keys]
    high = [get_dy(key) for key in high_keys]
    return np.array([low, high])


def main():
    node = rospy.get_param("node_name")
    rospy.init_node(node, anonymous=False)
    dd_server = setup_ddrec_server()
    ic = ImageConverter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.set_param("node_name", "image2angle")
    rospy.set_param("server_name", "cv_dyn_rec")

    topic = "/camera/color/image_raw"
    encoding = "bgr8"

    main()
