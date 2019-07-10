#!/usr/bin/env python

import rospy
import cv2

from uber_cv_ros.node import Node
from uber_cv_ros.setup_ddrec_server import setup_ddrec_server


def main():
    rospy.set_param("node_name", "image2angle")
    rospy.set_param("server_name", "cv_dyn_rec")

    node = rospy.get_param("node_name")
    rospy.init_node(node, anonymous=False)

    topic = "/camera/color/image_raw"
    encoding = "bgr8"

    dd_server = setup_ddrec_server()
    pubsub = Node(topic, encoding)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
