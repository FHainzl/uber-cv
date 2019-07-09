#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge, CvBridgeError
from publisher import Publisher
from img_publisher import ImgPublisher

from cv_ros_tools import get_dy, bounds_core_ball, bounds_edge_ball, human_time
from uber_cv.cv_tools import image_to_angle
from uber_cv.config import config as c


class PubSub:
    def __init__(self, topic, encoding):
        self.encoding = encoding

        self.bridge = CvBridge()

        # Publisher nodes
        self.pub_masks = {"core": ImgPublisher("mask_core"),
                          "edge": ImgPublisher("mask_edge")}
        self.pub_areas = {"core": Publisher("area_core", Float32),
                          "edge": Publisher("area_edge", Float32)}

        self.pub_rect_img = ImgPublisher("circles_image")
        self.pub_angle = Publisher("angle", JointState)

        # Subscriber node
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)

    def callback(self, data):
        header = data.header

        # Replace RealSense time stamps with current time, because they were off
        header.stamp = rospy.Time.now()

        try:
            img_cv = self.bridge.imgmsg_to_cv2(data, self.encoding)
        except CvBridgeError as e:
            print(e)
            return

        if c["print_img_received"]:
            time = human_time(data.header.stamp)
            rospy.loginfo("Message received from {}".format(time))

        results = image_to_angle(img_cv, bounds_core_ball(), bounds_edge_ball())

        # Check if both balls were found
        for ball in ["core", "edge"]:
            if results["center"][ball] is None:
                rospy.loginfo("No {} contour found!".format(ball))

        # Publish all results
        angle = results["angle"]
        if angle is not None:
            angle += get_dy("angle_offset")
            self.publish_angle(angle, header)
        for ball in ["core", "edge"]:
            self.pub_masks[ball].publish(results["mask"][ball])
            self.pub_areas[ball].publish(results["area"][ball])
        self.pub_rect_img.publish(results["rect_img"])

    def publish_angle(self, angle, header):
        angle_msg = JointState()
        angle_msg.header = header
        angle_msg.position = [angle]
        self.pub_angle.publish(angle_msg)
