#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge, CvBridgeError
from uber_cv_ros.publishers.publisher import Publisher
from uber_cv_ros.publishers.img_publisher import ImgPublisher

from ros_tools import get_dy, bounds, human_time
from uber_cv.cv_tools import image_to_angle
from uber_cv.config import config as c


class Node:
    def __init__(self, topic, encoding):
        self.encoding = encoding

        self.bridge = CvBridge()
        self.publish_angular_vel = rospy.get_param("angular_vel")
        # Last angle and time stamp to calculate angular acceleration
        self.last_angle = None
        self.last_stamp = None

        # Publisher nodes
        self.pub_masks = {"core": ImgPublisher("mask_core"),
                          "edge": ImgPublisher("mask_edge")}
        self.pub_areas = {"core": Publisher("area_core", Float32),
                          "edge": Publisher("area_edge", Float32)}

        self.pub_rect_img = ImgPublisher("circles_image")
        self.pub_angle = Publisher("angle", JointState)

        # Subscriber node
        self.image_sub = rospy.Subscriber(topic, Image, self.process_image)

    def process_image(self, data):
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

        results = image_to_angle(img_cv, bounds("core"), bounds("edge"))

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
        if not self.publish_angular_vel:
            self.pub_angle.publish(angle_msg)
            
        else:
            stamp = header.stamp.to_sec()
            angular_velocity = self.angular_velocity(angle, stamp)
            if angular_velocity is not None:
                angle_msg.velocity = [angular_velocity]
                self.pub_angle.publish(angle_msg)

            self.last_angle = angle
            self.last_stamp = stamp

    def angular_velocity(self, angle, stamp):
        if self.last_angle is None:
            return None
        dt = stamp - self.last_stamp
        if dt > 0.1:
            return None
        return (angle - self.last_angle) / dt
