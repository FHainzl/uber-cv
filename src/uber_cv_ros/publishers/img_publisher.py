import cv_bridge
from sensor_msgs.msg import Image

from uber_cv_ros.publishers.publisher import Publisher


class ImgPublisher(Publisher):
    def __init__(self, topic):
        Publisher.__init__(self, topic, Image)
        self.bridge = cv_bridge.CvBridge()

    def publish(self, cv_img):
        ros_image = self.bridge.cv2_to_imgmsg(cv_img, encoding="passthrough")
        self.pub.publish(ros_image)
