import rospy
from config import config as c


class Publisher(object):
    def __init__(self, topic, msg_type):
        self.pub = rospy.Publisher(topic, msg_type,
                                   queue_size=c["publisher_q_size"])

    def publish(self, msg):
        self.pub.publish(msg)
