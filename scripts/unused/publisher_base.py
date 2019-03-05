#!/usr/bin/env python

import sys
import rospy
from std_msgs import msg
import random


class publisher:
    def __init__(self, topic, type, node_name):
        self.pub = rospy.Publisher(topic, type, queue_size=10)
        rospy.init_node(node_name, anonymous=True)

    def publish(self, msg):
        self.pub.publish(msg)


def publisher():
    pub = rospy.Publisher('randnum', msg.String, queue_size=10)
    rospy.init_node('randnumgenerator', anonymous=True)
    rate = rospy.Rate(2)  # 10hz
    while not rospy.is_shutdown():
        x = random.normalvariate(0, 1)
        rospy.loginfo(x)
        pub.publish(str(x))
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
