#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def callback(data):
    number = data.data
    rospy.loginfo("Squared: {}".format(float(number) ** 2))


def listener():
    rospy.init_node('randnumreceiver', anonymous=True)
    rospy.Subscriber('randnum', String, callback)

    rospy.spin()


if __name__ == '__main__':
    listener()
