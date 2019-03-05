import rospy


class Publisher(object):
    def __init__(self, topic, msg_type):
        self.pub = rospy.Publisher(topic, msg_type, queue_size=10)

    def publish(self, msg):
        self.pub.publish(msg)
