import rospy
import numpy as np
from datetime import datetime


def bounds(ball):
    """
    Look up HSV bounds for range operation from dynamic parameter server

    :param ball: "Core" or "Edge"
    :return: array of shape (2,3) with lower and upper bounds in HSV
    """
    ball = ball.capitalize()
    low_keys = ["{}_Low_{}".format(t, ball) for t in "HSV"]
    high_keys = ["{}_High_{}".format(t, ball) for t in "HSV"]
    low = [get_dy(key) for key in low_keys]
    high = [get_dy(key) for key in high_keys]
    return np.array([low, high])


def get_dy(param):
    """
    Get a parameter from dynamic parameter server by prepending node and server

    :param param: string of parameter name
    :return: value from parameter server
    """
    node = rospy.get_param("node_name")
    server = rospy.get_param("server_name")
    full_param = "/{}/{}/{}".format(node, server, param)
    return rospy.get_param(full_param)


def human_time(time_stamp):
    ts = time_stamp.to_time()
    return datetime.utcfromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
