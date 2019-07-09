import rospy
import numpy as np
from datetime import datetime


def _bounds(ball):
    low_keys = ["{}_Low_{}".format(t, ball) for t in "HSV"]
    high_keys = ["{}_High_{}".format(t, ball) for t in "HSV"]
    low = [get_dy(key) for key in low_keys]
    high = [get_dy(key) for key in high_keys]
    return np.array([low, high])


def bounds_core_ball():
    return _bounds("Core")


def bounds_edge_ball():
    return _bounds("Edge")


def get_dy(param):
    node = rospy.get_param("node_name")
    server = rospy.get_param("server_name")
    full_param = "/{}/{}/{}".format(node, server, param)
    return rospy.get_param(full_param)


def human_time(time_stamp):
    ts = time_stamp.to_time()
    return datetime.utcfromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
