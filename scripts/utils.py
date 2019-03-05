from datetime import datetime

import rospy


def human_time(time_stamp):
    ts = time_stamp.to_time()
    return datetime.utcfromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')


def get_dy(param):
    node = rospy.get_param("node_name")
    server = rospy.get_param("server_name")
    full_param = "/{}/{}/{}".format(node, server, param)
    return rospy.get_param(full_param)
