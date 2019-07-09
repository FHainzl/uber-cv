import rospy
import json
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from cv_ros_tools import get_dy


def setup_ddrec_server():
    params_file = "/home/fab/catkin_ws/src/uber-cv/src/uber_cv_ros/params.json"

    def dyn_rec_callback(config, level):
        param_names = [
            # Core Ball
            "H_Low_Core",
            "H_High_Core",
            "S_Low_Core",
            "S_High_Core",
            "V_Low_Core",
            "V_High_Core",
            "H_Low_Core",
            # Edge Ball
            "H_Low_Edge",
            "H_High_Edge",
            "S_Low_Edge",
            "S_High_Edge",
            "V_Low_Edge",
            "V_High_Edge",
            # Angle
            "angle_offset"
        ]

        try:
            if get_dy("SaveToJson") == 1:
                new_params = {param: get_dy(param) for param in param_names}
                with open(params_file, 'w') as f:
                    json.dump(new_params, f)
                rospy.loginfo("Parameters saved to json")
        except KeyError:
            rospy.loginfo("Param not found, is parameter server running?")

        return config

    dd = DDynamicReconfigure("cv_dyn_rec")

    with open(params_file, 'r') as f:
        params = json.load(f)

    # In Range for central ball
    dd.add_variable("H_Low_Core", "Lower bound hue for central ball",
                    params["H_Low_Core"], 0, 180)
    dd.add_variable("H_High_Core", "Higher bound hue for central ball",
                    params["H_High_Core"], 0, 180)
    dd.add_variable("S_Low_Core", "Lower bound saturation for central ball",
                    params["S_Low_Core"], 0, 255)
    dd.add_variable("S_High_Core", "Higher bound saturation for central ball",
                    params["S_High_Core"], 0, 255)
    dd.add_variable("V_Low_Core", "Lower bound value for central ball",
                    params["V_Low_Core"], 0, 255)
    dd.add_variable("V_High_Core", "Higher bound value for central ball",
                    params["V_High_Core"], 0, 255)

    # In Range for edge ball
    dd.add_variable("H_Low_Edge", "Lower bound hue for edge ball",
                    params["H_Low_Edge"], 0, 180)
    dd.add_variable("H_High_Edge", "Higher bound hue for edge ball",
                    params["H_High_Edge"], 0, 180)
    dd.add_variable("S_Low_Edge", "Lower bound saturation for edge ball",
                    params["S_Low_Edge"], 0, 255)
    dd.add_variable("S_High_Edge", "Higher bound saturation for edge ball",
                    params["S_High_Edge"], 0, 255)
    dd.add_variable("V_Low_Edge", "Lower bound value for edge ball",
                    params["V_Low_Edge"], 0, 255)
    dd.add_variable("V_High_Edge", "Higher bound value for edge ball",
                    params["V_High_Edge"], 0, 255)

    # Angle offset
    dd.add_variable("angle_offset",
                    "Offset [rad] to added to angle such that hanging "
                    "pendulum yields an angle of zero",
                    params["angle_offset"], -3.142, 3.142)

    dd.add_variable("SaveToJson", "Evoke callback", 0, 0, 1)

    dd.start(dyn_rec_callback)

    return dd
