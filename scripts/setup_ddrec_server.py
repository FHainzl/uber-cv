from math import pi

from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure


def setup_ddrec_server():
    def dyn_rec_callback(config, level):
        return config

    dd = DDynamicReconfigure("cv_dyn_rec")

    # In Range for central ball
    dd.add_variable("H_Low_Core", "Lower bound hue for central ball",
                    74, 0, 180)
    dd.add_variable("H_High_Core", "Higher bound hue for central ball",
                    100, 0, 180)
    dd.add_variable("S_Low_Core", "Lower bound saturation for central ball",
                    180, 0, 255)
    dd.add_variable("S_High_Core", "Higher bound saturation for central ball",
                    255, 0, 255)
    dd.add_variable("V_Low_Core", "Lower bound value for central ball",
                    57, 0, 255)
    dd.add_variable("V_High_Core", "Higher bound value for central ball",
                    255, 0, 255)

    # In Range for edge ball
    dd.add_variable("H_Low_Edge", "Lower bound hue for edge ball",
                    19, 0, 180)
    dd.add_variable("H_High_Edge", "Higher bound hue for edge ball",
                    25, 0, 180)
    dd.add_variable("S_Low_Edge", "Lower bound saturation for edge ball",
                    193, 0, 255)
    dd.add_variable("S_High_Edge", "Higher bound saturation for edge ball",
                    255, 0, 255)
    dd.add_variable("V_Low_Edge", "Lower bound value for edge ball",
                    102, 0, 255)
    dd.add_variable("V_High_Edge", "Higher bound value for edge ball",
                    255, 0, 255)

    # Angle offset
    dd.add_variable("angle_offset",
                    "Offset [rad] to added to angle such that hanging "
                    "pendulum yields an angle of zero",
                    0.0, -3.142, 3.142)

    dd.start(dyn_rec_callback)

    return dd
