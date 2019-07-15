from os import path

import rospkg

rospack = rospkg.RosPack()

config = {

    "publisher_q_size": 1,

    # Largest contour within area range below will be considered the ball
    "min_area": 600,
    "max_area": 1800,

    "print_img_received": False,

    "params_rel_path": path.join("src", "uber_cv", "params.json")
}

# Build absolute path to params.json
uber_cv_path = rospack.get_path('uber-cv')
config["param_path"] = path.join(uber_cv_path, config["params_rel_path"])

config = config
