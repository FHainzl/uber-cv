from os import path

config = {
    "publisher_q_size": 1,

    "publish_velocity": True,

    # Largest contour within area range below will be considered the ball
    "min_area": 600,
    "max_area": 1800,

    "print_img_received": True,

    # Paths inside ros node are relative to ROS_HOME (~/.ros by default)
    # Use absolute path instead
    "catkin_path": "/home/fab/catkin_ws"
}

# Build absolute path to params.json
json_relative_path = "src/uber-cv/src/uber_cv/params.json"
config["param_path"] = path.join(config["catkin_path"], json_relative_path)

config = config
