# eddiebot_vel_controller

This package implements a function that convert ros standard velocity command message (i.e., [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)) into eddiebot [self-defined velocity command](eddiebot_msgs/msg/Velocity.msg).



It subscribes velocity command from [yocs_cmd_vel_mux](http://wiki.ros.org/yocs_cmd_vel_mux) whose configuration file is defined [here](../eddiebot_bringup/param/cmd_vel_mux_config.yaml), and publishes velocity command to eddiebot [controller](../eddiebot_bringup/src/eddie_controller.cpp) directly.