# eddiebot

This folder provides all the basic drivers for running and using the parallax [eddie](https://www.generationrobots.com/en/401394-eddie-robot-platform-parallax.html) robot with [ROS](http://wiki.ros.org).



Here is a brief introduction to every package,

- **[eddie_bringup](eddiebot_bringup)** provides roslaunch scripts for starting the eddiebot base functionality
- **[eddiebot_description](eddiebot_description)** provides urdf of the eddiebot
- **[eddiebot_launchers](eddiebot_launchers)** provides a collection of eddiebot launch files
- **[eddiebot_msgs](eddiebot_msgs)** defines eddiebot related messages and services
- **[eddiebot_teleop](eddiebot_teleop)** provides teleoperation for eddiebot by using keyboard
- **[eddiebot_odom](eddiebot_odom)** generates [Odometry](http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom) topic based on encoder information

