# eddiebot_bringup

The `eddiebot_bringup` package provided `roslaunch` scripts for starting the EddieBot base functionality, and implemented eddiebot basic functions and drivers



## 1. Dependency

For running the eddiebot_bringup, following packages should be installed

- [openni2_launch](http://wiki.ros.org/openni2_launch)



## 2. Launch File Description

- [minimal_primesense.launch](launch/minimal_primesense.launch): this file provides the minimal launch of the eddiebot **with Primesense Carmine** for running eddiebot applications, which will boot up the IR sensor, Ping sensor, motor controller, camera driver and Odometry publisher.
- [minimal_kinect_v2.launch](/launch/minimal_kinect_v2.launch): this file provides the minimal launch of the eddiebot **with Kinect v2**

