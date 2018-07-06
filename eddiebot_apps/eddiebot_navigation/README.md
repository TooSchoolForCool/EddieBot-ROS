# eddiebot_navigation

This package implements eddiebot navigation stack, which performs map building and navigating a known map given a target goal.



## 1. Dependency

For running the eddiebot navigation stack, following packages should be installed

- [openni2_launch](http://wiki.ros.org/openni2_launch): this package provides driver for [Primesense Carmine](http://xtionprolive.com/primesense-carmine-1.09)
- [rtabmap_ros](http://wiki.ros.org/rtabmap_ros): this package implements Real-Time Appearance-Based Mapping
- [iai_kinect2](https://github.com/code-iai/iai_kinect2): this package provides driver for Kinect v2



*Note: to install [iai_kinect2](https://github.com/code-iai/iai_kinect2), some dependencies should be installed as prerequisite.*



## 2. Usage

Here, we use eddiebot with Primesene camera as an example, if you want to use eddiebot with Kinect-v2, try `minimal_kinect_v2.launch` and `rtabmap_mapping_kinect2.launch`.



### 2.1 SLAM Map Building

Open a new terminal, boot up all eddiebot drivers

```bash
roslaunch eddiebot_bringup minimal_primesense.launch
```

Open a new terminal, start rtabmap mapping mode

```bash
roslaunch eddiebot_navigation rtabmap_mapping.launch
```

Open a new terminal, start teleoperation through keyboard (i.e., control the eddiebot through keyboard)

```bash
roslaunch eddiebot_teleop teleop_keyboard.launch
```

If you want to view the map building process through rviz, run the following command in a new terminal

```bash
roslaunch eddiebot_rviz_launchers view_mapping.launch
```



Note: when stopping the map building process, by default the map is stored in `.ros/rtabmap.db`, you can change the saving directory by adding parameter `database_path` in rtabmap launch process. 

For example,

```bash
roslaunch eddiebot_navigation rtabmap_mapping.launch database_path:=<new_path>
```



### 2.2 Navigating a known map

Once you have built a map or have a known map, you can have the eddiebot navigates the map given a target goal, it can then autonomously plan through the environment. 

To send a goal: 

1. Click the "2D Nav Goal" button in rviz
2. Click on the map where you want the TurtleBot to drive and drag in the direction the eddiebot should be pointing at the end. 

This can **fail** if the path or goal is blocked. 



Open a new terminal, boot up all eddiebot drivers

```bash
roslaunch eddiebot_bringup minimal.launch
```

Open a new terminal, start rtabmap localization mode

```bash
roslaunch eddiebot_navigation rtabmap_mapping.launch localization:=true
```

Open a new terminal, start teleoperation through keyboard (i.e., control the eddiebot through keyboard)

```bash
roslaunch eddiebot_teleop teleop_keyboard.launch 
```

If you want to view the map building process through rviz, run the following command in a new terminal

```bash
roslaunch eddiebot_rviz_launchers view_mapping.launch
```



### 3. Common Problems

**1. Cannot view rtabmap MapData rivz**

We use rtabmap [rviz plugin](http://wiki.ros.org/rtabmap_ros#RVIZ_plugins) to visualize rtabmap MapData. However, this plugin has an internal problem, that is rviz should be launched before `rtabmap_mapping.launch`. What's more, once you disable it in the rviz, you should restart all the pipeline to re-view the MapData.