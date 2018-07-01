# eddiebot_navigation

This package implements eddiebot navigation stack, which performs map building and navigating a known map given a target goal.



## 1. Dependency

For running the eddiebot navigation stack, following packages should be installed

- [openni2_launch](http://wiki.ros.org/openni2_launch)
- [rtabmap_ros](http://wiki.ros.org/rtabmap_ros)



## 2. Usage

### 2.1 SLAM Map Building

Open a new terminal, boot up all eddiebot drivers

```bash
roslaunch eddiebot_bringup minimal.launch
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

