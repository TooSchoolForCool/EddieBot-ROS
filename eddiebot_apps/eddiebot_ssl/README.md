# eddiebot_ssl

This package implements training process for eddiebot performing sound source localization in gazebo simulation world.

## 1. Demo

![demo](assets/demo/demo.png)

The robot will automatically explore the room according to the ranking result and find out the target.

## 2. Installation

### 2.1 Dependencies

For running the eddiebot gazebo simulation, following dependencies needs to be installed

- [turtlebot](http://wiki.ros.org/turtlebot)
- [turtlebot-gazebo](http://wiki.ros.org/turtlebot_gazebo)
- [rtabmap-ros](http://wiki.ros.org/rtabmap_ros)

Try following commands to install the dependencies

```
sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-gazebo ros-kinetic-rtabmap-ros
```

### 2.2 Environment Setup

For viewing our self-defined models in the simulation environment, you need to **update** a gazebo global environment variable. Use following commands to update,

```
export GAZEBO_MODEL_PATH=<path_to_eddiebot_gazebo>/objects:$GAZEBO_MODEL_PATH
```

For convenience, you can add this command to your `~/.bashrc` or `~/.zshrc`.

## 3. Usage

Firstly, we need to launch the simulation world and acml

*Note: In this step, you need to have a **pre-built** map for acml performing localization*.

```bash
roslaunch eddiebot_ssl bringup_world_explore.launch
```

Next we can launch the trainer,

```bash
roslaunch eddiebot_ssl start_trainer.launch
```

*For more information and configuration, please check the launch file [start_trainer.launch](launch/start_trainer.launch)*

Once the trainer is launched, the robot will automatically start learning the capability of room-based sound source localization. 