# Micro servo arm ROS package
A simple ROS package for a small robot arm found [here](https://www.thingiverse.com/thing:34829/files) on thingiverse.
The purpose of this is for familiarization with ROS, and Gazebo.

## Current state
Currently, the arm is an *extremely* crude model, simply consisting of the first
rotational base joint of the arm. This README is also in pretty rough shape. :)

## Usage
There is a simple launch file for starting the robot state publisher
```bash
ros2 launch micro_robot_arm robot_state_publisher.launch.py
```
Use this with Rviz and the joint state publisher gui

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

## Helpful links
[ROS link docs](http://wiki.ros.org/urdf/XML/link)
[ROS joint docs](http://wiki.ros.org/urdf/XML/joint)
[Gazebo element](http://wiki.ros.org/urdf/XML/Gazebo) (which won't be needed
for quite some time)
