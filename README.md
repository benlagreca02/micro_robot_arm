# Micro servo arm ROS package
A simple ROS package for a small robot arm found [here](https://www.thingiverse.com/thing:34829/files) 
on thingiverse.

The purpose of this is for familiarization with ROS, and Gazebo.

## Current state
Currently, the arm is a simplified, rough approximation of the small servo arm,
consisting of multiple joint segments simplified to large rectangles. The
gripper hasn't been implemented, however our 3D print of the arm has an unusable
gripper.

## Usage
You should have a "ros development workspace folder" (usually called `dev_ws`),
and the project structure should be `dev_ws/src/micro_robot_arm`. From the
`dev_ws` directory, run the following command to build and compile the package

```bash
colcon build --symlink-install
```

Next, you must source the project by running the following command. This assumes
you are using bash as your shell.

```bash
source dev_ws/install/setup.bash
```
This allows you to run the various launch files provided by the package as
listed below.

`robot_state_publisher.launch.py`
: Launches robot state publisher and nothing more or less

`rvizSim.launch.py`
: Launches Rviz only (useless on its own)

`rspGazebo.launch.py`
: Launches Robot state publisher, and Gazebo


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

[Gazebo element](http://wiki.ros.org/urdf/XML/Gazebo) 
