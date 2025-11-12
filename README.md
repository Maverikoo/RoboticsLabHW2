# HW2 - Kinematic and vision-based controller for ros2-iiwa

## Overview: 

This project implements Homework 2 of the Robotics Lab 2025 course.
The goal of this homework is to develop kinematic and a vision-based controller for a robotic manipulator arm in the simulation environment using KDL and aruco_ros.
The project is based on the ros2_kdl_package package and the ros2_iiwa package provided by the course.

Download the repository's content in the docker's image folder

We created a launch file for every point of the homework.

## 1 Kinematic control:

##  a: 
To launch the iiwa robot:
```bash
ros2 launch iiwa_bringup iiwa.launch.py
```
To launch the controller node:
```bash
ros2 launch ros2_kdl_package kdl_node_1a.launch.py
```

## b:
To launch the iiwa robot:
```bash
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```
To launch the velocity controller:

-Velocity_ctrl
```bash
ros2 launch ros2_kdl_package kdl_node_1b.launch.py cmd_interface:=velocity ctrl:=velocity_ctrl
```
-Velocity_ctrl_null
```bash
ros2 launch ros2_kdl_package kdl_node_1b.launch.py cmd_interface:=velocity ctrl:=velocity_ctrl_null
```

## c:
To launch the simulation:
```bash
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```

To launch the server with the velocity_ctrl_null controller:
```bash
ros2 launch ros2_kdl_package kdl_node_1c.launch.py cmd_interface:=velocity ctrl:=velocity_ctrl_null
```

To launch the client:
```bash
ros2 launch ros2_kdl_package execute_trajectory.launch.py
```

## 2 Vision-based control:

Not working...

To launch the gazebo enviroment
```bash
ros2 launch iiwa_description iiwa_world.launch.py
```

To launch the server with the vision controller:
```bash
ros2 launch ros2_kdl_package kdl_node_1c.launch.py cmd_interface:=velocity ctrl:=vision
```

To launch the client:
```bash
ros2 launch ros2_kdl_package execute_trajectory.launch.py
```
