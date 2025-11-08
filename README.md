# HW2 - Kinematic and vision-based controller for ros2-iiwa

## Overview: 

This project implements Homework 2 of the Robotics Lab 2025 course.
The goal of this homework is to develop kinematic and a vision-based controller for a robotic manipulator arm in the simulation environment using KDL and aruco_ros.
The project is based on the ros2_kdl_package package and the ros2_iiwa package provided by the course.

Download the repository's content in the docker's image folder
```bash
git clone https://github.com/Maverikoo/RoboticsLabHW2.git
```
We created a .launch.py file for every point of the homework.

## 1 Kinematic control:

##  a: 
Per lanciare la simulazione del robot:
```bash
ros2 launch iiwa_bringup iiwa.launch.py
```
Per lanciare il controllo:
```bash
ros2 launch ros2_kdl_package kdl_node_1a.launch.py
```