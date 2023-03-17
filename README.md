# HALO AUV

HALO AUV is an underwater robotic platform designed for student's in Northwestern University's M.S. in Robotics
program to explore underwater robotics. It can be controlled manually via a video game controller or through ROS2.
This repository consists of ROS packages to interface with the robot's camera and control it.

# Demonstration

[git_vid(1)(1)(1).webm](https://user-images.githubusercontent.com/46512429/225805866-1e266bba-3032-4d6f-bc3e-513fe9768ad3.webm)

## Documentation
More general setup documentation can be found at ..........

## System Architecture
`halo_auv` is a python ROS2 package that has node to set up communication and control the robot, obtain
the robot's camera stream and publish it as a ROS image, and ROS node for an autonomous docking capability.

`halo_auv_interfaces` consists of the custom srv to interact with the halo_auv package.

## Setup and External Packages
In order to use these packages you must have pymavlink (https://www.ardusub.com/developers/pymavlink.html),
OpenCV (https://opencv.org/), and Gstreamer (https://gstreamer.freedesktop.org/) installed.

## Launch File
`auv.launch.xml` launches the `auv_control` node, `auv_camera` node, and Rviz.

## Operation 
0. Call the launch file `ros2 launch halo_auv auv.launch.xml`. 

1. To move the robot call `ros2 service call /set_relative_pos halo_auv_interfaces/srv/AuvPose "x: 0.0
depth: 0.0
heading: 0.0" `

