# HALO AUV

HALO AUV is an underwater robotic platform designed for student's in Northwestern University's M.S. in Robotics
program to explore underwater robotics. It can be controlled manually via a video game controller or through ROS2.
This repository consists of ROS packages to interface with the robot's camera and control it.

# Demonstration

![bot_choc-min](https://user-images.githubusercontent.com/46512429/225804557-a2c09364-de29-4c79-979f-b97d44fbcdd3.mp4)


## Documentation
More general setup documentation can be found at ..........

## System Architecture
`halo_auv` is a python ROS2 package that has node to set up communication and control the robot, obtain
the robot's camera stream and publish it as a ROS image, and ROS node for an autonomous docking capability.

`halo_auv_interfaces` consists of the custom srv to interact with the halo_auv package.

## Setup and External Packages
In order to use these packages you must have pymavlink (https://www.ardusub.com/developers/pymavlink.html),
OpenCV (https://opencv.org/), and Gstreamer (https://gstreamer.freedesktop.org/) installed.

## Operation 
0. If running for the first time, calibration must be done. To calibrate the system, the end-effector 
AprilTag must be placed in the robot's gripper, aligned with the panda_link_tcp frame, and in the 
field of view of the camera. Then call `ros2 launch halo_auv auv.launch.xml`. 
This step can now be ignored for subsequent runs.

1. Plug the robot's Ethernet cable and the camera into your computer.

2. ssh into robot and go `https://panda0.robot` in your browser.

3. In the browser window, unlock the robot and active FCI.

4. In a separate terminal run `ros2 launch trajectory botchocolate.launch.py real:=true`.

5. In another terminal run `ros2 service call /make_hot_chocolate std_srvs/srv/Empty`

6. Enjoy hot chocolate.

### Launch File
`auv.launch.xml` launches the `auv_control` node, `auv_camera` node, and Rviz.