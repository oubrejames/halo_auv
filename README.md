# HALO AUV

HALO AUV is an underwater robotic platform designed for student's in Northwestern University's M.S. in Robotics
program to explore underwater robotics. It can be controlled manually via a video game controller or through ROS2.
This repository consists of ROS packages to interface with the robot's camera and control it.

## System Architecture
`halo_auv` is a python ROS2 package that has node to set up communication and control the robot, obtain
the robot's camera stream and publish it as a ROS image, and ROS node for an autonomous docking capability.

`halo_auv_interfaces` consists of the custom srv to interact with the halo_auv package.

# Demonstration

<iframe width="560" height="315" src="https://www.youtube.com/embed/hvmxipyYg5g" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

## Setup and External Packages
Before being able to run BotChocolate, you must ensure that you have all the necessary Franka 
packages installed (https://nu-msr.github.io/me495_site/franka.html).
It is convenient to have a workspace that is updated to the latest version of numsr_patches (https://github.com/m-elwin/numsr_patches)

Necessary external packages outside of numsr_patches can be installed with the botchoc.repos file using
`vcs import < botchoc.repos`.

## Operation 
0. If running for the first time, calibration must be done. To calibrate the system, the end-effector 
AprilTag must be placed in the robot's gripper, aligned with the panda_link_tcp frame, and in the 
field of view of the camera. Then call `ros2 launch bot_vis launch_vision.py calibration:=true`. 
This step can now be ignored for subsequent runs.

1. Plug the robot's Ethernet cable and the camera into your computer.

2. ssh into robot and go `https://panda0.robot` in your browser.

3. In the browser window, unlock the robot and active FCI.

4. In a separate terminal run `ros2 launch trajectory botchocolate.launch.py real:=true`.

5. In another terminal run `ros2 service call /make_hot_chocolate std_srvs/srv/Empty`

6. Enjoy hot chocolate.

## Documentation


### Launch File
`launch_vision.py` launches the `april_tf`, `apriltag_node`, and `realsense` nodes as well as rviz and the `calibration` node if specified.
