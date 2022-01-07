# WoLF: Whole-body Locomotion Framework for quadruped robots

This repo contains a collection of different robots and sensors used in WoLF.

## Setup

See the documentation [here](https://github.com/graiola/wolf-setup/blob/master/README.md).

## How to add a new robot

To add a new robot you can start by copying the structure of an existing one. The robot descriptions are defined as ROS packages, so you will need to add it to your ROS workspace.
The required files in the package are the following:

- `params/robot_name_params.yaml` it contains the controller parameters, use this file to tune a robot.
- `robots/robot_name.urdf.xacro` it is the robot URDF description describing the kinematics and dynamics of the robot.
- `robots/robot_name.srdf.xacro` it is the robot SRDF description, use this file to define the kinematic chains and default postures.

You can add plotjuggler and rviz visualization files in the respective folders `plotjuggler` and `rviz`. The folder `urdfs` can be used to store extra xacro files useful for the robot description. And last but not least, don't forget the robot's meshes in the `meshes` folder, unless your robot is a stealth one :)

To launch the new robot with the WoLF controller:

`roslaunch wolf_controller wolf_controller_bringup.launch robot_name:=new_robot`
