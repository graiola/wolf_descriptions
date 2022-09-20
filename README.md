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

## Parameters description

- `default_duty_factor` defines the ratio between the stance and total cycle time: `T_stance / T_cycle`.
- `default_swing_frequency` defines the frequency of the swing: `1 / T_swing`.
- `default_contact_threshold` defines the force magnitude  that is used to detect if the robot is contact with the enviroment or not.
- `default_step_reflex_contact_threshold` if the step reflex is active, it represents the magnitude of the contact force that triggers a step reflex, by default it is calculated as `default_contact_threshold * 1.5`.
- `default_step_reflex_max_retraction` if the step reflex is active, this is the maximum step retraction length, by default it is calculated as `max_step_height/2.0`.
- `default_base_linear_velocity_x` linear velocity along robot's x axis.
- `default_base_linear_velocity_y` linear velocity along robot's y axis.
- `default_base_linear_velocity_z` linear velocity along robot's z axis.
- `default_base_linear_velocity` defines a common velocity value along xyz axis, it overrides the previous linear velocities.
- `default_base_angular_velocity_roll` angular velocity along robot's roll axis.
- `default_base_angular_velocity_pitch` angular velocity along robot's pitch axis.
- `default_base_angular_velocity_yaw`  angular velocity along robot's yaw axis.
- `default_base_angular_velocity` defines a common velocity value along roll pitch and yaw axis, it overrides the previous angular velocities.
- `default_step_height` defines the step height.
- `default_cutoff_freq_gyroscope` cut off frequency for the gyroscope filter.
- `default_cutoff_freq_qdot` cut off frequency for the joint velocities filter.
- `default_friction_cones_mu` friction cones [mu](https://scaron.info/robot-locomotion/friction-cones.html).
- `max_step_height` defines the maximum step height allowed.
- `max_step_length` defines the maximum step length allowed.
- `max_base_roll` defines the maximum rotation along the robot's roll axis.
- `min_base_roll` defines the minimum rotation along the robot's roll axis.
- `max_base_pitch` defines the maximum rotation along the robot's pitch axis. 
- `min_base_pitch` defines the minimum rotation along the robot's pitch axis.
- `estimation_position_type [ground_truth (simulation only) | estimated_z]` 
- `estimation_orientation_type [ground_truth (simulation only) | imu_magnetometer | imu_gyroscope]` 
- `use_contact_sensors [true | false]` use the `hardware_interface::ContactSwitchSensorInterface` if available
- `compute_odom [true | false]` compute the odometry i.e. create the odom -> base_footprint transformation
- `activate_step_reflex [true | false]` activate the step reflex.
- `activate_push_recovery [true | false]` activate the push recovery.
- `initial_pose_simulation` Initial robot's pose in simulation.
- `activate_com_z [true | false]` activate the control of the robot's height through the CoM task, otherwise use the waist task. By default it is true.
- `activate_postural [true | false]` activate the postural task, by default it is false.
- `activate_angular_momentum [true | false]` activate the angular momentum task, by default it is true.
- `activate_joint_position_limits [true | false]` activate the joint position limits constraint (the values are loaded from the URDF), by default it is false.
- `regularization` regularization value for the solver, by default it is set to `1e-3` for the accelerations and `1e-6` for the forces.
```

gains:

      Kp_leg: {haa: xxx, hfe: xxx, kfe:xxx}
      Kd_leg: {haa: xxx, hfe: xxx, kfe: xxx}

      xx_foot:
          Kp: {x: xxx, y: xxx, z: xxx, roll: 0, pitch: 0, yaw: 0}
          Kd: {x: xxx, y: xxx, z: xxx, roll: 0, pitch: 0, yaw: 0}
          type: [force|acceleration]
          weight: xxx

      waist:
          Kp: {x: 0.0, y: 0.0, z: xxx, roll: xxx, pitch: xxx, yaw: xxx}
          Kd: {x: 0.0, y: 0.0, z: xxx, roll: xxx, pitch: xxx, yaw: xxx}
          type: [force|acceleration]
          weight: xxx

      CoM:
          Kp: {x: xxx, y: xxx z: xxx}
          Kd: {x: xxx, y: xxx, z: xxx}
          weight: xxx

      postural:
          weight: xxx

      angular_momentum:
          weight: xxx

      postural:
          weight: xxx
```

- `Kp_leg` and `Kd_leg` define the impedance gains for the the `hip abduction/adduction (haa)`, `hip flexion/extension (hfe)` and `knee flexion/extension (kfe)` joints in each leg. These gains are used for different things: 
	- to zero the joints during the init phase.
	- to perform a controlled shutdown in case of anomaly (using only the damping value of the impedance).
	- to set the gains for the postural task.
- The default tasks in WoLF are the following:
	- `xx_foot` these tasks allow the robot to track the swing trajectories with its legs. Since we are working with a quadruped we usually have 4 of them defined as `left front (lf)`, `right front (rf)`, `left hind (lh)` and `right hind (rh)`.
	- `waist` this task is used to control the attitude of the robot's base/trunk and its height if `activate_com_z` is false.
	- `CoM` this task is used to stabilize the Center of Mass (CoM) of the robot with respect to its [support polygon](https://scaron.info/robot-locomotion/zmp-support-area.html) along x and y, and control the CoM height if `activate_com_z` is true.
	- `postural` this task is used to impose a preferred joint configutation to the robot, by default the stand up posture is used. This task can be activated with `activate_postural` set to true.
	- `angular_momentum` this task is used to counteract angular momentum variations generated on the robot's base/trunk by disturbances. This task can be activated with `activate_angular_momentum` set to true.
	- `xx_arm` if the robot has an arm mounted on top, it is possible to add a task to control the arm end-effector position and orientation.
- tasks' gains:
	- `Kp` and `Kd` define the proportional and derivative gains for each task.
	- `type  [force|acceleration]` by default the tasks' gains are defined as force gains meaning that `Kp` and `Kd` work as Cartesian impedance gains. Please check Appendix A in the following [paper](https://hal.archives-ouvertes.fr/hal-03005133/document) for a better explanation of Cartesian impedance.
	- `weight` relative task weight. A higher value means that the selected task has a higher priority compared to the others and therefore the solver will try to minimize this task first.

## How to reset the robot pose and posture

To reset the robot pose in gazebo you can simply use the gazebo's shortcut `ctrl-r`, this command should set the robot pose to the origin. Then, to reset the robot's posture run the following:

`rosrun wolf_description_utils go0`

now your robot should be ready to stand up again!

## How to define the standup and standdown posture

To define the stand up and stand down posture in the SRDF file, you can use the `joint_state_publisher` by launching the following command in your terminal:

`roslaunch wolf_description_utils standalone.launch robot_name:=your_robot`

<p align="center">
<img src="https://user-images.githubusercontent.com/4747910/154806597-4985929f-7dc1-4f8f-8aa2-5a6daad717be.png" alt="joint_state_publisher" width="400"/>
</p>

## Disclaimer

Some robots have been tuned better than others, please don't be sad if your favorite robot doesn't work perfectly.
At the moment, the robots we feel more confident with are:

- Spot
- Aliengo

Feel free to tune and share your parameters with us if you feel they improve the control of a specific robot :)

## Legal notes

`wolf_description_utils` and `sensors_description` are licensed under a license GNU General Public License v3.0.
 For the robot descriptions all rights belong to their respective owners.
