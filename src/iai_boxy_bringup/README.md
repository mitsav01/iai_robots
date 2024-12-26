# Lightweight Boxy simulation

## Installation

First, make sure you have the following packages installed:
  * apt-get install ros-hydro-pr2-mechanism-model ros-hydro-pr2-controller-manager ros-hydro-control-toolbox ros-hydro-pr2-mechanism-controllers


In addition to this repo, you need to have two more repos from code-iai in your workspace:
  * https://github.com/code-iai/iai_control_pkgs
  * https://github.com/code-iai/iai_common_msgs
  * https://github.com/code-iai/iai_robot_drivers

Build them all by running ```catkin_make```

## Start-up
Run the following commands each in a new terminal:
  * ```roscore```
  * ```roslaunch iai_boxy_bringup boxy_sim.launch```
  * ```rosrun rviz rviz```

In rviz,
  * set the fixed frame to ```odom```
  * add a plugin of type ```RobotModel```
  * add the ```TF``` plugin

Then you should see something like this...

![rviz view](https://raw.github.com/code-iai/iai_robots/master/iai_boxy_bringup/doc/boxy_sim_rviz_new.png)


## Manually moving the joints through the velocity-resolved interface
For testing purposes, you can command the joints through publishing velocity commands from the console.

### Moving the torso
The torso of the robot offers a velocity-resolved interface. To periodically tell the torso to move the up with 2cm/s, call:

```rostopic pub -r 20 /torso_vel/command iai_control_msgs/MultiJointVelocityCommand '{velocity: [0.02]}'```

NOTE: The current version of the torso joint simulation comes with a watchdog. It stops the torso if it has not received a command for 100ms.

### Moving the head joints
The pan-tilt unit of the head also offers a velocity-resolved interface. To move all six joints with 0.1rad/s send:

```rostopic pub -r 20 /head_vel/command iai_control_msgs/MultiJointVelocityCommand '{velocity: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]}'```

NOTE: Also the simulated head controller comes with a watchdog. It stops the head joints if it has not received a command for 100ms.

### Moving the arms
The arms of the robot offer a velocity-resolved interface which also allows you to set desired joint stiffness with every command. The move, for instance, the first two joints of the right arm with a velocity of -0.1rad/s, and have all joints have a stiffness of 80Nm/rad call:

```rostopic pub -r 20 /r_arm_vel/command iai_control_msgs/MultiJointVelocityImpedanceCommand '{velocity: [-0.1, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0], stiffness: [80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0]}'```

NOTE: The simulated arm controllers also have watchdogs stopping them if you do not send a command at least every 100ms.

NOTE: In the current version of the arm simulation, all fields of the command messages but ```velocity``` are ignored, i.e. we do not have a stiffness simulation.

### Moving the base
The base of the robot offers a twist interface. This is how to command it to move from the terminal:

```rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0.2}, angular: {z: 0.1}}'```

This command asks for translations of 10cm/s in x- and 20cm/s in y-direction, and a rotation of 0.1rad/s around the z-axis of the base_footprint of the robot.

NOTE: All other fields of the command message will be ignored.

NOTE: There is a watchdog monitoring the commands. If none come in for 0.5s the base automatically stops.

## Manually moving the joints through the position-resolved interface
For convenience purposes, we are also running some naive position controllers on-top of the velocity-resolved. This section shows you how to use this interface from the console.

### Moving the head
```rostopic pub /head_pos_controller/command std_msgs/Float32MultiArray '{data: [-1.57, -0.89, 2.28, -1.21, 1.57, 3.14]}'```

### Moving the arms
```rostopic pub /l_arm_pos_controller/command std_msgs/Float32MultiArray '{data: [0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0]}'```

```rostopic pub /r_arm_pos_controller/command std_msgs/Float32MultiArray '{data: [0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0]}'```

### Moving the torso
```rostopic pub /torso_pos_controller/command std_msgs/Float32MultiArray '{data: [-0.25]}'```

## Controlling the gripper
Both fingers of a gripper always move symmetrical. The unit of the velocity is here mm/s and the position is the distance between the two fingers in mm.

### Controlling the gripper through the velocity-resolved interface
Open right gripper:

```rostopic pub /r_gripper_vel/goal_speed wsg_50_msgs/SpeedCmd '{speed: 50.0}' ```

Close left gripper:

```rostopic pub /l_gripper_vel/goal_speed wsg_50_msgs/SpeedCmd '{speed: -50.0}' ```

### Controlling the gripper through the position-resolved interface

Opening the right gripper:

```rostopic pub /r_gripper_pos/goal_position wsg_50_msgs/PositionCmd '{pos: 110, speed: 20.0}'```

Closing left gripper:

```rostopic pub /l_gripper_pos/goal_position wsg_50_msgs/PositionCmd '{pos: 0, speed: 20.0}'```


NOTE: The force field of the Messages will be ignored.
