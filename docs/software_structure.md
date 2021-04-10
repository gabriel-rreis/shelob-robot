---
layout: default
title: "Software Structure"
permalink: /software_structure/
---

# Software Structure

Apr 10, 2021 

It's common for a robotics software project to be divided into layers to abstract problems faced when dealing with high-level software to solve complex problems like navigation or vision and also with low-level software to move motors and read sensors. Based on standards used in [ROS](https://www.ros.org/) frameworks like the [Interbotix Research Robotics Open Standard (IRROS)](https://github.com/Interbotix/interbotix_ros_core) for robotics platforms, I divided the problem into four layers: **application**, **control**, **driver**, ****and **hardware**. The following diagram summarizes the responsibilities of each leayer.

![Software Diagram](https://raw.githubusercontent.com/gabriel-rreis/shelob-robot/main/docs/software_structure.png)

In this diagram the colour of the box represents the base language used:

- **Python** is in purple;
- **C++ (ROS)** is in blue;
- Other **C based languages** in green.

The boxes with a grey background represent the physical firmware that is running on the actuators and sensors when there is one.

## Application Layer

This layer is composed of Python modules to work on higher-level problems. The language was chosen because of Python's great community and also because this writer mainly knows Python. Most of the connections with the lower layers were already solved by other projects and with that, I could focus on the solving of complex problems.

The responsibilities of this layer are:

- **Inverse kinematics solvers**: how to convert position and velocity of the tip of the legs into motor angular positions.
- **Walking-gait pattern generators**: ****how to compute gait patterns for walking/crawling with the legs.
- **Sensor fusion**: how to filter multiple sensor signals to deal with noise and errors.
- **Navigation**: how to move in the environment presented, dealing with slopes, difficult terrain and obstacles.
- **Environment Manipulation**: how to act upon the environment with the robot's arms.
- **Vision**: how to transform sensor information into a internal environment model.
- **Robot tasking management**: how to prioritize tasks given with various routines and without the constraints imposed by the robot's body and outer environment.

## Control Layer

The second layer is for controlling the robot and work as a middleware. ROS was chosen for this task because its goal is exactly that. ROS provides services for heterogeneous computer clusters such as hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management.

This first ROS layer contains the YAML files that describe the parameters for actuators and sensors in the **[Unified Robot Description Format (URDF)](http://wiki.ros.org/urdf)**. This provides an easy way to switch between other robot models (future?) with little to no effort. Here is also some routines to **initialize the robot** and to serve as a **message tunnel** to all the actuators/sensors later described on the next layer.

## Driver Layer

The first high-level software layer contains the **ROS wrappers for both the actuators and sensors** plugged into the robot's computer. This allows to quickly substitute sensors and actuators abstractions for a possible upgrade or change needed. These wrappers provide ROS interfaces to set or get data to/from the physical devices. It essentially abstracts away all the lower-level 'plumbing' code (like serial protocols, register addresses, etc...). Third-party actuator or sensor ROS wrappers also should fit in this layer.

## Hardware Layer

This last layer describes the physical setup of the robot, specifically what **actuators** and/or **sensors** are plugged into the robot computer. The computer could be anything that can run ROS and I will be probably running this from a development computer board like a *Raspberry Pi* or a *BeagleBone* *Board* of sorts. The drivers controls the hobby servos used for this project. Also the sensor devices could be anything hooked up to some force sensing resistors, switches, potentiometers, or cameras.

* * *

[⌫ Back to home](/shelob-robot/)