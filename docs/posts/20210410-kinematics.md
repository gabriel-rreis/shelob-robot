---
layout: default
title: "Inverse Kinematics"
permalink: /posts/20210410-kinematics/
---

# Inverse Kinematics

Apr 10, 2021 

There are two type of solutions that I would like to try on this project:

- Using Inverse Kinematics "from the ground up" as the [Phoenix Code](https://github.com/KurtE/Phantom_Phoenix) in some low-level language that could be used directly on the robot. There are a couple of good examples of people solving the 3DOF leg problem using [JavaScript](https://github.com/mithi/hexapod-kinematics-library) and [C++](https://github.com/neuroprod/InsectRobotSimulation) this way.
- ROS (Robotic Operational System) [IKFast library](http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/ikfast/ikfast_tutorial.html), which is a more "academic" approach as I understand. From someone that worked in mechatronics engineering laboratories as an undergraduate student for a couple of years with robots running on ROS, this is something maybe too close to home for my comfort, but it's an alternative.

In anyway, here is the modeling for a leg of the hexapod:

![Leg Modeling](/assets/img/leg_model.png)

* * *

[⌫ Back](./../../)