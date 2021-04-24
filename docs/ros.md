---
layout: default
title: "ROS"
permalink: /ros/
---

# ROS

Apr 10, 2021 

## What is ROS?

ROS stands for **Robot Operating System** and could be simply described as robotics middleware suite. The framework was originally designed to facilitate research on the academic environment but it's used today also in companies and hobbyist communities. ROS can be divided into four main parts: plumbing, development tools, robot capabilities and the ROS ecosystem.

### Plumbing

This includes a variety of **process management** tools and **inter-process communication** for communication between the modules of the robot. This can come in different core libraries or **drivers** (wrappers) for different devices.

### Development Tools

The framework provides also a lot of tools for the development process. Since robots tend to operate in the real world, a lot of simulation is needed prior to jumping to the hardware. This is provided by a bundle of **visualization**, **graphical interfaces** and **data logging** tools for testing and debugging the software.

### Robot Capabilities

To solve a specific proposed novel problem, robots tend to rely on solutions for other complex problems already solved by the community. This is facilitated by libraries and modules for tasks such as **robot control**, **task planning**, **environment perception**, **surroundings mapping**, **object manipulation** and others.

### Ecosystem

As a framework, ROS is maintained by the community. This maintenance includes **package organization**, **software distribution**, **documentation** and **tutorials**. Most of this work can be found on the [ros.org](http://ros.org) website.

## ROS Philosophy

- **Peer-to-peer**: individual software communicate over defined APIs, usually ROS *messages* and *services*.
- **Distributed computing**: software runs on multiple computers and communicate over the network provided to achieve a goal.
- **Language agnostic**: modules can be written in any language for which a client library exists. C++ is most common mostly because the original project was written using it, but it's possible to find alternatives in Python, MATLAB, Java and others.
- **Lightweight**: libraries provide a simple approach to wrap code with a thin ROS layer.
- **Free and open-source**: as a community-based framework, most ROS software relies heavily on open source code and community engagement to keep itself novel and updated.  ****

## ROS Concepts

ROS is composed of elements that together can form a **graph** is comprised of many **nodes** working in concert**.** For more on the ROS concepts like graph, nodes, topics, services, services, parameters, and actions, consider reading the [ROS2 official documentation](https://docs.ros.org/en/foxy/Tutorials.html). 

## ROS Version

In this project, I will be using ROS2 Foxy running from my computer (Arch Linux) and on the embedded boards (Debian Linux).

![ROS2 Foxy](https://upload.wikimedia.org/wikipedia/commons/thumb/2/21/ROS2_Foxy_Fitzroy_poster.png/1200px-ROS2_Foxy_Fitzroy_poster.png)

* * *

[⌫ Back](./../)