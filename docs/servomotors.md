---
layout: default
title: "Inverse Kinematics"
permalink: /kinematics/
---

# Actuators

Apr 18, 2021 

Servomotors are userd for most robotics projects. From wikipedia:

> A servomotor is a rotary actuator or linear actuator that allows for **precise control** of angular or linear position, velocity and acceleration. It consists of a suitable motor coupled to a sensor for **position feedback**. It also requires a relatively sophisticated controller, often a dedicated module designed specifically for use with servomotors. Servomotors are not a specific class of motor, although the term servomotor is often used to refer to a **motor suitable for use in a closed-loop control system**.

There are two main types of servomotors used for hexapod robots: **"smart" servomotor** and **hobby servomotors**. 

## "Smart" Servomotors

The first type consists of a DC motor, a controller, a driver a sensor for feedback and a reduction gear. This type of motor excels in build quality and precision and usually come at a higher price. The encoder and electronics usually uses PID for position control. The final shaft usually is coupled with a high precision reduction gear and bearings for supporting the high forces provided. 

![https://blog.generationrobots.com/wp-content/uploads/2019/01/dynamixel-actuator-servo-schematics.jpg](https://blog.generationrobots.com/wp-content/uploads/2019/01/dynamixel-actuator-servo-schematics.jpg)

##### Image by [Génération Robots](https://blog.generationrobots.com/en/how-to-choose-the-right-dynamixel-servomotor/)

Another usual feature that comes with this type of motors is a network of communication that allows for a [daisy chain](https://en.wikipedia.org/wiki/Daisy_chain_(electrical_engineering)) setup for multiple motors providing a more organized way to deal with the cables for powering and communication with the motos. 

[Robotis's Dynamixel motors](https://en.wikipedia.org/wiki/Daisy_chain_(electrical_engineering)) are best know for this type of servos for smaller robots and they are praised for their quality... and not so praised for the price.

## Hobby Servomotors

But there is a cheaper alternative. For years hobbyists with a lower need for precision positioning used cheaper servomotors usually with lower quality, fewer features, and a very attractive price compared with their beefier cousins. Usually, this type of servomotor comes with different plastic horns (the part that mounts on the end shaft of the servomotor) for multiple applications. 

This type of servomotor normally is made with cheap materials and with lower specs, but there are few companies that are famous for their metal-geared servomotors like HiTec and TowerPro. The last one is the one commonly used in hexapod robots for the high return for a lower price.

![Multi-purpose Servomotor Bracket](/assets/img/servomotor_open.png)

##### Metal gears made with some brass like material (at least it's not plastic).

## Shelob's motors

For this project, I will be using [*TowerPro MG 996R*](https://components101.com/motors/mg996r-servo-motor-datasheet) motors for the legs. It's a cheap alternative readily available in my country and there is a lot of projects available using it. If I encounter any obstacle or problem with their operation I could just google about it and will probably find a solution or two. Their specs are a little bit better than their other model *MG 995.*

### Specifications

- Weight: 55 g
- Dimension: 40.7 x 19.7 x 42.9 mm
- Stall torque: 9.4 kgf·cm (4.8 V ), 11 kgf·cm (6 V)
- Operating speed: 0.17 s/60º (4.8 V), 0.14 s/60º (6 V)
- Operating voltage: 4.8 V to 7.2 V
- Running Current 500 mA to 900 mA (6V)
- Stall Current 2.5 A (6V)
- Dead band width: 5 µs

## Assembly Decisions

The fixations provide on the servo body are not ideal for joints so I will use a multi-purpose mount plate that also is readily available.

![Servomotor with Metal Gears](/assets/img/servo_bracket.png)

As for the horn, it's usually a mess to use the original plastic ones. Not only it's a very soft plastic but the provided holes are much too small for mechanical coupling and are not threaded. With a metal horn with M3 screws, the assembly would be easier and more reliable.

![Metal Servo Horn](/assets/img/servo_horn.png)