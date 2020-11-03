# OMRON ROS Robot driver

[![QUT Centre for Robotics Open Source](https://github.com/qcr/qcr.github.io/raw/master/misc/badge.svg)](https://qcr.github.io)

The OMRON LD-60 is a capable platform out of the box but has no ROS support. Fortunatelyt he LD-60 s still really a Pioneer at heart and there is significant resources in the public domain which can interface to the platform. 

This does not replace Mobile Planner. Mobile Planner is still used for map creation and robot configuration. *Note: Mobile planner will run inside Wine on Ubuntu 18.04

This driver currently assumes you have a user (which can be set via Mobile Planner) with no password. 

<img src="./docs/omron_robot.jpg" alt="LD-60 Robot" style="zoom:33%;" />

## Required Parameters

 **Host IP**: String     e.g. 172.168.1.1

 **Host Port**: String     e.g. 7272

 **User**: String  e.g. omron



## Topics 

### Published

* /amcl_pose
* /current_goal
* /laser
* /laser_low
* /bumper
* /status

### Actions

* /move_base

### Subscribed

* /cmd_vel
* /simple_goal
* /set_map



## Getting Started

*Coming Soon*
