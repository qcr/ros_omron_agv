# OMRON ROS Robot driver
===
The OMRON LD-60 is a capable platform out of the box but does not natively support any ROS capabilities. Fortunately being a legacy platform built on Adepbt, built on MobileRobots, built on ActivMedia .... there is significant documentattion in the public domain and the LD-60 at heart is still really a Pioneer. 

This driver currently assumes you have a user (which can be set via Mobile Planner) with no password. 

*Note: Mobile planner will run inside Wine on Ubuntu 18.04


## Parameters

These can be set via ROS Param
 
 host : IP String     e.g. host : 172.168.1.1
 port : Port String     e.g. port : 7272
 user : User String  e.g user : omron


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



