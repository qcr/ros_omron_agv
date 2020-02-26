#!/usr/bin/env python
from __future__ import print_function

import rospy
import actionlib
import numpy as np
import cv_bridge
import time
import rospkg
import atexit
import math
import tf2_ros

from sensor_msgs.msg import CompressedImage,Image, CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import TransformStamped, PoseStamped
from rv_omron_driver.msg import Omron

import py_trees as pt

import rv_trees.data_management as data_management

from py_trees.composites import Sequence, Selector
from py_trees.decorators import FailureIsRunning, SuccessIsRunning, Timeout, FailureIsSuccess, OneShot, SuccessIsFailure
from rv_trees.leaves import Leaf
from rv_trees.leaves_ros import ActionLeaf, PublisherLeaf, ServiceLeaf, SubscriberLeaf
from rv_trees.trees import BehaviourTree
from rv_tasks.leaves.console import Print

class MoveToPose(ActionLeaf):
  def __init__(self, action_namespace="/move_base", *args, **kargs):
    super(MoveToPose, self).__init__("Drive to pose", action_namespace=action_namespace , *args, **kargs)

class Dock(ServiceLeaf):
  def __init__(self,  *args, **kargs):
    super(Dock, self).__init__("Dock Robot", service_name="/dock", *args, **kargs)  

class GetRobotStatus(SubscriberLeaf):
  def __init__(self, topic_name="/robot_status", topic_class=Omron, *args, **kargs):
      super(GetRobotStatus, self).__init__(
      name="GetStatus",
      save=True,
      topic_name=topic_name,
      topic_class=topic_class,
      *args,
      **kargs
    )

class IsVoltageLow(Leaf):
  def __init__(self, topic_name="/robot_status", low_voltage=90, topic_class=Omron, *args, **kargs):
    super(IsVoltageLow, self).__init__(name="IsVoltageLow", save=True, result_fn=self.result_fn,*args, **kargs) 
    self.low_voltage = low_voltage,

  def result_fn(self):
    if self.loaded_data.batteryPercentage > self.low_voltage[0]:
      return False
    else:
      return True

#Engineers
pose1 = PoseStamped() 
pose1.pose.position.x = -0.9
pose1.pose.position.y = -4.4

#Manip Lab
pose2 = PoseStamped() 
pose2.pose.position.x = -9.7
pose2.pose.position.y = -27.43

#Peter's Office
pose3 = PoseStamped() 
pose3.pose.position.x = 1.56
pose3.pose.position.y = -0.5

#Manipulation lab
pose4 = PoseStamped() 
pose4.pose.position.x = -26.647
pose4.pose.position.y = -36.601


def main():
  BehaviourTree(
    "Test Drive",
    Selector("Charge if low Voltage", children = [
      Sequence("Charge", children= [
        GetRobotStatus(),
        IsVoltageLow(low_voltage=97),
          SuccessIsRunning(
            Sequence("Check Voltage", children = [
              Dock(),
              GetRobotStatus(),
              IsVoltageLow(low_voltage=98)
          ]) )
      ]), 
      Sequence("Move", children = [    
        Print(load_value="Starting the Omron Test behaviour"),
        Print(load_value="Goto Goal 1"),
        FailureIsSuccess(MoveToPose(load_value=pose1)),
        Print(load_value="Goto Goal 2"),
        FailureIsSuccess(MoveToPose(load_value=pose2)),
        Print(load_value="Goto Goal 3"),

        FailureIsSuccess(MoveToPose(load_value=pose3)),
        Print(load_value="Goto Goal 4"),
        FailureIsSuccess(MoveToPose(load_value=pose4)),
      ])
    ])
    
).run(hz=30, push_to_start=True, log_level='INFO')

if __name__ == '__main__':
  rospy.init_node("test_omron_tree")
  main()
