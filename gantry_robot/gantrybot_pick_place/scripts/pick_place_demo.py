#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman
#
## To use the Python MoveIt! interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. (More on these below)
##
## We also import `rospy`_ and some messages that we will use:
##

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

import yaml
import os

from GlobalRobotConstants import NO_OF_GRIPPERS, GRIPPER_NAME_PREFIX, GRIPPER_HOME_IDX, ROBOT_ARM_NAME

## END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupInterface(object):
  """MoveGroupInterface"""
  def __init__(self):
    super(MoveGroupInterface, self).__init__()

    ## Begin Basic Setup
    ## ^^^^^^^^^^^^^^^^^
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node('gantrybot_pick_place_demo',
    #                anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface to
    ## the group of prismatic joints which form the part of the Gantry robot arm.
    arm_group = moveit_commander.MoveGroupCommander(ROBOT_ARM_NAME)

    ## Instantiate a `MoveGroupCommander`_ objects for the group of grippers in the Gantry robot
    gripper_groups = list()
    for i in range(NO_OF_GRIPPERS):
        gripper_groups.append(moveit_commander.MoveGroupCommander(GRIPPER_NAME_PREFIX + str(i+1)))

    ## Create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)


    ## Get Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^
    # Get the name of the reference frame for this robot:
    planning_frame = arm_group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # Get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Print the entire state of the robot. Useful for debugging...
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    # Define Class Variables
    self.robot = robot
    self.scene = scene
    self.arm_group = arm_group
    self.gripper_groups = gripper_groups
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.group_names = group_names

  #####################################################################
  # Function to move the robot to the given position with orientation #
  # of the turret to use the appropriate gripper                      #
  #####################################################################
  def go_to_pose(self, position):
    if len(position) == 3:
        joint_values = { "x_joint": position[0],
                         "y_joint": position[1],
                         "z_joint": position[2]}
    else:
        joint_values = { "x_joint": position[0],
                         "y_joint": position[1],
                         "z_joint": position[2],
                         "turret_joint": position[3]}

    self.arm_group.set_joint_value_target(joint_values)

    # Call the planner to compute the plan and execute it.
    plan = self.arm_group.go(wait=True)

    # Call `stop()` to ensure that there is no residual movement
    self.arm_group.stop()

    # Clear all targets after planning with poses.
    self.arm_group.clear_pose_targets()

  #####################################################################
  # Function to operate the appropriate gripper by opening or closing #
  # its fingers depending on the required state                       #
  ##################################################################### 
  def operate_gripper(self, index, state, roll = 1.4527):
    finger_position = [0, 0]
    if state == "open":
        finger_position[0] = 0.35
        finger_position[1] = 0.35

    joint_values = { GRIPPER_NAME_PREFIX + "_" + str(index+1) + "_roll_joint": roll,
                     "grip" + str(index+1) + "_finger1_joint": finger_position[0],
                     "grip" + str(index+1) + "_finger2_joint": finger_position[1]}

    self.gripper_groups[index].set_joint_value_target(joint_values)

    ## Call the planner to compute the plan and execute it.
    plan = self.gripper_groups[index].go(wait=True)

    # Call `stop()` to ensure that there is no residual movement
    self.gripper_groups[index].stop()

    # Clear all targets after planning with poses.
    self.gripper_groups[index].clear_pose_targets()

  #####################################################################
  # Function to move the robot to its pre-defined home position and   #
  # and orientation of turret                                         #
  #####################################################################
  def go_to_home_pose(self):
    self.go_to_pose([0.00, 0.00, 0.25]) # Move to home position
    self.go_to_pose([0.00, 0.00, 0.25, 0.1562]) # Rotate turret to home position 

  #####################################################################
  # Function to move the robot to a pick position, orient the turret  #
  # and open & close the gripper fingers to pick the object           #
  #####################################################################
  def pick_object(self, position):
    self.go_to_pose([position[0], position[1], 0.25]) # Move to pick position
    self.go_to_pose([position[0], position[1], 0.25, position[2]]) # Rotate turret to required gripper
    self.operate_gripper(1, "open")
    self.go_to_pose([position[0], position[1], -0.29, position[2]])
    self.operate_gripper(1, "close")
    self.go_to_pose([position[0], position[1], 0.25, position[2]])

  #####################################################################
  # Function to move the robot to a place position and open & close   #
  # the gripper fingers to place the object.                          #
  #####################################################################
  def place_object(self, position):
    self.go_to_pose([position[0], position[1], 0.25, position[2]])
    self.go_to_pose([position[0], position[1], -0.29, position[2]])
    self.operate_gripper(1, "open")
    self.operate_gripper(1, "close")
    self.go_to_pose([position[0], position[1], 0.25, position[2]])

def main():
  try:
    # Define the x, y coordinates of a sample pick position and place position
    pick_position = [-0.80, 0.50]
    place_position = [0.80, -0.70]

    print "============ Press `Enter` to begin setting up the moveit_commander (press ctrl-d to exit) ..."
    raw_input()
    move_group_interface = MoveGroupInterface()

    print "============ Press `Enter` to begin Sample pick and place demo ..."
    raw_input()
    print "============ Moving to home position ..."
    move_group_interface.go_to_home_pose()

    print "============ Press `Enter` to plan and execute pick operation ..."
    raw_input()
    print "============ Moving to pick object ..."
    move_group_interface.pick_object([pick_position[0], pick_position[1], 3.2978])
    print "============ Picking of object done ..."

    print "============ Press `Enter` to plan and execute place operation ..."
    raw_input()
    print "============ Moving to place object ..."
    move_group_interface.place_object([place_position[0], place_position[1], 3.2978])
    print "============ Placing of object done ..."

    print "============ Press `Enter` to execute a movement to home position ..."
    raw_input()
    print "============ Returning to home position ..."
    move_group_interface.go_to_home_pose()
    print "============ Return to home position done ..."

    print "============ Sample pick and place demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/kinetic/api/rospy/html/
## CALL_SUB_TUTORIAL imports
## CALL_SUB_TUTORIAL setup
## CALL_SUB_TUTORIAL basic_info
## CALL_SUB_TUTORIAL plan_to_joint_state
## CALL_SUB_TUTORIAL plan_to_pose
## CALL_SUB_TUTORIAL plan_cartesian_path
## CALL_SUB_TUTORIAL display_trajectory
## CALL_SUB_TUTORIAL execute_plan
## CALL_SUB_TUTORIAL add_box
## CALL_SUB_TUTORIAL wait_for_scene_update
## CALL_SUB_TUTORIAL attach_object
## CALL_SUB_TUTORIAL detach_object
## CALL_SUB_TUTORIAL remove_object
## END_TUTORIAL
