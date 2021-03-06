#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from motion_control.moveit_helpers import load_joint_configurations_from_file, save_joint_configurations_to_file


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("moveit_pose_recorder", anonymous=True)

group_name = rospy.get_param("~move_group")
pose_name = rospy.get_param("~pose_name")
overwrite = rospy.get_param("~overwrite", default=False)

move_group = moveit_commander.MoveGroupCommander(group_name)
try:
    load_joint_configurations_from_file(move_group)
except FileNotFoundError:  # No file yet - create one
    pass
except KeyError:  # No data for this move group yet
    pass

# get_named_targets() gets hardcoded targets from the config (i.e. left_rest) but not remembered ones
if overwrite or pose_name not in move_group.get_remembered_joint_values().keys():
    move_group.remember_joint_values(pose_name)
    save_joint_configurations_to_file(move_group)
else:
    rospy.logerr("Attempted to modify an existing joing configuration without setting flag _overwrite")

moveit_commander.roscpp_shutdown()
