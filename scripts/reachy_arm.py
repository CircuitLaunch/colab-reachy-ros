#!/usr/bin/env python3

import rospy
import actionlib
import reachy
from std_msgs.msg import String
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from collections import OrderedDict


def patch_force_gripper(forceGripper):
    def __init__(self, root, io):
        """Create a new Force Gripper Hand."""
        reachy.parts.hand.Hand.__init__(self, root=root, io=io)

        dxl_motors = OrderedDict(
            {name: dict(conf) for name, conf in self.dxl_motors.items()}
        )

        self.attach_dxl_motors(dxl_motors)

        """
        self._load_sensor = self.io.find_module('force_gripper')
        self._load_sensor.offset = 4
        self._load_sensor.scale = 10000
        """

    forceGripper.__init__ = __init__

    return forceGripper

reachy.parts.arm.RightForceGripper = patch_force_gripper(reachy.parts.arm.RightForceGripper)

class ReachyArmNode:
    _feedback = 
    _result = 

    def __init__(self):
        # Change this to be able to set right or left arm, and provide io parameter as an argument
        self.reachy = reachy.Reachy(right_arm=reachy.parts.RightArm(io='ws', hand='force_gripper'))

    def execute_arm_cb(self, goal: FollowJointTrajectoryGoal):


if __name__ == "__main__":
    rospy.init_node("reachy_arm")