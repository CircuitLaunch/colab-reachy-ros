#!/usr/bin/env python3

# Adapted from https://github.com/poppy-project/poppy_controllers/blob/master/src/poppy_ros_control/joint_trajectory_action.py

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
# JTAS adapted by Yoan Mollard for e.DO robot, meeting the license hereunder
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import errno
import actionlib
import bisect
from copy import deepcopy
import math
import operator
import numpy as np
import reachy
import reachy_arm_control.bezier as bezier
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
)
from colab_reachy_ros.msg import JointTemperatures
from rospy import ROSException
from collections import OrderedDict

DEG_TO_RAD = 0.0174527


def patch_force_gripper(forceGripper):
    def __init__(self, root, io):
        """Create a new Force Gripper Hand, without the load sensor."""
        reachy.parts.hand.Hand.__init__(self, root=root, io=io)

        dxl_motors = OrderedDict({name: dict(conf) for name, conf in self.dxl_motors.items()})

        self.attach_dxl_motors(dxl_motors)

        """
        self._load_sensor = self.io.find_module('force_gripper')
        self._load_sensor.offset = 4
        self._load_sensor.scale = 10000
        """

    forceGripper.__init__ = __init__

    return forceGripper


reachy.parts.arm.RightForceGripper = patch_force_gripper(reachy.parts.arm.RightForceGripper)
reachy.parts.arm.LeftForceGripper = patch_force_gripper(reachy.parts.arm.LeftForceGripper)


class ReachyArmNode:
    _arm_feedback = FollowJointTrajectoryFeedback()
    _arm_result = FollowJointTrajectoryResult()
    _joint_state = JointState()
    _joint_temperatures = JointTemperatures()

    def __init__(self, name):
        # Change this to be able to set right or left arm, and provide io parameter as an argument
        self._side = rospy.get_param("~side")
        self._io = rospy.get_param("~io")
        self._name = name
        self._update_rate = rospy.get_param("~rate")

        if self._side == "right":
            self._reachy = reachy.Reachy(right_arm=reachy.parts.RightArm(io=self._io, hand="force_gripper"))
            dxl_list = self._reachy.right_arm.motors
            side_letter = "r"
        elif self._side == "left":
            self._reachy = reachy.Reachy(left_arm=reachy.parts.LeftArm(io=self._io, hand="force_gripper"))
            dxl_list = self._reachy.left_arm.motors
            side_letter = "l"
        else:
            raise ValueError("'side' private parameter must be 'left' or 'right")

        # Motors are of the Reachy motor wrapper type:
        # https://github.com/pollen-robotics/reachy/blob/master/software/reachy/parts/motor.py
        self._ros_motor_names = [
            f"{side_letter}_shoulder_pitch",
            f"{side_letter}_shoulder_roll",
            f"{side_letter}_arm_yaw",
            f"{side_letter}_elbow_pitch",
            f"{side_letter}_forearm_yaw",
            f"{side_letter}_wrist_pitch",
            f"{side_letter}_wrist_roll",
            f"{side_letter}_gripper"
        ]
        self._dxl_motors = dict(zip(self._ros_motor_names, dxl_list))

        self._joint_state_publisher = rospy.Publisher(f"{self._name}/joint_states", JointState, queue_size=10)
        self._joint_temp_publisher = rospy.Publisher(f"{self._name}/joint_temperatures", JointTemperatures, queue_size=10)
        self._update_spinner = rospy.Rate(self._update_rate)

        self._arm_action_server = actionlib.SimpleActionServer(
            f"{self._name}/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self.execute_arm_cb,
            auto_start=False,
        )
        self._arm_action_server.start()

        self._joint_state.name = self._ros_motor_names
        self._joint_temperatures.name = self._ros_motor_names
    
    def spin(self):
        while not rospy.is_shutdown():
            self._joint_state.header.stamp = rospy.Time.now()
            self._joint_state.position = [self._dxl_motors[m].present_position*DEG_TO_RAD for m in self._ros_motor_names]
            try:
                self._joint_state_publisher.publish(self._joint_state)
            except ROSException:
                pass

            self._joint_temperatures.header.stamp = rospy.Time.now()
            self._joint_temperatures.temperature = [self._dxl_motors[m].temperature for m in self._ros_motor_names]
            try:
                self._joint_temp_publisher.publish(self._joint_temperatures)
            except ROSException:
                pass

            self._update_spinner.sleep()

    def shutdown_hook(self):
        for m in self._dxl_motors.values():
            m.compliant = True

    def execute_arm_cb(self, goal: FollowJointTrajectoryGoal):
        pass


if __name__ == "__main__":
    rospy.init_node("reachy_arm", log_level=rospy.DEBUG)
    arm = ReachyArmNode(rospy.get_name())
    rospy.on_shutdown(arm.shutdown_hook)
    arm.spin()
