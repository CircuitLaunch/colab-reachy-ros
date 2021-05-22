#!/usr/bin/env python3

# Adapted from:
# https://github.com/poppy-project/poppy_controllers/blob/master/src/poppy_ros_control/joint_trajectory_action.py

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

################################################################################
# UPDATE LOG
#
# Edward Janne (edj 2021-05-08)
# This file was copied from reachy_arm_controller.py and modified to unify the
# left and right arm control functionality into one node. The reason for doing
# this is that the Pollen Robotics provided reachy module is not thread-safe
# and therefore performs unpredictably when accessed from two separate nodes
# which are necessarily operating on two independent threads. Unifying permits
# us to implment synchronization between asynchronous callbacks from ROS.
#
################################################################################

import rospy
import actionlib
import bisect
from copy import deepcopy
import operator
import numpy as np
# import reachy
from DXL import *
import motion_control.bezier as bezier
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
)
from std_srvs.srv import SetBool, SetBoolResponse
from colab_reachy_ros.msg import JointTemperatures
from rospy import ROSException
from collections import OrderedDict
from typing import List
# Required to synchronize access to reachy from multiple asynchronous callbacks
# (edj 2021-05-08)
from threading import Lock

DEG_TO_RAD = 0.0174527

'''
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
'''

# Updated class name to reflect change in paradigm
# class ReachyArmNode:
class ReachyArmController:

    '''
    # Introducing single reachy reference as a class variable to be shared by
    # all instances, and synchronized by a mutex (edj 2021-05-08)
    #_unified_reachy = None
    #_unified_reachy_mutex = Lock()

    @property
    def _reachy_mutex(self):
        return self.__class__._unified_reachy_mutex
    
    @property
    def _reachy(self):
        with self._reachy_mutex:
            if self.__class__._unified_reachy == None:
                self.__class__._unified_reachy = reachy.Reachy(
                    right_arm=reachy.parts.RightArm(io=rospy.get_param(f"~right_io"), hand="force_gripper"),
                    left_arm=reachy.parts.LeftArm(io=rospy.get_param(f"~left_io"), hand="force_gripper")
                )
        return self.__class__._unified_reachy
    '''

    # Why were these declared class variables? (edj 2021-05-08)
    """
    _arm_feedback = FollowJointTrajectoryFeedback()
    _arm_result = FollowJointTrajectoryResult()
    _joint_state = JointState()
    _joint_temperatures = JointTemperatures()
    """

    def __init__(self, name, side):
        self.u2d2 = DXLPort(rospy.get_param(f"~u2d2_port"), rospy.get_param(f"~u2d2_baud"))

        # Made these instance variables (edj 2021-05-08)
        self._arm_feedback = FollowJointTrajectoryFeedback()
        self._arm_result = FollowJointTrajectoryResult()
        self._joint_state = JointState()
        self._joint_temperatures = JointTemperatures()

        # Redundant (edj 2021-05-08)
        # self._side = rospy.get_param("~side")
        # self._io = rospy.get_param("~io")

        self._side = side
        self._name = name
        # Factored out into main() (edj 2021-05-08)
        # self._update_rate = rospy.get_param("~rate")
        self._arm_continuous = rospy.get_param("~arm_continuous_trajectories")

        if self._side == "right":
            # Factored out into a shared class variable (edj 2021-05-08)
            # self._reachy = reachy.Reachy(right_arm=reachy.parts.RightArm(io=self._io, hand="force_gripper"))
            #dxl_list = self._reachy.right_arm.motors
            self.armIds = [10, 11, 12, 13, 14, 15, 16, 17]
            dxl_list = [self.u2d2.getDXL(id) for id in self.armIds]
            side_letter = "r"
        elif self._side == "left":
            # Factored out into a shared class variable (edj 2021-05-08)
            # self._reachy = reachy.Reachy(left_arm=reachy.parts.LeftArm(io=self._io, hand="force_gripper"))
            #dxl_list = self._reachy.left_arm.motors
            self.armIds = [20, 21, 22, 23, 24, 25, 26, 27]
            dxl_list = [self.u2d2.getDXL(id) for id in self.armIds]
            side_letter = "l"
        else:
            raise ValueError("'side' private parameter must be 'left' or 'right")

        '''
        for m in dxl_list:
            # Access synchronization (edj 2021-05-08)
            with self._reachy_mutex:
                m.compliant = True
        '''

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
            f"{side_letter}_gripper",
        ]
        self._dxl_motors = dict(zip(self._ros_motor_names, dxl_list))

        self.setArmCompliance(True)

        # To maintain the correct topic name (edj 2021-05-08)
        self._joint_state_publisher = rospy.Publisher(f"{self._side}_arm_controller/joint_states", JointState, queue_size=10)
        self._joint_temp_publisher = rospy.Publisher(
            # To maintain the correct topic name (edj 2021-05-08)
            f"{self._side}_arm_controller/joint_temperatures", JointTemperatures, queue_size=10
        )
        # Factored out into main() (edj 2021-05-08)
        # self._update_spinner = rospy.Rate(self._update_rate)

        self._arm_compliant = True
        self._gripper_compliant = True

        self._arm_compliance_service = rospy.Service(
            # To maintain the correct topic name (edj 2021-05-08)
            f"{self._side}_arm_controller/set_arm_compliant", SetBool, self._set_arm_compliance
        )
        self._gripper_compliance_service = rospy.Service(
            # To maintain the correct topic name (edj 2021-05-08)
            f"{self._side}_arm_controller/set_gripper_compliant", SetBool, self._set_gripper_compliance
        )

        self._arm_action_server = actionlib.SimpleActionServer(
            # To maintain the correct action name (edj 2021-05-08)
            f"{self._side}_arm_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self._execute_arm_cb,
            auto_start=False,
        )
        self._arm_action_server.start()

        self._joint_state.name = self._ros_motor_names
        self._joint_temperatures.name = self._ros_motor_names

    def setArmCompliance(self, compliant):
        torqueEnable = 0 if compliant else 1
        self.u2d2.syncWrite(RAM_TORQUE_ENABLE, 1, dict(zip(self.armIds, [torqueEnable] * len(self.armIds))))

    # Factored out into main() (edj 2021-05-08)
    # def spin(self):
    #     while not rospy.is_shutdown()

    def report_joint_state(self):
        self._joint_state.header.stamp = rospy.Time.now()
        # Access synchronization (edj 2021-05-08)
        with self._reachy_mutex:
            self._joint_state.position = [
                # self._dxl_motors[m].present_position * DEG_TO_RAD for m in self._ros_motor_names
                math.radians(self._dxl_motors[m].presentPosition) for m in self._ros_motor_names
            ]
        try:
            self._joint_state_publisher.publish(self._joint_state)
        except ROSException:
            pass

        self._joint_temperatures.header.stamp = rospy.Time.now()
        # Access synchronization (edj 2021-05-08)
        with self._reachy_mutex:
            # self._joint_temperatures.temperature = [self._dxl_motors[m].temperature for m in self._ros_motor_names]
            self._joint_temperatures.temperature = [self._dxl_motors[m].presentTemperature for m in self._ros_motor_names]
        try:
            self._joint_temp_publisher.publish(self._joint_temperatures)
        except ROSException:
            pass

        # Factored out into main() (edj 2021-05-08)
        # self._update_spinner.sleep()

    def shutdown_hook(self):
        '''
        # Access synchronization (edj 2021-05-08)
        with self._reachy_mutex:
            for m in self._dxl_motors.values():
                m.compliant = True
        '''
        self.setArmCompliance(True)
        rospy.sleep(0.2)

    def _set_arm_compliance(self, request: SetBool):
        self._arm_compliant = request.data
        self.setArmCompliance(self._arm_compliant)
        '''
        for m in self._ros_motor_names[0:7]:  # End index is not included
            # Access synchronization (edj 2021-05-08)
            with self._reachy_mutex:
                self._dxl_motors[m].compliant = request.data
        '''
        # Indicate which side is reporting (edj 2021-05-08)
        msg = f"{self._side} arm compliance has been {'enabled' if request.data else 'disabled'}"
        return SetBoolResponse(success=True, message=msg)

    def _set_gripper_compliance(self, request: SetBool):
        self._gripper_compliant = request.data
        self._dxl_motors[f'{self.side_letter}_gripper'].torqueEnable = 0 if self._gripper_compliant else 1
        '''
        # Access synchronization (edj 2021-05-08)
        with self._reachy_mutex:
            self._dxl_motors[self._ros_motor_names[7]].compliant = request.data
        # Indicate which side is reporting (edj 2021-05-08)
        msg = f"{self._side} gripper compliance has been {'enabled' if request.data else 'disabled'}"
        '''
        return SetBoolResponse(success=True, message=msg)

    def _get_current_positions(self, joint_names: List[str]):
        # Access synchronization (edj 2021-05-08)
        '''
        with self._reachy_mutex:
            present_position = [self._dxl_motors[joint].present_position / DEG_TO_RAD for joint in joint_names]
        '''
        present_position = [math.radians(self._dxl_motors[name].presentPosition) for name in joint_names]
        return present_position

    def _update_feedback(self, cmd_point: JointTrajectoryPoint, joint_names: List[str], cur_time: float):
        self._arm_feedback.header.stamp = rospy.Duration.from_sec(rospy.get_time())
        self._arm_feedback.joint_names = joint_names
        self._arm_feedback.desired = cmd_point
        self._arm_feedback.desired.time_from_start = rospy.Duration.from_sec(cur_time)
        self._arm_feedback.actual.positions = self._get_current_positions(joint_names)
        self._arm_feedback.actual.time_from_start = rospy.Duration.from_sec(cur_time)
        self._arm_feedback.error.positions = list(map(
            operator.sub, self._arm_feedback.desired.positions, self._arm_feedback.actual.positions
        ))
        self._arm_feedback.error.time_from_start = rospy.Duration.from_sec(cur_time)
        self._arm_action_server.publish_feedback(self._arm_feedback)
        # Indicate which side is reporting (edj 2021-05-08)
        rospy.logdebug(f"{self._side}_arm_controller: Present positions: {self._get_current_positions(joint_names)}")

    def _command_joints(self, joint_names: List[str], point: JointTrajectoryPoint):
        # Indicate which side is reporting (edj 2021-05-08)
        rospy.logdebug(f"{self._side}_arm_controller: Setting motors to {point.positions}")
        if len(joint_names) != len(point.positions):
            # Indicate which side is reporting (edj 2021-05-08)
            rospy.logerr(f"{self._side}_arm_controller: Point is invalid: len(joint_names) != len(point.positions)")
            return False
        for i, m in enumerate(joint_names):
            if m not in self._dxl_motors.keys():
                # Indicate which side is reporting (edj 2021-05-08)
                rospy.logerr(f"{self._side}_arm_controller: Point is invalid: joint {m} not found")
                return False
            if self._arm_compliant:
                # Indicate which side is reporting (edj 2021-05-08)
                rospy.logerr(f"{self._side}_arm_controller: Arm is compliant, the trajectory cannot execute")
                return False
            '''
            # Access synchronization (edj 2021-05-08)
            with self._reachy_mutex:
                self._dxl_motors[m].goal_position = float(point.positions[i]) / DEG_TO_RAD
            '''
        dxlIds = [self._dxl_motors[name].id for name in joint_names]
        degrees = [math.degrees(pos) for pos in point.positions]
        self.u2d2.syncWrite(RAM_GOAL_POSITION, 2, dict(zip(dxlIds, degrees)))
        return True

    def _determine_trajectory_dimensions(self, trajectory_points: List[JointTrajectoryPoint]):
        position_flag = True
        velocity_flag = len(trajectory_points[0].velocities) != 0 and len(trajectory_points[-1].velocities) != 0
        acceleration_flag = (
            len(trajectory_points[0].accelerations) != 0 and len(trajectory_points[-1].accelerations) != 0
        )
        return {"positions": position_flag, "velocities": velocity_flag, "accelerations": acceleration_flag}

    def _compute_bezier_coeff(
        self, joint_names: List[str], trajectory_points: List[JointTrajectoryPoint], dimensions_dict: dict
    ):
        # Compute Full Bezier Curve
        num_joints = len(joint_names)
        num_traj_pts = len(trajectory_points)
        num_traj_dim = sum(dimensions_dict.values())
        num_b_values = len(["b0", "b1", "b2", "b3"])
        b_matrix = np.zeros(shape=(num_joints, num_traj_dim, num_traj_pts - 1, num_b_values))
        for jnt in range(num_joints):
            traj_array = np.zeros(shape=(len(trajectory_points), num_traj_dim))
            for idx, point in enumerate(trajectory_points):
                current_point = list()
                current_point.append(point.positions[jnt])
                if dimensions_dict["velocities"]:
                    current_point.append(point.velocities[jnt])
                if dimensions_dict["accelerations"]:
                    current_point.append(point.accelerations[jnt])
                traj_array[idx, :] = current_point
            d_pts = bezier.de_boor_control_pts(traj_array)
            b_matrix[jnt, :, :, :] = bezier.bezier_coefficients(traj_array, d_pts)
        return b_matrix

    def _get_bezier_point(self, b_matrix, idx, t, cmd_time, dimensions_dict: dict):
        pnt = JointTrajectoryPoint()
        pnt.time_from_start = rospy.Duration(cmd_time)
        num_joints = b_matrix.shape[0]
        pnt.positions = [0.0] * num_joints
        if dimensions_dict["velocities"]:
            pnt.velocities = [0.0] * num_joints
        if dimensions_dict["accelerations"]:
            pnt.accelerations = [0.0] * num_joints
        for jnt in range(num_joints):
            b_point = bezier.bezier_point(b_matrix[jnt, :, :, :], idx, t)
            # Positions at specified time
            pnt.positions[jnt] = b_point[0]
            # Velocities at specified time
            if dimensions_dict["velocities"]:
                pnt.velocities[jnt] = b_point[1]
            # Accelerations at specified time
            if dimensions_dict["accelerations"]:
                pnt.accelerations[jnt] = b_point[-1]
        return pnt

    def _execute_arm_cb(self, goal: FollowJointTrajectoryGoal):
        joint_names = goal.trajectory.joint_names
        trajectory_points = goal.trajectory.points
        dimensions_dict = self._determine_trajectory_dimensions(trajectory_points)

        for jnt in joint_names:
            if jnt not in self._dxl_motors.keys():
                # Indicate which side is reporting (edj 2021-05-08)
                rospy.logerr(f"{self._side}_arm_controller: Trajectory Aborted - Provided Invalid Joint Name {jnt}")
                self._arm_result.error_code = self._arm_result.INVALID_JOINTS
                self._arm_action_server.set_aborted(self._arm_result)
                return

        # Goal time tolerance - time buffer allowing goal constraints to be met
        if goal.goal_time_tolerance:
            goal_time_tolerance = goal.goal_time_tolerance.to_sec()
        else:
            goal_time_tolerance = rospy.get_param("~arm_goal_delay_tolerance")

        num_points = len(trajectory_points)
        if num_points == 0:
            # Indicate which side is reporting (edj 2021-05-08)
            rospy.logerr(f"{self._side}_arm_controller: Empty Trajectory")
            self._arm_action_server.set_aborted()
            return
        rospy.logwarn(
            # Indicate which side is reporting (edj 2021-05-08)
            f"{self._side}_arm_controller: Executing requested joint trajectory {trajectory_points[-1].time_from_start.to_sec()} sec"
        )
        # Factored out
        control_rate = rospy.Rate(rospy.get_param("~rate"))

        if num_points == 1:
            first_trajectory_point = JointTrajectoryPoint()
            first_trajectory_point.positions = self._get_current_position(joint_names)
            # To preserve desired velocities and accelerations, copy them to the first
            # trajectory point if the trajectory is only 1 point.
            if dimensions_dict["velocities"]:
                first_trajectory_point.velocities = deepcopy(trajectory_points[0].velocities)
            if dimensions_dict["accelerations"]:
                first_trajectory_point.accelerations = deepcopy(trajectory_points[0].accelerations)
            first_trajectory_point.velocities = deepcopy(trajectory_points[0].velocities)
            first_trajectory_point.accelerations = deepcopy(trajectory_points[0].accelerations)
            first_trajectory_point.time_from_start = rospy.Duration(0)
            trajectory_points.insert(0, first_trajectory_point)
            num_points = len(trajectory_points)

        if not self._arm_continuous:
            if dimensions_dict["velocities"]:
                trajectory_points[-1].velocities = [0.0] * len(joint_names)
            if dimensions_dict["accelerations"]:
                trajectory_points[-1].accelerations = [0.0] * len(joint_names)

        # Compute Full Bezier Curve Coefficients for all 7 joints
        pnt_times = [pnt.time_from_start.to_sec() for pnt in trajectory_points]
        try:
            b_matrix = self._compute_bezier_coeff(joint_names, trajectory_points, dimensions_dict)
        except Exception as ex:
            rospy.logerr(f"Failed to compute a Bezier trajectory: {repr(ex)}")
            self._arm_action_server.set_aborted()
            return

        start_time = goal.trajectory.header.stamp.to_sec()
        if start_time == 0.0:
            start_time = rospy.get_time()

        end_time = trajectory_points[-1].time_from_start.to_sec()

        now = rospy.get_time()
        now_from_start = now - start_time

        # Loop stages:
        # 1. Wait for start time if one is provided
        # 2. Set motors to time-derived point until end time
        # 3. Keep trying to reach goal until goal tolerance time expires
        while not rospy.is_shutdown() and now_from_start < end_time + goal_time_tolerance:
            # Indicate which side is reporting (edj 2021-05-08)
            if self._arm_action_server.is_preempt_requested():
                rospy.loginfo(f"{self._side}_arm_controller: Arm movement preempted")
                self._arm_action_server.set_preempted()
                return

            now = rospy.get_time()
            now_from_start = now - start_time

            if now < start_time:
                # If before movement time window, wait
                pass
            else:
                if now_from_start < end_time:
                    # If in movement time window, adjust motor positions

                    # Get the trajectory points the current time is between
                    idx = bisect.bisect(pnt_times, now_from_start)
                    # Calculate percentage of time passed in this interval
                    if idx >= num_points:
                        cmd_time = now_from_start - pnt_times[-1]
                        t = 1.0
                    elif idx >= 0:
                        cmd_time = now_from_start - pnt_times[idx - 1]
                        t = cmd_time / max(0.001, pnt_times[idx] - pnt_times[idx - 1])
                    else:
                        cmd_time = 0
                        t = 0

                    point = self._get_bezier_point(b_matrix, idx, t, cmd_time, dimensions_dict)
                else:
                    # If past movement time window, keep trying to meet goal until the goal time tolerance expires
                    point = trajectory_points[-1]

                if not self._command_joints(joint_names, point):
                    self._arm_action_server.set_aborted()
                    # Indicate which side is reporting (edj 2021-05-08)
                    rospy.logwarn(f"{self._side}_arm_controller: Arm movement aborted")
                    return

                self._update_feedback(deepcopy(point), joint_names, now_from_start)

            control_rate.sleep()

        # Indicate which side is reporting (edj 2021-05-08)
        rospy.loginfo(f"{self._side}_arm_controller: Arm movement succeeded")
        self._arm_result.error_code = self._arm_result.SUCCESSFUL
        self._arm_action_server.set_succeeded(self._arm_result)

# Since we are now instantiating two ReachyArmControllers in one node
# rather than running two nodes each with a single ReachyArmNode object
# the spinning behavior had to be factored out of the ReachyArmController.
def main():
    rospy.init_node("reachy_arm", log_level=rospy.DEBUG)
    update_spinner = rospy.Rate(rospy.get_param("~rate"))
    right_arm = ReachyArmController(rospy.get_name(), 'right')
    left_arm = ReachyArmController(rospy.get_name(), 'left')

    def shutdown_hook():
        right_arm.shutdown_hook()
        left_arm.shutdown_hook()

    rospy.on_shutdown(shutdown_hook)

    while not rospy.is_shutdown():
        right_arm.report_joint_state()
        left_arm.report_joint_state()
        update_spinner.sleep()

if __name__ == "__main__":
    main()
