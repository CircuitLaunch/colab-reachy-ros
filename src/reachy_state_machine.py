#!/usr/bin/env python

import rospy
import smach

from state_machine.idle_state import Idle


if __name__ == "__main__":
    rospy.init_node("reachy_state_machine")