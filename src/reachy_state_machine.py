#!/usr/bin/env python

import rospy
import smach

from state_machine.idle_state import Idle
from state_machine.greet_state import Greet


if __name__ == "__main__":
    rospy.init_node("reachy_state_machine")

    sm = smach.StateMachine(outcomes=["exit"])  # smach requires a StateMachine to have a list of outcomes

    with sm:
        smach.StateMachine.add("IDLE", Idle(), transitions={"person_detected": "GREET"})

        smach.StateMachine.add(
            "GREET",
            Greet(),
            transitions={"all_masks": NotImplementedError, "missing_mask": NotImplementedError, "nobody_here": "IDLE"},
        )

    outcome = sm.execute()

