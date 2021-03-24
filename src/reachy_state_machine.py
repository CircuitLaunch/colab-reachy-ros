#!/usr/bin/env python

import rospy
import smach

from state_machine.idle_state import Idle
from state_machine.greet_state import Greet


if __name__ == "__main__":
    rospy.init_node("reachy_state_machine")

    # Create state machine
    sm = smach.StateMachine(outcomes=["exit"])  # smach requires a StateMachine to have a list of outcomes

    with sm:
        # Add states and map state outcomes to subsequent states

        # remapping maps the userdata variable names used by states to the userdata variable names
        # used by the state machine.
        # Not necessary if the names are the same, but it's good to be explicit about this

        smach.StateMachine.add(
            "IDLE",
            Idle(),
            transitions={"person_detected": "GREET"},
            remapping={
                "conversation_started": "conversation_started",
                "num_faces": "num_faces",
                "num_masks": "num_masks",
            },
        )

        smach.StateMachine.add(
            "GREET",
            Greet(),
            transitions={"all_masks": NotImplementedError, "missing_mask": NotImplementedError, "nobody_here": "IDLE"},
            remapping={
                "conversation_started": "conversation_started",
                "num_faces": "num_faces",
                "num_masks": "num_masks",
            },
        )

    # Run state machine
    outcome = sm.execute()
