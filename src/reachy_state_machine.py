#!/usr/bin/env python

import rospy
import smach
import threading
import moveit_commander
import sys

from state_machine.idle_state import Idle
from state_machine.greet_state import Greet
from state_machine.has_mask_state import HasMask
from state_machine.no_mask_state import NoMask
from state_machine.goodbye_state import Goodbye
from state_machine.direct_to_kitchen_state import DirectToKitchen
from state_machine.joke_state import Joke


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("reachy_state_machine")

    # Create state machine
    sm = smach.StateMachine(outcomes=["exit"])  # A state machine will only exit if it transitions to an outcome

    with sm:
        # Add states and map state outcomes to subsequent states/state machine outcomes

        # remapping maps the userdata variable names used by states to the userdata variable names
        # used by the state machine.
        # Not necessary if the names are the same, but it's good to be explicit about this

        smach.StateMachine.add(
            "IDLE",
            Idle(),
            transitions={"person_detected": "GREET", "preempted": "exit"},
            remapping={"conversation_started": "conversation_started"},
        )

        smach.StateMachine.add(
            "GREET",
            Greet(),
            transitions={
                "all_masks": "HAS_MASK",
                "missing_mask": "NO_MASK",
                "nobody_here": "IDLE",
                "preempted": "exit",
            },
        )

        smach.StateMachine.add(
            "NO_MASK",
            NoMask(),
            transitions={
                "everybody_masked": "HAS_MASK",
                "nobody_here": "IDLE",
                "preempted": "exit",
            },
        )

        smach.StateMachine.add(
            "HAS_MASK",
            HasMask(),
            transitions={
                "one_not_masked": "NO_MASK",
                "nobody_here": "IDLE",
                "goodbye": "GOODBYE",
                "joke": "JOKE",
                "kitchen": "KITCHEN",
                "preempted": "exit",
            },
            remapping={"conversation_started": "conversation_started"},
        )

        smach.StateMachine.add(
            "GOODBYE",
            Goodbye(),
            transitions={"nobody_here": "IDLE", "preempted": "exit"},
        )

        smach.StateMachine.add("JOKE", Joke(), transitions={"completed": "HAS_MASK", "preempted": "exit"})

        smach.StateMachine.add("KITCHEN", DirectToKitchen(), transitions={"completed": "HAS_MASK", "preempted": "exit"})

    # Create a thread to execute the smach container
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    # Wait for ctrl-c
    rospy.spin()

    # Request the container to preempt
    sm.request_preempt()

    # Block until everything is preempted
    # (you could do something more complicated to get the execution outcome if you want it)
    smach_thread.join()
    moveit_commander.roscpp_shutdown()
