import rospy
import smach
from std_msgs.msg import String
import random


class Joke(smach.State):
    def __init__(self):
        # Invoke parent class constructor
        # outcomes is the list of possible state outcomes
        # input_keys is the list of state machine variables this state reads but does not write
        # output_keys is the list of state machine variables this state writes but does not read
        # io_keys is the list of state machine variables this state both reads and writes
        super().__init__(outcomes=["completed", "preempted"])

        self._speech_publisher = rospy.Publisher("/speak", String, queue_size=1)

        self._joke_list = [
            ["What do you call a robot who always runs into the wall?", "Wall-E"],
            ["How do doggy robots do?", "They byte!"],
            ["What happens to robots after they go defunct?", "They rust in peace!"],
        ]

    def execute(self, userdata: smach.UserData):
        # Userdata variables are accessed with the . operator
        # You can only access the variables you specified in input_keys, output_keys and io_keys
        # Variables specified in input_keys are wrapped to be immutable

        if self.preempt_requested():

            return "preempted"

        joke = random.choice(self._joke_list)
        self._speech_publisher.publish(joke[0])  # question
        rospy.sleep(2)
        self._speech_publisher.publish(joke[1])  # punch

        rospy.sleep(10)
        return "completed"
