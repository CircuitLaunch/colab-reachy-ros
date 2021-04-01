import rospy
import smach
from state_machine.helper_functions import say_something
from geometry_msgs.msg import PoseStamped


class DirectToKitchen(smach.State):
    def __init__(self):
        # Invoke parent class constructor
        # outcomes is the list of possible state outcomes
        # input_keys is the list of state machine variables this state reads but does not write
        # output_keys is the list of state machine variables this state writes but does not read
        # io_keys is the list of state machine variables this state both reads and writes
        super().__init__(outcomes=["completed", "preempted"])

        self._magni_publisher = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)

        x, y, z, qx, qy, qz, qw = (0, -27, 0, 0, 0, 0, 1.0)  # Kitchen pose

        self._msg = PoseStamped()
        self._msg.header.frame_id = "odom"
        self._msg.header.stamp = rospy.Time.now()
        self._msg.pose.position.x = x
        self._msg.pose.position.y = y
        self._msg.pose.position.z = z
        self._msg.pose.orientation.x = qx
        self._msg.pose.orientation.y = qy
        self._msg.pose.orientation.z = qz
        self._msg.pose.orientation.w = qw

    def execute(self, userdata: smach.UserData):
        # Userdata variables are accessed with the . operator
        # You can only access the variables you specified in input_keys, output_keys and io_keys
        # Variables specified in input_keys are wrapped to be immutable

        if self.preempt_requested():
            return "preempted"

        self._magni_publisher.publish(self._msg)  # Set a goal to Magni
        say_something("Please follow me to the Kitchen")

        rospy.sleep(10)
        return "completed"
