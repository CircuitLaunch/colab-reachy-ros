import rospy
import smach
from state_machine.helper_functions import say_something
from geometry_msgs.msg import PoseStamped

# head node inclues
from trajectory_msgs.msg import JointTrajectory
from motion_control.head_node_helpers import create_head_animation


class DirectToKitchen(smach.State):
    def __init__(self):
        # Invoke parent class constructor
        # outcomes is the list of possible state outcomes
        # input_keys is the list of state machine variables this state reads but does not write
        # output_keys is the list of state machine variables this state writes but does not read
        # io_keys is the list of state machine variables this state both reads and writes
        super().__init__(outcomes=["completed", "preempted"])

        self._magni_publisher = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
        
        self._head_publisher = rospy.Publisher("/head/position_animator_debug_degrees", JointTrajectory, queue_size=1)

        # head pose
        #   shake the head two the right to indicate that the robot it going 
        #    looking at the kitchen
        self._head_look_right_gesture = create_head_animation(
            [[94,80],[65,80],[65,80],[94,80]]
        )
        # Arm pose
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

        # move the head to the left indicate that reachy is looking toward the kitchen
        try:
            self._head_publisher.publish(self._head_yes_gesture)
            rospy.loginfo("Head gesture: 'YES'")
        except Exception as e:
            rospy.logerr(f"Error performing head gesture: {e}", exc_info=True)

        rospy.sleep(10)
        return "completed"
