import rospy
import smach
from colab_reachy_ros.msg import FaceAndMaskDetections
from motion_control.head_node_helpers import create_head_animation
from trajectory_msgs.msg import JointTrajectory

class Idle(smach.State):
    def __init__(self):
        # Invoke parent class constructor
        # outcomes is the list of possible state outcomes
        # input_keys is the list of state machine variables this state reads but does not write
        # output_keys is the list of state machine variables this state writes but does not read
        # io_keys is the list of state machine variables this state both reads and writes
        super().__init__(outcomes=["person_detected", "preempted"], output_keys=["conversation_started"])

        self._detected_faces = None
        self._detected_masks = None

        self._face_mask_subscriber = rospy.Subscriber(
            "/mask_detector/faces_detected", FaceAndMaskDetections, self._face_mask_callback, queue_size=10
        )

        self._head_start_gesture = create_head_animation(
                    [[94.0, 80.0]]
                )
        

        # this publisher moves the reachy's head around
        self._head_publisher = rospy.Publisher("/head/position_animator_debug_degrees", JointTrajectory, queue_size=1)



    def _face_mask_callback(self, data: FaceAndMaskDetections):
        # This callback will run regardless of which state is currently executing,
        # so the instance variables will always be up-to-date
        self._detected_faces = data.faces
        self._detected_masks = data.masks

    def execute(self, userdata: smach.UserData):
        # Userdata variables are accessed with the . operator
        # You can only access the variables you specified in input_keys, output_keys and io_keys
        # Variables specified in input_keys are wrapped to be immutable

        userdata.conversation_started = False

        # shake the head yes to indicate that reachy is listening
        try:
            self._head_publisher.publish(self._head_start_gesture)
            rospy.loginfo("Head gesture: 'YES'")
        except Exception as e:
            rospy.logerr(f"Error performing head gesture: {e}", exc_info=True)

        while not self._detected_faces:
            # Every state must check if a preempt has been requested (e.g. in case of Ctrl+C)
            # Smach does NOT handle this itself
            if self.preempt_requested():
                return "preempted"
            rospy.sleep(0.5)

        return "person_detected"
