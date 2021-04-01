import rospy
import smach
from std_msgs.msg import String
from colab_reachy_ros.msg import FaceAndMaskDetections


class Goodbye(smach.State):
    def __init__(self):
        # Invoke parent class constructor
        # outcomes is the list of possible state outcomes
        # input_keys is the list of state machine variables this state reads but does not write
        # output_keys is the list of state machine variables this state writes but does not read
        # io_keys is the list of state machine variables this state both reads and writes
        super().__init__(outcomes=["nobody_here", "preempted"])

        self._sees_a_face = True

        self._face_mask_subscriber = rospy.Subscriber(
            "/mask_detector/faces_detected", FaceAndMaskDetections, self._face_mask_callback, queue_size=10
        )

        self._speech_publisher = rospy.Publisher("/speak", String, queue_size=1)

    def _face_mask_callback(self, data: FaceAndMaskDetections):
        if not data.faces:
            self._sees_a_face = False

    def execute(self, userdata: smach.UserData):
        # Userdata variables are accessed with the . operator
        # You can only access the variables you specified in input_keys, output_keys and io_keys
        # Variables specified in input_keys are wrapped to be immutable
        self._sees_a_face = True

        self._speech_publisher.publish("Goodbye")

        while self._sees_a_face:
            # Every state must check if a preempt has been requested (e.g. in case of Ctrl+C)
            # Smach does NOT handle this itself
            if self.preempt_requested():
                return "preempted"
            rospy.sleep(0.5)

        return "nobody_here"
