import rospy
import smach
from std_msgs.msg import String
from colab_reachy_ros.msg import FaceAndMaskDetections


class HasMask(smach.State):
    def __init__(self):
        # Invoke parent class constructor
        # outcomes is the list of possible state outcomes
        # input_keys is the list of state machine variables this state reads but does not write
        # output_keys is the list of state machine variables this state writes but does not read
        # io_keys is the list of state machine variables this state both reads and writes
        super().__init__(
            outcomes=["one_not_masked", "nobody_here", "preempted", "kitchen", "joke", "goodbye"],
            io_keys=["conversation_started"],
        )

        self._detected_faces = None
        self._detected_masks = None
        self._everybody_masked = False
        self._keywords = None

        self._face_mask_subscriber = rospy.Subscriber(
            "/mask_detector/faces_detected", FaceAndMaskDetections, self._face_mask_callback, queue_size=10
        )

        self._keyword_subscriber = rospy.Subscriber("/voice_command", String, self._keyword_callback, queue_size=10)

        self._speech_publisher = rospy.Publisher("/speak", String, queue_size=1)

    def _face_mask_callback(self, data: FaceAndMaskDetections):
        self._detected_faces = data.faces
        self._detected_masks = data.masks

        self._everybody_masked = self._detected_faces == self._detected_masks

    def _keyword_callback(self, data: String):
        self._keywords = data.data

    def execute(self, userdata: smach.UserData):
        # Userdata variables are accessed with the . operator
        # You can only access the variables you specified in input_keys, output_keys and io_keys
        # Variables specified in input_keys are wrapped to be immutable

        if self.preempt_requested():
            return "preempted"

        if not userdata.conversation_started:
            userdata.conversation_started = True
            self._speech_publisher.publish("Thank you for wearing a mask.")
            self._speech_publisher.publish("How can I help you?")
            self._speech_publisher.publish("This is what I can do:")
            self._speech_publisher.publish("I can tell you a joke or direct you to the kitchen")
        else:
            self._speech_publisher.publish("What else can I do for you?")

        self._keywords = None
        while not self._keywords:
            if self.preempt_requested():
                return "preempted"

            if self._keywords == "kitchen":
                return "kitchen"

            if self._keywords == "joke":
                return "joke"

            if self._keywords == "goodbye":
                return "goodbye"

            if self._detected_faces == 0:
                return "nobody_here"

            if not self._everybody_masked:
                return "one_not_masked"

            rospy.sleep(0.5)
