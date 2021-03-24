import rospy
import smach
import threading
from colab_reachy_ros.msg import FaceAndMaskDetections


class Idle(smach.State):
    def __init__(self):
        # Invoke parent class constructor
        # outcomes is the list of possible state outcomes
        # input_keys is the list of state machine variables this state reads but does not write
        # output_keys is the list of state machine variables this state writes but does not read
        # io_keys is the list of state machine variables this state both reads and writes
        super().__init__(outcomes=["person_detected"], output_keys=["conversation_started", "num_faces", "num_masks"])

        # Lock prevensts execute() and subscriber callback from running at 
        self._mutex = threading.Lock()

        self._detected_faces = None
        self._detected_masks = None

        self._face_mask_subscriber = rospy.Subscriber(
            "/mask_detector/faces_detected", FaceAndMaskDetections, self._face_mask_callback, queue_size=10
        )

    def _face_mask_callback(self, data: FaceAndMaskDetections):
        with self._mutex:
            self._detected_faces = data.faces
            self._detected_masks = data.masks

    def execute(self, userdata: smach.UserData):
        # Userdata variables are accessed with the . operator
        # You can only access the variables you specified in input_keys, output_keys and io_keys
        # Variables specified in input_keys are wrapped to be immutable

        userdata.conversation_started = False

        while not self._detected_faces:
            rospy.sleep(0.5)

        with self._mutex:  # Lock to prevent detection variables from changing in this block
            userdata.num_faces = self._detected_faces
            userdata.num_masks = self._detected_masks

        return "person_detected"
