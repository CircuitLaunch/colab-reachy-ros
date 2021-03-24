import rospy
import smach
import threading
from colab_reachy_ros.msg import FaceAndMaskDetections


class Idle(smach.State):
    def __init__(self):
        super().__init__(outcomes=["person_detected"], output_keys=["conversation_started", "num_faces", "num_masks"])

        self._mutex = threading.Lock()

        self._face_mask_subscriber = rospy.Subscriber(
            "/mask_detector/faces_detected", FaceAndMaskDetections, self._face_mask_callback, queue_size=10
        )

        self._detected_faces = None
        self._detected_masks = None

    def _face_mask_callback(self, data: FaceAndMaskDetections):
        with self._mutex:
            self._detected_faces = data.faces
            self._detected_masks = data.masks

    def execute(self, userdata: smach.UserData):
        with self._mutex:
            userdata.conversation_started = False

            while not self._detected_faces:
                rospy.sleep(0.5)

            userdata.num_faces = self._detected_faces
            userdata.num_masks = self._detected_masks

            return "person_detected"
