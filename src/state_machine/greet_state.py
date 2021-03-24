import rospy
import smach
import threading
from colab_reachy_ros.msg import FaceAndMaskDetections


class Greet(smach.State):
    def __init__(self):
        super().__init__(
            outcomes=["all_masks", "missing_mask", "nobody_here"],
            input_keys=["num_faces", "num_masks"],
            output_keys=["conversation_started", "num_faces", "num_masks"],
        )

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
        with self._mutex:
            userdata.conversation_started = True

            raise NotImplementedError  # TODO: Speak, wave, gesture

            if self._detected_faces is not None:
                # New face detections, save and use these
                userdata.num_faces = self._detected_faces
                userdata.num_masks = self._detected_masks

            if userdata.num_masks == userdata.num_faces and userdata.num_faces > 0:
                return "all_masks"
            elif userdata.num_masks < userdata.num_faces:
                return "missing_mask"
            else:
                return "nobody_here"
