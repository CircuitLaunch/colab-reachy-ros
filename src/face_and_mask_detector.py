#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from mask_detection.detect_mask import MaskDetector, generate_debug_frame
from colab_reachy_ros.msg import FaceAndMaskDetections
from pathlib import Path


class MaskDetectorNode:
    def __init__(self, name: str):
        self._name = name

        self._source = rospy.get_param("~source")
        self._bridge = CvBridge()

        face_path = Path(rospy.get_param("~face_model_dir"))
        mask_path = Path(rospy.get_param("~mask_model"))
        self._detector = MaskDetector(face_path, mask_path)

        self._detection_buffer = []

        self._detection_publisher = rospy.Publisher(f"{self._name}/faces_detected", FaceAndMaskDetections, queue_size=1)
        self._subscriber = rospy.Subscriber(f"{self._source}/image_raw", Image, self._callback)

        self._debug_output = rospy.get_param("~debug_output")
        if self._debug_output:
            self._debug_publisher = rospy.Publisher(f"{self._name}/debug_image", Image, queue_size=1)

    def _callback(self, data: Image):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
            locs, preds = self._detector.detect_and_predict_mask(cv_image)

            # TODO: Some kind of averaging to handle fluctuations in detections, then publish
            faces = len(preds)
            masks = 0
            for pred in preds:
                if pred[0] >= 0.5:  # TODO: Better standards for confirming a face? Uncertainty threshold?
                    masks += 1

            # Store the last 5 frames, publish if at least 3 of 5 agree
            self._detection_buffer.append((faces, masks))
            if len(self._detection_buffer) > 5:
                self._detection_buffer.pop(0)  # Remove first element

                for start_index in range(0, 3):
                    if self._detection_buffer.count(self._detection_buffer[start_index]) >= 3:
                        self._detection_publisher.publish(
                            faces=self._detection_buffer[start_index][0], masks=self._detection_buffer[start_index][1]
                        )
                        break

            if self._debug_output:
                debug_image = generate_debug_frame(cv_image, locs, preds)
                self._debug_publisher.publish(self._bridge.cv2_to_imgmsg(debug_image, "bgr8"))
                rospy.logdebug(f"Predictions: {preds}")
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}", exc_info=True)


if __name__ == "__main__":
    rospy.init_node("mask_detector", log_level=rospy.DEBUG)
    detector = MaskDetectorNode(rospy.get_name())
    rospy.spin()
