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

        self._detection_publisher = rospy.Publisher(f"{self._name}/faces_detected", FaceAndMaskDetections, queue_size=1)
        self._debug_publisher = rospy.Publisher(f"{self._name}/debug_image", Image, queue_size=1)
        self._subscriber = rospy.Subscriber(f"{self._source}/image_raw", Image, self._callback)

    def _callback(self, data: Image):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
            locs, preds = self._detector.detect_and_predict_mask(cv_image)
            debug_image = generate_debug_frame(cv_image, locs, preds)
            self._debug_publisher.publish(self._bridge.cv2_to_imgmsg(debug_image, "bgr8"))
            rospy.logdebug(f"Predictions: {preds}")

            # TODO: Some kind of averaging to handle fluctuations in detections, then publish
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}", exc_info=True)


if __name__ == '__main__':
    rospy.init_node("mask_detector", log_level=rospy.DEBUG)
    detector = MaskDetectorNode(rospy.get_name())
    rospy.spin()
