import rospy
import smach
import threading
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from colab_reachy_ros.msg import FaceAndMaskDetections, JointTemperatures
from std_srvs.srv import SetBool
from typing import List
import moveit_commander
from motion_control.moveit_helpers import load_joint_configurations_from_file


class Greet(smach.State):
    def __init__(self):
        super().__init__(outcomes=["all_masks", "missing_mask", "nobody_here", "preempted"])

        self._mutex = threading.Lock()

        self._detected_faces = None
        self._detected_masks = None

        self._right_arm_overheating = False

        self._face_mask_subscriber = rospy.Subscriber(
            "/mask_detector/faces_detected", FaceAndMaskDetections, self._face_mask_callback, queue_size=10
        )
        self._right_arm_temperature_subscriber = rospy.Subscriber(
            "/right_arm_controller/joint_temperatures",
            JointTemperatures,
            self._right_arm_temperature_callback,
            queue_size=10,
        )

        self._speech_publisher = rospy.Publisher("/speak", String, queue_size=1)
        self._head_publisher = rospy.Publisher("/head/position_animator", JointTrajectory, queue_size=1)

        self._right_arm_commander = moveit_commander.MoveGroupCommander("right_arm")
        load_joint_configurations_from_file(self._right_arm_commander)

    def _face_mask_callback(self, data: FaceAndMaskDetections):
        with self._mutex:
            self._detected_faces = data.faces
            self._detected_masks = data.masks

    def _right_arm_temperature_callback(self, data: JointTemperatures):
        overheating = False
        for temp in data.temperature:
            if temp > 45:
                overheating = True

        self._right_arm_overheating = overheating

    def _right_arm_gesture(self, poses: List[str]):
        # Set arm stiff
        rospy.wait_for_service("/right_arm_controller/set_arm_compliant")  # TODO: Remove this? Add a timeout?
        compliance_service = rospy.ServiceProxy("/right_arm_controller/set_arm_compliant", SetBool)
        compliance_service(False)

        for pose in poses:
            if self._right_arm_overheating:
                rospy.logwarn("right arm too hot, aborting gesture")
                break
            elif self.preempt_requested():
                break
            # move right arm to pose (blocking)
            self._right_arm_commander.set_named_target(pose)
            self._right_arm_commander.go(wait=True)
            self._right_arm_commander.stop()

        # Move to rest position and set arm compliant
        self._right_arm_commander.set_named_target("right_rest")
        self._right_arm_commander.go(wait=True)
        self._right_arm_commander.stop()
        compliance_service(True)

    def execute(self, userdata: smach.UserData):
        # Every state must check if a preempt has been requested (e.g. in case of Ctrl+C)
        # Smach does NOT handle this itself
        if self.preempt_requested():
            return "preempted"

        self._speech_publisher.publish("Hi, I'm Reachy!")
        # self._head_publisher.publish(a_trajectory)
        rospy.sleep(0.05)  # If the function exits immediately, the publishes won't happen

        poses = ["hello_01", "hello_02"]
        if not self._right_arm_overheating:
            self._right_arm_gesture(poses)

        with self._mutex:  # Lock to prevent detection variables from changing in this block
            if self.preempt_requested():
                return "preempted"
            elif self._detected_masks == self._detected_faces and self._detected_faces > 0:
                return "all_masks"
            elif self._detected_masks < self._detected_faces:
                return "missing_mask"
            else:
                return "nobody_here"
