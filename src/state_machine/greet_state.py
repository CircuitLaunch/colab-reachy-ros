import rospy
import smach
import threading
from state_machine.helper_functions import say_something
from trajectory_msgs.msg import JointTrajectory
from colab_reachy_ros.msg import FaceAndMaskDetections, JointTemperatures
from std_srvs.srv import SetBool
from typing import List
import moveit_commander
from motion_control.moveit_helpers import load_joint_configurations_from_file
from motion_control.head_node_helpers import create_head_animation


class Greet(smach.State):
    def __init__(self):
        super().__init__(outcomes=["all_masks", "missing_mask", "nobody_here", "preempted"])

        self._mutex = threading.Lock()

        self._detected_faces = None
        self._detected_masks = None

        self._left_arm_overheating = False

        # creating a variable to hold the yes gesture animation
        #       for the array of points
        #       [trajectory_msgs/JointTrajectory Documentation]
        # (http://docs.ros.org/en/api/trajectory_msgs/html/msg/JointTrajectory.html)
        #       for the indivitual points
        #       [trajectory_msgs/JointTrajectoryPoint Documentation]
        # (http://docs.ros.org/en/melodic/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html)
        #       simulates head shaking yes motion up and down
        self._head_yes_gesture = create_head_animation(
            [[90.0, 90.0], [90.0, 115.0], [90.0, 90.0], [90.0, 115.0], [90.0, 90.0]]
        )

        self._face_mask_subscriber = rospy.Subscriber(
            "/mask_detector/faces_detected", FaceAndMaskDetections, self._face_mask_callback, queue_size=10
        )
        self._left_arm_temperature_subscriber = rospy.Subscriber(
            "/left_arm_controller/joint_temperatures",
            JointTemperatures,
            self._left_arm_temperature_callback,
            queue_size=10,
        )

        self._left_arm_commander = moveit_commander.MoveGroupCommander("left_arm", wait_for_servers=60)
        load_joint_configurations_from_file(self._left_arm_commander)

        # this publisher moves the reachy's head around
        self._head_publisher = rospy.Publisher("/head/position_animator_debug_degrees", JointTrajectory, queue_size=1)

    def _face_mask_callback(self, data: FaceAndMaskDetections):
        with self._mutex:
            self._detected_faces = data.faces
            self._detected_masks = data.masks

    def _left_arm_temperature_callback(self, data: JointTemperatures):
        overheating = False
        for temp in data.temperature:
            if temp > 45:
                overheating = True

        self._left_arm_overheating = overheating

    def _left_arm_gesture(self, poses: List[str]):
        # Set arm stiff
        rospy.wait_for_service("/left_arm_controller/set_arm_compliant")  # TODO: Remove this? Add a timeout?
        compliance_service = rospy.ServiceProxy("/left_arm_controller/set_arm_compliant", SetBool)
        compliance_service(False)

        for pose in poses:
            if self._left_arm_overheating:
                rospy.logwarn("left arm too hot, aborting gesture")
                break
            elif self.preempt_requested():
                break
            # move left arm to pose (blocking)
            self._left_arm_commander.set_named_target(pose)
            self._left_arm_commander.go(wait=True)
            self._left_arm_commander.stop()

        # Move to rest position and set arm compliant
        self._left_arm_commander.set_named_target("left_rest")
        self._left_arm_commander.go(wait=True)
        self._left_arm_commander.stop()
        compliance_service(True)

    def execute(self, userdata: smach.UserData):
        # Every state must check if a preempt has been requested (e.g. in case of Ctrl+C)
        # Smach does NOT handle this itself
        if self.preempt_requested():
            return "preempted"

        say_something("Hi, I'm Reachy!")
        rospy.sleep(0.05)  # If the function exits immediately, the publishes won't happen

        # shake the head yes to indicate that reachy is listening
        try:
            self._head_publisher.publish(self._head_yes_gesture)
            rospy.loginfo("Head gesture: 'YES'")
        except Exception as e:
            rospy.logerr(f"Error performing head gesture: {e}", exc_info=True)

        poses = ["left_hello_01", "left_hello_02"]
        if not self._left_arm_overheating:
            try:
                self._left_arm_gesture(poses)
            except Exception as e:
                rospy.logerr(f"Error performing arm gesture: {e}", exc_info=True)

        rospy.sleep(10)

        with self._mutex:  # Lock to prevent detection variables from changing in this block
            if self.preempt_requested():
                return "preempted"
            elif self._detected_masks == self._detected_faces and self._detected_faces > 0:
                return "all_masks"
            elif self._detected_masks < self._detected_faces:
                return "missing_mask"
            else:
                return "nobody_here"
