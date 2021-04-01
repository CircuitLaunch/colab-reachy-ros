import rospy
from colab_reachy_ros.srv import Speak


def say_something(sentence: str):
    rospy.wait_for_service("/speak")
    speech_service = rospy.ServiceProxy("/left_arm_controller/set_arm_compliant", Speak)
    speech_service(sentence)