#!usr/bin/env python

import rospy
import speech_recognition as sr
from std_msgs.msg import String


class ListenerNode:
    def __init__(self, name: str, index: int, frequency: float):
        self._device_index = rospy.get_param
        self._source = sr.Microphone(device_index=index)
        self._recognizer = sr.Recognizer()

        self._publisher = rospy.Publisher(name + "/microphone_speech", String, queue_size=10)
        self._rate = rospy.Rate(frequency)

    def spin(self):
        rospy.loginfo("Listening...")

        while not rospy.is_shutdown():
            # Obtain audio from microphone
            audio = self._recognizer.listen(self._source)
            listened = None

            # Recognize speech using Sphinx
            try:
                listened = self._recognizer.recognize_google(audio)
                rospy.logdebug(f'Heard: "{listened}"')
            except sr.UnknownValueError:
                rospy.logerr("Could not understand audio")
            except sr.RequestError as e:
                rospy.logerr(f"Recognizer RequestError: {e}")
            except sr.WaitTimeoutError:
                rospy.logdebug("No audio heard")

            if listened:
                self._publisher.publish(listened)

            self._rate.sleep()


if __name__ == "__main__":
    # add here the node name. In ROS, nodes are unique named.
    rospy.init_node("speech_listener_node")
    device_index = rospy.get_param("~device_index")
    listen_frequency = rospy.get_param("~rate")

    listener = ListenerNode(rospy.get_name(), device_index, listen_frequency)

    listener.spin()
