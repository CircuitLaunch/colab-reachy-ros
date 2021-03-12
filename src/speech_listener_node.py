#!usr/bin/env python

import rospy
import speech_recognition as sr
from typing import Optional
from std_msgs.msg import String


class ListenerNode:
    def __init__(self, name: str, index: Optional[int], frequency: float):
        self._device_index = index
        self._recognizer = sr.Recognizer()

        self._publisher = rospy.Publisher(name + "/microphone_speech", String, queue_size=10)
        self._rate = rospy.Rate(frequency)

    def spin(self):
        rospy.loginfo("Listening...")

        with sr.Microphone(device_index=self._device_index) as source:
            while not rospy.is_shutdown():
                # Obtain audio from microphone
                audio = self._recognizer.listen(source)
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
    device_index = rospy.get_param("~device_index", default=None)
    listen_frequency = rospy.get_param("~rate", default=10)

    listener = ListenerNode(rospy.get_name(), device_index, listen_frequency)

    listener.spin()
