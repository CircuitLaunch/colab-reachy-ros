#!usr/bin/env python3

import rospy
import speech_recognition as sr
from typing import Optional
from std_msgs.msg import String


class ListenerNode:
    def __init__(self, name: str, microphone_name: Optional[str], timeout: Optional[int], frequency: float):
        if microphone_name:
            try:
                self._device_index = [
                    index
                    for index, value in enumerate(sr.Microphone.list_microphone_names())
                    if microphone_name in value
                ][0]
            except IndexError:
                raise ValueError(f'Microphone name including "{microphone_name}" not found')

            rospy.loginfo(f'Microphone name including "{microphone_name}" found at device index {self._device_index}')
        else:
            rospy.loginfo("No microphone name given, using default device index")
            self._device_index = None

        self._recognizer = sr.Recognizer()
        self._microphone_timeout = timeout

        self._publisher = rospy.Publisher(name + "/microphone_speech", String, queue_size=10)
        self._rate = rospy.Rate(frequency)

    def spin(self):
        with sr.Microphone(device_index=self._device_index) as source:
            rospy.loginfo("Listening...")

            while not rospy.is_shutdown():
                # Obtain audio from microphone
                try:
                    audio = self._recognizer.listen(source, timeout=self._microphone_timeout, phrase_time_limit=5)
                except sr.WaitTimeoutError:
                    rospy.logwarn("No audio heard within timeout")
                else:
                    # Recognize speech using Google Web Speech API
                    try:
                        listened = self._recognizer.recognize_google(audio)
                    except sr.UnknownValueError:
                        rospy.logerr("Could not understand audio")
                    except sr.RequestError as e:
                        rospy.logerr(f"Recognizer RequestError: {e}")
                    except sr.WaitTimeoutError:
                        rospy.logdebug("No audio heard")
                    else:
                        rospy.logdebug(f'Heard: "{listened}"')
                        self._publisher.publish(listened)

                self._rate.sleep()


if __name__ == "__main__":
    # add here the node name. In ROS, nodes are unique named.
    rospy.init_node("speech_listener", log_level=rospy.DEBUG)
    device_name = rospy.get_param("~device_name", default=None)
    timeout = rospy.get_param("~listen_timeout", default=3)
    listen_frequency = rospy.get_param("~rate", default=10)

    listener = ListenerNode(rospy.get_name(), device_name, timeout, listen_frequency)

    listener.spin()
