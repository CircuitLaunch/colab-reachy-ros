#!usr/bin/env python3

import os
import rospy
from gtts import gTTS
from std_msgs.msg import String
from playsound import playsound
import threading


class SpeakerNode:
    def __init__(self):
        self._mutex = threading.Lock()

        self._language = "en-us"
        self._filename = "speech1.mp3"

        self._subscriber = rospy.Subscriber("speak", String, self._speak_callback)

    def __del__(self):
        try:
            os.remove(self._filename)
            os.remove("speech0.mp3")
        except FileNotFoundError:
            pass

    def _speak_callback(self, msg: String):
        with self._mutex:
            speech = gTTS(text=msg.data, lang=self._language, slow=False)
            speech.save(self._filename)
            playsound(self._filename)
            os.remove(self._filename)
            rospy.loginfo("Speaked: %s", msg.data)


if __name__ == "__main__":
    rospy.init_node("speaker_node")
    speaker_node = SpeakerNode()
    rospy.spin()
