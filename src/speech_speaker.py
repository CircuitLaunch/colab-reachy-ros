#!usr/bin/env python3

import sys
import os
import rospy
from gtts import gTTS
from std_msgs.msg import String
from playsound import playsound


def speak_callback(msg):
    language = "en-us"
    speech = gTTS(text=msg.data, lang=language, slow=False)
    filename = "speech1.mp3"
    speech.save(filename)
    playsound(filename)
    os.remove(filename)
    rospy.loginfo("Speaked: %s", msg.data)


def main():
    rospy.init_node("speaker_node")
    sub = rospy.Subscriber("speak", String, speak_callback)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Interrupted")
        os.remove("speech0.mp3")
        os.remove("speech1.mp3")
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)

    # Add here the name of the ROS. In ROS, names are unique named.
