#!usr/bin/env python

import rospy
from gtts import gTTS
from typing import Optional
from std_msgs.msg import String
from playsound import playsound


    

def speak_callback(msg):
    global count
    language = 'en-us'
    speech = gTTS(text = text, lang = language, slow = False)
    filename = f'speech{count%2}.mp3'
    speech.save(filename)
    playsound(filename)
    os.remove(filename)
    count +=1
    rospy.loginfo("Speaked: %s", msg.data)



def main():
    rospy.init_node('speaker_node')
    sub=rospy.Subscriber('speak', String, speak_callback)
    rospy.spin() 
    
if __name__=='__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Interrupted')
        os.remove("speech0.mp3")
        os.remove("speech1.mp3")
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)

    #Add here the name of the ROS. In ROS, names are unique named.
       
