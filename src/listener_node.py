#The Speech-to-text ROS node

#!usr/bin/env python


import rospy

import speech_recognition as sr

from std_msgs.msg import String
from time import sleep

def listen ():
    #obtain audio from microphone 
    r = sr.Recognizer()
    with sr.Microphone() as source:
        print("[...listening...]")
        audio = r.listen(source)
    #Recognize speech using Sphinx

    try:
        listened = r.recognize_google(audio) 
        print("You said: " + listened)
    except sr.UnknownValueError:
        listened = "Not understand"
        print("could not understand audio")
    except sr.RequestError as e:
        listened = "Error"
        print("Error; {0}".format(e))
    except sr.WaitTimeoutError:
        listened = "Did not hear"
        print("WaitTimeoutError")
    
    return listened

if __name__=='__main__':
    #add here the node name. In ROS, nodes are unique named.
    rospy.init_node("listener_node")

    pub =rospy.Publisher(rospy.get_name()+'/microphone_speech', String, queue_size=10)
    
    rate=rospy.Rate(10)

    while not rospy.is_shutdown():
        request = listen() #try to listen
        pub.publish(request)

rate.sleep()