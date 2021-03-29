#!usr/bin/env python3

# The Keyword finder & chatbot ROS node

import rospy
from chatterbot import ChatBot
from chatterbot.trainers import ListTrainer  # , ChatterBotCorpusTrainer


from std_msgs.msg import String


chatbot = ChatBot(
    name="Reachy",
    read_only=True,
    logic_adapters=[
        {
            "import_path": "chatterbot.logic.BestMatch",
            "default_response": "I am sorry, I do not understand. I am still learning. Please contact somebody for further assistance.",
            "maximum_similarity_threshold": 0.90,
        }
    ],
    trainer="chatterbot.trainers.ChatterBotCorpusTrainer",
)

# Create a new trainer for the chatbot
# trainer = ChatterBotCorpusTrainer(chatbot)

# Train the chatbot based on the english corpus
# trainer.train("chatterbot.corpus.english")

# Train the chatbot based on lists

small_talk = [
    "hi there!",
    "hi",
    "how do you do?",
    "how are you?",
    "i'm cool.",
    "fine,you?",
    "always cool.",
    "i\m ok",
    "glad to heat that",
    "i feel awesome",
    "excellent, glad to hear that",
    "not so good",
    "sorry to hear that.",
    "what's your name?",
    "i'm Reachy ",
]

specific = [
    "i want to go to bathroom",
    "ok, follow me to the bathroom",
    "i want to go to the kitchen",
    "ok, follow me to the kitchen",
    "i need to call Dan",
    "ok, please wait while I contact Dan",
    "i need to call Leah",
    "ok, please wait while I contact Leah",
]

list_trainer = ListTrainer(chatbot)

for item in (small_talk, specific):
    list_trainer.train(item)


def request_analyzer_callback(msg):
    key_words = ["kitchen", "bathroom", "call"]

    request = msg.data
    request_words = request.split()

    for word in request_words:
        if word in key_words:
            command = word
            rospy.loginfo('Keyword "' + command + '" detected.')
            pub_1.publish(command)
            return

    response = chatbot.get_response(request)
    rospy.loginfo("Chatbot generated response" + response.text)
    pub_2.publish(response.text)


if __name__ == "__main__":
    # add here the node name. In ROS, nodes are unique named.
    rospy.init_node("keyword_chatbot_node")

    pub_1 = rospy.Publisher("voice_command", String, queue_size=10)
    pub_2 = rospy.Publisher("speak", String, queue_size=10)

    sub = rospy.Subscriber("speech_listener/microphone_speech", String, request_analyzer_callback)
    rospy.spin()
