import rospy
import smach
from state_machine.helper_functions import say_something
from std_msgs.msg import String
from colab_reachy_ros.msg import FaceAndMaskDetections
import threading

from chatterbot import ChatBot
from chatterbot.trainers import ListTrainer  # , ChatterBotCorpusTrainer


class HasMask(smach.State):
    def __init__(self):
        # Invoke parent class constructor
        # outcomes is the list of possible state outcomes
        # input_keys is the list of state machine variables this state reads but does not write
        # output_keys is the list of state machine variables this state writes but does not read
        # io_keys is the list of state machine variables this state both reads and writes
        super().__init__(
            outcomes=["one_not_masked", "nobody_here", "preempted", "kitchen", "joke", "goodbye"],
            io_keys=["conversation_started"],
        )

        self._detected_faces = None
        self._detected_masks = None
        self._everybody_masked = False
        self._keywords = None

        self._face_mask_subscriber = rospy.Subscriber(
            "/mask_detector/faces_detected", FaceAndMaskDetections, self._face_mask_callback, queue_size=10
        )

        self._speech_subscriber = rospy.Subscriber("/respeaker/microphone_speech", String, self._req_callback)
        self._heard_sentence = None
        self._key_words = ["kitchen", "bathroom", "call", "joke", "goodbye"]
        self._listen_mutex = threading.Lock()

        self.chatbot = ChatBot(
            name="Reachy",
            read_only=True,
            logic_adapters=[
                {
                    "import_path": "chatterbot.logic.BestMatch",
                    "default_response": "I am sorry, I do not understand. I am still learning. Please contact somebody for further assistance.",
                    "maximum_similarity_threshold": 0.90,
                }
            ],
            trainer="chatterbot.trainers.ChatterBotCorpusTrainer",)

        # Create a new trainer for the chatbot
        # trainer = ChatterBotCorpusTrainer(chatbot)

        # Train the chatbot based on the english corpus
        # trainer.train("chatterbot.corpus.english")

        # Train the chatbot based on lists

        self.small_talk = [
            "um",
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

        self.specific = [
            "i want to go to bathroom",
            "ok, follow me to the bathroom",
            "i want to go to the kitchen",
            "ok, follow me to the kitchen",
            "i need to call Dan",
            "ok, please wait while I contact Dan",
            "i need to call Leah",
            "ok, please wait while I contact Leah",
        ]

        self.list_trainer = ListTrainer(self.chatbot)

        for item in (self.small_talk, self.specific):
            self.list_trainer.train(item)

    def _req_callback(self, msg):
        with self._listen_mutex:
            self._heard_sentence = msg.data

    def _face_mask_callback(self, data: FaceAndMaskDetections):
        self._detected_faces = data.faces
        self._detected_masks = data.masks

        self._everybody_masked = self._detected_faces == self._detected_masks

    def execute(self, userdata: smach.UserData):
        # Userdata variables are accessed with the . operator
        # You can only access the variables you specified in input_keys, output_keys and io_keys
        # Variables specified in input_keys are wrapped to be immutable

        if self.preempt_requested():
            return "preempted"

        if not userdata.conversation_started:
            userdata.conversation_started = True
            say_something("Thank you for wearing a mask.")
            say_something("How can I help you?")
            say_something("This is what I can do:")
            say_something("I can tell you a joke or direct you to the kitchen")
        else:
            say_something("What else can I do for you?")

        self._heard_sentence = None
        while True:
            if self.preempt_requested():
                return "preempted"

            with self._listen_mutex:
                if self._heard_sentence:
                    request_words = self._heard_sentence.split()

                    if "kitchen" in request_words:
                        return "kitchen"
                    elif "joke" in request_words:
                        return "joke"
                    elif "goodbye" in request_words:
                        return "goodbye"
                    else:
                        response = self.chatbot.get_response(self._heard_sentence)
                        rospy.loginfo("Chatbot generated response" + response.text)
                        say_something(response.text)
                        self._heard_sentence = None

            # if self._detected_faces == 0:
            #     return "nobody_here"

            # if not self._everybody_masked:
            #     return "one_not_masked"

            rospy.sleep(0.5)
