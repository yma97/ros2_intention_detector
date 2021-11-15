import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import speech_recognition as sr
import time
# import re


class IntentionPublisher(Node):

    def __init__(self):
        self.start_time = time.time()
        print("At {0:.2f}: Now we start!".format(time.time()-self.start_time))
        super().__init__('intention_publisher')
        print("At {0:.2f}: Initializing...".format(time.time()-self.start_time))
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        print("At {0:.2f}: Initialization done!".format(time.time()-self.start_time))

        # keep doing speach recognition
        self.publisher_ = self.create_publisher(String, 'intention', 3)
        print("At {0:.2f}: Publisher created!".format(time.time()-self.start_time))
        while(True):
           self.start_detection()

    
    def start_detection(self):
        msg = String()
        msg.data = self.intention_detection()
        self.publisher_.publish(msg)
        #self.get_logger().info('User wants to: "%s"' % msg.data)
        return msg.data

        # intention detection
    def intention_detection(self):
        """Main method to print prompt, start speech recognition, catch keyword,        
        and report error.

        Return a String of the intention keyword
        """
        # get command from the user
        prompt_limit = 3
        for i in range(prompt_limit):
            command = self.recognize_speech_from_mic(self.recognizer, self.microphone)
            if command["transcription"]:
                break
            if not command["success"]:
                break
            print("At {0:.2f}: I didn't catch that. What did you say?\n".format(time.time()-self.start_time))
    
        print("At {0:.2f}: Recoginition done!".format(time.time()-self.start_time))

        # if there was an error, stop
        if command["error"]:
            print("ERROR: {}".format(command["error"]))
            return "error"
        else:
            # show the user's transcription and pass it to intention detector
            print("You said: {}".format(command["transcription"]))
            return command["transcription"].lower()


    def recognize_speech_from_mic(self, recognizer, microphone):
        """Transcribe speech from recorded from 'microphone'.
    
        Returns a dictionary with three keys:
        "success": a boolean indicating whether or not the API request was
                   successful
        "error":   `None` if no error occured, otherwise a string containing
                   an error message if the API could not be reached or
                   speech was unrecognizable
        "transcription": `None` if speech could not be transcribed,
                   otherwise a string containing the transcribed text
        """
        # check that recognizer and microphone arguments are appropriate type
        if not isinstance(recognizer, sr.Recognizer):
            raise TypeError("`recognizer` must be `Recognizer` instance")
    
        if not isinstance(microphone, sr.Microphone):
            raise TypeError("`microphone` must be `Microphone` instance")
    
        # adjust the recognizer sensitivity to ambient noise and record audio
        # from the microphone
        with microphone as source:
            recognizer.adjust_for_ambient_noise(source)
            print("At {0:.2f}: Start listening".format(time.time()-self.start_time))
            audio = recognizer.listen(source, phrase_time_limit=10)
            print("At {0:.2f}: Finish listening".format(time.time()-self.start_time))

    
        print("At {0:.2f}: Recognizing...Please wait...".format(time.time()-self.start_time))

        # set up the response object
        response = {
            "success": True,
            "error": None,
            "transcription": None
        }
    
        # try recognizing the speech in the recording
        # if a RequestError or UnknownValueError exception is caught,
        #     update the response object accordingly
        try:
            response["transcription"] = recognizer.recognize_google(audio)
        except sr.RequestError:
            # API was unreachable or unresponsive
            response["success"] = False
            response["error"] = "API unavailable"
        except sr.UnknownValueError:
            # speech was unintelligible
            response["error"] = "Unable to recognize speech"
    
        return response


def main(args=None):
    rclpy.init(args=args)
    intention_publisher = IntentionPublisher()
    rclpy.spin(intention_publisher)        
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    intention_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
