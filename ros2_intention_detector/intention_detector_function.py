import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import speech_recognition as sr
import time

#def recognize_speech_from_mic(recognizer, microphone):
#    """Transcribe speech from recorded from 'microphone'.
#
#    Returns a dictionary with three keys:
#    "success": a boolean indicating whether or not the API request was
#               successful
#    "error":   `None` if no error occured, otherwise a string containing
#               an error message if the API could not be reached or
#               speech was unrecognizable
#    "transcription": `None` if speech could not be transcribed,
#               otherwise a string containing the transcribed text
#    """
#    # check that recognizer and microphone arguments are appropriate type
#    if not isinstance(recognizer, sr.Recognizer):
#        raise TypeError("`recognizer` must be `Recognizer` instance")
#
#    if not isinstance(microphone, sr.Microphone):
#        raise TypeError("`microphone` must be `Microphone` instance")
#
#    # adjust the recognizer sensitivity to ambient noise and record audio
#    # from the microphone
#    with microphone as source:
#        recognizer.adjust_for_ambient_noise(source)
#        audio = recognizer.listen(source)
#
#    # set up the response object
#    response = {
#        "success": True,
#        "error": None,
#        "transcription": None
#    }
#
#    # try recognizing the speech in the recording
#    # if a RequestError or UnknownValueError exception is caught,
#    #     update the response object accordingly
#    try:
#        response["transcription"] = recognizer.recognize_google(audio)
#    except sr.RequestError:
#        # API was unreachable or unresponsive
#        response["success"] = False
#        response["error"] = "API unavailable"
#    except sr.UnknownValueError:
#        # speech was unintelligible
#        response["error"] = "Unable to recognize speech"
#
#    return response


#def keyword_detection(transcript):
#    """Detect user's intention when speech recognization succeeds.
#
#    Returns a dictionary of five key intentions as boolean values:
#    "connect", "clearall","submit", "agree", "disagree"
#    Defalut value -  all False
#    """
#
#    keywords_connect = ['connect','kinect']
#    keywords_clearall = ['clear','erase','empty']
#    keywords_submit = ['submit']
#    keywords_agree = ['yes','yea','okay','agree']
#    keywords_disagree = ['no','not',"don't",'disagree']
#
#    # set up the response object
#    intention_dic = {
#        "connect": False,
#        "clearall": False,
#        "submit": False,
#        "agree": False,
#        "disagree": False
#    }
#
#    # set the corresponding intention to true if detected
#    if any([keyw in transcript for keyw in keywords_connect]):
#        intention_dic["connect"] = True
#    elif any([keyw in transcript for keyw in keywords_clearall]):
#        intention_dic["clearall"] = True
#    elif any([keyw in transcript for keyw in keywords_submit]):
#        intention_dic["submit"] = True
#    elif any([keyw in transcript for keyw in keywords_agree]):
#        intention_dic["agree"] = True
#    elif any([keyw in transcript for keyw in keywords_disagree]):
#        intention_dic["disagree"] = True
#
#
#    return intention_dic



## intention detection
#def intention_detection(recognizer, microphone):
#
#    # show instructions and wait 3 seconds before starting
#    instructions = "Tell me what you want to do!"
#    print(instructions)
#    time.sleep(1)
#
#    # get command from the user
#    prompt_limit = 3
#    for i in range(prompt_limit):
#        #print(instructions)
#        command = recognize_speech_from_mic(recognizer, microphone)
#        if command["transcription"]:
#            break
#        if not command["success"]:
#            break
#        print("I didn't catch that. What did you say?\n")
#
#    # if there was an error, stop
#    if command["error"]:
#        print("ERROR: {}".format(command["error"]))
#    else:
#        # show the user's transcription and pass it to intention detector
#        print("You said: {}".format(command["transcription"]))
#        intention_dic = keyword_detection(command["transcription"].lower())
#        intention = ""
#        if intention_dic["connect"]:
#            intention = "connect"
#            print("User wants to {} ".format(intention))
#        elif intention_dic["clearall"]:
#            intention = "clear all"
#            print("User wants to {} ".format(intention))
#        elif intention_dic["submit"]:
#            intention = "submit"
#            print("User wants to {} ".format(intention))
#        elif intention_dic["agree"]:
#            intention = "agree"
#            print("User wants to {} ".format(intention))
#        elif intention_dic["disagree"]:
#            intention = "disagree"
#            print("User wants to {} ".format(intention))
#        else:
#            print("Error: intention detection failed.")
#        
#        return intention
#

class IntentionPublisher(Node):

    def __init__(self):
        super().__init__('intention_publisher')
        print("Now we start!")
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        while(True):
            self.publisher_ = self.create_publisher(String, 'intention', 3)
            self.detection_callback()
            time.sleep(3)

    def detection_callback(self):
        msg = String()
        #msg.data = self.intention_detection(recognizer, microphone)
        msg.data = self.intention_detection()
        self.publisher_.publish(msg)
        self.get_logger().info('User wants to: "%s"' % msg.data)
        return msg.data

    # intention detection
    def intention_detection(self):
        """Main method to print prompt, start speech recognition, catch keyword,        
        and report error.

        Return a String of the intention keyword
        """
    
        # show instructions and wait 3 seconds before starting
        instructions = "Tell me what you want to do!"
        print(instructions)
        time.sleep(1)
    
        # get command from the user
        prompt_limit = 3
        for i in range(prompt_limit):
            #print(instructions)
            command = self.recognize_speech_from_mic(self.recognizer, self.microphone)
            if command["transcription"]:
                break
            if not command["success"]:
                break
            print("I didn't catch that. What did you say?\n")
    
        # if there was an error, stop
        if command["error"]:
            print("ERROR: {}".format(command["error"]))
        else:
            # show the user's transcription and pass it to intention detector
            print("You said: {}".format(command["transcription"]))
            intention_dic = self.keyword_detection(command["transcription"].lower())
            intention = ""
            if intention_dic["connect"]:
                intention = "connect"
                #print("User wants to {} ".format(intention))
            elif intention_dic["clearall"]:
                intention = "clear all"
                #print("User wants to {} ".format(intention))
            elif intention_dic["submit"]:
                intention = "submit"
                #print("User wants to {} ".format(intention))
            elif intention_dic["agree"]:
                intention = "agree"
                #print("User wants to {} ".format(intention))
            elif intention_dic["disagree"]:
                intention = "disagree"
                #print("User wants to {} ".format(intention))
            else:
                print("Error: intention detection failed.")
    
            return intention

    def keyword_detection(self,transcript):
        """Detect user's intention when speech recognition succeeds.
        Compare the words in the transcript with teh keywords set.
    
        Returns a dictionary of five key intentions as boolean values:
        "connect", "clearall","submit", "agree", "disagree"
        Defalut value -  all False
        """
    
        keywords_connect = ['connect','kinect']
        keywords_clearall = ['clear','erase','empty']
        keywords_submit = ['submit']
        keywords_agree = ['yes','yea','okay','agree']
        keywords_disagree = ['no','not',"don't",'disagree']
    
        # set up the response object
        intention_dic = {
            "connect": False,
            "clearall": False,
            "submit": False,
            "agree": False,
            "disagree": False
        }
    
        # set the corresponding intention to true if detected
        if any([keyw in transcript for keyw in keywords_connect]):
            intention_dic["connect"] = True
        elif any([keyw in transcript for keyw in keywords_clearall]):
            intention_dic["clearall"] = True
        elif any([keyw in transcript for keyw in keywords_submit]):
            intention_dic["submit"] = True
        elif any([keyw in transcript for keyw in keywords_agree]):
            intention_dic["agree"] = True
        elif any([keyw in transcript for keyw in keywords_disagree]):
            intention_dic["disagree"] = True
    
    
        return intention_dic

    
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
            audio = recognizer.listen(source)
    
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

    # create recognizer and mic instances
    #print("Now we start!")
    #recognizer = sr.Recognizer()
    #microphone = sr.Microphone()

    #while(True):
    #intention = intention_detection(recognizer, microphone)
    rclpy.init(args=args)
    intention_publisher = IntentionPublisher()
    rclpy.spin(intention_publisher)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    intention_publisher.destroy_node()
    rclpy.shutdown()
    #    time.sleep(3)

if __name__ == '__main__':
    main()

