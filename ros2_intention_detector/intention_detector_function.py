import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import speech_recognition as sr
import time
import wave
import pyaudio
import librosa
import soundfile as sf


import wave
import pyaudio


class ReachyAudioPlayerRecorder():
    """ReachyAudioPlayerRecorder class.

    This class allows to record audio samples and save them as WAV file.
    It also allows to play WAV files.
    """

    def __init__(self):
        """Define constants for playing and recording."""
        # number of frames the signal is split into
        self.chunk = 1024

        # number of frames per second
        self.rate = 44100

        # number of samples per frame
        self.channels = 2

        # number of bytes per sample
        self.format = pyaudio.paInt16

    def recordAudio(self, recordTime=5, wavOutputFileName="output.wav"):
        """Record audio samples and save them as a WAV file.

        :param recordTime: Duration of the recording.
        :param wavOutputFileName: Name of the WAV output file.
        """
        try:
            # Create the PyAudio object and open the PyAudio stream
            p = pyaudio.PyAudio()
            stream = p.open(format=self.format,
                            channels=self.channels,
                            rate=self.rate,
                            input=True,
                            frames_per_buffer=self.chunk)

            print("* recording")

            frames = []

            # our signal is composed of rate*recordTime frames. Since our for
            # loop is not repeated for each frame but only for each chunk,
            # the number of loops has to be divided by the chunk size
            for _ in range(int(self.rate / self.chunk * recordTime)):
                data = stream.read(self.chunk)
                frames.append(data)

            print("* done recording")

            # Close and terminate the PyAudio processes
            stream.stop_stream()
            stream.close()
            p.terminate()

            # Save the recorded audio data
            wf = wave.open(wavOutputFileName, 'wb')
            wf.setnchannels(self.channels)
            wf.setsampwidth(p.get_sample_size(self.format))
            wf.setframerate(self.rate)
            wf.writeframes(b''.join(frames))
            wf.close()

        except Exception as e:
            print("Exception: " + str(e))

    def playAudio(self, wavFileName):
        """Play a WAV file.

        :param wavFileName: Name of the WAV file to play.
        """
        try:
            # Open the wav file
            wf = wave.open(wavFileName, 'rb')

            # Create the PyAudio object and open the PyAudio stream
            p = pyaudio.PyAudio()
            stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                            channels=wf.getnchannels(),
                            rate=wf.getframerate(),
                            output=True)

            # Read the wav file until his end
            data = wf.readframes(self.chunk)

            while data != b'':
                stream.write(data)
                data = wf.readframes(self.chunk)

            # Terminate and close all the processes
            stream.stop_stream()
            stream.close()

            p.terminate()

            wf.close()

        except Exception as e:
            print("Exception: " + str(e))


class IntentionPublisher(Node):

    def __init__(self):

        """Initialization"""
        self.start_time = time.time()
        print("At {0:.2f}: Now we start!".format(time.time()-self.start_time))
        super().__init__('intention_publisher')
        print("At {0:.2f}: Initializing...".format(time.time()-self.start_time))
        # number of frames the signal is split into
        self.reachyAudio = ReachyAudioPlayerRecorder()
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        #self.reachy_audio = reachyAudio.ReachyAudio()
        #self.reachy_audio.pixel_ring.mono(self.reachy_audio.COLORS['CYAN'])
        print("At {0:.2f}: Initialization done!".format(time.time()-self.start_time))

        # keep doing speach recognition
        self.publisher_ = self.create_publisher(String, 'intention', 3)
        print("At {0:.2f}: Publisher created!".format(time.time()-self.start_time))
        while(True):
           msg = String()
           msg.data = self.intention_detection()
           self.publisher_.publish(msg)
           #self.get_logger().info('User wants to: "%s"' % msg.data)


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
            time_before_listen = time.time()
            # play start sound indicating recording
            self.reachyAudio.playAudio("start.wav")
            #print('playing sound using  playsound')
            audio = recognizer.listen(source, phrase_time_limit=8)
            time_listened = time.time()-time_before_listen
            # set led lights to be red indicating recording ends
            self.reachyAudio.playAudio("finish.wav")
            print("At {0:.2f}: Finish listening".format(time.time()-self.start_time))
            if (time_listened >= 8):print("TIME OUT - listened more than 8 seconds")

    
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
