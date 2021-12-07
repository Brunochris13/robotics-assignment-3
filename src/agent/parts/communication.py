import os
import subprocess

import rospy
import speech_recognition as sr
from gtts import gTTS

from ..actions import Action


class Communication():
    """Communication part responsible for robot's hearing and speech.
    """
    def __init__(self, basename=""):
        """Initializes the communication part.

        Args:
            basename (str): The name of the robot this part belongs to
        """
        self.name = basename + "[COMMUNICATION] "
        self.FNULL = open(os.devnull, 'w')


    def play(self, filepath):
        """Plays the specified audio file via terminal command.

        Args:
            filepath (str): The path to the audio file (`.mp3` format)
        """
        command = ["ffplay", filepath, "-nodisp", "-autoexit"]
        subprocess.call(command, stdout=self.FNULL, stderr=subprocess.STDOUT)


    def record(self, filepath, dur=3):
        """Records audio via microphone to a specified output file.

        Args:
            filepath (str): The path to the output file (`.wav` format)
            dur (int): The duration of the recording in seconds
        """
        command = ["arecord", "-d", str(dur), "-f", "cd", filepath]
        subprocess.call(command, stdout=self.FNULL, stderr=subprocess.STDOUT)


    def prompt(self):
        """Plays the prompt sound for UI."""
        rospy.loginfo(self.name + "Prompt")
        self.play("resources/audio/prompt.mp3")
    

    def say(self, message):
        """Reads aloud the given message in google voice.

        Args:
            message (str): The message to say aloud
        """
        # Log message and convert to audio
        rospy.loginfo(self.name + message)
        tts = gTTS(text=message, lang='en-uk')

        # Save, play, remove
        tts.save("temp.mp3")
        self.play("temp.mp3")
        os.remove("temp.mp3")
    

    def yesno_to_bool(self, voice_data):
        """Converts "yes" to `True`, "no" to `False`.

        Args:
            voice_data (str): The recorded voice data
        
        Returns:
            (bool): Whether the answer was yes or no. `None` if none.
        """
        if voice_data in ["yes", "yup", "yeah", "y", "true", "positive"]:
            return True
        elif voice_data in ["no", "nope", "nah", "n", "false", "negative"]:
            return False
        else:
            return None


    def recognize_voice(self, path=None):
        """Records voice data from a microphone or an audio file.

        Args:
            path (str): The path to audio file to recognize or `None`

        Returns:
            (tuple(Action, str)): A tuple consisting of action and voice data
        """
        # Initialize the recognizer
        recognizer = sr.Recognizer()
        
        with sr.Microphone() if path is None else sr.AudioFile(path) as source:
            # Record voice data from microphone or audio
            audio = recognizer.listen(source, timeout=7)
            
            try:
                # Convert audio to text data using a recognizer
                voice_data = recognizer.recognize_google(audio)
            except sr.UnknownValueError:
                # Could not process the audio
                return Action.BASE.REPEAT, ""
            except sr.WaitTimeoutError:
                # Timeout was reached, reject
                return Action.BASE.REJECT, ""
            except sr.RequestError:
                # Speech service doesn't work
                return Action.BASE.REJECT, ""
        
        return Action.BASE.ACCEPT, voice_data

    
    def listen(self, max_prompts=3, source=None, dur=5):
        """Listens for the user response and recognizes it.

        Args:
            max_prompts (int): The maximum number of prompts
            source (str): The path to recorded audio or `None`
            dur (int): The duration to record voice or `None`

        Returns:
            (str): The recognized answer 
        """
        # Init variables
        num_prompts = 0
        action = None

        while action != Action.BASE.ACCEPT and num_prompts < max_prompts:
            # Increase count
            num_prompts += 1
            
            # Wait answer
            self.prompt()

            # Inform when exactly client must start speaking
            rospy.loginfo(self.name + "Collecting audio...")

            if source is None:
                if dur is not None:
                    # Record manually if `dur` given
                    self.record("temp.wav", dur=dur)
                    action, voice_data = self.recognize_voice("temp.wav")
                    os.remove("temp.wav")
                else:
                    # If no dur provided, record dynamical audio
                    action, voice_data = self.recognize_voice()
            else:
                # If source is provided, use it to get access to the mp3 file
                raise NotImplementedError("Can't yet listen to audio files")

            if action is Action.BASE.REPEAT and num_prompts < max_prompts:
                # Ask to repeat if the recognizer failed
                self.say("Sorry, please repeat your answer.")
                
            if action is Action.BASE.REJECT and num_prompts < max_prompts:
                # Ask to type it instead if recognizer service is down
                self.say("Sorry, could you instead type your answer?")
                voice_data = input("Type your answer:\n")
                break
        
        return voice_data


    def say_listen(self, message, binary=False, max_prompts=3, source=None):
        """Says the message and listens for response.

        Args:
            message (str): The message to say
            binary (bool): Whether the message is a yes/no response
            max_prompts (int): The maximum number of prompts
            source (str): The path to recorded audio or `None`
        
        Returns:
            (str|bool): The recognized response (could be parsed to bool)
        """
        # Ask the question
        self.say(message)

        # Get the response voice data from a customer
        voice_data = self.listen(max_prompts, source)

        if binary:
            # If it is a yes/no question, parse to bool
            voice_data = self.yesno_to_bool(voice_data)

        return voice_data

        
    def ask(self, message, valid_answers=[], binary=False, max_prompts=(3, 3)):
        """Asks the provided question and checks if resposne is valid.

        Args:
            message (str): The message to say
            valid_answers (list(str)): The list of valid responses
            binary (bool): Whether to convert response to boolean
            max_prompts (tuple(int, int)): The max num of prompts for
                recognizing the response and for checking if it is in
                `valid_answers`
        
        Returns:
            (tuple(Action, (str|bool))): An action and recognized answer
        """
        if isinstance(max_prompts, int):
            # Convert to tuple if single int is given
            max_prompts = (max_prompts, max_prompts)
        
        if binary:
            # True/False valid for binary
            valid_answers = [True, False]

        # Init variables
        num_prompts = 0
        action = None
        
        while action != Action.BASE.ACCEPT and num_prompts < max_prompts[1]:
            # Increase count
            num_prompts += 1

            # Ask the question and wait until some sort of an answer is acquired
            voice_data = self.say_listen(message, binary, max_prompts[0])
            rospy.loginfo(self.name + "Acquired answer: " + str(voice_data))

            if voice_data not in valid_answers and num_prompts < max_prompts[1]:
                # Define all valid options for the provided question
                options =  ["yes", "no"] if binary else valid_answers

                # Generate 3 sentences that chould inform about failure
                sentence1 = f"Your answer, {voice_data}, is invalid."
                sentence2 = f"Valid options are: {', '.join(options)}."
                sentence3 = "Let me repeat the question."

                # Say the 3 sentences and change action to REPEAT
                self.say(f"{sentence1} {sentence2} {sentence3}")
                action = Action.BASE.REPEAT
            
            if voice_data in valid_answers:
                # ACCEPT if answer is valid
                action = Action.BASE.ACCEPT
        
        if action is Action.BASE.REPEAT:
            # Give the final chance by letting user to type response
            voice_data = input("Please type your answer instead:\n")

            if binary:
                # If it is a yes/no question, parse to bool
                voice_data = self.yesno_to_bool(voice_data)
            
            if voice_data in valid_answers:
                # ACCEPT if answer is valid
                action = Action.BASE.ACCEPT
            
        return action, voice_data


if __name__ == "__main__":
    communication = Communication()
    data = communication.listen(dur=5)
    print(data)
