#!/usr/bin/env python

from gtts import gTTS
import os
import subprocess
import speech_recognition as sr
from ..state_actions import Action

class Communication():
    def __init__(self):
        self.mname = "[COMMUNICATION] "
        self.FNULL = open(os.devnull, 'w')

    def play(self, f):
        command = ["ffplay", f, "-nodisp", "-autoexit"]
        subprocess.call(command, stdout=self.FNULL, stderr=subprocess.STDOUT)

    def say(self, message):
        print(message)
        tts = gTTS(text=message, lang='en-uk')
        tts.save("temp.mp3")
        self.play("temp.mp3")
        os.remove("temp.mp3")

    def beep(self):
        print(self.mname + "Beeping")
        # self.play("sound/ap-success.mp3")

    def prompt(self):
        print(self.mname + "Prompt")
        # self.play("sound/hw-insert.mp3")


    def ask(self, message, valid_answers=[], binary=False, max_prompts=(3, 3)):
        
        if isinstance(max_prompts, int):
            # Convert to tuple if single int is given
            max_prompts = (max_prompts, max_prompts)
        
        if binary:
            # True/False valid for binary
            valid_answers = [True, False]

        # Init variables
        num_prompts = 0
        action = None
        
        while action != Action.BASE.ACCEPT or num_prompts < max_prompts[1]:
            # Increase count
            num_prompts += 1

            # Ask the question and wait until some sort of an answer is acquired
            voice_data = self.say_listen(self, message, binary, max_prompts[0])

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
            
        return action, voice_data


    def say_listen(self, message, binary=False, max_prompts=3, source=None):
        # Ask the question
        self.say(message)

        # Get the response voice data from a customer
        voice_data = self.listen(max_prompts, source)

        if binary:
            # If it's a yes/no question, convert to bool
            voice_data = self.yesno_to_bool(voice_data)

        return voice_data
    

    def listen(self, max_prompts=3, source=None):
        # Init variables
        num_prompts = 0
        action = None

        while action != Action.BASE.ACCEPT or num_prompts < max_prompts:
            # Increase count
            num_prompts += 1
            
            # Wait answer
            self.prompt()

            if source is None:
                # If no source provided, use available microphone
                action, voice_data = self.record_from_microphone()
            else:
                # If source is provided, use it to get access to the mp3 file
                raise NotImplementedError("Can't yet listen to audio files")

            if action is Action.BASE.REPEAT:
                # Ask to repeat if the recognizer failed
                self.say("Sorry, could you repeat that?")
                
            if action is Action.BASE.REJECT:
                # Ask to type it instead if recognizer service is down
                self.say("Sorry, could you instead type your answer?")
                voice_data = input("Type your answer:")
                break
        
        return voice_data     


    def yesno_to_bool(self, voice_data):
        """Converts yes to `True`, no to `False`.

        Args:
            voice_data (str): The recorded voice data
        
        Returns:
            (bool): Whether the answer was yes or no. `None` if none.
        """
        if voice_data in ["yes", "yup", "yeah"]:
            # If positive
            return True
        elif voice_data in ["no", "nope", "nah"]:
            # If negative
            return False
        else:
            # Otherwise
            return None


    def record_from_microphone(self):
        """Records voice data from a microphone.

        Returns:
            (tuple): A tuple consisting of a logic action and voice data
        """
        # Initialize the recognizer
        recognizer = sr.Recognizer()

        with sr.Microphone() as source:
            # Record voice data from microphone
            audio = recognizer.listen(source)
            
            try:
                # Convert audio to text data using a recognizer
                voice_data = recognizer.recognize_google(audio)
            except sr.UnknownValueError:
                # Could not process the audio
                return Action.BASE.REPEAT, ""
            except sr.RequestError:
                # Speech service doesn't work
                return Action.BASE.REJECT, ""
            
        return Action.BASE.ACCEPT, voice_data


if __name__ == "__main__":
    communication = Communication()
    print("hi")
    communication.say("Hi there. This is a very long paragraph")
    print("Prints only after speaking")
