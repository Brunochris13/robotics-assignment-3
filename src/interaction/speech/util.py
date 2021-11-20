import speech_recognition as sr
from src.logic.state_actions import Action


def record_from_microphone():
    """Records voice data from a microphone.

    Returns:
        (tuple): A tuple consisting of a logic action and recorded data
    """
    # Initialize the recognizer
    recognizer = sr.Recognizer()

    with sr.Microphone() as source:
        # Record audio data
        audio = recognizer.listen(source)
        
        try:
            # Convert audio to text data using Google recognizer
            voice_data = recognizer.recognize_google(audio)
        except sr.UnknownValueError:
            return Action.LOGIC.REPEAT, ""
        except sr.RequestError:
            return Action.LOGIC.IGNORE, ""
        
    return Action.LOGIC.ACCEPT, voice_data