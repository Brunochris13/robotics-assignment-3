# Waiter Robot

## Running (temp)
To run a specific package file, e.g., `recognizer`, do:
```shell
$ python -m src.interaction.speech.recognizer
```

## Requirements
This code was implemented using _Python 3.8.10_ and the libraries detailed in [requirements.txt](requirements.txt). You can install these libraries as: `pip install -r requirements.txt` or using conda (see [this](https://stackoverflow.com/questions/51042589/conda-version-pip-install-r-requirements-txt-target-lib)).

Please also ensure your environment supports `portaudio` and `pyaudio`:
```shell
$ sudo apt install portaudio19-dev python3-pyaudio
```