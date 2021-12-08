# Waiter Robot

## Running (temp)
To run a specific package file, e.g., `recognizer`, do:
```shell
$ python -m src.interaction.speech.recognizer
```

To launch waiter_robot in an empty world in Gazebo
```
roscore
roslaunch waiter_robot stage_navigation.launch
```

## Requirements
### General
For a quick setup just run the following scripts inside **waiter_robot** package:
```shell
$ ./scripts/turtlebot3_setup.sh && ./scripts/venv_setup.sh
```

### ROS
The **waiter_bot** package was implemented using _ROS Noetic_. The simulation uses _gazebo_ for visualization and _turtlebot3_ as the waiter robot. To install _gazebo_ run:
```shell
$ sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```

### Python
The code was implemented using _Python 3.8.10_ and the libraries detailed in [requirements.txt](requirements.txt). You can install these libraries by running the following command or using conda (see [this](https://stackoverflow.com/questions/51042589/conda-version-pip-install-r-requirements-txt-target-lib)):
```shell
$ pip install -r requirements.txt
```

Please also ensure your environment supports `portaudio` and `pyaudio` for speech recognition:
```shell
$ sudo apt install portaudio19-dev python3-pyaudio
```