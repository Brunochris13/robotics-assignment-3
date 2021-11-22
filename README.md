# Waiter Robot

## Running (temp)
To run a specific package file, e.g., `recognizer`, do:
```shell
$ python -m src.interaction.speech.recognizer
```

To launch turtlebot3 in an empty world in Gazebo
```
roscore
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
or
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

If you want to try the built-in Navigation node run in a new terinal
```
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
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
_turtlebot3_ and dependencies for it can be installed by running:
```shell
$ sudo apt-get install ros-noetic-dynamixel-sdk ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3 ros-noetic-dwa-local-planner
```

To make _turtlebot3_ work on simulations install the simulation package:
```shell
$ cd ~/catkin_ws/src/
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make
```

If you have _Ignition Rendering_ problems when running the built in navigator, the following may fix it:
```shell
$ sudo apt -y install wget lsb-release gnupg
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt update
$ sudo apt install libignition-rendering3-dev
```

Before running any command you need to export `TURTLEBOT3_MODEL` for each terminal that will use a node from a _turtlebot3_ package.
It can take value `burger`, `waffle` and `waffle_pi`. Current implementation uses `waffle`. Run this before every execution:
```shell
export TURTLEBOT3_MODEL=waffle
```

Or to automatically choose it every time, just add the export to `bashrc`:
```shell
echo "export TURTLEBOT3_MODEL=waffle" >> ~/bashrc
source ~/.bashrc
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