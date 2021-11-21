# To automatically choose waffle robot as turtlebot
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc

# To install Ros-Gazebo package
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

# To get the package of turtlebot3_simulations
cd ~/catkin_ws/src/
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws && catkin_make

# Install all required packages
sudo apt-get install ros-noetic-dynamixel-sdk ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3 ros-noetic-dwa-local-planner

# If you have Ignition Rendering problems when running the built in navigator, uncomment below lines
# sudo apt -y install wget lsb-release gnupg
# sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
# wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
# sudo apt update
# sudo apt install libignition-rendering3-dev