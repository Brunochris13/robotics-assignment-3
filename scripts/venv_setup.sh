# Install the dependent packages
sudo apt install portaudio19-dev python3-pyaudio ffmpeg

# Create virtual environment inside waiter_robot
cd ~/catkin_ws/src/waiter_robot/
python3 -m venv venv
source venv/bin/activate

# Install the dependet libraries
pip install -r requirements.txt
