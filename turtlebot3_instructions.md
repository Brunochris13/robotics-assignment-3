# Turtlebot3 General Instructions

Before running any command you need to export TURTLEBOT3_MODEL for each terminal that will use a node from a turtlebot3 package.
It can take value burger, waffle and waffle_pi. (I personally liked waffle)
```
export TURTLEBOT3_MODEL=waffle
```

To launch turtlebot3 in an empty world in Gazebo
```
roscore
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
or
roslaunch turtlebot3_gazebo turtlebot3_world.launc
```

If you want to try the built-in Navigation node run in a new terinal
```
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
```

