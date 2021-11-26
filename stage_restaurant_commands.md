# Stage Restaurant Useful Commands

### To run stage with the restaurant world file
```
rosrun stage_ros stageros src/waiter_robot/wlds/stage_restaurant.world
```

### To move the robot using the keyboard
```
roslaunch socspioneer keyboard_teleop.launch
```

### Rviz
```
rosrun rviz rviz
```

### Amcl
```
rosrun amcl amcl scan:=base_scan
```

### Map Server
```
rosrun map_server map_server src/waiter_robot/maps/stage_restaurant.yaml
```

## For Navigation
It is move_base, you can look at amcl.launch how it calls move_base and about the param files in the config directory.