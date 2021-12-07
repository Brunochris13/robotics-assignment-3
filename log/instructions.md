# Experiments Instructions

## For the Navigation Testing

First in `test_navigation.py` and in `moving.py` turn `self.test_amcl_params` to **False** in BOTH files.

Then in 1 terminal run
```
roslaunch waiter_robot stage_navigation.launch
```
and in a 2nd terminal go to `/src/agent/parts` and run
```
python3 test_navigation.py > test_name.log
```

`test_name` can be anything you want

The above command will redirect all output to a new file called `test_name.log`.

You can then open this file and copy the values to the report.


## For the Noise Parameters and Max/Min num. of particles experiments

First in `test_navigation.py` and in `moving.py` turn `self.test_amcl_params` to **True** in BOTH files.

Then in 1 terminal run
```
roslaunch waiter_robot stage_navigation.launch
```
and in a 2nd terminal go to `/src/agent/parts` and run
```
python3 test_navigation.py > test_name.log
```

`test_name` can be anything you want

The above command will redirect all output to a new file called `test_name.log`.

Then copy this file to `/log`, where the `graph_gen.py` is.

### In graph_gen.py

For every `error_over_time_*` variable change the name of the file to the log file you want to read from.

Change all the titles of the graph as well.
Then you can just run
```
python3 graph_gen.py
```
The image will be saved under `/graph`, which is under `/log`.