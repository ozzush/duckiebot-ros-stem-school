#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun camera src/camera_reader.py

# wait for app to end
dt-launchfile-join
