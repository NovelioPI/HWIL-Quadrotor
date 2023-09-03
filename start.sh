#!/bin/bash
sudo chmod a+rw /dev/ttyACM*
if !(echo >/dev/ttyACM*) &>/dev/null
then
    echo "/dev/ttyACM0 is not available"
    exit 1
else
    roslaunch hwil_sim start.launch
fi
