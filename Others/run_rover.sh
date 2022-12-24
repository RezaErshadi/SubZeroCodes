#!/bin/bash

cd /home/subzero

# sudo screen -s /home/subzero/rover_startup.sh

# touch 'afterscreenstart.txt'

source /opt/ros/foxy/setup.sh
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source ~/ros2_ws/install/setup.sh
source ~/ros2_ws/install/local_setup.sh

# touch beforerosstart.txt

# ros2 run subzero_v1 LogWriter
python3 ctl_run.py

# touch ihavestarted0.txt
