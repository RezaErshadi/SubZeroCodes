#!/bin/bash

touch 'afterscreenstart.txt'

source /opt/ros/foxy/setup.sh
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source ~/ros2_ws/install/setup.sh
source ~/ros2_ws/install/local_setup.sh

touch beforerosstart.txt

ros2 run subzero_v1 LogWriter

touch ihavestarted0.txt
