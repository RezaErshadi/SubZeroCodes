#!/bin/bash

# copy the rover's log file to the operator's computer

destinationUser=subzero
# destinationIP=192.168.30.113
destinationIP=192.168.1.10
sendRemote=$destinationUser@$destinationIP:/

M1Home=/media/psf/Home
OneDrivePath=$M1Home/OneDrive\ -\ bwedu
ROS2WS=$OneDrivePath/WorkTub/Devices/SubZero/Codes/PIO_CodeDevelopment/test/ros2_ws
SourceROS2=$ROS2WS/src

destinationROS2=home/subzero/ros2_ws/src

scp -r $destinationUser@$destinationIP:/home/subzero/ros2lf /home/reza/

# scp -r $destinationUser@$destinationIP:/home/subzero/ros2lf/SubZero/drive /home/reza/ros2lf/SubZero/drive
# scp -r $destinationUser@$destinationIP:/home/subzero/ros2lf/SubZero/nodes /home/reza/ros2lf/SubZero/nodes
# scp -r $destinationUser@$destinationIP:/home/subzero/ros2lf/SubZero/apres /home/reza/ros2lf/SubZero/apres

