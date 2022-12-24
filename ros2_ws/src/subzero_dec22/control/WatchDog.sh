#!/bin/bash

function PyCheck
    {
        if pgrep -f "RunRover.py" &>/dev/null; then
            # echo "Python RunRover is already running"
            exit
        else
            echo "Initiating Python RunRover"
            python3 ~/RunRover.py
        fi
    }

while true;
    do PyCheck & sleep 10; done