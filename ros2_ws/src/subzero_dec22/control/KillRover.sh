#!/bin/bash





kill $(pgrep -f LogWriter.py)
kill $(pgrep -f Radio.py)
kill $(pgrep -f PingPongRadio.py)
kill $(pgrep -f Arduino.py)
kill $(pgrep -f WriteSpeed.py)
kill $(pgrep -f ManualDrive.py)
kill $(pgrep -f AutoDrive.py)
kill $(pgrep -f ApRES.py)
kill $(pgrep -f Garmin.py)
kill $(pgrep -f Trimble.py)
kill $(pgrep -f Telemetry.py)









echo $(ros2 node list)

# Radio=$(pgrep -f 'Radio')
# Arduino=$(pgrep -f 'Arduino')
# Trimble=$(pgrep -f 'Trimble')
# Garmin=$(pgrep -f 'Garmin')
# ApRES=$(pgrep -f 'ApRES')
# ManualDrive=$(pgrep -f 'ManualDrive')
# AutoDrive=$(pgrep -f 'AutoDrive')
# Telemetry=$(pgrep -f 'Telemetry')
# WriteSpeed=$(pgrep -f 'WriteSpeed')

# function CheckNodes
#     {
#         Nodes=$(ros2 node list)
#     }

# if pgrep -f "WatchDog.sh" &>/dev/null; then
#     kill $(pgrep -f "WatchDog.sh")
#     exit
# fi

# if pgrep -f "RunRover.py" &>/dev/null; then
#     kill $(pgrep -f "RunRover.py")
#     exit
# fi

# if $Radio &>/dev/null; then
#     kill $Radio
#     exit
# fi

# if $Arduino &>/dev/null; then
#     kill $Arduino
#     exit
# fi

# if $Trimble &>/dev/null; then
#     kill $Trimble
#     exit
# fi

# if $Garmin &>/dev/null; then
#     kill $Garmin
#     exit
# fi

# if $ManualDrive &>/dev/null; then
#     kill $ManualDrive
#     exit
# fi

# if $ApRES &>/dev/null; then
#     kill $ApRES
#     exit
# fi

# if $AutoDrive &>/dev/null; then
#     kill $AutoDrive
#     exit
# fi

# if $Telemetry &>/dev/null; then
#     kill $Telemetry
#     exit
# fi

# if $WriteSpeed &>/dev/null; then
#     kill $WriteSpeed
#     exit
# fi