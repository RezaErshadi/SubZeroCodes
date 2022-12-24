import subprocess
import time
import datetime

def InitiateAllNodes(AllNodes):
    for i in range(len(AllNodes)):
        n = AllNodes[i]
        print(f"Initiating: {n}")
        rws = "/home/subzero/ros2_ws/src/subzero_v1/subzero_v1/"
        subprocess.Popen(f"python3 {rws}{n}.py",shell=True,stdin=None,stdout=None,stderr=None,close_fds=True)
        time.sleep(0.5)

#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
AllNodes = ['LogWriter',
            'Radio',
            'PingPongRadio',
            'Arduino',
            'WriteSpeed',
            'ManualDrive',
            'AutoDrive',
            'ApRES',
            'Garmin',
            'Trimble',
            'Telemetry']

now = datetime.datetime.now()
print(now)
InitiateAllNodes(AllNodes)
while True:
    time.sleep(2)