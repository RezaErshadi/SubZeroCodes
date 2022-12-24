import subprocess
import time
import os

def InitiateAllNodes(AllNodes):
    for i in range(len(AllNodes)):
        n = AllNodes[i]
        print(f"Initiating: {n}")
        subprocess.Popen(f"python3 {n}.py",shell=True,stdin=None,stdout=None,stderr=None,close_fds=True)
        time.sleep(1)

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
InitiateAllNodes(AllNodes)