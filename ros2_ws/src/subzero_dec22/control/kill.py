import subprocess
import os

def ActiveNodes(n):
    # print(f"pgrep -f '{n}'")
    # res = subprocess.Popen(f"pgrep -f {n}.py",shell=True,stdin=None,stdout=subprocess.PIPE,stderr=None,close_fds=True)
    # nID = res.stdout.read().decode('utf-8')
    os.system(f"pgrep -f {n}.py")
    print(os.system(f"pgrep -f {n}.py"))
    # nID = nID.replace('\n','')
    # nID = nID.split('/')
    # nID = nID[1:]
    # print(f"{n}: {nID}")
    # return nID

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

for i in range(len(AllNodes)):
    ActiveNodes(AllNodes[i])
