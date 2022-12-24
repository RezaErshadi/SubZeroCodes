import subprocess

def ActiveNodes():
    res = subprocess.Popen("ros2 node list",shell=True,stdin=None,stdout=subprocess.PIPE,stderr=None,close_fds=True)
    Nodes = res.stdout.read().decode('utf-8')
    Nodes = Nodes.replace('\n','')
    Nodes = Nodes.split('/')
    Nodes = Nodes[1:]
    return Nodes

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
while True:
    Nodes = ActiveNodes()
    if len(Nodes) == 0:
        print("All nodes are dead")
        break
    else:
        for i in range(len(Nodes)):
            n = Nodes[i]
            print(f"Killing {n}")
            subprocess.Popen(f"killall {n}",shell=True,stdin=None,stdout=None,stderr=None,close_fds=True)