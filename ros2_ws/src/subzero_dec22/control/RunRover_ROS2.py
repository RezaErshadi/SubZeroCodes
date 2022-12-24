import subprocess
import time
import datetime
import logging
import os

def InitiateAllNodes(AllNodes,logger):
    logger.info("Initiating all nodes")
    for i in range(len(AllNodes)):
        n = AllNodes[i]
        print(f"Initiating: {n}")
        subprocess.Popen(f"ros2 run subzero_v1 {n}",shell=True,stdin=None,stdout=None,stderr=None,close_fds=True)
        time.sleep(0.5)

def ActiveNodes():
    res = subprocess.Popen("ros2 node list",shell=True,stdin=None,stdout=subprocess.PIPE,stderr=None,close_fds=True)
    Nodes = res.stdout.read().decode('utf-8')
    Nodes = Nodes.replace('\n','')
    Nodes = Nodes.split('/')
    Nodes = Nodes[1:]
    return Nodes

def PrintListOfNodesStatus(AllNodes,logger):
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>")
    print(f"ACTIVE NODES @ {datetime.datetime.now()}")
    logger.info("List Of Active Nodes:")
    print("<<<<<<<<<<<<<<<<<<<<<<<<<<")
    Nodes = ActiveNodes()
    for i in range(len(Nodes)):
        n = AllNodes[i]
        chk = Nodes.count(n)
        print(f"{n}: {chk}")
        logger.info(f"{n}: {chk}")
    print("-----------------------------")

def RunNewNode(n,nfalse,logger):
    nfalse += 1
    print(f"@ {datetime.datetime.now()}")
    print(f"Node {n} is missing")
    logger.warning(f"Node {n} is missing")
    PrintListOfNodesStatus(AllNodes,logger)
    print(f"Trying to rerun {n}")
    logger.info(f"Trying to rerun {n}")
    subprocess.Popen(f"ros2 run subzero_v1 {n}",shell=True,stdin=None,stdout=None,stderr=None,close_fds=True)
    time.sleep(2)
    PrintListOfNodesStatus(AllNodes,logger)
    return nfalse

def DeltaTime(t0):
    now =  datetime.datetime.now()
    d = now - t0
    return d.seconds + (d.microseconds)/1000000


now = datetime.datetime.now()
Ros2LogFolder = 'Ros2LogFiles/Ros2NodesBehavior/'
nf = len(os.listdir(Ros2LogFolder))
LogFileName = f"{nf+1}_NBsubzero_{now.strftime('%m-%d-%Y_%H-%M-%S')}.txt"
LogFilePath = Ros2LogFolder + LogFileName
logging.basicConfig(filename=LogFilePath, filemode='w', format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger=logging.getLogger()
logger.setLevel(logging.DEBUG)
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

InitiateAllNodes(AllNodes,logger)
time.sleep(2)    

PrintListOfNodesStatus(AllNodes,logger)

while True:
    time.sleep(2)
    if DeltaTime(now) >= 1:
        nfalse = 0
        now = datetime.datetime.now()
        Nodes = ActiveNodes()
        for i in range(len(AllNodes)):
            n = AllNodes[i]
            chk = Nodes.count(n)
            # print(f"{n}: {chk}")
            if chk == 0:
                nfalse = RunNewNode(n,nfalse,logger)
            elif chk > 1:
                NewChk = chk
                while NewChk > 0:
                    print(f"@ {datetime.datetime.now()}")
                    print(f"Repeated Node: {NewChk} times {n}, trying to kill them")
                    logger.error(f"Repeated Node: {NewChk} times {n}, trying to kill them")
                    subprocess.Popen(f"killall {n}",shell=True,stdin=None,stdout=None,stderr=None,close_fds=True)
                    print(f"Killing {n} :o")
                    logger.warning(f"Killing {n} :o")
                    time.sleep(20)
                    NewNodes = ActiveNodes()
                    NewChk = NewNodes.count(n)
                if NewChk == 0:
                    print(f"@ {datetime.datetime.now()}")
                    print(f"All {n} nodes are dead, Trying a fresh run")
                    logger.warning(f"All {n} nodes are dead, Trying a fresh run")
                    nfalse = RunNewNode(n,nfalse,logger)
        # if nfalse == 0:
        #     print(f"{datetime.datetime.now()} All nodes are running")
        # print("-----------------------------")