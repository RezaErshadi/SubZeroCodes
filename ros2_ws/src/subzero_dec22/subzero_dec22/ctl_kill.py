import subprocess

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

subprocess.Popen(f"kill $(pgrep -f ctl_run.py)",shell=True,stdin=None,stdout=None,stderr=None,close_fds=True)

for i in range(len(AllNodes)):
    n = AllNodes[i]
    subprocess.Popen(f"kill $(pgrep -f {n}+.py)",shell=True,stdin=None,stdout=None,stderr=None,close_fds=True)

subprocess.Popen(f"kill $(pgrep -f ctl_run.py)",shell=True,stdin=None,stdout=None,stderr=None,close_fds=True)