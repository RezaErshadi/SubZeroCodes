from launch import LaunchDescription
from launch_ros.actions import Node

# AllNodes = ['LogWriter',
#             'Radio',
#             'PingPongRadio',
#             'Arduino',
#             'WriteSpeed',
#             'ManualDrive',
#             'AutoDrive',
#             'ApRES',
#             'Garmin',
#             'Trimble',
#             'Telemetry']

def generate_launch_description():
    ld = LaunchDescription()
    radionode = Node(
        package='subzero_dec22',
        executable='Radio',
        respawn=True,
        respawn_delay=2
    )
    trimblenode = Node(
        package='subzero_dec22',
        executable='Trimble',
        respawn=True,
        respawn_delay=1
    )
    garminnode = Node(
        package='subzero_dec22',
        executable='Garmin',
        respawn=True,
        respawn_delay=1
    )
    arduinonode = Node(
        package='subzero_dec22',
        executable='Arduino',
        respawn=True,
        respawn_delay=1
    )
    telemetrynode = Node(
        package='subzero_dec22',
        executable='Telemetry',
        respawn=True,
        respawn_delay=1
    )
    apresnode = Node(
        package='subzero_dec22',
        executable='ApRES',
        respawn=True,
        respawn_delay=1
    )
    manualdrivenode = Node(
        package='subzero_dec22',
        executable='ManualDrive',
        respawn=True,
        respawn_delay=1
    )
    autodrivenode = Node(
        package='subzero_dec22',
        executable='AutoDrive',
        respawn=True,
        respawn_delay=1
    )
    writespeednode = Node(
        package='subzero_dec22',
        executable='WriteSpeed',
        respawn=True,
        respawn_delay=1
    )
    # ld.add_action(radionode)
    ld.add_action(arduinonode)
    ld.add_action(trimblenode)
    ld.add_action(garminnode)
    ld.add_action(apresnode)
    ld.add_action(manualdrivenode)
    ld.add_action(autodrivenode)
    ld.add_action(telemetrynode)
    ld.add_action(writespeednode)
    return ld