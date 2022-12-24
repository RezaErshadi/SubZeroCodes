from setuptools import setup
import os
from glob import glob

package_name = 'subzero_dec22'
submodules = "subzero_dec22/submodules"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RezaErshadi',
    maintainer_email='mohammadreza.ershadi@uni-tuebingen.de',
    description='subzero control system',
    license='Nothing',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Radio = subzero_dec22.Radio:main',
            'Arduino = subzero_dec22.Arduino:main',
            'Trimble = subzero_dec22.Trimble:main',
            'Garmin = subzero_dec22.Garmin:main',
            'ApRES = subzero_dec22.ApRES:main',
            'ManualDrive = subzero_dec22.ManualDrive:main',
            'AutoDrive = subzero_dec22.AutoDrive:main',
            'Telemetry = subzero_dec22.Telemetry:main',
            'WriteSpeed = subzero_dec22.WriteSpeed:main',
            'LogWriter = subzero_dec22.LogWriter:main',
            'PingPongRadio = subzero_dec22.PingPongRadio:main'
        ],
    },
)
