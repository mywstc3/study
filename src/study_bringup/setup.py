import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'study_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mywstc3',
    maintainer_email='a2830189086@qq.com',
    description=(
        'One-shot launch files for the 5-axis robotic arm stack '
        '(sy4 serial driver + sy5 motor control + sy6 forward kinematics '
        '+ sy7 inverse kinematics + tip_position_bridge WebSocket relay).'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
