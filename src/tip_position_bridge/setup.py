from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tip_position_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mywstc3',
    maintainer_email='a2830189086@qq.com',
    description='ROS 2 bridge for tip position communication via Rosbridge',
    license='Apache-2.0',  # 修改为适当的许可证
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tip_position_client = tip_position_bridge.tip_position_client:main',
        ],
    },
)