from setuptools import find_packages, setup

package_name = 'location_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mywstc3',
    maintainer_email='a2830189086@qq.com',
    description='Local CLI publisher for /location_target (target end-effector position).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'location_target_publisher = location_publisher.location_target_publisher:main',
        ],
    },
)
