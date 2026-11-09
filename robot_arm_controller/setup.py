import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_arm_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='SRIVARDHAN S.',
    maintainer_email='you@example.com',
    description='ESP32 6-DOF robot arm controller (ROS 2)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_gui = robot_arm_controller.arm_gui:main',
            'arm_node = robot_arm_controller.arm_node:main',
        ],
    },
)
