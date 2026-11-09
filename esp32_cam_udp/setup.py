import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'esp32_cam_udp'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'firmware', 'esp32_cam_udp'),
         glob('firmware/esp32_cam_udp/*.ino')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ESP32-CAM UDP video receiver for ROS 2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'udp_camera_node = esp32_cam_udp.udp_camera_node:main',
            'udp_test_sender = esp32_cam_udp.udp_test_sender:main',
            'qr_gripper_node = esp32_cam_udp.qr_gripper_node:main',
        ],
    },
)
