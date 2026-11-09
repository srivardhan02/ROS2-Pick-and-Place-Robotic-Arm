import os
from glob import glob
from setuptools import setup

package_name = 'rviz_arm_sync'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SRIVARDHAN S.',
    maintainer_email='you@example.com',
    description='RViz sliders to real arm',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rviz_to_arm = rviz_arm_sync.rviz_to_arm:main',
            'servo_wifi_bridge = rviz_arm_sync.servo_wifi_bridge:main',
        ],
    },
)
