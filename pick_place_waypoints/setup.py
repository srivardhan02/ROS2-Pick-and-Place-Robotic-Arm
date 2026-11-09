from setuptools import setup
from glob import glob
import os

package_name = 'pick_place_waypoints'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Srivardhan',
    maintainer_email='you@example.com',
    description='Waypoint GUI and pick-place sequence for a ROS 2 robotic arm',
    license='MIT',
    entry_points={
        'console_scripts': [
            'waypoint_gui = pick_place_waypoints.waypoint_gui:main',
        ],
    },
)
