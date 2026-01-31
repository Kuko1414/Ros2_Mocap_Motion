from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros_action'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kuko',
    maintainer_email='kuko@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'circular_trajectory = ros_action.ros_action:main',
            'send_target = ros_action.send_target:main',
            'lattice_follower = ros_action.lattice_path_follower:main',
            'pid_track = ros_action.pid_track:main',
            'path_generation = ros_action.path_generation:main',
        ],
    },
)
