import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rm_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lucas',
    maintainer_email='lucas@example.com',
    description='Nav2 navigation stack for the differential robot in Gazebo',
    license='TODO: License declaration',
    entry_points={'console_scripts': []},
)
