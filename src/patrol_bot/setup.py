from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'patrol_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arthur',
    maintainer_email='arthurgb@al.insper.edu.br',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'patrol_node = patrol_bot.patrol_node:main',
            'rotate_node = patrol_bot.rotate_node:main',
            'start_patrol = patrol_bot.start_patrol:main',
        ],
    },
)

