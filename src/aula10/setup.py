from setuptools import find_packages, setup

package_name = 'aula10'

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
            'action_server = aula10.action_server:main',
            'action_client = aula10.action_client:main',
        ],
    },
)
