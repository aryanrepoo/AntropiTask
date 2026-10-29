import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'antropi_amr_sim'

setup(
    name=package_name,
    version='0.1.0',
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
    maintainer='Aryan',
    maintainer_email='aryanrajcontact@gmail.com',
    description='Autonomous CNC Factory Material Handler using TurtleBot3 and Nav2 in ROS 2 Jazzy',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'factory_dispatcher = antropi_amr_sim.factory_dispatcher:main',
            'factory_telemetry_hud = antropi_amr_sim.factory_telemetry_hud:main',
        ],
    },
)
