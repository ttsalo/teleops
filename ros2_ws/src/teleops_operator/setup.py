from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'teleops_operator'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ttsalo',
    maintainer_email='todo@todo.com',
    description='Operator laptop joystick driver and launch for teleoperated rover',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
