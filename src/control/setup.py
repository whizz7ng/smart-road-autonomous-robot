from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jsdbswjd',
    maintainer_email='jsdbswjd@todo.todo',
    description='Keyboard-driven simple FSM (manual + auto) for cmd_vel testing.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = control.control_node:main',
            'fsm_node = control.fsm_node:main',
        ],
    },
)
