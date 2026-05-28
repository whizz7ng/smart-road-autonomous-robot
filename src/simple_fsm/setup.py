from setuptools import find_packages, setup

package_name = 'simple_fsm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ws',
    maintainer_email='ws@todo.todo',
    description='Keyboard-driven simple FSM (manual + auto) for cmd_vel testing.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fsm_node = simple_fsm.fsm_node:main',
        ],
    },
)
