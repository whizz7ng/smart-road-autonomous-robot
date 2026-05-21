import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('agv_robot')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'agv_robot_urdf.xacro')
    world_file = os.path.join(pkg_dir, 'worlds', 'agv_track.sdf')

    # xacro → URDF 변환
    robot_description = subprocess.check_output(
        ['xacro', xacro_file]
    ).decode('utf-8')

    # robot_state_publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    # Gazebo Harmonic - 절대경로로 world 지정
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 로봇 스폰 (Gazebo 뜨고 나서)
    spawn_entity = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', 'robot_description',
                    '-name',  'agv_robot',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.05',
                ],
                output='screen'
            )
        ]
    )

    # ROS ↔ Gazebo 브릿지
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rsp_node,
        spawn_entity,
        bridge,
    ])