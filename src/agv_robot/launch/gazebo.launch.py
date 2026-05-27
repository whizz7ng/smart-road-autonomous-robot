import os
import subprocess
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import math


def generate_launch_description():
    pkg_dir = get_package_share_directory('agv_robot')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'agv_robot_urdf.xacro')
    world_file = os.path.join(pkg_dir, 'worlds', 'agv_track.sdf')

    pkg_share = get_package_share_directory('agv_robot')
    models_path = os.path.join(pkg_share, 'models')
    plugin_path = os.path.join(get_package_prefix('traffic_light_plugin'), 'lib')

    # 환경변수 dict (자식 프로세스에 명시적으로 전달)
    gz_env = {
        **os.environ,  # 기존 환경 유지
        'GZ_SIM_RESOURCE_PATH': models_path
            + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        'GZ_SIM_SYSTEM_PLUGIN_PATH': plugin_path
            + os.pathsep + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
    }

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

    # ★ gz sim 직접 실행 (환경변수 명시 전달)
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_file],
        output='screen',
        additional_env=gz_env,
    )

    # 로봇 스폰
    spawn_entity = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', 'robot_description',
                    '-name', 'agv_robot',
                    '-x', '0.65',
                    '-y', '-0.2',
                    '-z', '0.01',
                    '-R', '0',
                    '-P', '0',
                    '-Y', str((math.pi)/2),
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