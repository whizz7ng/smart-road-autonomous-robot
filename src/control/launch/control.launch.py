#!/usr/bin/env python3
"""
control 패키지 통합 런치 (bridge 포함, 순차 기동)
기동 순서:
    1) bridge_node, fsm_node 즉시 시작
    2) STARTUP_DELAY 초 뒤 control_node 시작
       -> bridge 가 시리얼 열고 /wheel_rpm 흘리기 시작한 뒤 PID 가 붙도록

PID 게인 / 지연 시간은 런치 인자로 조정:
    ros2 launch control control.launch.py kp:=1.0 startup_delay:=3.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    startup_delay = LaunchConfiguration('startup_delay')

    # --- 1) 먼저 올라가는 노드들 ---
    bridge_node = Node(
        package='serial_bridge',         
        executable='bridge_node',
        name='bridge_node',
        output='screen',
    )

    fsm_node = Node(
        package='control',
        executable='fsm_node',
        name='fsm_node',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('startup_delay', default_value='2.0',
                              description='bridge 기동 후 control_node 시작까지 대기(초)'),

        bridge_node,
        fsm_node,
    ])
