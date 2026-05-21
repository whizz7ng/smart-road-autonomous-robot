"""
bringup.launch.py - 전체 시스템 통합 런처

사용법:
  # 시뮬레이션 (Gazebo)
  ros2 launch system bringup.launch.py mode:=sim

  # 실물 (RPi4 + ESP32)
  ros2 launch system bringup.launch.py mode:=real

작성: Eng A (Tech Lead)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    # ===== Launch 인자 =====
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        description='실행 모드: "sim" (Gazebo 시뮬레이션) 또는 "real" (실물 RPi)'
    )

    mode = LaunchConfiguration('mode')

    # ===== 공통 노드 (sim/real 양쪽 모두 실행) =====
    # TODO: Leader (Perception) - YOLOv8 추론 노드 등록 예정
    perception_node = Node(
        package='perception',
        executable='yolo_detector',
        name='yolo_detector',
        output='screen',
        condition=IfCondition('true'),  # 노드 개발 완료 후 'true'로 변경
    )

    # TODO: Eng B (Control) - FSM/PID 제어 노드 등록 예정
    control_node = Node(
        package='control',
        executable='driving_fsm',
        name='driving_fsm',
        output='screen',
        condition=IfCondition('false'),  # 노드 개발 완료 후 'true'로 변경
    )

    # ===== Sim 모드 전용 (Gazebo) =====
    # TODO: Eng C (Simulation) - Gazebo bringup 등록 예정
    sim_group = GroupAction(
        actions=[
            LogInfo(msg='[bringup] === SIM MODE: Gazebo 시뮬레이션 실행 ==='),
            # Node(
            #     package='simulation',
            #     executable='gazebo_bringup',
            #     name='gazebo_bringup',
            #     output='screen',
            # ),
        ],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('mode'), 'sim')),
    )

    # ===== Real 모드 전용 (실물 H/W) =====
    # TODO: Eng A (System) - 시리얼 브릿지 노드 등록 예정
    real_group = GroupAction(
        actions=[
            LogInfo(msg='[bringup] === REAL MODE: 실물 로봇 (RPi+ESP32) 실행 ==='),
            # Node(
            #     package='system',
            #     executable='serial_bridge',
            #     name='serial_bridge',
            #     output='screen',
            # ),
        ],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('mode'), 'real')),
    )

    return LaunchDescription([
        mode_arg,
        LogInfo(msg=['[bringup] 시작 모드: ', mode]),
        perception_node,
        control_node,
        sim_group,
        real_group,
    ])
