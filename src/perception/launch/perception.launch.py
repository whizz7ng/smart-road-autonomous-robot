#!/usr/bin/env python3
"""
perception.launch.py

카메라 노드와 YOLO 디텍터 노드를 함께 실행하는 런치 파일.

사용 예:
    ros2 launch perception perception.launch.py
    ros2 launch perception perception.launch.py device_id:=2 frame_width:=1280 frame_height:=720
    ros2 launch perception perception.launch.py show_window:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- 런치 인자 (CLI에서 변경 가능) ---
    device_id = LaunchConfiguration('device_id')
    frame_width = LaunchConfiguration('frame_width')
    frame_height = LaunchConfiguration('frame_height')
    fps = LaunchConfiguration('fps')
    image_topic = LaunchConfiguration('image_topic')
    show_window = LaunchConfiguration('show_window')

    declare_device_id = DeclareLaunchArgument(
        'device_id', default_value='0',
        description='카메라 디바이스 번호 (/dev/videoN의 N)',
    )
    declare_frame_width = DeclareLaunchArgument(
        'frame_width', default_value='640',
        description='카메라 프레임 가로 해상도',
    )
    declare_frame_height = DeclareLaunchArgument(
        'frame_height', default_value='480',
        description='카메라 프레임 세로 해상도',
    )
    declare_fps = DeclareLaunchArgument(
        'fps', default_value='30.0',
        description='카메라 프레임레이트',
    )
    declare_image_topic = DeclareLaunchArgument(
        'image_topic', default_value='/camera/image_raw',
        description='카메라 이미지 토픽 이름 (카메라가 발행, YOLO가 구독)',
    )
    declare_show_window = DeclareLaunchArgument(
        'show_window', default_value='true',
        description='YOLO 결과 창 띄울지 여부 (헤드리스면 false)',
    )

    # --- 카메라 노드 ---
    camera_node = Node(
        package='perception',
        executable='camera_node',
        name='camera_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'device_id': device_id,
            'frame_width': frame_width,
            'frame_height': frame_height,
            'fps': fps,
            'topic_name': image_topic,
            'frame_id': 'camera_link',
        }],
    )

    # --- YOLO 디텍터 노드 ---
    yolo_detector = Node(
        package='perception',
        executable='yolo_detector',
        name='yolo_detector',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'show_window': show_window,
            'image_topic': image_topic,
        }],
    )

    return LaunchDescription([
        declare_device_id,
        declare_frame_width,
        declare_frame_height,
        declare_fps,
        declare_image_topic,
        declare_show_window,
        LogInfo(msg='[perception.launch] camera_node + yolo_detector 시작'),
        camera_node,
        yolo_detector,
    ])
