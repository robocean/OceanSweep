from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 릴레이 서버로부터 GPS 오차 수신
        #Node(
            #package='arduino_interface',
            #executable='bridge_server_node',
            #name='bridge_server_node',
            #output='screen'
        #),

        # 아두이노에서 raw GPS + Yaw 수신
        Node(
            package='arduino_interface',
            executable='broker_node',
            name='broker_node',
            output='screen'
        ),

        # 보정된 GPS 계산
        #Node(
            #package='arduino_interface',
            #executable='dgps_node',
            #name='dgps_node',
            #output='screen'
        #),

        # WGS84 → TM 좌표 변환
        Node(
            package='arduino_interface',
            executable='tm_converter_node',
            name='tm_converter_node',
            output='screen'
        ),

        #  칼만 필터 적용 노드 추가
        Node(
            package='arduino_interface',
            executable='kalman_node',
            name='kalman_node',
            output='screen'
        ),

        # Waypoint 추종 제어 노드
        Node(
            package='arduino_interface',
            executable='waypoint_node',
            name='waypoint_node',
            output='screen'
        ),

        # 바퀴 모터 속도 명령 노드
        Node(
            package='arduino_interface',
            executable='diff_drive_node',
            name='diff_drive_node',
            output='screen'
        ),
    ])

