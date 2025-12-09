import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'gravity_compensation'
    urdf_file = 'my_robot.urdf'

    # 1. URDF 파일 경로 잡기
    pkg_share = get_package_share_directory(pkg_name)
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file)

    # 2. URDF 내용 읽기
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # [핵심] Robot State Publisher: URDF를 읽어서 좌표계(TF)를 뿌려줌
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),

        # [우리 노드] Gravity Node (Monitor Mode여야 함!)
        Node(
            package=pkg_name,
            executable='gravity_node',
            name='gravity_node',
            output='screen',
        ),

        # [시각화] RViz2 자동 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])