from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ssafy_bridge",
                node_executable="udp_to_pub",
                node_name="udp_to_pub",
                # output="screen", # ✅ DEBUG 시 로그 확인
            ),
            Node(
                package="ssafy_bridge",
                node_executable="sub_to_udp",
                node_name="sub_to_udp",
                # output="screen", # ✅ DEBUG 시 로그 확인
            ),
            Node(
                package="ssafy_bridge",
                node_executable="udp_to_cam",
                node_name="udp_to_cam",
                # output="screen", # ✅ DEBUG 시 로그 확인
            ),
            Node(
                package="ssafy_bridge",
                node_executable="udp_to_laser",
                node_name="udp_to_laser",
                # output="screen", # ✅ DEBUG 시 로그 확인
            ),
        ]
    )
