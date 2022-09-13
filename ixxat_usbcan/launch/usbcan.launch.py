import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'ixxat_usbcan',
            executable = 'usbcan_node',
            output = 'screen',
            parameters = [
                {"port_name": "can0"},
                {"topic_rx": "can_rx"},
                {"topic_tx": "can_tx"}
            ]
        )
    ])