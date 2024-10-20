from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Đường dẫn tới file YAML
    param_file_path = os.path.join(
        get_package_share_directory('param_file_loader'),
        'param',
        'params.yaml'
    )

    # Tạo node và nạp tham số từ file YAML
    return LaunchDescription([
        Node(
            package='param_file_loader',  # Tên package
            executable='param_loader_node',  # Tên executable của node
            name='param_loader_node',
            parameters=[param_file_path]  # Nạp tham số từ file YAML
        )
    ])
