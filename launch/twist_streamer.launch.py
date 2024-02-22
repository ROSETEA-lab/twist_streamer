import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    node=Node(
        package = 'twist_streamer',
        name = 'twist_streamer',
        executable = 'twist_streamer',
        namespace = '',
        parameters = [
            {"csv_filename": os.path.join(get_package_share_directory('twist_streamer'),'data','data.csv')}
        ]
    )
    
    ld.add_action(node)
    return ld

