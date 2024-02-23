import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    ld = LaunchDescription()
    
    pkg_name = 'twist_streamer'
    csvfile_name = 'data.csv'
    
    twist_streamer_params = RewrittenYaml(
        source_file = os.path.join(get_package_share_directory(pkg_name),'config', "twist_streamer_params.yaml"),
        root_key = '',
        param_rewrites = {
            "csv_filename": PathJoinSubstitution(
                [get_package_share_directory(pkg_name),'data',csvfile_name]
            )
        },
        convert_types=True
    )
    
    twist_streamer_node = Node(
        package = pkg_name,
        name = pkg_name,
        executable = pkg_name,
        namespace = '',
        parameters = [twist_streamer_params],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),	# Add a remapping if it is needed
        ]
    )
    
    ld.add_action(twist_streamer_node)
    return ld

