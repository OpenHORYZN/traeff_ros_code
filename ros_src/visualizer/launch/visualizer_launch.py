from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Optional: Path to a specific RViz configuration file
    rviz_config_dir = os.path.join(
        get_package_share_directory('visualizer'),
        'rviz',
        'visualizer.rviz')
    
    map_saver_node = Node(
        package='visualizer',
        executable='map_exporter',
        name='map_exporter'
    )

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir], # Pass config file
            output='screen'),
        map_saver_node
    ])