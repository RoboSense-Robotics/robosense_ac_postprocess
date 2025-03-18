from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('robosense_ac_postprocess')
    rviz_config_file = os.path.join(
        package_dir,
        'rviz2_config',
        'rviz2_config.rviz'
    )
    config_file = os.path.join(package_dir, "config", "usr_config.yaml")

    postprocess_node = Node(
        package='robosense_ac_postprocess',
        executable='postprocess_node',
        name='postprocess_node',
        output='screen',
        arguments=['--config', config_file]
    )
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_postprocess',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    ld = LaunchDescription()
    ld.add_action(postprocess_node)
    ld.add_action(rviz2_node)
    return ld