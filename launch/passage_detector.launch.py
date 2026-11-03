from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('mid360_passage_detector')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='mid360_passage_detector', executable='passage_detector',
             name='passage_detector', output='screen',
             parameters=[os.path.join(pkg, 'config', 'params.yaml'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
