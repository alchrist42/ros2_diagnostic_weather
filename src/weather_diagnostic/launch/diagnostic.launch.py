from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os import path

import launch


def generate_launch_description():
    params_filepath = path.join(get_package_share_directory('weather_diagnostic'), "weather_status.yaml")
    weather_data_filler = Node(
        package='weather_diagnostic',
        executable='weather_data_filler',
        output='screen',)
    
    weather_diagnostic = Node(
        package='weather_diagnostic',
        executable="weather_diagnostic",
        parameters=[params_filepath],)
    
    return LaunchDescription([
        weather_data_filler,
        weather_diagnostic,
    ])
        
