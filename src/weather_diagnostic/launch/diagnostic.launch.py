from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os import path

import launch


def generate_launch_description():
    params_filepath = path.join(get_package_share_directory('weather_diagnostic'), "weather_status.yaml")
    aggregator = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        output='screen',
        parameters=[params_filepath],)
    
    weather_diagnostic = Node(
        package='weather_diagnostic',
        executable="weather_diagnostic",
        parameters=[params_filepath],)
    
    return LaunchDescription([
        aggregator,
        weather_diagnostic,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=aggregator,
                on_exit=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown())],
            )),
    ])
