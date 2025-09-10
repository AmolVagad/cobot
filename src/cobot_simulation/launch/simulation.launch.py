from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches the two simulation nodes.
    - The proximity sensor runs in the background.
    - The emergency stop node gets its own terminal for keyboard input.
    """
    return LaunchDescription([
        # Proximity sensor simulator node runs in the background
        Node(
            package='cobot_simulation',
            executable='proximity_sensor_node',
            name='proximity_sensor_node',
            output='screen',
        ),
        # Emergency stop simulator node gets its own terminal for user input
        Node(
            package='cobot_simulation',
            executable='emergency_stop_node',
            name='emergency_stop_node',
            output='screen',
            prefix='gnome-terminal --',
        ),
    ])

