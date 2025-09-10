from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Generates the main launch description to bring up the entire system.
    This launch file is compatible with ROS2 Foxy.
    """
    cobot_simulation_pkg_path = get_package_share_directory('cobot_simulation')

    # Include the simulation launch file, which will handle the simulation nodes
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cobot_simulation_pkg_path, 'launch', 'simulation.launch.py')
        )
    )

    # Launch the main control node in its own terminal
    control_node = Node(
        package='cobot_control',
        executable='control_node',
        name='control_node',
        output='screen',
        prefix='gnome-terminal --',
    )

    return LaunchDescription([
        simulation_launch,
        control_node
    ])

