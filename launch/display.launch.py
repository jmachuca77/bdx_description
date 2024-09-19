from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    package_name = 'bdx_description'
    urdf_file = 'bdx.urdf'
    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        urdf_file)

    # Declare the 'launch_rviz' launch argument with default value 'false'
    launch_rviz = LaunchConfiguration('launch_rviz')

    # Read the URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Set to "true" to launch RViz'),

        # Pass the robot_description parameter
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # Conditionally launch RViz if 'launch_rviz' is true
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(launch_rviz)
        ),
    ])
