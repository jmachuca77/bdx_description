from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    package_name = 'bdx_description'
    urdf_file = 'uki_bdx.urdf'
    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        urdf_file)

    # Declare the 'launch_rviz' launch argument with default value 'false'
    launch_rviz = LaunchConfiguration('launch_rviz')

    joint_state_package_share = get_package_share_directory('serial_joint_state_publisher')
    joint_state_config_file = os.path.join(joint_state_package_share, 'config', 'joint_config.yaml')

    if not os.path.exists(joint_state_config_file):
        print("Config file not found: {}".format(joint_state_config_file))
    else:
        print("Using config file: {}".format(joint_state_config_file))


    # Read the URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    declare_foxglove_bridge_arg = DeclareLaunchArgument('use_foxglove_bridge', default_value='true')

    declare_rviz_cmd = DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Set to "true" to launch RViz')

    declare_motor_config_file_cmd = DeclareLaunchArgument(
        'motor_config_file',
        default_value=os.path.join(
            get_package_share_directory('unitree_motor_controller'),
            'config',
            'unitree_motor_config.yaml'
        ),
        description='Full path to the motor config file to load'
    )

    declare_joint_state_publish_rate_cmd = DeclareLaunchArgument(
            'joint_state_publish_rate',
            default_value='10',
            description='Rate at which to publish joint states')
    
    declare_motor_command_publish_rate_cmd = DeclareLaunchArgument(
            'motor_command_publish_rate',
            default_value='100',
            description='Rate at which to send motor commands')

    motor_controller_node = Node(
        package='unitree_motor_controller',
        executable='motor_controller',
        name='motor_controller',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('motor_config_file'),
            'joint_state_publish_rate': LaunchConfiguration('joint_state_publish_rate'),
            'motor_command_publish_rate': LaunchConfiguration('motor_command_publish_rate'),
        }]
    )

    # Pass the robot_description parameter
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    joint_state_publisher_node = Node(
        package='serial_joint_state_publisher',
        executable='serial_joint_state_publisher',
        name='serial_joint_state_publisher',
        output='screen',
        parameters=[joint_state_config_file],
        # Uncomment the next line to set the log level to debug:
        # arguments=['--ros-args', '--log-level', 'debug']
    )

    # Conditionally launch RViz if 'launch_rviz' is true
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(launch_rviz),
        )
    
    foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([os.path.join(
            get_package_share_directory('foxglove_bridge'), 'launch/'),
            'foxglove_bridge_launch.xml'
        ]),
        condition=IfCondition(LaunchConfiguration('use_foxglove_bridge'))
    )

    return LaunchDescription([
        declare_foxglove_bridge_arg,
        declare_rviz_cmd,
        declare_motor_config_file_cmd,
        declare_joint_state_publish_rate_cmd,
        declare_motor_command_publish_rate_cmd,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        foxglove_bridge
    ])
