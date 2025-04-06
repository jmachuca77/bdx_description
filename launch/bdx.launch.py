from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.substitutions import LaunchConfiguration
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

    launch_rviz = LaunchConfiguration('launch_rviz')
    use_joy_bridge = LaunchConfiguration('use_joy_bridge')

    puddleduck_control_package_share = get_package_share_directory('puddleduck_control')
    puddleduck_control_config_file = os.path.join(puddleduck_control_package_share, 'config', 'puddleduck_control.yaml')

    if not os.path.exists(puddleduck_control_config_file):
        print("Config file not found: {}".format(puddleduck_control_config_file))
    else:
        print("Using config file: {}".format(puddleduck_control_config_file))


    # Read the URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    declare_foxglove_bridge_arg = DeclareLaunchArgument('use_foxglove_bridge', default_value='true')
    declare_joy_bridge_arg = DeclareLaunchArgument('use_joy_bridge', default_value='true', description='Set to "true" to launch joy_serial_bridge_node')
    declare_rviz_cmd = DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Set to "true" to launch RViz')

    # Pass the robot_description parameter
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Conditionally launch RViz if 'launch_rviz' is true
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(launch_rviz),
        )

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='log',  # Redirect output to log
        arguments=['--ros-args', '--log-level', 'fatal'],  # Only show error logs
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'tls': False,
            'certfile': '',
            'keyfile': '',
            'topic_whitelist': ['.*'],
            'param_whitelist': ['.*'],
            'service_whitelist': ['.*'],
            'client_topic_whitelist': ['.*'],
            'min_qos_depth': 1,
            'max_qos_depth': 10,
            'num_threads': 0,
            'send_buffer_limit': 50000000,
            'use_sim_time': False,
            'capabilities': ['clientPublish', 'parameters', 'parametersSubscribe', 'services', 'connectionGraph', 'assets'],
            'include_hidden': False,
            'asset_uri_allowlist': ['^package://(?:\\w+/)*\\w+\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$']
        }]
    )

    puddleduck_control_node = Node(
        package='puddleduck_control',
        executable='puddleduck_control_node',
        name='puddleduck_control',
        output='screen',
        parameters=[puddleduck_control_config_file],
        # condition=IfCondition(use_joy_bridge)
        # Uncomment the next line to set the log level to debug:
        # arguments=['--ros-args', '--log-level', 'debug']
    )

    xbee_joy_node = Node(
        package='puddleduck_control',
        executable='xbee_joy_node',
        name='xbee_joy_node',
        output='screen',
        parameters=[puddleduck_control_config_file]
    )

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('bdx_description'),
                'config', 'apm_config.yaml'
            ),
            {
                'fcu_url': 'udp://:11000@',
                'gcs_url': 'udp://@',  # Adjust or remove if unused
            }
        ]
    )

    madgwick_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[
            {'use_mag': False,  # set True if your IMU has magnetometer data
            'publish_tf': True,
            'fixed_frame': 'odom',  # or 'map' if no odometry yet
            'imu_frame': 'imu_link',
            'world_frame': 'enu',   # adjust if your IMU uses NED or other
            'gain': 0.01  # Default: 0.1, try values like 0.05, 0.01, or even smaller
            }
        ],
        remappings=[
            ('imu/data_raw', '/mavros/imu/data'),  # input from mavros
            ('imu/data', '/imu/data_filtered')         # output filtered data
        ]
    )

    return LaunchDescription([
        declare_foxglove_bridge_arg,
        declare_joy_bridge_arg,
        declare_rviz_cmd,
        robot_state_publisher_node,
        puddleduck_control_node,
        xbee_joy_node,
        rviz_node,
        mavros_node,
        madgwick_node,
        foxglove_bridge_node
    ])
