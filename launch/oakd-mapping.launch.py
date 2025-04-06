#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions

def generate_launch_description():
    # Launch DepthAI ROS driver
    depthai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('depthai_ros_driver'),
            'launch',
            'camera.launch.py'
        )),
        launch_arguments={
            'enable_depth': 'true',
            'enable_imu': 'true'
        }.items()
    )

    # RTAB-Map Visual Odometry Node (correct executable from rtabmap_odom)
    vo_node = launch_ros.actions.Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[
            {'subscribe_depth': True,
             'subscribe_imu': True,
             'approx_sync': True,
             'approx_sync_max_interval': 10.2,  # Increase sync tolerance
             'frame_id': 'oak-d-base-frame',
             'odom_frame_id': 'odom',
             'publish_tf': True,          # Explicitly publish odom->base_frame transform
             'use_sim_time': False}
        ],
        remappings=[
            ('rgb/image', '/oak/rgb/image_raw'),
            ('depth/image', '/oak/stereo/image_raw'),
            ('rgb/camera_info', '/oak/rgb/camera_info'),
            ('imu', '/oak/imu/data'),
            ('odom', '/odom')  # Publishes the odometry data
        ]
    )

    # RTAB-Map Node (SLAM) that uses odometry from rgbd_odometry
    rtabmap_node = launch_ros.actions.Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            {'subscribe_depth': True,
             'approx_sync': True,
             'frame_id': 'oak-d-base-frame',
             'subscribe_odom': True,
             'odom_frame_id': 'odom',
             'use_sim_time': False}
        ],
        remappings=[
            ('rgb/image', '/oak/rgb/image_raw'),
            ('depth/image', '/oak/stereo/image_raw'),
            ('rgb/camera_info', '/oak/rgb/camera_info'),
            ('odom', '/odom')
        ]
    )

    return LaunchDescription([
        depthai_launch,
        vo_node,
        rtabmap_node
    ])
