# save this as usb_cam_multi.launch.py in your package's launch/ folder

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Camera 0
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam0',
            namespace='camera0',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                # you can also tweak resolution, pixel_format, framerate, etc:
                # 'image_width': 640,
                # 'image_height': 480,
                # 'pixel_format': 'yuyv',
                # 'framerate': 30.0,
            }],
        ),

        # Camera 1
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam1',
            namespace='camera1',
            output='screen',
            parameters=[{
                'video_device': '/dev/video2',
                # match other settings to camera0 if needed
            }],
        ),
    ])
