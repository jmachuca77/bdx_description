# BD-X Robot Description

This package contains the URDF description of the BD-X robot.

## Overview

The `bdx_description` package provides the URDF model for the BD-X robot, including its physical and visual properties.

## Usage

Usage:

To launch without RViz (default behavior):
```bash
ros2 launch your_package your_launch_file.launch.py
```

To launch with RViz:
```bash
ros2 launch your_package your_launch_file.launch.py launch_rviz:=true
```

## Installation

Clone this package into your ROS2 workspace and build it:

```bash
cd ~/ros2_ws/src
git clone <repository_url>
cd ~/ros2_ws
colcon build
```

## Dependencies

- ROS2 Foxy (or later)
- `urdf` package
- `robot_state_publisher`
- `rviz2`

