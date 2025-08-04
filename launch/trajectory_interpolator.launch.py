#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get the package directory
    pkg_dir = get_package_share_directory('traj_interp')
    
    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'trajectory_interpolator.yaml')
    
    # Trajectory Interpolator Node
    trajectory_interpolator_node = Node(
        package='traj_interp',
        executable='trajectory_interpolator',
        name='trajectory_interpolator',
        output='screen',
        parameters=[config_file],
        remappings=[
            # Add remappings if needed
        ]
    )
    
    return LaunchDescription([
        trajectory_interpolator_node
    ])
