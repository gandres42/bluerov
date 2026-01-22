#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    sonar_arg = DeclareLaunchArgument(
        'sonar',
        default_value='true',
        description='Enable the sonar node'
    )
    
    viewer_arg = DeclareLaunchArgument(
        'viewer',
        default_value='true',
        description='Enable the oculus viewer node'
    )

    # Get the package share directory for parameters
    pkg_share = FindPackageShare('sonar_oculus')
    
    # Sonar oculus driver node
    sonar_node = Node(
        package='sonar_oculus',
        executable='sonar_oculus_node',
        name='sonar_oculus_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'sonar_params.yaml'])
        ],
        condition=IfCondition(LaunchConfiguration('sonar'))
    )
    
    # Oculus viewer node
    viewer_node = Node(
        package='sonar_oculus',
        executable='oculus_viewer.py',
        name='oculus_viewer',
        output='screen',
        parameters=[{
            'raw': False,
            'colormap': 2
        }],
        condition=IfCondition(LaunchConfiguration('viewer'))
    )
    
    # Static transform publisher (sonar to base_link)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sonar_base_link',
        arguments=['0.2', '0', '0.15', '0', '0', '0', 'base_link', 'sonar']
    )

    return LaunchDescription([
        sonar_arg,
        viewer_arg,
        sonar_node,
        viewer_node,
        static_tf_node,
    ])
