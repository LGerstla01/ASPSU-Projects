from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

rviz_config_path = LaunchConfiguration('rviz_config')
num_robots = LaunchConfiguration('num_robots')
num_cameras = LaunchConfiguration('num_cameras')
size_room = LaunchConfiguration('size_room')
FOV = LaunchConfiguration('FOV')

def generate_launch_description():
    return LaunchDescription([
        # Start the visualizer node
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                get_package_share_directory('robot_estimator'),
                'config',
                'visualization.rviz'
            ),
            description='Path to the RViz config file'
        ),

        DeclareLaunchArgument(
            'num_robots',
            default_value='2',
            description='Number of robots'
        ),

        DeclareLaunchArgument(
            'num_cameras',
            default_value='2',
            description='Number of cameras'
        ),

        DeclareLaunchArgument(
            'size_room',
            default_value='(10, 10)',
            description='Size of the room'
        ),

        DeclareLaunchArgument(
            'FOV',
            default_value='40.0',
            description='Field of view of the cameras'
        ),
        
        Node(
            package='robot_estimator',
            executable='VisualizationNode',
            name='VisualizationNode',
            output='screen',
            parameters=[
                {'num_robots': num_robots},
                {'num_cameras': num_cameras},
                {'size_room': size_room},
                {'FOV': FOV},
            ],
        ),

        Node(
            package='robot_estimator',
            executable='robotControllerNode',
            name='robotControllerNode',
            output='screen',
            parameters=[
                {'num_robots': num_robots},
                {'size_room': size_room},
            ],
        ),

        Node(
            package='robot_estimator',
            executable='cameraDetectionNode',
            name='cameraDetectionNode',
            output='screen',
            parameters=[
                {'num_cameras': num_cameras},
                {'size_room': size_room},
                {'FOV': FOV},
                {'noise': 0.2},
            ],
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
        ),
    ])

