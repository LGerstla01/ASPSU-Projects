from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

# Declare Launch Arguments at the top for clarity
rviz_config_path = LaunchConfiguration('rviz_config')
num_robots = LaunchConfiguration('num_robots')
num_cameras = LaunchConfiguration('num_cameras')
room_size = LaunchConfiguration('room_size')
FOV = LaunchConfiguration('FOV')
num_lidar = LaunchConfiguration('num_lidar')

def generate_launch_description():
    pkg_name = 'robot_estimator' # Stellen Sie sicher, dass dies Ihr Paketname ist

    # Pfad zum Paket-Share-Verzeichnis (falls für Konfigurationsdateien benötigt)
    pkg_share_dir = get_package_share_directory(pkg_name)

    return LaunchDescription([
        # Declare Launch Arguments
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                get_package_share_directory('robot_estimator'),
                'config',
                'visualization.rviz'),
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
            'num_lidar',
            default_value='1',
            description='Number of lidars'
        ),

        DeclareLaunchArgument(
            'room_size',
            default_value='[40.0, 30.0, 3.0]',
            description='Size of the room'
        ),

        DeclareLaunchArgument(
            'FOV',
            default_value='40.0',
            description='Field of view of the cameras'
        ),
        
        # Define and launch your nodes
        Node(
            package='robot_estimator',
            executable='VisualizationNode',
            name='VisualizationNode',
            output='screen',
            parameters=[
                {'num_robots': num_robots},
                {'num_cameras': num_cameras},
                {'num_lidar': num_lidar},
                {'room_size': room_size},
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
                {'room_size': room_size},
            ],
        ),

        Node(
            package='robot_estimator',
            executable='cameraDetectionNode',
            name='cameraDetectionNode',
            output='screen',
            parameters=[
                {'num_cameras': num_cameras},
                {'room_size': room_size},
                {'FOV': FOV},
            ],
        ),
        
        Node(
            package='robot_estimator',
            executable='lidarDetectionNode',
            name='lidarDetectionNode',
            output='screen',
            parameters=[
                {'room_size': room_size},
                {'num_lidar': num_lidar},
                {'angle_lidar': 10},
            ],
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
        ),

        Node(
            package='robot_estimator',
            executable='ekf', 
            name='ekf_tracker',
            output='screen',
            parameters=[
                {'room_size': room_size},
            ],
            # Wenn Sie Debug-Meldungen vom EKF sehen möchten, können Sie die folgende Zeile einkommentieren:
            arguments=['--ros-args', '--log-level', 'ekf_tracker:=debug'],
        ),
    ])