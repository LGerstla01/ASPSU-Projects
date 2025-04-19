from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Pfad zur Weltdatei
    world_file = os.path.join(
        os.getenv('HOME'),
        'ASPSU-Projects/src/robot_estimator/worlds/room.world'
    )

    return LaunchDescription([
        # Gazebo starten mit der Weltdatei
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file],
            output='screen'
        ),

        # Roboter-Controller-Knoten
        Node(
            package='robot_estimator',
            executable='robotControllerNode',
            name='robotControllerNode',
            output='screen',
        ),

        # State Estimator (Kalman-Filter)
        Node(
            package='robot_estimator',
            executable='state_estimator',
            name='state_estimator',
            output='screen',
        ),

        # Visualizer-Knoten
        Node(
            package='robot_estimator',
            executable='visualizer',
            name='visualizer',
            output='screen',
        ),
    ])