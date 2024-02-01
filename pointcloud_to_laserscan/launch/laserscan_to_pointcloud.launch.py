from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='laserscan_to_pointcloud_node',
            name='laserscan_to_pointcloud',
            remappings=[('scan_in', '/scan'),
                        ('cloud',  '/cloud')],
            parameters=[{'target_frame': '', 
                         'transform_tolerance': 0.01, 
                         'use_sim_time': True}]
        ),
    ])

