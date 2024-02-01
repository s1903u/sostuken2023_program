from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/livox/lidar'),
                        ('scan', '/scan')],
            parameters=[{
                'target_frame': '',
                'transform_tolerance': 0.1,
                'min_height': 0.3,
                'max_height': 5.0,
                'angle_min': -3.14159,  # -M_PI/2
                'angle_max': 3.14159,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.1,
                'range_min': 0.1,
                'range_max': 300.0,
                'use_inf': True,
                'inf_epsilon': 1000.0,
                'use_sim_time':True
            }],
            name='pointcloud_to_laserscan'
        )
    ])

