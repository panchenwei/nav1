from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            output='screen',
            parameters=['src/robot_navig/config/nav2_params.yaml'],
            remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]
        ),
        Node(
            package='my_robot_navigation',
            executable='icp_localization',
            name='icp_localization',
            output='screen',
            parameters=['config/icp_params.yaml'],
            remappings=[('/input', '/livox/lidar')]
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -1.57,
                'angle_max': 1.57,
                'angle_increment': 0.01,
                'scan_time': 0.1,
                'range_min': 0.0,
                'range_max': 10.0,
                'use_inf': True
            }],
            remappings=[
                ('cloud_in', '/livox/lidar'),
                ('scan', '/scan')
            ]
        )
    ])