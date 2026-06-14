import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = get_package_share_directory('home_sim_navigation')
    params_file = os.path.join(nav_pkg, 'config', 'pointcloud_to_laserscan.yaml')

    scan_converter = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[params_file, {'use_sim_time': True}],
        remappings=[
            ('cloud_in', '/camera/points'),
            ('scan', '/scan'),
        ],
        output='screen',
    )

    gazebo_rgbd_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gazebo_rgbd_frame_tf',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--qx', '0',
            '--qy', '0.1305262',
            '--qz', '0',
            '--qw', '0.9914449',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'home_bot/camera_link/rgbd_camera',
        ],
        output='screen',
    )

    return LaunchDescription([
        gazebo_rgbd_frame_tf,
        scan_converter,
    ])
