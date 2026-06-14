import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('home_sim_gazebo')
    nav_pkg = get_package_share_directory('home_sim_navigation')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'sim.launch.py')
        )
    )

    rgbd_scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'rgbd_scan.launch.py')
        )
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            os.path.join(nav_pkg, 'config', 'mapper_params_rgbd_slice.yaml'),
            {'use_sim_time': True},
        ],
        output='screen',
    )

    return LaunchDescription([
        sim_launch,
        rgbd_scan_launch,
        slam_toolbox,
    ])

