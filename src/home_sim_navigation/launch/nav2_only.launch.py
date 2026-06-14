import os
import shutil
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def prepare_nav2_map(nav_pkg):
    src_yaml = os.path.join(nav_pkg, 'maps', 'rgbd_two_room_map_dense_views_cleaned.yaml')
    src_pgm = os.path.join(nav_pkg, 'maps', 'rgbd_two_room_map_dense_views_cleaned.pgm')
    runtime_dir = '/tmp/home_sim_project_nav2_maps'
    os.makedirs(runtime_dir, exist_ok=True)

    runtime_pgm = os.path.join(runtime_dir, 'rgbd_two_room_map_dense_views_cleaned.pgm')
    runtime_yaml = os.path.join(runtime_dir, 'rgbd_two_room_map_dense_views_cleaned.yaml')
    shutil.copy2(src_pgm, runtime_pgm)

    with open(src_yaml, 'r', encoding='utf-8') as f:
        map_data = yaml.safe_load(f)
    map_data['image'] = runtime_pgm
    with open(runtime_yaml, 'w', encoding='utf-8') as f:
        yaml.safe_dump(map_data, f, default_flow_style=False, sort_keys=False)

    return runtime_yaml


def generate_launch_description():
    nav_pkg = get_package_share_directory('home_sim_navigation')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    map_yaml = prepare_nav2_map(nav_pkg)

    rgbd_scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'rgbd_scan.launch.py')
        )
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_yaml,
            'params_file': os.path.join(nav_pkg, 'config', 'my_nav2_params.yaml'),
        }.items(),
    )

    return LaunchDescription([
        rgbd_scan_launch,
        nav2_launch,
    ])
