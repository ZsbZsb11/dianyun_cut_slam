import os
from glob import glob

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def _xauthority_path():
    if os.environ.get('XAUTHORITY'):
        return os.environ['XAUTHORITY']

    candidates = sorted(glob(f"/run/user/{os.getuid()}/.mutter-Xwaylandauth.*"))
    if candidates:
        return candidates[0]

    home_auth = os.path.expanduser('~/.Xauthority')
    if os.path.exists(home_auth):
        return home_auth

    return ''


def _launch_setup(context, *args, **kwargs):
    gazebo_pkg = get_package_share_directory('home_sim_gazebo')
    description_pkg = get_package_share_directory('home_sim_description')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    gui_enabled = LaunchConfiguration('gui').perform(context).lower() in ('1', 'true', 'yes', 'on')

    world_file = os.path.join(gazebo_pkg, 'worlds', 'two_room_home.sdf')
    robot_sdf = os.path.join(description_pkg, 'models', 'home_bot', 'model.sdf')
    robot_xacro = os.path.join(description_pkg, 'urdf', 'home_bot.urdf.xacro')
    gz_args = f'-r {world_file}' if gui_enabled else f'-r -s {world_file}'

    resource_paths = os.pathsep.join([
        os.path.join(description_pkg, 'models'),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'home_bot',
            '-file', robot_sdf,
            '-x', '-2.0',
            '-y', '0.0',
            '-z', '0.05',
            '-Y', '0.0',
        ],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': Command(['xacro ', robot_xacro])},
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel_gz@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/home_bot/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
            '/world/two_room_home/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/model/home_bot/pose', '/gz_model_pose'),
            ('/world/two_room_home/pose/info', '/gz_world_pose_tf'),
            ('/camera/image', '/camera/color/image_raw'),
            ('/camera/depth_image', '/camera/depth/image_raw'),
        ],
        output='screen',
    )
    
    cmd_vel_adapter = Node(
        package='home_sim_gazebo',
        executable='cmd_vel_adapter',
        name='cmd_vel_adapter',
        parameters=[{
            'invert_linear_x': True,
            'invert_angular_z': True,
        }],
        output='screen',
    )

    ground_truth_odom = Node(
        package='home_sim_gazebo',
        executable='ground_truth_odom',
        name='ground_truth_odom',
        parameters=[
            {'use_sim_time': True},
            {'input_topic': '/gz_world_pose_tf'},
            {'input_type': 'tf_message'},
            {'initial_x': -2.0},
            {'initial_y': 0.0},
            {'initial_yaw': 0.0},
        ],
        output='screen',
    )
    
    return [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_paths),
        SetEnvironmentVariable('DISPLAY', os.environ.get('DISPLAY') or ':0'),
        SetEnvironmentVariable('XAUTHORITY', _xauthority_path()),
        SetEnvironmentVariable('QT_QPA_PLATFORM', os.environ.get('QT_QPA_PLATFORM') or 'xcb'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', os.environ.get('LIBGL_ALWAYS_SOFTWARE') or '1'),
        gazebo,
        spawn_robot,
        robot_state_publisher,
        bridge,
        cmd_vel_adapter,
        ground_truth_odom,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='Start the Gazebo GUI. The default is server-only for stable RGB-D rendering in the VM.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
