import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # params_file = os.path.join(
    #     get_package_share_directory('sensor'), 'config', 'sensor.yaml')

    camera_info_path = os.path.join(
        'config',
        'camera_info.yaml'
    )

    livox_launch_dir = get_package_share_directory('livox_ros2_driver')
    livox_launch_file = os.path.join(livox_launch_dir, 'launch', 'livox_lidar_rviz_launch.py')
    livox_config_file = os.path.join(livox_launch_dir, 'config', 'livox_lidar_config.json')    # 包含 Livox Launch 文件

    include_livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_launch_file),
        launch_arguments={
            'param_file': livox_config_file,  # 替换为实际配置文件路径
            'use_sim_time': 'true'
        }.items()
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_info',
            default_value=camera_info_path
        ),
        Node(
            package='sensor',
            executable='sensor_node',
            name='sensor',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('camera_info')]
        ),
        Node(
            package='livox_ros2_driver',
            executable="livox_ros2_driver_node"
        ),
        include_livox_launch
    ])