from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_slam(context, *args, **kwargs):
    """根据 slam_mode 参数选择对应的 SLAM 节点配置"""
    slam_mode = LaunchConfiguration('slam_mode').perform(context)
    slam_toolbox_dir = get_package_share_directory('wheeltec_slam_toolbox')

    if slam_mode == 'lifelong':
        config_file = os.path.join(slam_toolbox_dir, 'config', 'mapper_params_lifelong.yaml')
    else:
        config_file = os.path.join(slam_toolbox_dir, 'config', 'mapper_params_online_async.yaml')

    slam_node = launch_ros.actions.Node(
        parameters=[config_file],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        remappings=[('odom', 'odom_combined')]
    )

    return [slam_node]


def generate_launch_description():
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    slam_mode_arg = DeclareLaunchArgument(
        'slam_mode',
        default_value='localization',
        description='SLAM模式: localization(纯定位) 或 lifelong(动态建图)'
    )

    wheeltec_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),
    )
    wheeltec_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_lidar.launch.py')),
    )

    return LaunchDescription([
        slam_mode_arg,
        wheeltec_robot,
        wheeltec_lidar,
        OpaqueFunction(function=launch_slam),
    ])
