"""黄线巡线驾驶测试 launch。

启动 Astra 相机（可选）+ yellow_line_drive_test_node，用于现场验证巡线驾驶效果。

用法：
  ros2 launch agv_inventory_system yellow_line_drive_test.launch.py launch_front_camera:=true

然后：
  ros2 service call /yellow_line_drive_test_node/forward std_srvs/srv/Trigger
  ros2 service call /yellow_line_drive_test_node/start std_srvs/srv/Trigger
  ros2 service call /yellow_line_drive_test_node/stop std_srvs/srv/Trigger
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('agv_inventory_system')
    params_file = os.path.join(pkg_share, 'config', 'inventory_system.yaml')

    launch_front_camera_arg = DeclareLaunchArgument(
        'launch_front_camera', default_value='true',
        description='是否启动前方 Astra 相机')
    image_topic_arg = DeclareLaunchArgument(
        'yellow_line_image_topic', default_value='/camera/color/image_raw',
        description='相机图像话题')
    debug_arg = DeclareLaunchArgument(
        'yellow_line_debug_image_enabled', default_value='true',
        description='是否发布调试图像')
    target_x_ratio_arg = DeclareLaunchArgument(
        'yellow_line_target_x_ratio', default_value='0.88',
        description='黄线目标 x 位置比例')
    linear_speed_arg = DeclareLaunchArgument(
        'test_linear_speed', default_value='0.15',
        description='前进线速度 m/s')
    backward_linear_speed_arg = DeclareLaunchArgument(
        'test_backward_linear_speed', default_value='0.08',
        description='后退线速度 m/s；比前进慢以增加纠偏窗口')
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='/cmd_vel',
        description='底盘速度话题（需与底盘驱动一致）')

    # Astra 相机
    astra_launch_file = os.path.join(
        get_package_share_directory('astra_camera'), 'launch', 'astra.launch.xml')
    front_camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(astra_launch_file),
        condition=IfCondition(LaunchConfiguration('launch_front_camera')))

    # 黄线驾驶测试节点
    drive_test = Node(
        package='agv_inventory_system',
        executable='yellow_line_drive_test_node',
        name='yellow_line_drive_test_node',
        output='screen',
        parameters=[params_file, {
            'yellow_line_image_topic': LaunchConfiguration('yellow_line_image_topic'),
            'yellow_line_debug_image_enabled': LaunchConfiguration('yellow_line_debug_image_enabled'),
            'yellow_line_target_x_ratio': LaunchConfiguration('yellow_line_target_x_ratio'),
            'test_linear_speed': LaunchConfiguration('test_linear_speed'),
            'test_backward_linear_speed': LaunchConfiguration('test_backward_linear_speed'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
        }]
    )

    return LaunchDescription([
        launch_front_camera_arg,
        image_topic_arg,
        debug_arg,
        target_x_ratio_arg,
        linear_speed_arg,
        backward_linear_speed_arg,
        cmd_vel_topic_arg,
        front_camera,
        drive_test,
    ])
