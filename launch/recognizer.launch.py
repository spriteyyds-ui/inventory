from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('wheeltec_inventory_system'),
            'config',
            'inventory_system.yaml'
        ]),
        description='数字识别节点参数文件'
    )

    params_file = LaunchConfiguration('params_file')

    recognizer_node = Node(
        package='wheeltec_inventory_system',
        executable='number_recognizer_node',
        name='number_recognizer_node',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        params_file_arg,
        recognizer_node,
    ])
