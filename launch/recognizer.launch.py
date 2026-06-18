from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('agv_inventory_system'),
            'config',
            'inventory_system.yaml'
        ]),
        description='数字识别节点参数文件'
    )

    params_file = LaunchConfiguration('params_file')

    # Camera manager: manages HJ camera processes on demand.
    # No static usb_cam nodes — camera_manager starts/stops cameras as needed.
    camera_manager_node = Node(
        package='agv_inventory_system',
        executable='camera_manager_node.py',
        name='camera_manager_node',
        output='screen',
        parameters=[params_file]
    )

    recognizer_node = Node(
        package='agv_inventory_system',
        executable='number_recognizer_node',
        name='number_recognizer_node',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        params_file_arg,
        camera_manager_node,
        recognizer_node,
    ])
