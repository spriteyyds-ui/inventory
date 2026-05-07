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
        description='盘库系统参数文件'
    )

    params_file = LaunchConfiguration('params_file')

    corridor_follower_node = Node(
        package='wheeltec_inventory_system',
        executable='corridor_follower_node',
        name='corridor_follower_node',
        output='screen',
        parameters=[params_file]
    )

    c100_right_camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='c100_right_camera',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'camera_name': 'c100_right',
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0,
            'pixel_format': 'mjpeg2rgb',
        }],
        remappings=[
            ('image_raw', '/c100_right/image_raw'),
            ('camera_info', '/c100_right/camera_info'),
        ]
    )

    number_recognizer_node = Node(
        package='wheeltec_inventory_system',
        executable='number_recognizer_node',
        name='number_recognizer_node',
        output='screen',
        parameters=[params_file]
    )

    distance_estimator_node = Node(
        package='wheeltec_inventory_system',
        executable='distance_estimator_node',
        name='distance_estimator_node',
        output='screen',
        parameters=[params_file]
    )

    gap_detector_node = Node(
        package='wheeltec_inventory_system',
        executable='gap_detector_node',
        name='gap_detector_node',
        output='screen',
        parameters=[params_file]
    )

    mission_manager_node = Node(
        package='wheeltec_inventory_system',
        executable='mission_manager_node',
        name='mission_manager_node',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        params_file_arg,
        corridor_follower_node,
        c100_right_camera_node,
        number_recognizer_node,
        distance_estimator_node,
        gap_detector_node,
        mission_manager_node,
    ])
