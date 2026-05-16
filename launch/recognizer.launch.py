from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


RIGHT_CAMERA_DEVICE_DEFAULT = (
    '/dev/v4l/by-path/platform-3610000.usb-usb-0:2.1.1:1.0-video-index0'
)
LEFT_CAMERA_DEVICE_DEFAULT = (
    '/dev/v4l/by-path/platform-3610000.usb-usb-0:2.1.2:1.0-video-index0'
)


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

    c100_right_video_device_arg = DeclareLaunchArgument(
        'c100_right_video_device',
        default_value=RIGHT_CAMERA_DEVICE_DEFAULT,
        description='C100 右相机稳定 by-path 设备路径'
    )

    c100_left_video_device_arg = DeclareLaunchArgument(
        'c100_left_video_device',
        default_value=LEFT_CAMERA_DEVICE_DEFAULT,
        description='C100 左相机稳定 by-path 设备路径'
    )

    params_file = LaunchConfiguration('params_file')
    c100_right_video_device = LaunchConfiguration('c100_right_video_device')
    c100_left_video_device = LaunchConfiguration('c100_left_video_device')

    c100_right_camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='c100_right_camera',
        output='screen',
        parameters=[{
            'video_device': c100_right_video_device,
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

    c100_left_camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='c100_left_camera',
        output='screen',
        parameters=[{
            'video_device': c100_left_video_device,
            'camera_name': 'c100_left',
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0,
            'pixel_format': 'mjpeg2rgb',
        }],
        remappings=[
            ('image_raw', '/c100_left/image_raw'),
            ('camera_info', '/c100_left/camera_info'),
        ]
    )

    c100_right_camera_timer = TimerAction(
        period=10.0,
        actions=[c100_right_camera_node]
    )

    recognizer_node = Node(
        package='wheeltec_inventory_system',
        executable='number_recognizer_node',
        name='number_recognizer_node',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        params_file_arg,
        c100_right_video_device_arg,
        c100_left_video_device_arg,
        c100_left_camera_node,
        c100_right_camera_timer,
        recognizer_node,
    ])
