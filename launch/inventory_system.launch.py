from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_prefix, get_package_share_directory
import os


RIGHT_CAMERA_DEVICE_DEFAULT = (
    '/dev/v4l/by-path/platform-3610000.usb-usb-0:2.1.1:1.0-video-index0'
)
LEFT_CAMERA_DEVICE_DEFAULT = (
    '/dev/v4l/by-path/platform-3610000.usb-usb-0:2.1.2:1.0-video-index0'
)

INIT_CAMERA_SCRIPT = os.path.join(
    get_package_prefix('agv_inventory_system'),
    'lib', 'agv_inventory_system', 'init_camera_controls.sh'
)

ROBOT_API_SERVER_DIR = (
    '/home/wheeltec/wheeltec_ros2/src/agv_inventory_system/scripts/robot_inventory_client'
)


def generate_launch_description():
    launch_nav2_arg = DeclareLaunchArgument(
        'launch_nav2',
        default_value='true',
        description='是否随盘库系统启动 wheeltec_nav2'
    )

    enable_inventory_operation_gui_arg = DeclareLaunchArgument(
        'enable_inventory_operation_gui',
        default_value='true',
        description='是否随盘库系统启动操作总控 GUI'
    )

    enable_robot_api_server_arg = DeclareLaunchArgument(
        'enable_robot_api_server',
        default_value='true',
        description='是否随盘库系统启动网页接收 FastAPI 服务'
    )

    robot_api_host_arg = DeclareLaunchArgument(
        'robot_api_host',
        default_value='0.0.0.0',
        description='网页接收 FastAPI 服务监听地址'
    )

    robot_api_port_arg = DeclareLaunchArgument(
        'robot_api_port',
        default_value='8000',
        description='网页接收 FastAPI 服务监听端口'
    )

    inventory_params_file_arg = DeclareLaunchArgument(
        'inventory_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('agv_inventory_system'),
            'config',
            'inventory_system.yaml'
        ]),
        description='盘库系统参数文件'
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

    enable_camera_controls_init_arg = DeclareLaunchArgument(
        'enable_camera_controls_init',
        default_value='true',
        description='相机启动后是否自动初始化 v4l2 参数'
    )

    launch_nav2 = LaunchConfiguration('launch_nav2')
    enable_inventory_operation_gui = LaunchConfiguration('enable_inventory_operation_gui')
    enable_robot_api_server = LaunchConfiguration('enable_robot_api_server')
    robot_api_host = LaunchConfiguration('robot_api_host')
    robot_api_port = LaunchConfiguration('robot_api_port')
    inventory_params_file = LaunchConfiguration('inventory_params_file')
    c100_right_video_device = LaunchConfiguration('c100_right_video_device')
    c100_left_video_device = LaunchConfiguration('c100_left_video_device')
    enable_camera_controls_init = LaunchConfiguration('enable_camera_controls_init')
    nav2_launch_file = os.path.join(
        get_package_share_directory('wheeltec_nav2'),
        'launch',
        'wheeltec_nav2.launch.py'
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        condition=IfCondition(launch_nav2)
    )

    robot_api_server_process = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            [
                'source /opt/ros/humble/setup.bash && '
                'source /home/wheeltec/wheeltec_ros2/install/setup.bash && '
                'exec python3 -m uvicorn robot_api:app --host ',
                robot_api_host,
                ' --port ',
                robot_api_port,
            ],
        ],
        cwd=ROBOT_API_SERVER_DIR,
        output='screen',
        condition=IfCondition(enable_robot_api_server),
    )

    corridor_follower_node = Node(
        package='agv_inventory_system',
        executable='corridor_follower_node',
        name='corridor_follower_node',
        output='screen',
        parameters=[inventory_params_file]
    )

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
            'brightness': 0,
            'contrast': 32,
            'gain': 0,
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
            'brightness': 0,
            'contrast': 32,
            'gain': 0,
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

    # v4l2 参数初始化：两个相机都已启动后统一执行，延迟 15 秒
    init_camera_controls_action = ExecuteProcess(
        cmd=[
            'bash', INIT_CAMERA_SCRIPT,
            LaunchConfiguration('c100_right_video_device'),
            LaunchConfiguration('c100_left_video_device'),
        ],
        output='screen',
    )

    init_camera_controls_timer = TimerAction(
        period=15.0,
        actions=[init_camera_controls_action],
        condition=IfCondition(enable_camera_controls_init),
    )

    number_recognizer_node = Node(
        package='agv_inventory_system',
        executable='number_recognizer_node',
        name='number_recognizer_node',
        output='screen',
        parameters=[inventory_params_file, {
            'camera_topic': '/c100_right/image_raw',
            'recognized_topic': '/inventory/recognized_number',
            'enable_control_topic': '/inventory/recognizer_enable',
            'visualization_topic': '/inventory/visualization',
            'debug_a4_topic': '/inventory/debug_a4_region',
            'debug_digits_topic': '/inventory/debug_digits',
            'enable_on_start': True,
        }]
    )

    distance_estimator_node = Node(
        package='agv_inventory_system',
        executable='distance_estimator_node',
        name='distance_estimator_node',
        output='screen',
        parameters=[inventory_params_file]
    )

    gap_detector_node = Node(
        package='agv_inventory_system',
        executable='gap_detector_node',
        name='gap_detector_node',
        output='screen',
        parameters=[inventory_params_file]
    )

    mission_manager_node = Node(
        package='agv_inventory_system',
        executable='mission_manager_node',
        name='mission_manager_node',
        output='screen',
        parameters=[inventory_params_file]
    )

    inventory_auto_recharger_node = Node(
        package='agv_inventory_system',
        executable='inventory_auto_recharger.py',
        output='screen',
        parameters=[inventory_params_file]
    )

    lift_relay_controller_node = Node(
        package='agv_inventory_system',
        executable='lift_relay_controller.py',
        name='lift_relay_controller',
        output='screen',
        parameters=[inventory_params_file]
    )

    inventory_operation_gui_node = Node(
        package='agv_inventory_system',
        executable='inventory_operation_gui.py',
        name='inventory_operation_gui',
        output='screen',
        parameters=[inventory_params_file],
        condition=IfCondition(enable_inventory_operation_gui)
    )

    return LaunchDescription([
        launch_nav2_arg,
        enable_inventory_operation_gui_arg,
        enable_robot_api_server_arg,
        robot_api_host_arg,
        robot_api_port_arg,
        inventory_params_file_arg,
        c100_right_video_device_arg,
        c100_left_video_device_arg,
        enable_camera_controls_init_arg,
        robot_api_server_process,
        nav2_launch,
        corridor_follower_node,
        c100_left_camera_node,
        c100_right_camera_timer,
        init_camera_controls_timer,
        number_recognizer_node,
        distance_estimator_node,
        gap_detector_node,
        inventory_auto_recharger_node,
        lift_relay_controller_node,
        inventory_operation_gui_node,
        mission_manager_node,
    ])
