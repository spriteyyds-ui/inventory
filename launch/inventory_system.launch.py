from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_prefix, get_package_share_directory
import os
import yaml


RIGHT_CAMERA_DEVICE_DEFAULT = (
    '/dev/v4l/by-path/platform-3610000.usb-usb-0:2.4.1:1.0-video-index0'
)
LEFT_CAMERA_DEVICE_DEFAULT = (
    '/dev/v4l/by-path/platform-3610000.usb-usb-0:2.4.2:1.0-video-index0'
)

ROBOT_API_SERVER_DIR = (
    '/home/wheeltec/wheeltec_ros2/src/agv_inventory_system/scripts/robot_inventory_client'
)

SLAM_TOOLBOX_CONFIG_DIR = os.path.join(
    get_package_share_directory('wheeltec_slam_toolbox'), 'config'
)


def read_slam_mode_from_config(params_file):
    """从 inventory_system.yaml 读取 slam_mode 参数"""
    try:
        # 尝试直接读取文件路径
        if os.path.isfile(params_file):
            with open(params_file, 'r') as f:
                cfg = yaml.safe_load(f)
            return cfg.get('slam_mode', 'localization')
    except Exception:
        pass
    return 'localization'


def launch_slam_toolbox(context, *args, **kwargs):
    """根据 slam_mode 参数启动对应的 SLAM Toolbox 节点"""
    slam_mode = LaunchConfiguration('slam_mode').perform(context)

    if slam_mode == 'none':
        return []

    if slam_mode == 'lifelong':
        config_file = os.path.join(SLAM_TOOLBOX_CONFIG_DIR, 'mapper_params_lifelong.yaml')
    else:
        config_file = os.path.join(SLAM_TOOLBOX_CONFIG_DIR, 'mapper_params_online_async.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[config_file],
        remappings=[('odom', 'odom_combined')]
    )

    return [slam_node]


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

    launch_front_camera_arg = DeclareLaunchArgument(
        'launch_front_camera',
        default_value='true',
        description='是否启动前方 Astra 相机（黄线巡线需要）'
    )

    enable_front_depth_arg = DeclareLaunchArgument(
        'enable_front_depth',
        default_value='false',
        description='是否启用 Astra 深度/IR/点云流（默认关闭以节省 USB 带宽；仅彩色足以满足黄线巡线需求）'
    )

    launch_nav2 = LaunchConfiguration('launch_nav2')
    enable_inventory_operation_gui = LaunchConfiguration('enable_inventory_operation_gui')
    enable_robot_api_server = LaunchConfiguration('enable_robot_api_server')
    robot_api_host = LaunchConfiguration('robot_api_host')
    robot_api_port = LaunchConfiguration('robot_api_port')
    inventory_params_file = LaunchConfiguration('inventory_params_file')
    c100_right_video_device = LaunchConfiguration('c100_right_video_device')
    c100_left_video_device = LaunchConfiguration('c100_left_video_device')
    launch_front_camera = LaunchConfiguration('launch_front_camera')
    enable_front_depth = LaunchConfiguration('enable_front_depth')
    nav2_launch_file = os.path.join(
        get_package_share_directory('wheeltec_nav2'),
        'launch',
        'wheeltec_nav2.launch.py'
    )

    # slam_mode 参数：none=使用AMCL | localization=SLAM纯定位 | lifelong=SLAM动态建图
    # 从 inventory_system.yaml 读取默认值
    inventory_yaml_path = os.path.join(
        get_package_share_directory('agv_inventory_system'), 'config', 'inventory_system.yaml'
    )
    default_slam_mode = read_slam_mode_from_config(inventory_yaml_path)

    slam_mode_arg = DeclareLaunchArgument(
        'slam_mode',
        default_value=default_slam_mode,
        description='SLAM模式: none(使用AMCL) | localization(SLAM纯定位) | lifelong(SLAM动态建图)'
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={'slam_mode': LaunchConfiguration('slam_mode')}.items(),
        condition=IfCondition(launch_nav2)
    )

    # 前方 Astra 相机（黄线巡线需要）
    # 延迟 15 秒启动，等待 Nav2 等重节点完成初始化后再独占 USB/带宽资源
    astra_launch_file = os.path.join(
        get_package_share_directory('astra_camera'),
        'launch',
        'astra.launch.xml'
    )
    front_camera_launch_raw = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(astra_launch_file),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': enable_front_depth,
            'enable_ir': 'false',
            'enable_point_cloud': enable_front_depth,
            'enable_colored_point_cloud': 'false',
            'depth_registration': enable_front_depth,
        }.items(),
        condition=IfCondition(launch_front_camera)
    )
    front_camera_launch = TimerAction(
        period=15.0,
        actions=[front_camera_launch_raw],
        condition=IfCondition(launch_front_camera)
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

    # Camera manager: manages HJ camera processes on demand (left/right).
    # No static usb_cam nodes or init_camera_controls.sh needed.
    camera_manager_node = Node(
        package='agv_inventory_system',
        executable='camera_manager_node.py',
        name='camera_manager_node',
        output='screen',
        parameters=[inventory_params_file, {
            'left_camera_device': c100_left_video_device,
            'right_camera_device': c100_right_video_device,
            'left_camera_topic': '/c100_left/image_raw',
            'right_camera_topic': '/c100_right/image_raw',
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'mjpeg2rgb',
            'startup_timeout_sec': 10.0,
            'startup_retry_count': 1,
            'first_frame_timeout_sec': 10.0,
        }]
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
        slam_mode_arg,
        enable_inventory_operation_gui_arg,
        enable_robot_api_server_arg,
        robot_api_host_arg,
        robot_api_port_arg,
        inventory_params_file_arg,
        c100_right_video_device_arg,
        c100_left_video_device_arg,
        launch_front_camera_arg,
        enable_front_depth_arg,
        robot_api_server_process,
        nav2_launch,
        OpaqueFunction(function=launch_slam_toolbox),
        front_camera_launch,
        corridor_follower_node,
        camera_manager_node,
        number_recognizer_node,
        distance_estimator_node,
        gap_detector_node,
        inventory_auto_recharger_node,
        lift_relay_controller_node,
        inventory_operation_gui_node,
        mission_manager_node,
    ])
