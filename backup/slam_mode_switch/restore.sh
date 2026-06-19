#!/bin/bash
# 恢复 SLAM 模式切换相关的外部包文件
# 用法: bash restore.sh

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "恢复 SLAM 模式切换文件..."

# 恢复 wheeltec_nav2 launch 文件
cp "$SCRIPT_DIR/wheeltec_nav2.launch.py" \
   "/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/launch/wheeltec_nav2.launch.py"
echo "✓ wheeltec_nav2.launch.py"

# 恢复 lifelong 配置文件
cp "$SCRIPT_DIR/mapper_params_lifelong.yaml" \
   "/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_slam/wheeltec_slam_toolbox/config/mapper_params_lifelong.yaml"
echo "✓ mapper_params_lifelong.yaml"

# 恢复 SLAM Toolbox launch 文件
cp "$SCRIPT_DIR/online_async_launch.py" \
   "/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_slam/wheeltec_slam_toolbox/launch/online_async_launch.py"
echo "✓ online_async_launch.py"

echo "恢复完成！"
