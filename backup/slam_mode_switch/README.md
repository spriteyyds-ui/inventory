# SLAM 模式切换 - 备份文件

这些文件是实现 SLAM Toolbox 模式切换功能时修改的外部包文件。
由于这些包不是 git 仓库，修改不会被版本控制跟踪。

## 文件说明

| 备份文件 | 原始路径 | 修改内容 |
|----------|----------|----------|
| `wheeltec_nav2.launch.py` | `wheeltec_robot_nav2/launch/wheeltec_nav2.launch.py` | 添加 slam_mode 参数，禁用 AMCL |
| `mapper_params_lifelong.yaml` | `wheeltec_slam_toolbox/config/mapper_params_lifelong.yaml` | 新建 lifelong 模式配置 |
| `online_async_launch.py` | `wheeltec_slam_toolbox/launch/online_async_launch.py` | 支持 slam_mode 参数选择配置 |

## 恢复方法

如果这些包被重新安装，需要将备份文件复制回原位：

```bash
cd /home/wheeltec/wheeltec_ros2/src/agv_inventory_system/backup/slam_mode_switch

# 恢复 wheeltec_nav2 launch 文件
cp wheeltec_nav2.launch.py /home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/launch/wheeltec_nav2.launch.py

# 恢复 lifelong 配置文件
cp mapper_params_lifelong.yaml /home/wheeltec/wheeltec_ros2/src/wheeltec_robot_slam/wheeltec_slam_toolbox/config/mapper_params_lifelong.yaml

# 恢复 SLAM Toolbox launch 文件
cp online_async_launch.py /home/wheeltec/wheeltec_ros2/src/wheeltec_robot_slam/wheeltec_slam_toolbox/launch/online_async_launch.py
```

## 使用方式

在 `config/inventory_system.yaml` 中修改 `slam_mode` 参数：

```yaml
slam_mode: "localization"   # 纯定位，地图不变（默认）
slam_mode: "lifelong"       # 动态建图，地图持续更新
slam_mode: "none"           # 使用原来的 AMCL 定位
```
