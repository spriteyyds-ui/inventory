# SLAM 模式切换 - 备份文件

这些文件是实现 SLAM Toolbox 模式切换功能时修改的外部包文件。
由于这些包不是 git 仓库，修改不会被版本控制跟踪。

## 文件说明

| 备份文件 | 原始路径 | 修改内容 |
|----------|----------|----------|
| `wheeltec_nav2.launch.py` | `wheeltec_robot_nav2/launch/wheeltec_nav2.launch.py` | 添加 slam_mode 参数，禁用 AMCL |
| `mapper_params_lifelong.yaml` | `wheeltec_slam_toolbox/config/mapper_params_lifelong.yaml` | 新建 lifelong 模式配置 |
| `online_async_launch.py` | `wheeltec_slam_toolbox/launch/online_async_launch.py` | 支持 slam_mode 参数选择配置 |
| `WHEELTEC_left.pgm/yaml` | `wheeltec_robot_nav2/map/WHEELTEC_left.pgm/yaml` | 左半地图（涂掉Y=185以上），扫描19-36货柜时用 |
| `WHEELTEC_right.pgm/yaml` | `wheeltec_robot_nav2/map/WHEELTEC_right.pgm/yaml` | 右半地图（涂掉Y=385以下），扫描18-1货柜时用 |

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

# 恢复半地图文件
cp WHEELTEC_left.pgm WHEELTEC_left.yaml /home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/map/
cp WHEELTEC_right.pgm WHEELTEC_right.yaml /home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/map/
```

## 使用方式

### SLAM 模式切换

在 `config/inventory_system.yaml` 中修改 `slam_mode` 参数：

```yaml
slam_mode: "localization"   # 纯定位，地图不变（默认）
slam_mode: "lifelong"       # 动态建图，地图持续更新
slam_mode: "none"           # 使用原来的 AMCL 定位
```

### 半地图定位

根据扫描的货柜位置选择对应地图：

```bash
# 扫描右侧货柜（18-1）时：使用右半地图
ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py map:=/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/map/WHEELTEC_right.yaml

# 扫描左侧货柜（19-36）时：使用左半地图
ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py map:=/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/map/WHEELTEC_left.yaml
```

### 地图说明

- **WHEELTEC_left.pgm**: 保留Y=185以下区域，涂掉Y=185以上（用于扫描左侧19-36货柜）
- **WHEELTEC_right.pgm**: 保留Y=385以上区域，涂掉Y=385以下（用于扫描右侧18-1货柜）
