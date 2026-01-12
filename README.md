# Mid-360 激光雷达 Gazebo 仿真

本功能包提供 Livox Mid-360 激光雷达在 Gazebo 仿真环境中的完整仿真支持。

## 功能特性

- 基于真实 Mid-360 扫描模式的点云仿真
- 发布标准 `sensor_msgs/PointCloud2` 消息
- 包含真实 3D 模型 (STL mesh)
- 支持 URDF/Xacro 集成
- 内置 IMU 坐标系定义

## 目录结构

```
mid360_simulation/
├── CMakeLists.txt              # 构建配置
├── package.xml                 # 包描述文件
├── README.md                   # 本文档
├── include/mid360_simulation/  # 头文件
│   ├── mid360_points_plugin.h      # 点云插件头文件
│   ├── mid360_ode_multiray_shape.h # ODE 射线形状头文件
│   └── csv_reader.hpp              # CSV 读取工具
├── src/                        # 源文件
│   ├── mid360_points_plugin.cpp    # 点云插件实现
│   └── mid360_ode_multiray_shape.cpp # ODE 射线形状实现
├── urdf/                       # URDF 文件
│   └── mid360.xacro                # Mid-360 宏定义
├── meshes/                     # 3D 模型
│   └── livox_frame.STL             # Mid-360 外壳模型
└── scan_mode/                  # 扫描模式配置
    └── mid360.csv                  # 扫描角度数据
```

## 依赖项

- ROS2 (Humble/Foxy)
- Gazebo 11
- gazebo_ros_pkgs

## 编译

```bash
cd ~/your_ws
colcon build --packages-select mid360_simulation
source install/setup.bash
```

## 使用方法

### 1. 在 URDF 中添加激光雷达

在你的机器人 URDF/Xacro 文件中添加以下内容:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- 包含 Mid-360 宏定义 -->
  <xacro:include filename="$(find mid360_simulation)/urdf/mid360.xacro"/>

  <!-- 添加 Mid-360 激光雷达 -->
  <xacro:mid360 name="livox" parent="base_link">
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </xacro:mid360>

</robot>
```

### 2. 宏参数说明

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `name` | string | 是 | - | 激光雷达 link 名称 |
| `parent` | string | 是 | - | 父坐标系名称 |
| `origin` | block | 是 | - | 相对于父坐标系的位姿 (xyz, rpy) |
| `update_rate` | int | 否 | 10 | 发布频率 (Hz)，每帧点数自动计算 |

### 3. 发布频率与点云速率

基于官方规格，Mid-360 的点云速率为 **200,000 points/s**。

每帧点数根据发布频率自动计算：`每帧点数 = 200,000 / update_rate`

| 发布频率 | 每帧点数 |
|----------|----------|
| 10 Hz (默认) | 20,000 |
| 20 Hz | 10,000 |
| 5 Hz | 40,000 |



### 4. 生成的坐标系

调用宏后会生成以下坐标系:

- `${name}` - 激光雷达主体坐标系 (点云参考系)
- `${name}_imu` - 内置 IMU 坐标系

### 4. 发布的话题

| 话题名称 | 消息类型 | 说明 |
|----------|----------|------|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | 点云数据 |

## 技术规格

基于 Livox Mid-360 官方规格:

| 参数 | 值 |
|------|-----|
| 测距范围 | 0.1m - 200m |
| 水平视场角 | 360° |
| 垂直视场角 | -7.22° ~ +55.22° |
| 点云速率 | 200,000 points/s |
| 默认发布频率 | 10 Hz |
| 默认每帧点数 | 20,000 |
| 质量 | 265g |

## 示例

### 完整的机器人 URDF 示例

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- 基座 -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </visual>
  </link>

  <!-- 包含并添加 Mid-360 -->
  <xacro:include filename="$(find mid360_simulation)/urdf/mid360.xacro"/>
  <xacro:mid360 name="livox" parent="base_link">
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </xacro:mid360>

</robot>
```

### 在 Launch 文件中使用

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取 URDF 文件路径
    urdf_file = os.path.join(
        get_package_share_directory('your_robot_description'),
        'urdf', 'robot.urdf.xacro'
    )

    return LaunchDescription([
        # 启动 Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'),
                            'launch', 'gazebo.launch.py')
            ])
        ),

        # 生成机器人模型
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),

        # 在 Gazebo 中生成机器人
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'my_robot']
        ),
    ])
```

## 注意事项

1. 本仿真插件仅支持 ODE 物理引擎
2. 点云话题名称固定为 `/livox/lidar`，与真实驱动保持一致
3. 扫描模式基于真实 Mid-360 的非重复扫描模式

## 许可证

Apache-2.0

## 致谢

本项目基于 [livox_laser_simulation](https://github.com/Livox-SDK/livox_laser_simulation) 修改而来。
