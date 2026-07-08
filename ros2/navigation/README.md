# 导航与通信子系统

> [!WARNING]
> ## ⚠️ 重要声明
> 本模块是对真实部署代码的重构版本。**不能直接部署。** 详见[仓库根目录 README](../../README.md)。

## 包含的 ROS2 包

| 包 | 语言 | 功能 |
|----|------|------|
| `k1muse_mcu_bridge` | C++17 | STM32 串口桥接节点（UART 帧协议、里程计发布、cmd_vel 路由） |
| `k1muse_mobile_bridge` | C++17 + Python | 蓝牙 App 桥接（K1MB 协议、地图分块/zlib、位姿发布） |
| `k1muse_slam_nav` | Python/Launch | Cartographer SLAM + Nav2 导航编排 |
| `k1muse_description` | URDF/Xacro | 机器人模型（X-omni 底盘 + SO101 机械臂） |
| `ldlidar_stl_ros2` | C++14 | LD06 激光雷达驱动 |
| `imu_ros2_device` | Python | YB IMU 驱动 |
| `k1muse_exploration` | Python | RRT/Frontier 自主探索 |

## 硬件接口

| 设备 | 端口 | 波特率 |
|------|------|--------|
| STM32 MCU | /dev/mymcu (UART4) | 115200 |
| LD06 LiDAR | /dev/mylidar (UART5) | 230400 |
| YB IMU | /dev/myimu (CH340 USB) | 115200 |
| 蓝牙 | /dev/rfcomm0 | — |

## 分布式部署

支持 MUSE Pi Pro + Raspberry Pi 4B 双机 ROS2 通信：
- MUSE: 传感器驱动 + 通信桥（实时 I/O）
- RPi4: Cartographer + Nav2 + 探索（计算密集型）
- 通信: 有线以太网 CycloneDDS, ROS_DOMAIN_ID=42

## 许可

Apache License 2.0
