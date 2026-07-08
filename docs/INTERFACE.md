# MUSE Pi Pro 工作区外部接口

本文只描述本工作区对外暴露的稳定接口，以及外部系统应该如何使用这些接口。

## 工作区入口

- 本机仓库：`E:\Embed_Game\MusePiPro_prj`
- 板端通信工作区：`/home/bianbu/k1muse_communicate_ros/`
- 板端 SLAM 工作区：`/home/bianbu/k1muse_slam_ros/`
- 树莓派计算工作区：`/home/user/k1muse_slam_ros/`
- 树莓派 MPPI overlay：`/home/user/mppi_overlay/`
- 通信源码：`k1muse_communicate_ros/src/`
- SLAM/Nav 源码：`k1muse_slam_ros/src/`
- 地图目录：`/home/bianbu/k1muse_slam_ros/src/k1muse_slam_nav/maps/`

外部系统不要直接依赖 `build/`、`install/`、`log/`、缓存目录或开发临时脚本目录。板端运行前按 `AGENTS.md` 和 `ENV_DEV.md` 同步源码并构建。

默认运行是双机分布式：

- MUSE Pi Pro：硬件桥接、传感器发布、蓝牙 App、地图目录、手动 RViz。
- Raspberry Pi 4B：Cartographer、Nav2、自动探索和地图保存计算。

外部系统仍以 MUSE 为唯一产品入口。树莓派只作为内部计算端，不改变 Android App 协议、脚本名称、地图目录语义或 ROS2 topic/service/action 名称。

## 硬件设备接口

| 设备 | 板端别名 | 物理设备 | 用途 | 独占方 |
|---|---|---|---|---|
| STM32 MCU | `/dev/mymcu` | UART4 `/dev/ttyS4`，115200 8N1 | 底盘状态和控制 | `mcu_bridge_node` |
| LD06 | `/dev/mylidar` | UART5 `/dev/ttyS5`，230400 8N1 | 激光雷达 | `ldlidar_stl_ros2` |
| YB IMU | `/dev/myimu` | CH340 USB 串口，115200 8N1 | IMU 数据 | `imu_ros2_device` |
| Android App | `/dev/rfcomm0` | Bluetooth RFCOMM/SPP | 手机桥接 | `mobile_supervisor.py` 或 `mobile_bridge_node` |

STM32 串口只能由 `mcu_bridge_node` 独占打开。外部底盘、机械臂、升降、导航或 App 逻辑通过 ROS2 topic/service/action 访问，不直接抢 `/dev/mymcu`。

## 网络拓扑

```
MUSE Pi Pro (192.168.137.194 Wi-Fi / 10.10.10.2 eth)  ←→  Raspberry Pi 4B (10.10.10.1 eth)
       ↑ Wi-Fi SSH 管理面                                        ↑ Wi-Fi DHCP 管理面
       ↑ 10.10.10.0/24 有线 ROS2 数据面 (CycloneDDS peer)
```

两端 CycloneDDS 配置文件确保 ROS2 数据面走有线，内容详见 `AGENTS.md`。

## 坐标和 TF

坐标遵循 REP-103：`+X` 前、`+Y` 左、`+Z` 上，yaw 逆时针为正。

目标 TF 树：

```text
map -> odom -> base_footprint -> base_link -> base_laser
       ↑                                        ↑
  Cartographer                              -> imu_link
  (或 Nav2/AMCL)
```

- `odom -> base_footprint`：由 `k1muse_mcu_bridge/mcu_bridge_node` 发布。
- `base_footprint -> base_link -> base_laser / imu_link`：由 `k1muse_description` 的 URDF 经 `robot_state_publisher` 发布。
- `map -> odom`：建图时由 Cartographer 发布，导航时由 Nav2/AMCL 发布。

## ROS2 Topic

| Topic | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/mcu/chassis/status` | `k1muse_mcu_bridge/msg/ChassisStatus` | MCU -> ROS | 底盘状态原始上行数据 |
| `/mcu/chassis/mov` | `k1muse_mcu_bridge/msg/ChassisMov` | ROS -> MCU bridge | 底盘 MOV 下行命令 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 外部控制 -> MCU bridge | 可选速度控制入口，需 `mcu_bridge_node enable_cmd_vel_output:=true` |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | Nav2 -> (remap) | Nav2 controller 输出的速度指令，由 launch 文件 remap 到 `/cmd_vel` |
| `/odom` | `nav_msgs/msg/Odometry` | MCU bridge -> ROS | 里程计，child frame 为 `base_footprint` |
| `/scan_raw` | `sensor_msgs/msg/LaserScan` | LD06 -> ROS | 原始雷达扫描，frame 为 `base_laser` |
| `/scan` | `sensor_msgs/msg/LaserScan` | 过滤节点 -> ROS | 过滤自车体后的雷达扫描，供 Cartographer/Nav2/App 使用 |
| `/imu/data` | `sensor_msgs/msg/Imu` | IMU -> ROS | IMU 数据，frame 为 `imu_link` |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Cartographer/Nav2 -> ROS/App | 当前地图 |
| `/plan` | `nav_msgs/msg/Path` | Nav2 -> App | 自动探索时发送给 App 的路径 |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | rrt_frontier_explorer -> Nav2 | 自动探索目标位姿 |
| `/exploration_status` | `std_msgs/msg/String` | 自动探索 -> ROS/App | 自动探索状态文本 |
| `/exploration_markers` | `visualization_msgs/msg/MarkerArray` | 自动探索 -> RViz | 自动探索候选点和目标可视化 |

## ROS2 Service

| Service | 类型 | 说明 |
|---|---|---|
| `/mcu/chassis/stop` | `k1muse_mcu_bridge/srv/ChassisStop` | 向 MCU 发送底盘停止命令 |
| `/mcu/chassis/odom` | `k1muse_mcu_bridge/srv/ChassisOdom` | 向 MCU 写入底盘 odom 重置值：`direction`、`x`、`y` |
| `/write_state` | `cartographer_ros_msgs/srv/WriteState` | Cartographer 保存 `.pbstream` |

## ROS2 Action

| Action | 类型 | 使用方 |
|---|---|---|
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 提供，`k1muse_exploration` 可向它发送自动探索目标 |

## 启动脚本接口

安装后的脚本是 App、systemd 或人工操作应优先使用的稳定入口。

| 脚本 | 工作区 | 说明 |
|---|---|---|
| `k1_start_mcu_bridge.sh` | communication | 启动 MCU bridge，默认开启 `/cmd_vel` 到 MCU 的路由 |
| `k1_stop_mcu_bridge.sh` | communication | 发布零 `/cmd_vel` 后停止 MCU bridge |
| `k1_start_mapping.sh` | slam | 启动 MCU bridge、传感器和 Cartographer 建图（默认远程） |
| `k1_save_map.sh manual` | slam | 保存 `manual_*.yaml/.pgm/.pbstream`，远程模式下由树莓派计算后同步回 MUSE 地图目录 |
| `k1_stop_mapping.sh` | slam | 停止前尝试保存 `stop_*`，失败也继续停止 |
| `k1_stop_mapping.sh fast` | slam | 快速停止，不做自动保存 |
| `k1_start_exploration.sh MIN_X MAX_X MIN_Y MAX_Y` | slam | 在指定边界内启动自动探索 |
| `k1_stop_exploration.sh fast` | slam | 快速停止自动探索和 MCU bridge |
| `k1_stop_exploration.sh save` | slam | 停止前调用 `k1_save_map.sh manual` |

远程计算模式下，启动脚本会检查树莓派关键节点后再返回成功。建图至少要求 Cartographer 和 occupancy grid 节点可见；自动探索还要求 Nav2 controller、planner、BT navigator 和 `rrt_frontier_explorer` 可见。等待超时后输出日志尾部帮助定位问题。

脚本状态和日志：

- 建图：`/home/bianbu/.ros/k1muse_slam_nav/run/`、`/home/bianbu/.ros/k1muse_slam_nav/logs/`
- 自动探索：`/home/bianbu/.ros/k1muse_exploration/run/`、`/home/bianbu/.ros/k1muse_exploration/logs/`
- 树莓派计算端日志：`/home/user/.ros/k1muse_slam_nav/logs/`、`/home/user/.ros/k1muse_exploration/logs/`

远程编排环境变量：

| 变量 | 默认值 | 说明 |
|---|---|---|
| `K1_REMOTE_COMPUTE` | `1` | `1` 使用树莓派计算端；`0` 回退到 MUSE 本地运行 Cartographer/Nav2/探索 |
| `K1_REMOTE_SLAM_HOST` | `user@10.10.10.1` | MUSE 到树莓派的 SSH 目标 |
| `K1_REMOTE_COMMS_WS` | `/home/user/k1muse_communicate_ros` | 树莓派通信工作区，用于 source 接口包 |
| `K1_REMOTE_SLAM_WS` | `/home/user/k1muse_slam_ros` | 树莓派 SLAM 工作区 |
| `K1_REMOTE_STATE_DIR` | `/home/user/.ros/k1muse_slam_nav` | 树莓派建图日志和 pid 目录 |
| `K1_REMOTE_EXPLORATION_STATE_DIR` | `/home/user/.ros/k1muse_exploration` | 树莓派探索日志和 pid 目录 |
| `K1_MAPPING_USE_RVIZ` | `false` | `K1_REMOTE_COMPUTE=0` 本地建图时是否随 `k1_start_mapping.sh` 启动 RViz；远程计算仍固定不自动启动 RViz |
| `K1_EXPLORE_USE_RVIZ` | `false` | 本地或远程探索 launch 是否请求 RViz；App 默认不设置 |
| `K1_SKIP_ODOM_PREFLIGHT` | `0` | `1` 跳过 MCU bridge 后的 `/odom` 可见性等待 |
| `K1_MAPPING_USE_RVIZ` | `false` | 本地建图时是否随脚本启动 RViz |

`slam.launch.py`、`nav.launch.py` 和 `explore.launch.py` 支持 `start_sensors:=false`。树莓派计算端必须使用该参数，避免尝试打开 MUSE 上的 `/dev/mylidar`、`/dev/myimu` 或 `/dev/mymcu`。

## App 蓝牙接口

`k1muse_mobile_bridge` 使用经典蓝牙 RFCOMM/SPP。帧格式固定为：

```text
magic[4] = "K1MB"
version  = 1
type     = uint8
flags    = uint16
seq      = uint32
len      = uint32
crc32    = uint32 payload CRC32
payload  = len bytes
```

所有数值小端序。`magic` 按 `K1MB` 原样写入。

### 帧优先级

发送队列使用两级优先级控制蓝牙带宽：

| 优先级 | 帧类型 | 说明 |
|---|---|---|
| `NORMAL` | `HELLO`, `HEARTBEAT`, `ROBOT_POSE`, `BRIDGE_STATUS`, `MAP_CONTROL_STATUS`, `TELEOP_CMD` 回执 | 正常发送，不限制 |
| `LOW` | `MAP_TILE`, `MAP_INFO`, `NAV_PATH`, `MAP_LIBRARY_LIST` | 受队列限制 |

遥控活跃时（最近 1.5s 内收到 `TELEOP_CMD`），所有 `LOW` 帧从队列清除并不再入队，确保遥控指令不被地图数据阻塞。

### 主要消息类型

| 类型 | 数值 | 方向 | 说明 |
|---|---:|---|---|
| `HELLO` | `0x01` | K1 -> App | 链路问候 |
| `HEARTBEAT` | `0x02` | K1 -> App | 心跳 |
| `MAP_INFO` | `0x10` | K1 -> App | 地图元信息（宽、高、分辨率、origin） |
| `MAP_TILE` | `0x11` | K1 -> App | 地图 tile，支持 raw/zlib 编码 |
| `ROBOT_POSE` | `0x20` | K1 -> App | `map -> base_footprint` 位姿（复合 TF 查询） |
| `TELEOP_CMD` | `0x30` | App -> K1 | `vx`、`vy`、`omega` 手动速度 |
| `STOP` | `0x31` | App -> K1 | 发布零 `/cmd_vel` |
| `NAV_PATH` | `0x43` | K1 -> App | 自动探索/导航路径抽样 |
| `BRIDGE_CONTROL` | `0x48` | App -> K1 | 启动、停止、查询 full bridge |
| `BRIDGE_STATUS` | `0x49` | K1 -> App | bridge 状态 |
| `MAP_CONTROL` | `0x50` | App -> K1 | 建图启动、保存、停止、查询 |
| `MAP_CONTROL_STATUS` | `0x51` | K1 -> App | 建图状态和保存路径 |
| `MAP_LIBRARY_REQUEST` | `0x60` | App -> K1 | 地图库列表、加载、区域读写 |
| `MAP_LIBRARY_STATUS` | `0x61` | K1 -> App | 地图库命令结果 |
| `MAP_LIBRARY_LIST` | `0x62` | K1 -> App | 地图库 `.yaml` 列表 |
| `MAP_REGIONS_DATA` | `0x63` | K1 -> App | 地图区域 JSON |
| `ERROR` | `0x7F` | K1 -> App | 协议错误 |

### MAP_CONTROL 命令

| 命令 | 数值 | 说明 |
|---|---:|---|
| `START_MAPPING` | `1` | 启动正式建图脚本 |
| `SAVE_MAP_MANUAL` | `2` | 保存 `manual_*` 地图 |
| `STOP_MAPPING` | `3` | 当前 App 路径使用 `k1_stop_mapping.sh fast` |
| `QUERY_MAPPING` | `4` | 查询建图状态 |

### MAP_LIBRARY_REQUEST 命令

| 命令 | 数值 | 说明 |
|---|---:|---|
| `LIST_MAPS` | `1` | 列出地图目录下的 `.yaml/.yml` |
| `LOAD_MAP` | `2` | 读取指定地图并通过 `MAP_INFO/MAP_TILE` 发送 |
| `READ_REGIONS` | `3` | 读取同名 `.regions.json` |
| `SAVE_REGIONS` | `4` | 保存同名 `.regions.json` |

### 地图传输参数

这些参数在 `mobile_bridge.yaml` 中配置，影响蓝牙地图传输性能：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `tile_size` | 32 | 每 tile 像素边长，影响每帧大小 |
| `max_queue_frames` | 64 | 发送队列最大帧数 |
| `max_low_queue_frames` | 4 | LOW 优先级帧最大排队数 |
| `max_map_tiles_per_update` | 8 | 每次 `/map` 回调最多发送 tile 数 |
| `map_publish_hz` | 2.0 | 地图变化检查频率 |
| `pose_publish_hz` | 10.0 | 机器人位姿发送频率 |
| `path_publish_hz` | 1.0 | 导航路径发送频率 |
| `teleop_timeout_ms` | 300 | 遥控超时后自动停止（ms） |

### ROBOT_POSE TF 查询机制

`send_pose()` 使用 2D 复合 TF fallback 策略：

1. 先直接查 `map -> base_footprint` at `TimePointZero`（一次查询）
2. 失败时分别查 `map -> odom` 和 `odom -> base_footprint` at `TimePointZero`，用 2D 合成 `(x, y, yaw)`

这避免了因 `map -> odom` 和 `odom -> base_footprint` 缓存时间不重叠导致的位姿发送卡住。

## 外部使用规则

- 需要控制底盘时，先确认 `mcu_bridge_node` 以 `enable_cmd_vel_output:=true` 运行。
- 只读建图或外部遥控时，保持 `enable_cmd_vel_output:=false`，避免板端向 MCU 输出控制。
- 不要从外部直接操作树莓派上的地图文件作为 App 地图库；App 地图库以 MUSE 的 `maps/` 目录为准。
- MUSE 手动 RViz 只需要订阅分布式 ROS graph 中的 `/map`、`/scan`、`/odom` 和 TF，不需要在 App 启动时自动打开。
- SLAM/Nav 只消费过滤后的 `/scan`，单独调试雷达时才使用独立 `ld06.launch.py` 的 `/scan`。
- IMU 当前不参与 `/odom` 融合，只作为 Cartographer 可选输入。
- 地图保存后同步回本机仓库，再纳入 Git。
- 不要把 `/home/bianbu/tmp_sh` 当作产品入口。
- 树莓派上的 MPPI overlay（`/home/user/mppi_overlay/`）是必要的运行时补丁，删除后探索/导航的 `controller_server` 会因 SIGILL 崩溃。
