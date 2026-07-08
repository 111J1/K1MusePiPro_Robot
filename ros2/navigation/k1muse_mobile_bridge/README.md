# k1muse_mobile_bridge

`k1muse_mobile_bridge` 是 K1 侧的 ROS2 蓝牙桥接包，服务 Android RobotController 的 K1 Map/Nav 模式。它通过经典蓝牙 RFCOMM/SPP 把实时 `/map` 和机器人位姿发给手机，同时接收手机发来的手动控制指令。

## 运行职责

- `mobile_supervisor.py` 开机常驻，低资源监听手机连接，只处理 bridge 生命周期控制。
- `mobile_bridge_node` 由 supervisor 按需启动，负责 ROS2 地图、位姿、手动控制和建图命令。
- `bluetooth_pairing_agent.py` 常驻 BlueZ default agent，负责自动接受配对和授权。
- 订阅 `/map`，消息类型为 `nav_msgs/msg/OccupancyGrid`。
- 向 Android 发送 `MAP_INFO` 和变化的 `MAP_TILE`。
- 从 TF 查询 `map -> base_footprint`，向 Android 发送 `ROBOT_POSE`。
- 接收 `TELEOP_CMD`，发布 `/cmd_vel`。
- 接收 `STOP`，发布零速度 `/cmd_vel`。
- 接收 `MAP_CONTROL`，调用 SLAM 工作区安装的正式建图脚本。
- 接收 `MAP_LIBRARY_REQUEST`，列出、加载地图并读写同名区域 JSON。
- 自动探索运行时订阅 `/plan`，向 Android 发送 `NAV_PATH`。
- 接收 `STOP_BRIDGE`，退出完整 bridge，让 supervisor 恢复等待连接。
- 手动控制帧超时时自动发布零速度，避免断联后继续运动。

本包不直接打开 `/dev/mymcu`。STM32 串口仍由 `k1muse_mcu_bridge` 独占；`mcu_bridge_node` 同生命周期提供 `/odom`/TF 和可选 `/cmd_vel` 路由，底盘运动经由 `/cmd_vel` 进入 MCU 桥接节点。

默认分布式运行时，App 仍只连接 MUSE。`MAP_CONTROL` 调用的脚本会在 MUSE 启动 MCU bridge 和 sensors，并通过 SSH 让树莓派运行 Cartographer/Nav2/自动探索。App 协议、命令号、地图目录和地图库接口不变。

对应 Android App 仓库：`E:\Android\Projects\RobotController`。

## 协议概要

手机和 K1 的帧都使用小端二进制包头：

```text
magic[4]  = "K1MB"
version   = 1
type      = 消息类型
flags     = uint16
seq       = uint32
len       = uint32 payload 长度
crc32     = uint32 payload CRC32
payload   = len 字节
```

主要消息类型：

| 类型 | 数值 | 方向 | 含义 |
| --- | ---: | --- | --- |
| `HELLO` | `0x01` | K1 -> 手机 | 桥接节点问候帧 |
| `HEARTBEAT` | `0x02` | K1 -> 手机 | 链路心跳 |
| `MAP_INFO` | `0x10` | K1 -> 手机 | 地图元信息 |
| `MAP_TILE` | `0x11` | K1 -> 手机 | 原始或 zlib 压缩 tile |
| `ROBOT_POSE` | `0x20` | K1 -> 手机 | `x`、`y`、`yaw` |
| `TELEOP_CMD` | `0x30` | 手机 -> K1 | `vx`、`vy`、`omega` |
| `STOP` | `0x31` | 手机 -> K1 | 零速度停止 |
| `NAV_GOAL` | `0x40` | 预留 | 导航目标 |
| `NAV_CANCEL` | `0x41` | 预留 | 取消导航 |
| `NAV_STATUS` | `0x42` | 预留 | 导航状态 |
| `NAV_PATH` | `0x43` | K1 -> 手机 | 自动探索/导航路径抽样 |
| `BRIDGE_CONTROL` | `0x48` | 手机 -> K1 | bridge 启动、停止、查询 |
| `BRIDGE_STATUS` | `0x49` | K1 -> 手机 | bridge 状态和命令结果 |
| `MAP_CONTROL` | `0x50` | 手机 -> K1 | 建图启动、保存、停止、查询 |
| `MAP_CONTROL_STATUS` | `0x51` | K1 -> 手机 | 建图状态、保存路径和错误文本 |
| `MAP_LIBRARY_REQUEST` | `0x60` | 手机 -> K1 | 地图库列表、加载、区域读写 |
| `MAP_LIBRARY_STATUS` | `0x61` | K1 -> 手机 | 地图库命令结果 |
| `MAP_LIBRARY_LIST` | `0x62` | K1 -> 手机 | 地图 `.yaml/.yml` 列表 |
| `MAP_REGIONS_DATA` | `0x63` | K1 -> 手机 | 地图区域 JSON |
| `ERROR` | `0x7F` | K1 -> 手机 | 协议错误 |

`TELEOP_CMD` payload：

```text
uint32 stamp_ms
float32 vx
float32 vy
float32 omega
```

`BRIDGE_CONTROL` payload：

```text
uint32 stamp_ms
uint8 command
```

命令：

| 命令 | 数值 | 含义 |
| --- | ---: | --- |
| `START_BRIDGE` | `1` | supervisor 启动完整 `mobile_bridge_node` |
| `STOP_BRIDGE` | `2` | 完整 bridge 退出，supervisor 恢复等待 |
| `QUERY_BRIDGE` | `3` | 查询当前入口状态 |

`MAP_CONTROL` payload：

```text
uint32 stamp_ms
uint8 command
uint8 mode       # 可选：0=MANUAL, 1=AUTO
uint8 room_size  # 可选：0=SMALL, 1=MEDIUM, 2=LARGE, 3=CUSTOM
float32 custom_size_x  # 可选，仅 CUSTOM 时使用
float32 custom_size_y  # 可选，仅 CUSTOM 时使用
```

命令：

| 命令 | 数值 | 含义 |
| --- | ---: | --- |
| `START_MAPPING` | `1` | 启动 MCU bridge、传感器和 Cartographer 建图 |
| `SAVE_MAP_MANUAL` | `2` | 保存 `manual_YYYYmmdd_HHMMSS.yaml/.pgm/.pbstream` |
| `STOP_MAPPING` | `3` | 当前 App 路径快速停止建图/探索，不做自动保存 |
| `QUERY_MAPPING` | `4` | 查询建图是否运行 |

`START_MAPPING` 默认使用手动建图脚本。payload 中 `mode=1` 时使用自动探索脚本；`room_size` 会转换为探索边界，`CUSTOM` 时用 `custom_size_x/custom_size_y` 限幅后生成边界。

`MAP_LIBRARY_REQUEST` payload：

```text
uint32 stamp_ms
uint8 command
string yaml_name
string json
```

命令：

| 命令 | 数值 | 含义 |
| --- | ---: | --- |
| `LIST_MAPS` | `1` | 列出地图目录下 `.yaml/.yml` |
| `LOAD_MAP` | `2` | 加载指定地图并发送 `MAP_INFO/MAP_TILE` |
| `READ_REGIONS` | `3` | 读取同名 `.regions.json` |
| `SAVE_REGIONS` | `4` | 保存同名 `.regions.json` |

状态回包均包含 `stamp_ms`、状态、原命令、success 和 UTF-8 文本。`MAP_CONTROL_STATUS` 额外包含保存基路径。

## 构建

```bash
cd /home/bianbu/k1muse_communicate_ros
source /opt/ros/humble/setup.bash
colcon build --packages-select k1muse_mobile_bridge
source install/setup.bash
```

## 启动

产品流程推荐使用 systemd supervisor，不需要手动启动 full bridge：

```bash
sudo cp /home/bianbu/k1muse_communicate_ros/install/k1muse_mobile_bridge/share/k1muse_mobile_bridge/systemd/k1-mobile-supervisor.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now k1-mobile-supervisor.service
systemctl is-active k1-mobile-supervisor.service
```

App 流程：

1. 手机连接 K1 后先进入 supervisor，界面只显示 `Start Bridge`。
2. 点击 `Start Bridge`，supervisor 释放 RFCOMM 并启动完整 `mobile_bridge_node`。
3. App 自动重连完整 bridge。
4. 完整 bridge 负责地图显示、手动控制、建图启动/保存/停止。
5. 点击 `Stop Bridge` 后完整 bridge 退出，supervisor 重新等待手机连接。

手动调试时仍可直接启动 full bridge：

```bash
cd /home/bianbu/k1muse_communicate_ros
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run k1muse_mobile_bridge start_rfcomm_server.sh
ros2 launch k1muse_mobile_bridge mobile_bridge.launch.py
```

`/home/bianbu/tmp_sh` 只保留为手动测试脚本目录，不作为产品流程入口。

建图由 App 通过 `MAP_CONTROL` 调用安装后的正式脚本。手动模式 `START_MAPPING` 调用 `k1_start_mapping.sh`，自动模式调用 `k1_start_exploration.sh MIN_X MAX_X MIN_Y MAX_Y`。脚本会先启动 MCU bridge，再启动地图流程。默认远程计算模式下，地图流程的 Cartographer/Nav2/探索在树莓派运行，MUSE 保留硬件桥接、传感器发布和 App 入口。若要从手机 K1 Map/Nav 页面手动控制底盘，建图脚本会以 `enable_cmd_vel_output:=true` 启动 `mcu_bridge_node`，让 `/cmd_vel` 由 MCU bridge 内置路由进入底盘控制。

远程模式由 `K1_REMOTE_COMPUTE=1` 控制，默认开启；设置 `K1_REMOTE_COMPUTE=0` 可回到 MUSE 本地计算。`QUERY_MAPPING` 会同时识别 MUSE 本地进程和远程编排脚本留下的 MUSE 状态标记。

当前 `STOP_MAPPING` 会按正在运行的模式调用快速停止脚本：手动建图使用 `k1_stop_mapping.sh fast`，自动探索使用 `k1_stop_exploration.sh fast`。快速停止跳过停止前自动保存，优先保证 App 停止流程快速收敛。需要保存地图时，App 应先发送 `SAVE_MAP_MANUAL`，或人工在板端调用不带 `fast` 的停止脚本。

## RFCOMM 权限

`rfcomm listen` 可能把 `/dev/rfcomm0` 创建成：

```text
root:dialout 660
```

如果 `bianbu` 用户不在 `dialout` 组，`mobile_bridge_node` 会一直等待设备可打开，即使 `/dev/rfcomm0` 已经存在。`mobile_supervisor.py` 和 `start_rfcomm_server.sh` 都会在 `rfcomm listen` 生命周期内保留权限 watcher，持续把设备修正为：

```text
root:<当前用户主组> 660
```

期望检查结果：

```bash
ls -l /dev/rfcomm0
# crw-rw---- 1 root bianbu ... /dev/rfcomm0
```

## 配对和产品化行为

经典蓝牙配对应该是一次性的初始化步骤，不应该在每次点击 Link 时都要求双端确认。

K1 使用 `k1-bluetooth-pairing-agent.service` 常驻注册 BlueZ default agent：

- agent capability 为 `NoInputNoOutput`。
- 手机发起 pairing、confirmation、authorization 时，K1 自动接受。
- 配对成功后自动把手机标记为 trusted。
- 服务随系统启动，避免 K1 重启后又回到桌面端手动确认。

推荐流程：

1. 首次使用时让手机和 K1 完成配对。
2. 正常情况下，只需要在手机端点击配对；K1 侧 agent 会自动确认并 trust。
3. 如需手动确认 trusted 状态，可在 K1 上查看：

   ```bash
   bluetoothctl
   devices
   devices Trusted
   quit
   ```

4. 只在初始化阶段保持 K1 discoverable/pairable。
5. 日常连接和断开通过 App 的 K1 Map/Nav `Link` / `Off` 按钮完成。

Android K1 客户端会先尝试固定 RFCOMM channel 1，再退回 UUID/SPP 发现，以匹配 `start_rfcomm_server.sh`。这样可以减少 SDP 或安全确认弹窗。如果仍然每次连接都要求确认，删除 Android 和 K1 两边的旧配对记录，重新配对一次，并在 K1 上 trust 手机。

agent 服务安装：

```bash
sudo cp /home/bianbu/k1muse_communicate_ros/install/k1muse_mobile_bridge/share/k1muse_mobile_bridge/systemd/k1-bluetooth-pairing-agent.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now k1-bluetooth-pairing-agent.service
systemctl status k1-bluetooth-pairing-agent.service
```

## 验证

```bash
source /opt/ros/humble/setup.bash
source /home/bianbu/k1muse_communicate_ros/install/setup.bash
source /home/bianbu/k1muse_slam_ros/install/setup.bash

ros2 node list
ros2 topic echo --once /map --field info
ros2 topic info /cmd_vel -v
journalctl -u k1-mobile-supervisor.service -f
tail -f /home/bianbu/.ros/k1muse_slam_nav/logs/*.log
```

App 手动建图回归时，先重启 supervisor，再边操作 App 边观察：

```bash
sudo systemctl restart k1-mobile-supervisor.service
systemctl is-active k1-mobile-supervisor.service
journalctl -u k1-mobile-supervisor.service -f
```

另开终端检查 ROS graph：

```bash
watch -n 1 '
echo === nodes ===
ros2 node list | grep -E "mobile_bridge|mcu_bridge|cartographer|occupancy|robot_state|ldlidar|ybimu" || true
echo === topics ===
ros2 topic list | grep -E "^/(cmd_vel|map|scan|scan_raw|odom|imu/data|tf|tf_static)$" || true
'
```

手动控制链路：

```bash
timeout 15 ros2 topic echo /cmd_vel
ros2 topic info /cmd_vel -v
ros2 topic info /mcu/chassis/mov -v
tail -n 120 /home/bianbu/.ros/k1muse_mcu_bridge/logs/mcu_bridge.log
```

期望现象：

- `k1-mobile-supervisor.service` 为 `active`。
- 手机点击 Link 后先连接 supervisor。
- 点击 `Start Bridge` 后 full bridge 启动并出现 `/mobile_bridge_node`。
- `mobile_bridge_node` 日志出现 `Mobile RFCOMM connected on /dev/rfcomm0`。
- 点击 `Start Mapping` 后出现 `/cartographer_node` 和 `/cartographer_occupancy_grid_node`。
- 默认远程计算模式下，`/cartographer_node` 进程在树莓派上；MUSE 侧应仍能看到 `/map` topic。
- 推动手机摇杆时日志能看到非零 `vx` 或 `vy`。
- 拨动手机角速度滑块时日志能看到非零 `wz`。
- `Save Map` 生成 `manual_*.yaml/.pgm/.pbstream`。
- `Stop Mapping` 使用 fast 停止路径，停止建图和 MCU bridge；保存地图应先点 `Save Map` 生成 `manual_*`。
- `Stop Bridge` 后 full bridge 退出，supervisor 回到等待下一次手机连接的状态。

## 安全边界

- 只有 Android K1 Map/Nav 的手动模式开启时才发布手动控制命令。
- 手动模式开启后，Android 每 50 ms 发送一次 `TELEOP_CMD`。
- `teleop_timeout_ms` 默认 300 ms。超过该时间没有收到控制帧时，K1 发布零速度 `/cmd_vel`。
- 隐藏手动控制、断开连接或返回页面都会发送 `STOP`。
- Direct SPP 和 K1 Map/Nav 不应同时主动驱动底盘。App 会在模式切换时关闭另一条 App 侧蓝牙链路，但无法停止外部终端已经启动的板端进程。

## 蓝牙传输优化

`mobile_bridge.yaml` 中的参数控制蓝牙 RFCOMM 传输性能。核心原则是：**遥控指令（NORMAL 优先级）永远优先于地图数据（LOW 优先级）**。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `tile_size` | 32 | 每 tile 像素边长，越小单帧越小但 tile 越多 |
| `max_queue_frames` | 64 | 发送队列最大帧数 |
| `max_low_queue_frames` | 4 | LOW 优先级帧（地图 tile 等）最多排队数 |
| `max_map_tiles_per_update` | 8 | 每次 `/map` 回调最多发送的 tile 数 |
| `map_publish_hz` | 2.0 | 地图变化检查频率 (Hz) |
| `pose_publish_hz` | 10.0 | 机器人位姿发送频率 (Hz) |
| `path_publish_hz` | 1.0 | 导航路径发送频率 (Hz) |
| `teleop_timeout_ms` | 300 | 遥控超时后自动停止 (ms) |

**优先级机制：**

1. `LOW` 帧最多排队 `max_low_queue_frames` 个，超出直接丢弃
2. 收到遥控指令后立即清除队列中所有 `LOW` 帧 (`purge_low_priority_frames()`)
3. 遥控活跃窗口（1.5s）内禁止 `LOW` 帧入队
4. 发送线程在取出 `LOW` 帧时二次检查遥控状态

**地图传输流控：**

每收到一次 `/map` 更新，`send_changed_tiles()` 做增量 tile hash 对比，只发变化 tile，且：
- 最多发 `max_map_tiles_per_update` 个
- 遥控活跃时立即返回，不发任何 tile
- tile 只有成功入队后才更新 hash（确保未传输 tile 下次仍被检测为变化）

**ROBOT_POSE 发送：**

`send_pose()` 使用 2D 复合 TF fallback：
1. 直接查 `map -> base_footprint` at `TimePointZero`
2. 失败时分别查 `map -> odom` 和 `odom -> base_footprint` at `TimePointZero`，做 2D 合成 (x, y, yaw)

这避免了 `map -> odom` 和 `odom -> base_footprint` 缓存时间不重叠时的位姿发送卡住。
