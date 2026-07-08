# MCU 桥接节点设计方案

## 目录结构

```
k1muse_communicate_ros/src/
    ├── udev/
    │   ├── 99-ros2-communicate.rules       # ttyS4 → /dev/mymcu
    │   └── README.md
    └── k1muse_mcu_bridge/
        ├── CMakeLists.txt
        ├── package.xml
        ├── msg/                        # 自定义 ROS 消息
        │   ├── ChassisStatus.msg       # 上行：底盘状态（32B 原样）
        │   └── ChassisMov.msg          # 下行：底盘 MOV 命令（13B 原样）
        ├── srv/                        # 自定义 ROS 服务
        │   ├── ChassisStop.srv         # 空请求
        │   └── ChassisOdom.srv         # direction, x, y
        ├── include/k1muse_mcu_bridge/
        │   ├── cmd_vel_router.hpp      # /cmd_vel 到 chassis MOV/STOP 的可选路由
        │   ├── odom_publisher.hpp      # /odom 和 odom -> base_footprint
        │   ├── protocol.hpp            # 帧格式、CRC8、全部枚举（含 arm/lift）
        │   ├── serial_port.hpp         # 串口操作封装
        │   └── mcu_bridge_node.hpp     # 节点主类
        ├── scripts/
        │   ├── k1_start_mcu_bridge.sh
        │   └── k1_stop_mcu_bridge.sh
        └── src/
            ├── cmd_vel_router.cpp
            ├── odom_publisher.cpp
            ├── protocol.cpp
            ├── serial_port.cpp
            └── mcu_bridge_node.cpp
```

---

## 一、udev 规则

`99-ros2-communicate.rules`：

```
KERNEL=="ttyS4", MODE:="0777", SYMLINK+="mymcu"
```

波特率 115200 8N1 由节点代码设置。

---

## 二、协议常量（protocol.hpp）

与 STM32 端 `mdl_control_protocol.h` 保持一致。**枚举定义全部目标模块**，即使本阶段只实现 chassis，代码层面已预留给 arm/lift 的解析入口。

```cpp
// 帧格式
constexpr uint8_t SOF1 = 0xA5;
constexpr uint8_t SOF2 = 0x5A;
constexpr uint8_t MAX_PAYLOAD = 64;
constexpr uint8_t HEADER_SIZE = 7;
constexpr uint8_t MAX_FRAME = HEADER_SIZE + MAX_PAYLOAD + 1;  // 72

// 源标识
enum class Src : uint8_t {
    NONE = 0x00,
    BT   = 0x01,
    HOST = 0x02,    // 本节点固定使用
    MCU  = 0x10,
};

// 模块（全部定义，方便后续扩展）
enum class Target : uint8_t {
    SYSTEM  = 0x00,
    CHASSIS = 0x01,
    ARM     = 0x02,
    LIFT    = 0x03,
};

// 底盘命令
enum class ChassisCmd : uint8_t {
    STOP = 0x00,
    MOV  = 0x01,
    ODOM = 0x02,
};

// 状态帧命令号
constexpr uint8_t STATUS_CMD = 0x80;
```

**扩展时**：在此文件追加 `ArmCmd`、`LiftCmd` 枚举即可。

---

## 三、ROS 接口（本阶段：仅 chassis）

### 3.1 上行 Topic（MCU → ROS）

| Topic | 消息 | Payload |
|---|---|---|
| `/mcu/chassis/status` | `ChassisStatus.msg` | 32B |
| `/odom` | `nav_msgs/Odometry` | 由 chassis status 同生命周期换算发布 |
| `odom -> base_footprint` | TF | 默认发布，可由 `publish_tf` 控制 |

**ChassisStatus.msg**（与 MCU `ctrl_chassis_status_payload_t` 一致）：
```
uint32 tick_ms
uint8  state
uint8  move_cs
uint8  motor_block_flags
uint8  reserved
float32 wcs_vx
float32 wcs_vy
float32 omega
float32 wcs_x
float32 wcs_y
float32 wcs_direction
```

**扩展 arm/lift 时**：新增 `ArmStatus.msg`、`LiftStatus.msg`，在 `CMakeLists.txt` 的 `rosidl_generate_interfaces` 中追加，节点中新增 publisher。

### 3.2 下行 Topic（ROS → MCU）

| Topic | 消息 | 说明 |
|---|---|---|
| `/mcu/chassis/mov` | 自定义消息 | 下行 MOV 帧 |
| `/cmd_vel` | `geometry_msgs/Twist` | 可选订阅，`enable_cmd_vel_output=true` 时路由到 `/mcu/chassis/mov` 或 `/mcu/chassis/stop` |

`msg/ChassisMov.msg`（与 MCU `ctrl_chassis_mov_payload_t` 一致）：
```
uint8  move_cs     # 0=LCS, 1=WCS
float32 direction
float32 v
float32 omega
```

### 3.3 下行 Service

| Service | 请求字段 | 响应 |
|---|---|---|
| `/mcu/chassis/stop` | 空 | 空 |
| `/mcu/chassis/odom` | `float32 direction`, `float32 x`, `float32 y` | 空 |

**扩展 arm/lift 时**：新增对应的 `.srv` 文件和 service server，模式完全一致。

---

## 四、节点架构

**单节点，双线程。**

```
┌──────────────────────────────────────────────────────────┐
│                   mcu_bridge_node                         │
│                                                           │
│  ┌──────────────┐          ┌────────────────────────────┐ │
│  │  ReadThread   │──发布──→│  /mcu/chassis/status        │ │
│  │               │         │  (后续 + /mcu/arm/status    │ │
│  │  字节流→SOF   │         │        + /mcu/lift/status)  │ │
│  │  →CRC8→反序列│         └────────────────────────────┘ │
│  │  →TARGET分发  │                                        │
│  └──────┬───────┘                                        │
│         │                                                │
│     /dev/mymcu                                           │
│         │                                                │
│  ┌──────┴───────┐          ┌────────────────────────────┐ │
│  │  主线程       │←─────────│  /mcu/chassis/mov (Topic)   │ │
│  │              │←─────────│  /mcu/chassis/stop (Service) │ │
│  │  SEQ→组帧   │←─────────│  /mcu/chassis/odom (Service) │ │
│  │  →CRC8→写   │          │  (后续 + arm/*, lift/*)      │ │
│  └──────────────┘          └────────────────────────────┘ │
└──────────────────────────────────────────────────────────┘
```

### 读线程

```
loop:
  read bytes from serial port → append to buffer
  while buffer has complete frame:
    find 0xA5 0x5A
    read SRC + TARGET + CMD + SEQ + LEN (5 bytes)
    read LEN bytes payload
    read 1 byte CRC
    if CRC8 mismatch: discard, skip 1 byte, continue

    if SRC == MCU (0x10) && CMD == STATUS_CMD (0x80):
      switch TARGET:
        CHASSIS → deserialize → publish /mcu/chassis/status
        ARM     → (预留，暂不处理)
        LIFT    → (预留，暂不处理)
    else:
      // 非状态帧（如 BT 中继），暂忽略
```

**扩展 arm/lift 时**：在 switch 的 ARM/LIFT 分支中分别 deserialize + publish。

### 主线程

**Subscriber `/mcu/chassis/mov`**：
```
callback(msg):
  payload = serialize(msg)   // 13 bytes
  frame = build(Target::CHASSIS, ChassisCmd::MOV, seq++, payload)
  write(frame)
```

**Service `/mcu/chassis/stop`**：
```
callback(req, resp):
  frame = build(Target::CHASSIS, ChassisCmd::STOP, seq++, empty)
  write(frame)
  return resp
```

**Service `/mcu/chassis/odom`**：
```
callback(req, resp):
  payload = serialize(req.direction, req.x, req.y)  // 12 bytes
  frame = build(Target::CHASSIS, ChassisCmd::ODOM, seq++, payload)
  write(frame)
  return resp
```

**build 函数**（通用）：
```
输入: target, cmd, seq, payload
输出: [SOF1, SOF2, SRC_HOST, target, cmd, seq, len, payload..., crc8]
```

**扩展 arm/lift 时**：新增 subscriber/service callback，调用同一个 `build()`，仅 target/cmd/payload 不同。

---

## 五、关键细节

| 项目 | 决策 |
|---|---|
| **SRC** | 固定 0x02（HOST） |
| **SEQ** | 节点内部自增 uint8_t，溢出回绕 |
| **CRC8** | ATM 多项式 0x07，初始 0x00，覆盖 SRC~PAYLOAD |
| **字节序** | 全小端，float32 用 `memcpy` |
| **串口读** | 非阻塞，缓冲区 ≥ 256 字节 |
| **串口写** | 加锁保护，避免读写线程竞争 |
| **重连** | 串口异常 → 1s 间隔重试，恢复后继续 |
| **CRC 失败** | 丢弃帧，跳过 1 字节重新搜 SOF |

---

## 六、扩展模式

以新增 arm 为例，后续只需三步：

1. **定义接口**：新增 `msg/ArmStatus.msg`、`srv/ArmStop.srv`、`srv/ArmMoveXyz.srv` 等
2. **CMakeLists.txt**：`rosidl_generate_interfaces` 追加新文件
3. **节点代码**：各约 5 行
   - 成员变量加一个 `rclcpp::Publisher<ArmStatus>::SharedPtr`
   - 构造函数加 `create_publisher` + `create_service` 调用
   - 读线程 switch 的 ARM 分支加 deserialize + publish
   - 主线程加 service callback

无需改动 `protocol.hpp`（枚举已预定义）、`serial_port`、`build()` 等公共逻辑。

---

## 七、里程计和 `/cmd_vel` 路由

`mcu_bridge_node` 现在把桥接、里程计发布和速度路由放在同一个生命周期内，避免多个节点同时依赖底盘状态。

### `/odom` 发布

默认参数：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `odom_frame` | `odom` | 里程计父坐标系 |
| `base_frame` | `base_footprint` | 车体投影坐标系 |
| `publish_tf` | `true` | 是否发布 `odom -> base_footprint` |
| `odom_publish_rate_hz` | `50.0` | 发布频率 |

`/odom` 来自 `/mcu/chassis/status` 中的 `wcs_x`、`wcs_y`、`wcs_direction`、`wcs_vx`、`wcs_vy`、`omega`。协方差使用温和 2D 数值，避免 RViz 因极大协方差显示异常。

### `/cmd_vel` 路由

默认参数：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `enable_cmd_vel_output` | `false` | 默认只读，不向 MCU 发 MOV/STOP |
| `vx_limit` | `1.4` | `linear.x` 限幅 |
| `vy_limit` | `1.4` | `linear.y` 限幅 |
| `omega_limit` | `3.7` | `angular.z` 限幅 |
| `cmd_vel_timeout_ms` | `250` | 超时后发零速/STOP |
| `cmd_vel_rate_hz` | `20` | MOV 重发频率 |
| `zero_vel_send_stop` | `true` | 全零速度调用 `/mcu/chassis/stop` |

只读建图或排查时保持默认：

```bash
ros2 run k1muse_mcu_bridge mcu_bridge_node
```

App 手动控制或 Nav2 控车时显式开启：

```bash
ros2 run k1muse_mcu_bridge mcu_bridge_node --ros-args -p enable_cmd_vel_output:=true
```

安装脚本 `k1_start_mcu_bridge.sh` 当前会先确认 `/dev/mymcu` 有数据，再以 `enable_cmd_vel_output:=true` 启动节点；`k1_stop_mcu_bridge.sh` 停止前会尝试发布一次零 `/cmd_vel`。

---

## 八、依赖

**package.xml**：
- `rclcpp`
- `geometry_msgs`
- `nav_msgs`
- `tf2`
- `tf2_ros`
- `rosidl_default_generators` / `rosidl_default_runtime`

**CMakeLists.txt**：
- `find_package(rclcpp REQUIRED)`
- `find_package(geometry_msgs REQUIRED)`
- `find_package(nav_msgs REQUIRED)`
- `find_package(tf2 REQUIRED)`
- `find_package(tf2_ros REQUIRED)`
- `rosidl_generate_interfaces(...)` 生成 msg/srv
- 可执行文件 `mcu_bridge_node`

