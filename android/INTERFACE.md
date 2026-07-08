# RobotController Interface

本文档描述外部软件如何与 RobotController Android App 通信。面向对象包括：

- STM32 侧 HC-05 固件：接收 Direct SPP 底盘控制帧。
- K1 侧 supervisor / mobile bridge：通过经典蓝牙 RFCOMM 与 App 双向交换地图、状态和控制命令。

RobotController 本身不提供 HTTP、WebSocket、ADB 或文件接口。当前所有运行时通信都走经典蓝牙。

## 1. 通信角色

### 1.1 Direct SPP

链路：

```text
RobotController App -> Android Bluetooth SPP -> HC-05 -> STM32 firmware
```

App 是客户端，STM32/HC-05 是服务端或串口接收端。App 只写控制帧，不依赖 STM32 回包。

适用场景：

- 基础底盘直连控制。
- 不经过 K1 或 ROS 的保底控制链路。

### 1.2 K1 Map/Nav

链路：

```text
RobotController App <-> Android Bluetooth RFCOMM/SPP <-> K1 supervisor / mobile bridge
```

App 是蓝牙客户端，K1 侧需要监听 RFCOMM。App 连接时优先尝试 channel 1，然后退回 SPP UUID。

K1 侧推荐暴露两个逻辑端点：

- `supervisor`：常驻轻量服务，负责启动完整 bridge。
- `bridge`：完整服务，负责地图、位姿、路径、手动控制、建图和地图库。

App 通过 `HELLO` payload 中的名称判断当前端点类型：名称包含 `supervisor` 时视为 supervisor，否则视为 bridge。

## 2. 蓝牙要求

App 只列出 Android 系统已配对设备。外部设备需要先在系统蓝牙设置中完成配对。

K1 端建议：

```bash
sudo systemctl enable --now k1-bluetooth-pairing-agent.service
sudo systemctl enable --now k1-mobile-supervisor.service
```

首次配对后，建议 trust 手机：

```bash
bluetoothctl
devices
trust <PHONE_MAC>
quit
```

K1 侧 RFCOMM 建议使用 channel 1。App 的连接尝试顺序为：

1. RFCOMM channel 1
2. insecure RFCOMM channel 1
3. SPP UUID `00001101-0000-1000-8000-00805F9B34FB`
4. insecure SPP UUID

## 3. Direct SPP 帧

Direct SPP 由 App 发送，STM32 固件接收。

字节序：payload 中的浮点数使用 little-endian IEEE 754 float32。

帧格式：

```text
SOF1 SOF2 SRC TARGET CMD SEQ LEN PAYLOAD CRC8
```

字段：

| 字段 | 长度 | 说明 |
| --- | ---: | --- |
| `SOF1` | 1 | 固定 `0xA5` |
| `SOF2` | 1 | 固定 `0x5A` |
| `SRC` | 1 | 固定 `0x01`，表示蓝牙来源 |
| `TARGET` | 1 | 固定 `0x01`，表示底盘目标 |
| `CMD` | 1 | 命令 |
| `SEQ` | 1 | App 侧递增序号，按 8 bit 回绕 |
| `LEN` | 1 | payload 字节数 |
| `PAYLOAD` | `LEN` | 命令数据 |
| `CRC8` | 1 | CRC-8/ATM |

CRC 覆盖范围：

```text
SRC TARGET CMD SEQ LEN PAYLOAD
```

CRC 参数：

- 多项式：`0x07`
- 初值：`0x00`
- 无反射
- 无 xorout

命令：

| 命令 | CMD | Payload |
| --- | ---: | --- |
| `STOP` | `0x00` | 空 |
| `MOV` | `0x01` | `uint8 mode`, `float32 direction`, `float32 velocity`, `float32 omega` |
| `ODOM` | `0x02` | `float32 direction`, `float32 x`, `float32 y` |

坐标模式：

| 模式 | 值 | 说明 |
| --- | ---: | --- |
| `LCS` | `0x00` | 车体坐标系 |
| `WCS` | `0x01` | 世界坐标系 |

App 输出限制：

- 最大线速度：`1.4 m/s`
- 最大角速度：`3.7 rad/s`
- 速度档位：25%、50%、75%、100%

Direct SPP 行为：

- 运动中 App 约每 50 ms 发送一次 `MOV`。
- 静止时 App 发送低频 `STOP` keepalive。
- 断开连接前 App 尝试发送一次 `STOP`。
- `Master` 关闭、页面返回、控件回中或发送失败时，App 会回到停止状态。

## 4. K1MB 通用帧

K1 Map/Nav 使用 `K1MB` 二进制协议。所有多字节整数和浮点数均为 little-endian。

帧头固定 20 字节：

```text
magic[4] version[1] type[1] flags[2] seq[4] len[4] payload_crc32[4] payload[len]
```

字段：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `magic` | bytes[4] | 固定 ASCII `K1MB` |
| `version` | uint8 | 当前固定 `1` |
| `type` | uint8 | 消息类型 |
| `flags` | uint16 | 当前 App 发送为 `0` |
| `seq` | uint32 | 发送方递增序号 |
| `len` | uint32 | payload 长度 |
| `payload_crc32` | uint32 | payload 的 CRC32 |
| `payload` | bytes | 消息体 |

payload 最大长度：App 解析器接受最大 `4 MiB`。

字符串编码：

```text
uint16 byte_len
utf8 bytes[byte_len]
```

## 5. K1MB 消息类型

| 类型 | 值 | 方向 | 说明 |
| --- | ---: | --- | --- |
| `HELLO` | `0x01` | K1 -> App | 端点识别 |
| `HEARTBEAT` | `0x02` | K1 -> App | 心跳，可选 |
| `MAP_INFO` | `0x10` | K1 -> App | 地图元信息 |
| `MAP_TILE` | `0x11` | K1 -> App | 地图 tile |
| `ROBOT_POSE` | `0x20` | K1 -> App | 机器人位姿 |
| `TELEOP_CMD` | `0x30` | App -> K1 | 手动控制速度 |
| `STOP` | `0x31` | App -> K1 | 停止手动输出 |
| `NAV_PATH` | `0x43` | K1 -> App | 导航路径 |
| `BRIDGE_CONTROL` | `0x48` | App -> K1 | bridge 生命周期命令 |
| `BRIDGE_STATUS` | `0x49` | K1 -> App | bridge 状态 |
| `MAP_CONTROL` | `0x50` | App -> K1 | 建图控制 |
| `MAP_CONTROL_STATUS` | `0x51` | K1 -> App | 建图状态 |
| `MAP_LIBRARY_REQUEST` | `0x60` | App -> K1 | 地图库请求 |
| `MAP_LIBRARY_STATUS` | `0x61` | K1 -> App | 地图库请求结果 |
| `MAP_LIBRARY_LIST` | `0x62` | K1 -> App | 地图列表 |
| `MAP_REGIONS_DATA` | `0x63` | K1 -> App | 区域 JSON |
| `ERROR` | `0x7F` | K1 -> App | 错误 |

## 6. K1 -> App payload

### 6.1 HELLO

```text
string endpoint_name
```

要求：

- supervisor 端建议发送类似 `k1-mobile-supervisor`。
- 完整 bridge 建议发送类似 `k1-mobile-bridge`。
- App 收到名称包含 `supervisor` 的 `HELLO` 后只开放 `Start Bridge`。
- App 收到其他 `HELLO` 后视为完整 bridge，并开放地图、建图、地图库和手动控制能力。

### 6.2 MAP_INFO

```text
uint32 map_version
uint32 width
uint32 height
float32 resolution
float32 origin_x
float32 origin_y
float32 origin_yaw
uint16 tile_size
```

K1 应在发送 tile 前先发送 `MAP_INFO`。当地图尺寸、分辨率、原点或版本变化时，应重新发送 `MAP_INFO`。

### 6.3 MAP_TILE

```text
uint32 map_version
uint32 x
uint32 y
uint16 width
uint16 height
uint32 raw_len
uint8 encoding
bytes encoded_data
```

`encoding`：

| 值 | 说明 |
| ---: | --- |
| `0` | raw occupancy bytes |
| `1` | zlib 压缩后的 occupancy bytes |

要求：

- 解压后的长度必须等于 `raw_len`。
- `raw_len` 必须等于 `width * height`。
- tile 必须位于当前 `MAP_INFO` 范围内。
- `map_version` 必须等于当前地图版本，否则 App 会丢弃该 tile。

occupancy 建议值：

- `< 0`：unknown
- `0`：free
- `100` 或更大：occupied
- `1..99`：中间占据概率

### 6.4 ROBOT_POSE

```text
uint32 stamp_ms
float32 x
float32 y
float32 yaw
```

坐标应与 `MAP_INFO` 的 map 坐标系一致。

### 6.5 NAV_PATH

```text
uint32 stamp_ms
uint32 path_id
uint16 count
repeat count:
  float32 x
  float32 y
```

App 要求 `count >= 2` 才会解析路径。

### 6.6 BRIDGE_STATUS

```text
uint32 stamp_ms
uint8 state
uint8 command
uint8 success
string message
```

`state`：

| 状态 | 值 |
| --- | ---: |
| `SUPERVISOR` | `0` |
| `STARTING` | `1` |
| `ONLINE` | `2` |
| `STOPPING` | `3` |
| `ERROR` | `4` |

`command`：

| 命令 | 值 |
| --- | ---: |
| `START_BRIDGE` | `1` |
| `STOP_BRIDGE` | `2` |
| `QUERY_BRIDGE` | `3` |

### 6.7 MAP_CONTROL_STATUS

```text
uint32 stamp_ms
uint8 state
uint8 command
uint8 success
string map_base
string message
```

`state`：

| 状态 | 值 |
| --- | ---: |
| `IDLE` | `0` |
| `STARTING` | `1` |
| `MAPPING` | `2` |
| `SAVING` | `3` |
| `STOPPING` | `4` |
| `ERROR` | `5` |

`command`：

| 命令 | 值 |
| --- | ---: |
| `START_MAPPING` | `1` |
| `SAVE_MAP_MANUAL` | `2` |
| `STOP_MAPPING` | `3` |
| `QUERY_MAPPING` | `4` |

`map_base` 用于显示当前保存或加载的地图基名。

### 6.8 MAP_LIBRARY_STATUS

```text
uint32 stamp_ms
uint8 command
uint8 success
string message
```

`command`：

| 命令 | 值 |
| --- | ---: |
| `LIST_MAPS` | `1` |
| `LOAD_MAP` | `2` |
| `READ_REGIONS` | `3` |
| `SAVE_REGIONS` | `4` |

### 6.9 MAP_LIBRARY_LIST

```text
uint32 stamp_ms
uint16 count
repeat count:
  string yaml_name
  string image_name
  uint8 has_regions
```

App 使用该列表填充 Edit 模式中的地图选择框。

### 6.10 MAP_REGIONS_DATA

```text
uint32 stamp_ms
string yaml_name
uint8 exists
string json
```

`exists = 0` 表示该地图没有区域 sidecar。此时 `json` 可为空字符串。

## 7. App -> K1 payload

### 7.1 BRIDGE_CONTROL

```text
uint32 stamp_ms
uint8 command
```

命令：

| 命令 | 值 | App 发送时机 |
| --- | ---: | --- |
| `START_BRIDGE` | `1` | 连接 supervisor 后点击 `Start Bridge` |
| `STOP_BRIDGE` | `2` | 连接完整 bridge 且未建图时点击 `Stop Bridge` |
| `QUERY_BRIDGE` | `3` | 查询状态 |

建议 K1 行为：

- supervisor 收到 `START_BRIDGE` 后启动完整 bridge，然后释放当前 RFCOMM，使 App 自动重连。
- bridge 收到 `STOP_BRIDGE` 后退出完整 bridge，让 supervisor 恢复等待。

### 7.2 MAP_CONTROL

基础格式：

```text
uint32 stamp_ms
uint8 command
```

启动建图时会追加选项：

```text
uint8 mode
uint8 room_size
```

当 `mode = AUTO` 且 `room_size = CUSTOM` 时继续追加：

```text
float32 custom_size_x
float32 custom_size_y
```

命令：

| 命令 | 值 |
| --- | ---: |
| `START_MAPPING` | `1` |
| `SAVE_MAP_MANUAL` | `2` |
| `STOP_MAPPING` | `3` |
| `QUERY_MAPPING` | `4` |

模式：

| 模式 | 值 |
| --- | ---: |
| `MANUAL` | `0` |
| `AUTO` | `1` |

自动建图尺寸：

| 尺寸 | 值 | 说明 |
| --- | ---: | --- |
| `SMALL` | `0` | 36 m2，约 +/-3 m |
| `MEDIUM` | `1` | 100 m2，约 +/-5 m |
| `LARGE` | `2` | 225 m2，约 +/-7.5 m |
| `CUSTOM` | `3` | 自定义 X/Y，App 限制 1-30 m |

### 7.3 TELEOP_CMD

```text
uint32 stamp_ms
float32 vx
float32 vy
float32 omega
```

App 在手动控制启用且连接完整 bridge 时周期发送。K1 侧应把它转换为底盘控制或 `/cmd_vel`。

App 输出限制：

- `vx` / `vy` 最大幅值：`1.4 m/s * speed_scale`
- `omega` 最大幅值：`3.7 rad/s * omega_scale`
- `speed_scale` 和 `omega_scale` 可选 0.25、0.5、0.75、1.0

### 7.4 STOP

```text
uint32 stamp_ms
```

App 在以下场景发送：

- 用户关闭手动控制。
- 进入 Edit 模式。
- 自动建图开始。
- 蓝牙断开或页面退出前的停止流程。
- 手动控件回中或控制失效。

K1 侧收到后应立即停止手动底盘输出。

### 7.5 MAP_LIBRARY_REQUEST

```text
uint32 stamp_ms
uint8 command
string map_name
string regions_json
```

命令：

| 命令 | 值 | `map_name` | `regions_json` |
| --- | ---: | --- | --- |
| `LIST_MAPS` | `1` | 空 | 空 |
| `LOAD_MAP` | `2` | 目标 yaml 文件名 | 空 |
| `READ_REGIONS` | `3` | 目标 yaml 文件名 | 空 |
| `SAVE_REGIONS` | `4` | 目标 yaml 文件名 | 区域 JSON |

App 端限制：

- `map_name` UTF-8 编码后不超过 65535 字节。
- `regions_json` UTF-8 编码后不超过 65535 字节。

## 8. 区域 JSON

App 保存区域时发送 `SAVE_REGIONS`，其中 `regions_json` 是 UTF-8 JSON 字符串。

顶层结构：

```json
{
  "schema_version": 1,
  "map": {
    "yaml": "manual.yaml",
    "image": "manual.pgm",
    "resolution": 0.05,
    "origin": [0.0, 0.0, 0.0],
    "width": 100,
    "height": 100,
    "image_sha256": ""
  },
  "regions": [
    {
      "id": "uuid",
      "name": "A区",
      "color": "#4F8CFF",
      "vertices": [
        { "x": 0.0, "y": 0.0 },
        { "x": 1.0, "y": 0.0 },
        { "x": 1.0, "y": 1.0 }
      ],
      "center": { "x": 0.5, "y": 0.5 },
      "center_type": "nearest_free_cell"
    }
  ]
}
```

约定：

- `vertices` 使用 map 坐标系。
- App 保存前会为每个区域计算 `center`。
- `center_type` 当前固定为 `nearest_free_cell`。
- K1 端保存时应把 JSON 和对应地图 yaml 关联。

## 9. 推荐交互时序

### 9.1 Supervisor 启动 Bridge

```text
App connects to K1 supervisor
K1 -> App: HELLO("k1-mobile-supervisor")
App -> K1: BRIDGE_CONTROL START_BRIDGE
K1 -> App: BRIDGE_STATUS STARTING
K1 starts full bridge and closes RFCOMM
App reconnects
K1 bridge -> App: HELLO("k1-mobile-bridge")
K1 bridge -> App: BRIDGE_STATUS ONLINE
```

### 9.2 实时地图

```text
K1 -> App: MAP_INFO
K1 -> App: MAP_TILE map_version=当前版本
K1 -> App: MAP_TILE ...
K1 -> App: ROBOT_POSE
K1 -> App: NAV_PATH
```

地图尺寸或原点变化时重新发送 `MAP_INFO`。

### 9.3 建图

```text
App -> K1: MAP_CONTROL START_MAPPING mode=MANUAL/AUTO
K1 -> App: MAP_CONTROL_STATUS STARTING
K1 -> App: MAP_CONTROL_STATUS MAPPING
K1 -> App: MAP_INFO / MAP_TILE / ROBOT_POSE
App -> K1: MAP_CONTROL SAVE_MAP_MANUAL
K1 -> App: MAP_CONTROL_STATUS SAVING
K1 -> App: MAP_CONTROL_STATUS MAPPING
App -> K1: MAP_CONTROL STOP_MAPPING
K1 -> App: MAP_CONTROL_STATUS STOPPING
K1 -> App: MAP_CONTROL_STATUS IDLE
```

### 9.4 地图库和区域编辑

```text
App -> K1: MAP_LIBRARY_REQUEST LIST_MAPS
K1 -> App: MAP_LIBRARY_LIST
App -> K1: MAP_LIBRARY_REQUEST LOAD_MAP map_name=<yaml>
K1 -> App: MAP_LIBRARY_STATUS LOAD_MAP success=1
K1 -> App: MAP_INFO / MAP_TILE
App -> K1: MAP_LIBRARY_REQUEST READ_REGIONS map_name=<yaml>
K1 -> App: MAP_REGIONS_DATA
App -> K1: MAP_LIBRARY_REQUEST SAVE_REGIONS map_name=<yaml> regions_json=<json>
K1 -> App: MAP_LIBRARY_STATUS SAVE_REGIONS success=1
```

## 10. 兼容性注意

- K1 端必须先发有效 `HELLO`，否则 App 会停留在等待识别状态。
- App 会丢弃 CRC32 错误、未知类型、超长 payload、版本不匹配或 magic 不匹配的 K1MB 帧。
- App 不会主动扫描蓝牙设备，外部设备必须先配对。
- Direct SPP 和 K1 Map/Nav 是互斥控制模式，外部软件不应同时向底盘输出两套控制链路。
- K1 侧如果启用自动建图，应停止接受或转发 App 手动控制，App 侧也会锁定手动面板。
