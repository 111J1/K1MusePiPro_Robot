# Communication Protocol

本文档定义机器人外部控制通信协议。当前定义 `SYSTEM`、`CHASSIS`、`ARM` 和 `LIFT`，其中 `SYSTEM` 暂不定义具体命令。

## 1. 基本约定

- 协议类型：二进制帧
- 数据字节序：little-endian
- 浮点数：IEEE754 float32，4 bytes
- payload 结构体：1 byte packed，对齐后长度必须与 `LEN` 一致
- payload 最大长度：`64` bytes
- 校验：CRC-8/ATM

CRC-8/ATM 参数：

| 参数 | 值 |
| --- | --- |
| poly | `0x07` |
| init | `0x00` |
| xorout | `0x00` |
| refin | `false` |
| refout | `false` |

## 2. 帧格式

```text
SOF1 SOF2 SRC TARGET CMD SEQ LEN PAYLOAD CRC8
```

| 字段 | 长度 | 值/说明 |
| --- | ---: | --- |
| `SOF1` | 1 | `0xA5` |
| `SOF2` | 1 | `0x5A` |
| `SRC` | 1 | 命令来源 |
| `TARGET` | 1 | 目标模块 |
| `CMD` | 1 | 模块命令 |
| `SEQ` | 1 | 帧序号，发送端递增 |
| `LEN` | 1 | payload 长度，`0..64` |
| `PAYLOAD` | `LEN` | 命令载荷 |
| `CRC8` | 1 | CRC8 校验 |

CRC8 计算范围：

```text
SRC TARGET CMD SEQ LEN PAYLOAD
```

不包含 `SOF1`、`SOF2`、`CRC8`。

## 3. 通用枚举

### 3.1 SRC

```c
typedef enum {
    CTRL_SRC_NONE = 0x00,
    CTRL_SRC_BT = 0x01,
    CTRL_SRC_HOST = 0x02,
    CTRL_SRC_MCU = 0x10,
} control_source_e;
```

### 3.2 TARGET

```c
typedef enum {
    CTRL_TARGET_SYSTEM = 0x00,
    CTRL_TARGET_CHASSIS = 0x01,
    CTRL_TARGET_ARM = 0x02,
    CTRL_TARGET_LIFT = 0x03,
    CTRL_TARGET_PERIPHERAL = 0x04,
} ctrl_target_e;
```

### 3.3 MCU 上报调度

底盘、机械臂和升降任务的执行周期均保持为 `10 ms`，状态采样和控制计算周期不因 telemetry 调度而改变。周期 `STATUS` 的提交周期为 `50 ms`，相位安排如下：

| 模块 | 任务周期 | `STATUS` 周期 | 首次提交相位 |
| --- | ---: | ---: | ---: |
| CHASSIS | `10 ms` | `50 ms` | `0 ms` |
| ARM | `10 ms` | `50 ms` | `20 ms` |
| LIFT | `10 ms` | `50 ms` | `40 ms` |
| PERIPHERAL | `10 ms` | `50 ms` | `10 ms` |

首次提交相位相对各模块任务启动时刻计算，用于避免三个模块在同一任务周期集中提交状态帧。上位机不应依赖严格的帧间隔或固定到达顺序，应使用各 payload 中的 `tick_ms` 判断状态采样时间。

MCU 将上报分为两类进行调度：

| 类型 | 触发方式 | 缓存方式 | 调度规则 |
| --- | --- | --- | --- |
| `RESULT` | 命令处理事件触发 | 独立 FIFO 队列 | 优先于 `STATUS` 发送，保持入队顺序，不参与 `STATUS` 轮询和覆盖 |
| `STATUS` | 周期触发 | 每个模块一个最新值槽位 | 无 `RESULT` 待发送时按模块轮询；同模块尚未发送的旧状态可被新状态覆盖 |

`RESULT` 入队最多等待 `5 ms`；若队列持续满载，提交函数返回失败。`STATUS` 的最新值策略可能跳过中间状态快照，但不会影响模块任务的控制周期。上位机应将 `RESULT` 用作命令结果依据，将 `STATUS` 用作当前状态快照，不应通过统计 `STATUS` 帧数判断命令是否完成。

每个上报帧仍通过同一 telemetry 发送任务广播到蓝牙串口和上位机串口。串口发送忙时，telemetry 任务分别等待各串口可接受当前帧，不因正常的 UART busy 直接丢弃该帧；两路串口共享同一帧内容和 `SEQ`。若某一路串口长期无法恢复可发送状态，后续 telemetry 上报会被阻塞。

## 4. SYSTEM

`SYSTEM` 用于系统级宏命令。当前已定义比赛演示宏命令，正式上位机接口走 `USART2`，帧内 `SRC` 必须为 `CTRL_SRC_HOST = 0x02`。蓝牙调试口走 `USART3`，帧内 `SRC` 必须为 `CTRL_SRC_BT = 0x01`。下位机会校验物理串口和 `SRC` 是否匹配，不匹配的帧会被丢弃。

### 4.1 CMD

```c
typedef enum {
    CTRL_SYS_CMD_RESERVED = 0x00,
    CTRL_SYS_CMD_DEMO_STOP = 0x01,
    CTRL_SYS_CMD_DEMO_RUN = 0x02,
    CTRL_SYS_CMD_DEMO_HOME = 0x03,
    CTRL_SYS_RPT_DEMO_STATUS = 0x80,
    CTRL_SYS_RPT_DEMO_RESULT = 0x81,
} ctrl_system_cmd_e;
```

### 4.2 Demo Variant

```c
typedef enum {
    CTRL_DEMO_VARIANT_AUTO = 0x00,
    CTRL_DEMO_VARIANT_DOWN = 0x01,
    CTRL_DEMO_VARIANT_UP = 0x02,
} ctrl_demo_variant_e;
```

| `variant` | 含义 |
| ---: | --- |
| `0x00` | `AUTO`，根据 `src_layer` / `dst_layer` 自动推断路线 |
| `0x01` | `DOWN`，下行标定，主要用于 `3 -> 2` |
| `0x02` | `UP`，上行标定，主要用于 `2 -> 3`，当前三层目标约上调 `0.02 m` |

`DEMO_ID_LAYER_TRANSFER` 使用 `AUTO` 时，若 `dst_layer > src_layer` 则按 `UP` 处理；若 `dst_layer < src_layer` 则按 `DOWN` 处理。

### 4.3 Demo ID

```c
typedef enum {
    DEMO_ID_NONE = 0x00,
    DEMO_ID_STATIC_PICK_PLACE = 0x01,
    DEMO_ID_LAYER_PICK = 0x02,
    DEMO_ID_LAYER_PLACE = 0x03,
    DEMO_ID_LAYER_TRANSFER = 0x04,
    DEMO_ID_KEY_TURN = 0x05,
    DEMO_ID_CABINET_PULL = 0x06,
} demo_id_e;
```

| `demo_id` | 含义 | `src_layer` | `dst_layer` |
| ---: | --- | --- | --- |
| `0x01` | 平面抓取放置 | `0` | `0` |
| `0x02` | 从指定层抓取 | 抓取层 `1..3` | `0` |
| `0x03` | 放置到指定层 | `0` | 放置层 `1..3` |
| `0x04` | 层间搬运 | 源层 `1..3` | 目标层 `1..3` |
| `0x05` | 钥匙旋转拉出 | `0` | `0` |
| `0x06` | 夹柜门后拉 | `0` | `0` |

### 4.4 DEMO_RUN

`CTRL_SYS_CMD_DEMO_RUN = 0x02`，payload 长度固定为 `4` bytes：

```c
typedef struct {
    uint8_t demo_id;
    uint8_t src_layer;
    uint8_t dst_layer;
    uint8_t variant;
} ctrl_demo_run_payload_t;
```

下位机收到合法 `DEMO_RUN` 后会封装为内部 `demo_cmd_msg_t` 并放入 `DemoCmdQueue`，由 `DemoTask` 选择对应动作步骤表执行。上位机不需要逐条下发 ARM / LIFT / CHASSIS 动作。

正式上位机 `USART2` 示例，假设 `SRC = 0x02`、`SEQ = 0x01`：

```text
STATIC_PICK_PLACE:
A5 5A 02 00 02 01 04 01 00 00 00 FB

LAYER_TRANSFER 3 -> 2, DOWN:
A5 5A 02 00 02 01 04 04 03 02 01 25

LAYER_TRANSFER 2 -> 3, UP:
A5 5A 02 00 02 01 04 04 02 03 02 52

LAYER_TRANSFER 2 -> 3, AUTO:
A5 5A 02 00 02 01 04 04 02 03 00 5C

KEY_TURN:
A5 5A 02 00 02 01 04 05 00 00 00 A3

CABINET_PULL:
A5 5A 02 00 02 01 04 06 00 00 00 94
```

若通过蓝牙调试口 `USART3` 测试，则 `SRC` 改为 `0x01` 并重新计算 CRC。

### 4.5 DEMO_STOP / DEMO_HOME

`CTRL_SYS_CMD_DEMO_STOP = 0x01`，payload 长度为 `0`，用于中止当前 demo。

`CTRL_SYS_CMD_DEMO_HOME = 0x03`，payload 长度为 `0`，用于执行 demo 归位链。

正式上位机 `USART2` 示例，假设 `SRC = 0x02`、`SEQ = 0x01`：

```text
DEMO_STOP:
A5 5A 02 00 01 01 00 BA

DEMO_HOME:
A5 5A 02 00 03 01 00 6C
```

### 4.6 DEMO_STATUS

`CTRL_SYS_RPT_DEMO_STATUS = 0x80`，payload 长度固定为 `16` bytes：

```c
typedef struct {
    uint32_t tick_ms;
    uint8_t state;
    uint8_t fault;
    uint8_t demo_id;
    uint8_t src_layer;
    uint8_t dst_layer;
    uint8_t variant;
    uint8_t step_index;
    uint8_t step_count;
    uint8_t active;
    uint8_t reserved[3];
} ctrl_demo_status_payload_t;
```

`DemoTask` 运行时会通过 telemetry 周期上报该状态。上位机可用 `step_index / step_count` 显示执行进度，用 `active` 判断当前是否正在执行 demo。

### 4.7 DEMO_RESULT

`CTRL_SYS_RPT_DEMO_RESULT = 0x81`，payload 长度固定为 `20` bytes：

```c
typedef struct {
    uint32_t tick_ms;
    uint8_t request_seq;
    uint8_t request_cmd;
    uint8_t request_source;
    uint8_t result;
    uint8_t reject_reason;
    uint8_t fault_reason;
    uint8_t state_after;
    uint8_t demo_id;
    uint8_t src_layer;
    uint8_t dst_layer;
    uint8_t variant;
    uint8_t step_index;
    uint8_t step_count;
    uint8_t reserved[3];
} ctrl_demo_result_payload_t;
```

`result` 复用 ARM / LIFT 的结果语义：

| `result` | 含义 |
| ---: | --- |
| `0x01` | `ACCEPTED`，命令已入队 |
| `0x02` | `REJECTED`，命令被拒绝 |
| `0x03` | `COMPLETED`，demo 完成 |
| `0x04` | `ABORTED`，demo 被中止 |
| `0x05` | `FAILED`，demo 失败 |

`reject_reason`：

| `reject_reason` | 含义 |
| ---: | --- |
| `0x00` | `NONE` |
| `0x01` | `BAD_LENGTH` |
| `0x02` | `QUEUE_FULL` |
| `0x03` | `BUSY` |

`fault_reason`：

| `fault_reason` | 含义 |
| ---: | --- |
| `0x00` | `NONE` |
| `0x01` | `UNKNOWN_DEMO` |
| `0x02` | `BAD_LAYER` |
| `0x03` | `BAD_VARIANT` |
| `0x04` | `QUEUE_FULL` |
| `0x05` | `ARM` |
| `0x06` | `LIFT` |
| `0x07` | `TIMEOUT` |

典型生命周期：

```text
上位机发送 DEMO_RUN
MCU 回 DEMO_RESULT / ACCEPTED
MCU 周期上报 DEMO_STATUS
动作完成后 MCU 回 DEMO_RESULT / COMPLETED、FAILED 或 ABORTED
```

## 5. CHASSIS

### 5.1 CMD

```c
typedef enum {
    CTRL_CHS_CMD_STOP = 0x00,
    CTRL_CHS_CMD_MOV = 0x01,
    CTRL_CHS_CMD_ODOM = 0x02,
    CTRL_CHS_RPT_STATUS = 0x80,
} ctrl_chassis_cmd_e;
```

### 5.2 坐标系

```c
typedef enum {
    CTRL_CHS_MOVE_LCS = 0x00,
    CTRL_CHS_MOVE_WCS = 0x01,
} ctrl_chassis_move_cs_e;
```

### 5.3 STOP

| 字段 | 值 |
| --- | --- |
| `TARGET` | `CTRL_TARGET_CHASSIS` |
| `CMD` | `CTRL_CHS_CMD_STOP` |
| `LEN` | `0` |
| `PAYLOAD` | 无 |

语义：停止底盘。

### 5.4 MOV

| 字段 | 值 |
| --- | --- |
| `TARGET` | `CTRL_TARGET_CHASSIS` |
| `CMD` | `CTRL_CHS_CMD_MOV` |
| `LEN` | `13` |

payload：

```c
#pragma pack(push, 1)
typedef struct {
    uint8_t move_cs;
    float direction;
    float v;
    float omega;
} ctrl_chassis_mov_payload_t;
#pragma pack(pop)
```

| 字段 | 单位 | 说明 |
| --- | --- | --- |
| `move_cs` | - | `CTRL_CHS_MOVE_LCS` 或 `CTRL_CHS_MOVE_WCS` |
| `direction` | rad | 运动方向 |
| `v` | m/s | 线速度绝对值 |
| `omega` | rad/s | 角速度，逆时针为正 |

### 5.5 ODOM

| 字段 | 值 |
| --- | --- |
| `TARGET` | `CTRL_TARGET_CHASSIS` |
| `CMD` | `CTRL_CHS_CMD_ODOM` |
| `LEN` | `12` |

payload：

```c
#pragma pack(push, 1)
typedef struct {
    float direction;
    float x;
    float y;
} ctrl_chassis_odom_payload_t;
#pragma pack(pop)
```

| 字段 | 单位 | 说明 |
| --- | --- | --- |
| `direction` | rad | 世界坐标系方向 |
| `x` | m | 世界坐标系 X |
| `y` | m | 世界坐标系 Y |

### 5.6 STATUS

`STATUS` 为 MCU 主动上报帧。底盘任务保持每 `10 ms` 执行一次，按 `50 ms` 周期、`0 ms` 首次提交相位生成状态上报。该帧只表示底盘当前状态，不作为控制命令。

| 字段 | 值 |
| --- | --- |
| `SRC` | `CTRL_SRC_MCU` |
| `TARGET` | `CTRL_TARGET_CHASSIS` |
| `CMD` | `CTRL_CHS_RPT_STATUS` |
| `LEN` | `32` |

payload：

```c
typedef enum {
    CTRL_CHS_STATE_IDLE = 0x00,
    CTRL_CHS_STATE_MOVING = 0x01,
    CTRL_CHS_STATE_TIMEOUT = 0x02,
    CTRL_CHS_STATE_FAULT = 0x03,
} ctrl_chassis_state_e;

#pragma pack(push, 1)
typedef struct {
    uint32_t tick_ms;

    uint8_t state;
    uint8_t move_cs;
    uint8_t motor_block_flags; /* bit0 LF, bit1 RF, bit2 RB, bit3 LB */
    uint8_t reserved[1];

    float WCS_vx;
    float WCS_vy;
    float omega;

    float WCS_x;
    float WCS_y;
    float WCS_direction;
} ctrl_chassis_status_payload_t;
#pragma pack(pop)
```

| 字段 | 单位 | 说明 |
| --- | --- | --- |
| `tick_ms` | ms | MCU 当前 tick |
| `state` | - | 底盘状态，取 `ctrl_chassis_state_e` |
| `move_cs` | - | 当前运动坐标系，取 `CTRL_CHS_MOVE_LCS` 或 `CTRL_CHS_MOVE_WCS` |
| `motor_block_flags` | - | 电机堵转位图，`bit0=LF`，`bit1=RF`，`bit2=RB`，`bit3=LB` |
| `reserved[1]` | - | 保留，发送端填 `0` |
| `WCS_vx` | m/s | 世界坐标系 X 方向速度 |
| `WCS_vy` | m/s | 世界坐标系 Y 方向速度 |
| `omega` | rad/s | 当前角速度，逆时针为正 |
| `WCS_x` | m | 世界坐标系 X |
| `WCS_y` | m | 世界坐标系 Y |
| `WCS_direction` | rad | 世界坐标系方向 |

`STATUS` 完整帧长度为 `40` bytes：`2` bytes 帧头 + `5` bytes 头字段 + `32` bytes payload + `1` byte CRC8。

## 6. CHASSIS 控制规则

- `STOP`：任何来源允许，立即停止底盘，并释放当前控制来源
- `MOV`：`BT` 和 `HOST` 均允许
- `MOV`：无当前控制来源时，发送方成为当前控制来源
- `MOV`：当前控制来源未超时前，只接受同来源 `MOV`
- `MOV`：每次接受后刷新控制时间
- `ODOM`：`BT` 和 `HOST` 均允许
- `ODOM`：不改变当前控制来源，不刷新运动超时
- 运动超时：300 ms 内未收到当前控制来源的新 `MOV`，底盘自动 `STOP`，并释放当前控制来源
- `STATUS`：MCU 主动上报到所有外部控制来源对应的物理串口；当前实现同时从蓝牙串口和上位机串口回传
- `STATUS`：状态上报不依赖最近一次有效控制来源；控制来源仲裁只影响命令接收，不影响 telemetry 广播

## 7. LIFT

### 7.1 CMD

```c
typedef enum {
    CTRL_LIFT_CMD_STOP = 0x00,
    CTRL_LIFT_CMD_HOME = 0x01,
    CTRL_LIFT_CMD_MOVE_Z = 0x02,
    CTRL_LIFT_CMD_CLEAR_FAULT = 0x03,
    CTRL_LIFT_RPT_STATUS = 0x80,
    CTRL_LIFT_RPT_RESULT = 0x81,
} ctrl_lift_cmd_e;
```

### 7.2 升降状态与结果枚举

```c
typedef enum {
    LIFT_STATE_IDLE = 0,
    LIFT_STATE_HOMING,
    LIFT_STATE_MOVING,
    LIFT_STATE_REACHED,
    LIFT_STATE_FAULT,
} lift_state_e;

typedef enum {
    LIFT_HOME_STATE_IDLE = 0,
    LIFT_HOME_STATE_WAIT_RISING_EDGE,
    LIFT_HOME_STATE_WAIT_FALLING_EDGE,
    LIFT_HOME_STATE_DONE,
    LIFT_HOME_STATE_FAULT,
} lift_home_state_e;

typedef enum {
    LIFT_FAULT_NONE = 0,
    LIFT_FAULT_HOME_SENSOR_NOT_INIT,
    LIFT_FAULT_HOME_TIMEOUT,
    LIFT_FAULT_MOTOR_BLOCKED,
    LIFT_FAULT_CONTROL_TIMEOUT,
    LIFT_FAULT_SENSOR_POSITION_MISMATCH,
} lift_fault_reason_t;

typedef enum {
    LIFT_REJECT_NONE = 0,
    LIFT_REJECT_NOT_HOMED,
    LIFT_REJECT_BUSY,
    LIFT_REJECT_TARGET_OUT_OF_RANGE,
    LIFT_REJECT_IN_FAULT,
    LIFT_REJECT_UNKNOWN_CMD,
} lift_reject_reason_t;

typedef enum {
    LIFT_RESULT_NONE = 0,
    LIFT_RESULT_ACCEPTED,
    LIFT_RESULT_REJECTED,
    LIFT_RESULT_COMPLETED,
    LIFT_RESULT_ABORTED,
    LIFT_RESULT_FAILED,
    LIFT_RESULT_SUPERSEDED,
} lift_result_t;
```

`STATUS` 只描述升降当前状态；`RESULT` 只描述某条命令的处理结果。上位机调度时推荐用 `RESULT.request_seq` 关联命令，用 `STATUS.state/is_busy/has_fault/current_z` 判断当前执行器状态。

### 7.3 STOP

| 字段 | 值 |
| --- | --- |
| `TARGET` | `CTRL_TARGET_LIFT` |
| `CMD` | `CTRL_LIFT_CMD_STOP` |
| `LEN` | `0` |
| `PAYLOAD` | 无 |

语义：停止升降当前运动，并释放当前控制来源。

### 7.4 HOME

| 字段 | 值 |
| --- | --- |
| `TARGET` | `CTRL_TARGET_LIFT` |
| `CMD` | `CTRL_LIFT_CMD_HOME` |
| `LEN` | `0` |
| `PAYLOAD` | 无 |

语义：启动升降回零流程。当前实现中，回零依赖升降原点传感器；回零成功后当前位置和目标位置均置为 `0.0 m`。

### 7.5 MOVE_Z

| 字段 | 值 |
| --- | --- |
| `TARGET` | `CTRL_TARGET_LIFT` |
| `CMD` | `CTRL_LIFT_CMD_MOVE_Z` |
| `LEN` | `4` |

payload：
```c
#pragma pack(push, 1)
typedef struct {
    float z;
} ctrl_lift_z_payload_t;
#pragma pack(pop)
```

| 字段 | 单位 | 说明 |
| --- | --- | --- |
| `z` | m | 升降目标高度 |

语义：控制升降运动到目标高度。当前实现要求升降已经完成回零，且目标值必须位于 `LIFT_Z_MIN_M` 到 `LIFT_Z_MAX_M` 范围内。若目标值超出范围，下位机拒绝该命令并上报 `RESULT_REJECTED`，`reject_reason` 为 `LIFT_REJECT_TARGET_OUT_OF_RANGE`，不会进入故障状态。

### 7.6 CLEAR_FAULT

| 字段 | 值 |
| --- | --- |
| `TARGET` | `CTRL_TARGET_LIFT` |
| `CMD` | `CTRL_LIFT_CMD_CLEAR_FAULT` |
| `LEN` | `0` |
| `PAYLOAD` | 无 |

语义：清除升降故障状态，停止电机，清除堵转计数，将升降任务恢复到空闲状态，并释放当前控制来源。当前实现会清除已回零标志，恢复后需要重新执行 `HOME`。

### 7.7 STATUS

`STATUS` 为 MCU 主动上报帧。升降任务保持每 `10 ms` 执行一次，按 `50 ms` 周期、`40 ms` 首次提交相位生成状态上报。该帧只表示升降当前状态，不作为控制命令。

| 字段 | 值 |
| --- | --- |
| `SRC` | `CTRL_SRC_MCU` |
| `TARGET` | `CTRL_TARGET_LIFT` |
| `CMD` | `CTRL_LIFT_RPT_STATUS` |
| `LEN` | `36` |

payload：

```c
#pragma pack(push, 1)
typedef struct {
    uint32_t tick_ms;

    uint8_t state;
    uint8_t home_state;
    uint8_t is_homed;
    uint8_t is_busy;

    uint8_t has_fault;
    uint8_t fault_reason;
    uint8_t motor_blocked;
    uint8_t home_sensor_level;

    float current_z;
    float target_z;
    float current_v;
    float target_v;
    float position_error;

    int32_t motor_total_encoder_count;
} ctrl_lift_status_payload_t;
#pragma pack(pop)
```

| 字段 | 单位 | 说明 |
| --- | --- | --- |
| `tick_ms` | ms | MCU 当前 tick |
| `state` | - | 升降状态，取 `lift_state_e` |
| `home_state` | - | 升降回零状态，取 `lift_home_state_e` |
| `is_homed` | - | 是否已经完成回零，`0` 为否，非 `0` 为是 |
| `is_busy` | - | 是否正在执行回零或移动，`0` 为否，非 `0` 为是 |
| `has_fault` | - | 是否处于故障状态，`0` 为否，非 `0` 为是 |
| `fault_reason` | - | 升降故障原因，取 `lift_fault_reason_t` |
| `motor_blocked` | - | 升降电机是否堵转，`0` 为否，非 `0` 为是 |
| `home_sensor_level` | - | 升降原点传感器当前电平 |
| `current_z` | m | 当前升降高度 |
| `target_z` | m | 目标升降高度 |
| `current_v` | m/s | 当前升降速度 |
| `target_v` | m/s | 目标升降速度 |
| `position_error` | m | 目标高度与当前高度的差值 |
| `motor_total_encoder_count` | count | 升降电机累计编码器计数 |

`STATUS` 完整帧长度为 `44` bytes：`2` bytes 帧头 + `5` bytes 头字段 + `36` bytes payload + `1` byte CRC8。

### 7.8 RESULT

`RESULT` 为 MCU 对升降命令的结果上报帧。该帧用于回答某条命令是否被接受、拒绝、完成、中止或失败。`RESULT` 中的 `request_seq` 等于对应命令帧的 `SEQ`。

若升降正在移动，且同一控制来源下发新的合法 `MOVE_Z`，旧的 `MOVE_Z` 命令会先上报 `LIFT_RESULT_SUPERSEDED`，新的 `MOVE_Z` 命令随后上报 `LIFT_RESULT_ACCEPTED`。

| 字段 | 值 |
| --- | --- |
| `SRC` | `CTRL_SRC_MCU` |
| `TARGET` | `CTRL_TARGET_LIFT` |
| `CMD` | `CTRL_LIFT_RPT_RESULT` |
| `LEN` | `32` |

payload：

```c
#pragma pack(push, 1)
typedef struct {
    uint32_t tick_ms;

    uint8_t request_seq;
    uint8_t request_cmd;
    uint8_t request_source;
    uint8_t result;

    uint8_t reject_reason;
    uint8_t fault_reason;
    uint8_t state_after;
    uint8_t reserved0;

    float requested_z;
    float accepted_z;
    float current_z;
    float z_min;
    float z_max;
} ctrl_lift_result_payload_t;
#pragma pack(pop)
```

| 字段 | 单位 | 说明 |
| --- | --- | --- |
| `tick_ms` | ms | MCU 当前 tick |
| `request_seq` | - | 被响应命令帧的 `SEQ` |
| `request_cmd` | - | 被响应命令帧的 `CMD` |
| `request_source` | - | 被响应命令帧的 `SRC` |
| `result` | - | 命令结果，取 `lift_result_t` |
| `reject_reason` | - | 拒绝原因，取 `lift_reject_reason_t`；非拒绝结果填 `LIFT_REJECT_NONE` |
| `fault_reason` | - | 故障原因，取 `lift_fault_reason_t`；无故障填 `LIFT_FAULT_NONE` |
| `state_after` | - | 发送该结果时的升降状态，取 `lift_state_e` |
| `reserved0` | - | 保留，发送端填 `0` |
| `requested_z` | m | 命令请求的目标高度；非高度命令可填 `0` |
| `accepted_z` | m | 实际接受的目标高度；拒绝时为当前目标高度 |
| `current_z` | m | 发送结果时的当前升降高度 |
| `z_min` | m | 当前允许的最小目标高度 |
| `z_max` | m | 当前允许的最大目标高度 |

`RESULT` 完整帧长度为 `40` bytes：`2` bytes 帧头 + `5` bytes 头字段 + `32` bytes payload + `1` byte CRC8。

## 8. LIFT 控制规则

- `STOP`：任何来源允许，立即停止升降，并释放当前控制来源
- `HOME`：`BT` 和 `HOST` 均允许，但需要通过控制来源仲裁
- `MOVE_Z`：`BT` 和 `HOST` 均允许，但需要通过控制来源仲裁；只有升降已回零且不处于故障状态时才会执行
- `CLEAR_FAULT`：`BT` 和 `HOST` 均允许，不经过控制来源仲裁；执行后释放当前控制来源
- `MOVE_Z` 目标超出 `LIFT_Z_MIN_M` 到 `LIFT_Z_MAX_M` 范围时，下位机拒绝该命令并上报 `RESULT_REJECTED`，不进入故障状态
- 升降正在回零时，新的 `HOME` 或 `MOVE_Z` 会被拒绝并上报 `LIFT_REJECT_BUSY`
- 升降正在移动时，同一控制来源的新 `MOVE_Z` 会被接受并切换到新目标；旧 `MOVE_Z` 命令上报 `LIFT_RESULT_SUPERSEDED`，新 `MOVE_Z` 命令上报 `LIFT_RESULT_ACCEPTED`
- 升降正在移动时，其他控制来源的新 `MOVE_Z` 会被拒绝并上报 `LIFT_REJECT_BUSY`
- 升降正在移动时，新的 `HOME` 会被拒绝并上报 `LIFT_REJECT_BUSY`
- 升降未回零时，`MOVE_Z` 会被拒绝并上报 `LIFT_REJECT_NOT_HOMED`
- 升降处于故障状态时，`MOVE_Z` 会被拒绝并上报 `LIFT_REJECT_IN_FAULT`
- 无当前控制来源时，发送方成为当前控制来源
- 当前控制来源未释放或未超时前，只接受同来源的 `HOME` 或 `MOVE_Z`
- 每次接受 `HOME` 或 `MOVE_Z` 后刷新控制时间
- 目标到达后，释放当前控制来源
- 控制超时：`10000 ms` 内当前控制来源未刷新控制命令，升降自动 `STOP`，并释放当前控制来源
- 回零超时：超过 `LIFT_HOME_TIMEOUT_MS` 后仍未完成回零，升降自动 `STOP`，进入故障状态，并释放当前控制来源
- 检测到升降电机堵转时，升降自动 `STOP`，进入故障状态
- middle/top 参考传感器仅用于位置和编码器读数校准；若参考点位置误差超过允许范围，升降进入 `LIFT_FAULT_SENSOR_POSITION_MISMATCH`
- top 参考传感器不作为硬件冲顶保护；升降超限保护由软件目标范围 `LIFT_Z_MIN_M` / `LIFT_Z_MAX_M` 约束
- 帧头错误、CRC 错误、payload 长度超过协议最大值或 `SRC` 与物理串口来源不匹配时，当前实现直接丢弃，不上报 `RESULT`
- `target = LIFT` 且 CRC 正确但 `CMD` 未知或 payload 长度错误时，当前实现直接忽略；后续可扩展为 `RESULT_REJECTED`

## 9. ARM

### 9.1 CMD

```c
typedef enum {
    CTRL_ARM_CMD_STOP = 0x00,
    CTRL_ARM_CMD_HOME = 0x01,
    CTRL_ARM_CMD_MOVE_XYZ = 0x02,
    CTRL_ARM_CMD_MOVE_POSE = 0x03,
    CTRL_ARM_CMD_GRIPPER = 0x04,
    CTRL_ARM_CMD_CLEAR_FAULT = 0x05,
    CTRL_ARM_CMD_DISABLE_TORQUE = 0x06,
    CTRL_ARM_RPT_STATUS = 0x80,
    CTRL_ARM_RPT_RESULT = 0x81,
} ctrl_arm_cmd_e;
```

### 9.2 STOP

| 字段 | 值 |
| --- | --- |
| `TARGET` | `CTRL_TARGET_ARM` |
| `CMD` | `CTRL_ARM_CMD_STOP` |
| `LEN` | `0` |
| `PAYLOAD` | 无 |

语义：停止机械臂当前运动目标，保持当前姿态，并释放当前控制来源。

### 9.3 HOME

| 字段 | 值 |
| --- | --- |
| `TARGET` | `CTRL_TARGET_ARM` |
| `CMD` | `CTRL_ARM_CMD_HOME` |
| `LEN` | `0` |
| `PAYLOAD` | 无 |

语义：机械臂回零。当前实现中，5 个主动关节目标角为 `0 rad`，夹爪目标角为 `0 rad`。

### 9.4 MOVE_XYZ

| 字段 | 值 |
| --- | --- |
| `TARGET` | `CTRL_TARGET_ARM` |
| `CMD` | `CTRL_ARM_CMD_MOVE_XYZ` |
| `LEN` | `12` |

payload：

```c
#pragma pack(push, 1)
typedef struct {
    float x;
    float y;
    float z;
} ctrl_arm_move_xyz_payload_t;
#pragma pack(pop)
```

| 字段 | 单位 | 说明 |
| --- | --- | --- |
| `x` | m | 机械臂末端目标 X |
| `y` | m | 机械臂末端目标 Y |
| `z` | m | 机械臂末端目标 Z |

语义：只控制机械臂末端移动到目标位置，不改变夹爪目标角。夹爪必须通过独立的 `GRIPPER` 命令控制。

### 9.5 MOVE_POSE

| 字段 | 值 |
| --- | --- |
| `TARGET` | `CTRL_TARGET_ARM` |
| `CMD` | `CTRL_ARM_CMD_MOVE_POSE` |
| `LEN` | `20` |

payload：

```c
#pragma pack(push, 1)
typedef struct {
    float x;
    float y;
    float z;
    float roll;
    float pitch;
} ctrl_arm_move_pose_payload_t;
#pragma pack(pop)
```

| 字段 | 单位 | 说明 |
| --- | --- | --- |
| `x` | m | 机械臂末端目标 X |
| `y` | m | 机械臂末端目标 Y |
| `z` | m | 机械臂末端目标 Z |
| `roll` | rad | 机械臂末端目标 roll |
| `pitch` | rad | 机械臂末端目标 pitch |

语义：控制机械臂末端移动到目标位置，并约束末端 `roll` / `pitch`。该命令用于正式位姿控制；目标可达性、IK 状态和到达状态通过机械臂状态上报诊断。

### 9.6 GRIPPER

| 字段 | 值 |
| --- | --- |
| `TARGET` | `CTRL_TARGET_ARM` |
| `CMD` | `CTRL_ARM_CMD_GRIPPER` |
| `LEN` | `4` |

payload：

```c
#pragma pack(push, 1)
typedef struct {
    float gripper_rad;
} ctrl_arm_gripper_payload_t;
#pragma pack(pop)
```

| 字段 | 单位 | 说明 |
| --- | --- | --- |
| `gripper_rad` | rad | 夹爪目标角度 |

语义：单独控制夹爪目标角，不改变机械臂末端目标位置。

### 9.7 CLEAR_FAULT

| 字段 | 值 |
| --- | --- |
| `TARGET` | `CTRL_TARGET_ARM` |
| `CMD` | `CTRL_ARM_CMD_CLEAR_FAULT` |
| `LEN` | `0` |
| `PAYLOAD` | 无 |

语义：清除机械臂故障状态或扭矩关闭状态，并重新进入使能扭矩与回零流程。

### 9.8 DISABLE_TORQUE

| 字段 | 值 |
| --- | --- |
| `TARGET` | `CTRL_TARGET_ARM` |
| `CMD` | `CTRL_ARM_CMD_DISABLE_TORQUE` |
| `LEN` | `0` |
| `PAYLOAD` | 无 |

语义：关闭机械臂舵机扭矩，释放当前控制来源。关闭扭矩后机械臂进入扭矩关闭状态，不再继续执行位置目标更新；需要通过 `CLEAR_FAULT` 恢复到使能扭矩与回零流程。

### 9.9 STATUS

`STATUS` 为 MCU 主动上报帧。机械臂任务保持每 `10 ms` 执行一次，按 `50 ms` 周期、`20 ms` 首次提交相位生成状态上报。该帧只表示机械臂当前状态，不作为控制命令。

| 字段 | 值 |
| --- | --- |
| `SRC` | `CTRL_SRC_MCU` |
| `TARGET` | `CTRL_TARGET_ARM` |
| `CMD` | `CTRL_ARM_RPT_STATUS` |
| `LEN` | `48` |

payload：

```c
#pragma pack(push, 1)
typedef struct {
    uint32_t tick_ms;

    uint8_t state;
    uint8_t status;
    uint8_t is_busy;
    uint8_t has_fault;

    uint8_t active_cmd;
    uint8_t active_source;
    uint8_t active_seq;
    uint8_t diag_code;

    float current_x;
    float current_y;
    float current_z;

    float target_x;
    float target_y;
    float target_z;

    float current_gripper_rad;
    float target_gripper_rad;

    uint32_t fault_flags;
} ctrl_arm_status_payload_t;
#pragma pack(pop)
```

| 字段 | 单位 | 说明 |
| --- | --- | --- |
| `tick_ms` | ms | MCU 当前 tick |
| `state` | - | 机械臂任务状态，取 `arm_task_state_t` |
| `status` | - | 机械臂任务执行状态，取 `arm_task_status_t` |
| `is_busy` | - | 是否正在执行已接受命令，`0` 为否，非 `0` 为是 |
| `has_fault` | - | 是否处于故障状态，`0` 为否，非 `0` 为是 |
| `active_cmd` | - | 当前 active 命令，取 `arm_cmd_type_e`；无 active 命令时为 `0` |
| `active_source` | - | 当前 active 命令来源；无 active 命令时为 `CTRL_SRC_NONE` |
| `active_seq` | - | 当前 active 命令的 `SEQ`；无 active 命令时为 `0` |
| `diag_code` | - | 机械臂诊断码，取 `arm_diag_code_t` |
| `current_x` | m | 当前末端 X 坐标 |
| `current_y` | m | 当前末端 Y 坐标 |
| `current_z` | m | 当前末端 Z 坐标 |
| `target_x` | m | 目标末端 X 坐标 |
| `target_y` | m | 目标末端 Y 坐标 |
| `target_z` | m | 目标末端 Z 坐标 |
| `current_gripper_rad` | rad | 当前夹爪角度 |
| `target_gripper_rad` | rad | 目标夹爪角度 |
| `fault_flags` | - | 当前机械臂故障位图 |

`STATUS` 完整帧长度为 `56` bytes：`2` bytes 帧头 + `5` bytes 头字段 + `48` bytes payload + `1` byte CRC8。

### 9.10 RESULT

`RESULT` 为 MCU 对机械臂命令的结果上报帧。该帧只描述某条命令的处理结果，`request_seq` 等于对应命令帧的 `SEQ`；周期状态仍由 `STATUS` 描述。

若机械臂正在执行移动目标，且同一控制来源下发新的合法 `HOME` / `MOVE_XYZ` / `MOVE_POSE` / `GRIPPER`，旧命令先上报 `ARM_RESULT_SUPERSEDED`，新命令随后上报 `ARM_RESULT_ACCEPTED`。若新目标 IK 不可达，旧命令仍视为已被取代，机械臂保持当前位姿等待下一个目标，新命令上报 `ARM_RESULT_REJECTED`，`reject_reason` 为 `ARM_REJECT_IK_UNREACHABLE`。

| 字段 | 值 |
| --- | --- |
| `SRC` | `CTRL_SRC_MCU` |
| `TARGET` | `CTRL_TARGET_ARM` |
| `CMD` | `CTRL_ARM_RPT_RESULT` |
| `LEN` | `60` |

payload：

```c
#pragma pack(push, 1)
typedef struct {
    uint32_t tick_ms;

    uint8_t request_seq;
    uint8_t request_cmd;
    uint8_t request_source;
    uint8_t result;

    uint8_t reject_reason;
    uint8_t fault_source;
    uint8_t state_after;
    uint8_t diag_code;

    float requested_x;
    float requested_y;
    float requested_z;
    float requested_gripper_rad;

    float accepted_x;
    float accepted_y;
    float accepted_z;
    float accepted_gripper_rad;

    float current_x;
    float current_y;
    float current_z;
    float current_gripper_rad;
} ctrl_arm_result_payload_t;
#pragma pack(pop)
```

`requested_x/y/z` 仅对位置或位姿类命令有效，`requested_gripper_rad` 仅对 `GRIPPER` 命令有效。`accepted_*` 和 `current_*` 字段提供结果产生时的执行器目标与当前状态快照；未被该命令控制的执行轴不会因此改变目标。

```c
typedef enum {
    ARM_RESULT_NONE = 0,
    ARM_RESULT_ACCEPTED,
    ARM_RESULT_REJECTED,
    ARM_RESULT_COMPLETED,
    ARM_RESULT_ABORTED,
    ARM_RESULT_FAILED,
    ARM_RESULT_SUPERSEDED,
} arm_result_t;

typedef enum {
    ARM_REJECT_NONE = 0,
    ARM_REJECT_BUSY,
    ARM_REJECT_OWNER,
    ARM_REJECT_IN_FAULT,
    ARM_REJECT_TORQUE_DISABLED,
    ARM_REJECT_PARAM,
    ARM_REJECT_TARGET_OUT_OF_RANGE,
    ARM_REJECT_IK_UNREACHABLE,
    ARM_REJECT_UNKNOWN_CMD,
} arm_reject_reason_t;

typedef enum {
    ARM_FAULT_SRC_NONE = 0,
    ARM_FAULT_SRC_TASK = 1,
    ARM_FAULT_SRC_MDL = 2,
    ARM_FAULT_SRC_SERVO_BUS = 3,
    ARM_FAULT_SRC_SERVO = 4,
    ARM_FAULT_SRC_IK = 5,
    ARM_FAULT_SRC_TIMEOUT = 6,
} arm_fault_source_t;

typedef enum {
    ARM_DIAG_OK = 0,
    ARM_DIAG_BUSY = 1,
    ARM_DIAG_PARAM_ERROR = 2,
    ARM_DIAG_IK_UNREACHABLE = 3,
    ARM_DIAG_SERVO_FAULT = 4,
    ARM_DIAG_SERVO_BUS_TIMEOUT = 5,
    ARM_DIAG_MDL_FAULT = 6,
    ARM_DIAG_TASK_TIMEOUT = 7,
} arm_diag_code_t;
```

`RESULT` 完整帧长度为 `68` bytes：`2` bytes 帧头 + `5` bytes 头字段 + `60` bytes payload + `1` byte CRC8。

## 10. ARM 控制规则

- `STOP`：任何来源允许，立即停止机械臂当前运动目标，保持当前姿态，并释放当前控制来源
- `DISABLE_TORQUE`：任何来源允许，关闭机械臂舵机扭矩，并释放当前控制来源
- `CLEAR_FAULT`：`BT` 和 `HOST` 均允许，不改变当前控制来源；在故障状态或扭矩关闭状态下触发恢复流程
- `HOME`、`MOVE_XYZ`、`MOVE_POSE`、`GRIPPER`：`BT` 和 `HOST` 均允许，但需要通过控制来源仲裁
- `MOVE_XYZ` 和 `MOVE_POSE` 不改变夹爪目标角；`GRIPPER` 不改变机械臂末端目标位置
- 无当前控制来源时，发送方成为当前控制来源
- 当前控制来源未释放或未超时前，只接受同来源的运动类命令；同来源新运动命令会覆盖当前执行命令
- 机械臂执行运动目标时，同一控制来源的新合法目标必须立即覆盖旧目标；旧 active 命令上报 `ARM_RESULT_SUPERSEDED`，新命令上报 `ARM_RESULT_ACCEPTED`
- 新目标不可达或 IK 失败时，旧目标仍被废弃；机械臂保持当前位姿，进入 `ARM_TASK_WAIT_TARGET`，新命令上报 `ARM_RESULT_REJECTED`
- `STATUS` 只描述机械臂当前状态；`RESULT` 只描述某条命令的处理结果，上位机调度应使用 `RESULT.request_seq` 关联命令
- 每次接受运动类命令后刷新控制时间
- 目标到达且任务状态为 OK 后，释放当前控制来源
- 控制超时：10000 ms 内当前控制来源未刷新控制命令，机械臂自动 `STOP`，并释放当前控制来源

## 11. PERIPHERAL

`PERIPHERAL` 为 MCU 外围功能状态上报模块，当前包含气体传感器状态和 3S 电池电源电压状态。该模块当前不接收控制命令，只周期上报 `STATUS`。

### 11.1 CMD

```c
typedef enum {
    CTRL_PERIPH_RPT_STATUS = 0x80,
} ctrl_peripheral_cmd_e;
```

### 11.2 电源状态

```c
typedef enum {
    CTRL_PERIPH_POWER_NORMAL = 0x00,
    CTRL_PERIPH_POWER_LOW = 0x01,
    CTRL_PERIPH_POWER_CRITICAL = 0x02,
    CTRL_PERIPH_POWER_FAULT = 0x03,
} ctrl_peripheral_power_state_e;
```

当前默认阈值按 3S 锂电设置：

| 状态 | 条件 |
| --- | --- |
| `CTRL_PERIPH_POWER_NORMAL` | `power_mv >= 11100` |
| `CTRL_PERIPH_POWER_LOW` | `10500 <= power_mv < 11100` |
| `CTRL_PERIPH_POWER_CRITICAL` | `9000 <= power_mv < 10500` |
| `CTRL_PERIPH_POWER_FAULT` | ADC 读取失败，或 `power_mv < 9000` |

`LOW`、`CRITICAL`、`FAULT` 均需要对应条件连续持续 `5000 ms` 后才会上报为当前 `power_state`；若电压恢复到 `NORMAL` 条件，则立即恢复上报 `CTRL_PERIPH_POWER_NORMAL`。

PB1 接入的是真实电源电压缩小 4 倍后的 ADC 电压，因此 MCU 端按 `power_mv = adc_pin_mv * 4` 还原真实电源电压。协议上报的 `power_mv` 为 MCU 端滤波后的电压值。

### 11.3 STATUS

`STATUS` 为 MCU 主动上报帧。外围功能任务保持每 `10 ms` 执行一次，按 `50 ms` 周期、`10 ms` 首次提交相位生成状态上报。

| 字段 | 值 |
| --- | --- |
| `SRC` | `CTRL_SRC_MCU` |
| `TARGET` | `CTRL_TARGET_PERIPHERAL` |
| `CMD` | `CTRL_PERIPH_RPT_STATUS` |
| `LEN` | `12` |

payload：

```c
#pragma pack(push, 1)
typedef struct {
    uint32_t tick_ms;
    uint8_t gas_detected;
    uint8_t power_state;
    uint16_t power_mv;
    int16_t temperature_centi_c;
    uint16_t humidity_centi_pct;
} ctrl_peripheral_status_payload_t;
#pragma pack(pop)
```

| 字段 | 单位 | 说明 |
| --- | --- | --- |
| `tick_ms` | ms | MCU 当前 tick |
| `gas_detected` | - | 气体检测状态，`1` 为检测到气体，`0` 为未检测到 |
| `power_state` | - | 电源状态，取 `ctrl_peripheral_power_state_e` |
| `power_mv` | mV | 还原并滤波后的真实电源电压 |
| `temperature_centi_c` | 0.01 deg C | SHT31 温度，摄氏度乘以 100 |
| `humidity_centi_pct` | 0.01 %RH | SHT31 相对湿度百分比乘以 100 |

气体传感器有效电平由 MCU 端配置宏决定，当前默认为低电平有效。

`STATUS` 完整帧长度为 `20` bytes：`2` bytes 帧头 + `5` bytes 头字段 + `12` bytes payload + `1` byte CRC8。
