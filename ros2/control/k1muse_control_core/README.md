# k1muse_control_core 纯C控制核心

`k1muse_control_core` 是一套与操作系统和ROS2解耦的纯C控制核心，同时服务于：

- PC阶段：由Python脚本暂时替代K1主控板，通过USB-UART控制真实STM32。
- K1阶段：由ROS2 C++节点直接链接本C库，复用协议、规划和任务状态机。

本库不包含Python、ROS2、串口设备名、线程或操作系统相关代码。因此从PC迁移到K1时，只需替换外层通信和ROS2接口，不需要重写核心控制流程。

## 分层结构

本库参考STM32下位机的分层方式组织：

| 层级 | 目录 | 职责 |
|---|---|---|
| Algorithm | `src/algorithm` | CRC8及通用数学算法 |
| Protocol | `src/protocol` | 字节流帧编码、解析、粘包和拆包处理 |
| Device | `src/device` | STM32命令构造，以及STATUS/RESULT解析 |
| Module | `src/module` | 抓取配置校验、底盘和升降布置计算 |
| Task | `src/task` | 抓取、携带、运输和放置状态机 |

平台外壳只需提供：

- 当前时间。
- 收到的串口字节。
- 配置数据。
- 命令完成、失败、超时或取消事件。
- 将C核心输出的动作发送给STM32。

PC平台外壳位于 `tools/pc_controller`；最终K1平台外壳将由ROS2 C++节点实现。

## 协议层

协议层实现STM32当前使用的帧格式：

```text
SOF1 SOF2 SRC TARGET CMD SEQ LEN PAYLOAD CRC8
```

当前目标编号以STM32源码为准：

```text
SYSTEM  = 0x00
CHASSIS = 0x01
ARM     = 0x02
LIFT    = 0x03
```

协议层支持：

- CRC-8/ATM。
- 最大64字节载荷。
- 小端整数和IEEE754浮点数。
- 半帧、连续多帧和错误CRC处理。
- ARM和LIFT命令RESULT解析。
- CHASSIS、ARM和LIFT周期STATUS解析。

协议常量应始终以STM32工程中的 `mdl_control_protocol.h` 为准，不应复制旧版上位机说明中的编号。

## 布置规划

规划器输入：

- 目标TCP在底盘坐标系中的位置。
- 机械臂相对底盘的安装参数。
- 升降范围。
- 物品抓取配置。

规划器输出：

- 底盘平移方向和距离。
- 升降目标高度。
- 最终抓取位姿。
- 预抓取位姿。
- 试提位姿。
- 抓取后的退出位姿。

当前第一版要求输入坐标已经位于底盘坐标系：X向前、Y向左、Z向上。

## 抓取和放置状态机

状态机采用事件驱动方式，不在C核心内部使用线程或阻塞等待：

```text
安全停止
→ 机械臂回安全位
→ 升降回零
→ 底盘移动到抓取位置
→ 升降移动到抓取高度
→ 打开夹爪
→ 移动到预抓取点
→ 移动到最终TCP
→ 固定角度闭合夹爪
→ 等待夹爪稳定
→ 小幅试提
→ 移动到携带姿态
→ 底盘运输
→ 升降移动到放置高度
→ 移动到预放置点
→ 移动到最终放置点
→ 打开夹爪
→ 等待释放
→ 机械臂退出
→ 完成
```

外部收到命令完成、失败、超时或取消事件后，再调用状态机推进到下一阶段。PC阶段由Python循环调用；K1阶段由ROS2回调或定时器调用。

## 安全设计

- 未标定的物品配置禁止真实执行。
- 任务失败、超时或取消时，状态机输出 `STOP_ALL`。
- `STOP_ALL`要求分别向底盘、升降和机械臂发送STOP。
- 底盘运动必须持续刷新MOV，避免超过STM32的300 ms控制超时。
- ARM和LIFT命令必须等待匹配SEQ的最终RESULT。
- 后一个机械臂命令必须等待前一个命令完成，避免触发 `SUPERSEDED`。

## 本地测试

纯C测试覆盖：

- CRC标准校验值。
- 协议帧编码和解析。
- 错误CRC拒绝。
- 抓取配置校验。
- 底盘与升降布置计算。
- 顶部抓取预抓取点计算。
- 完整抓取—运输—放置状态机。
- 超时后的安全停止。

## 在K1上构建

```bash
cd <workspace_root>
colcon build --packages-select k1muse_control_core
colcon test --packages-select k1muse_control_core
```

最终ROS2 C++节点可通过 `extern "C"` 直接包含本库头文件并链接该库。
