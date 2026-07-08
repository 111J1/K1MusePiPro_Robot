# STM32G474 固件 — 实时控制层

> [!WARNING]
> ## ⚠️ 重要声明
> 本模块是对真实部署代码的重构版本，用于展示核心算法和系统架构。**不能直接部署运行。**
> 详见[仓库根目录 README](../README.md)。

## 概述

基于 STM32G474VCT6 (Cortex-M4F, 144MHz) 的实时控制固件，运行 FreeRTOS V10.3.1。

## 架构

```
Application/
├── Algorithm/      CRC8, PID 控制器
├── Device/         硬件抽象层（电机、舵机、传感器、OLED）
├── Driver/         寄存器级驱动（HRTIM, UART DMA, ADC, I2C, DWT）
├── Module/         业务逻辑层
│   ├── SO101ARM/   SO101 5-DOF 机械臂 + 逆运动学
│   ├── mdl_chassis   麦克纳姆底盘运动学
│   ├── mdl_lift      升降机构 PID 控制
│   ├── mdl_control_protocol   二进制帧协议解析
│   └── mdl_control_arbitration  控制权仲裁
└── Task/           FreeRTOS 8 个任务入口
```

## FreeRTOS 任务拓扑

| 任务 | 优先级 | 周期 | 功能 |
|------|--------|------|------|
| CommandTask | High (24) | 10ms | UART 帧解析与分发 |
| ChassisTask | AboveNormal (22) | 10ms | 底盘运动控制 |
| ArmTask | AboveNormal (22) | 10ms | 机械臂状态机 |
| LiftTask | AboveNormal (22) | 10ms | 升降台 PID + 归零 |
| TelemetryTask | Normal (20) | 事件驱动 | 遥测编码与发送 |
| PeripheralTask | Normal (20) | 周期 | 传感器轮询 |
| DemoTask | Normal (20) | 10ms | 演示序列编排 |
| OledTask | Low (18) | 100ms | OLED 显示 |

## 通信协议

与上位机 (K1) 通过 UART 通信，二进制帧格式：
`0xA5 0x5A SRC TARGET CMD SEQ LEN PAYLOAD(0-64) CRC8`

详见 [`docs/protocol.md`](../docs/protocol.md)。

## 关键算法

- **机械臂 IK**: 螺旋转轴 (PoE) 表示，位置用阻尼最小二乘法 (LM)，姿态用闭式解析解
- **底盘**: 麦克纳姆轮逆运动学 (4 轮)，1/√2 缩放，330 RPM 饱和保护
- **升降台**: P-only PID (Kp=7.0)，霍尔传感器归零 + 运行时参考修正

## 构建

- IDE: Keil MDK-ARM (Arm Compiler 6)
- STM32CubeMX 配置文件: `EmbodiedAI_Robot.ioc`

## 许可

Apache License 2.0
