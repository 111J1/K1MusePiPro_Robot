# Android 蓝牙控制器 App

> [!WARNING]
> ## ⚠️ 重要声明
> 本模块是对真实部署代码的重构版本，用于展示核心算法和系统架构。**不能直接部署运行。**
> 详见[仓库根目录 README](../README.md)。

## 概述

基于 Kotlin + Jetpack Compose 的 Android 机器人遥控器，通过经典蓝牙 (RFCOMM/SPP) 与机器人通信。

## 双模式

- **Direct SPP 模式**: 通过 HC-05 蓝牙模块直接控制 STM32 底盘 (只写)
- **K1 Map/Nav 模式**: 连接 K1 上位机，实时查看 SLAM 地图、机器人位姿、导航路径 (双向 K1MB 协议)

## 技术栈

| 项目 | 版本/值 |
|------|---------|
| Kotlin | 2.2.10 |
| Compose BOM | 2026.02.01 |
| AGP | 9.2.1 |
| Min SDK | 28 |
| Target SDK | 36 |
| 包名 | com.embodiedai.robotcontroller |

## 关键特性

- 自定义 K1MB 二进制协议（17 种消息类型，最大 4 MiB 载荷）
- zlib 压缩地图分块渲染（ByteArray 环形缓冲区，避免 GC 卡顿）
- 4 种蓝牙连接 fallback 策略
- 监督器自动探测 → 启动 bridge → 5 次自动重连
- 地图多边形区域标注（最近自由栅格中心计算）
- 拖拽平移、双指缩放 (0.25x-16x)、0/90/180/270 度旋转

## 核心文件

| 文件 | 功能 |
|------|------|
| `K1MapModeScreen.kt` | 地图/Nav 主界面 (~3900 行) |
| `K1MobileProtocol.kt` | K1MB 协议编解码 + 帧解析器 |
| `ControlProtocol.kt` | STM32 Direct SPP 帧编码 |
| `K1MapBluetoothClient.kt` | 蓝牙连接 + 遥操线程 |
| `RegionModels.kt` | 地图区域数据模型 |

## 许可

Apache License 2.0
