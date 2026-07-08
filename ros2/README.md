# ROS2 包 — 上层计算层

> [!WARNING]
> ## ⚠️ 重要声明
> 本模块是对真实部署代码的重构版本，用于展示核心算法和系统架构。**不能直接部署运行。**
> 详见[仓库根目录 README](../README.md)。

## 概述

基于 ROS2 Humble 的上层计算软件栈，运行在 SpacemiT K1 MusePi Pro 上。分为三个子系统：

| 子系统 | 目录 | 功能 |
|--------|------|------|
| 导航与通信 | [`navigation/`](navigation/) | SLAM 建图、路径规划、传感器驱动、通信桥 |
| AI 运行时 | [`ai/`](ai/) | 语音交互、视觉检测、LLM、多模态监督器 |
| 上层控制 | [`control/`](control/) | 取放任务规划、抓取配置、串口客户端 |

## 依赖关系

```
navigation/  ← 独立构建（LiDAR/IMU 驱动 + Cartographer + Nav2）
ai/          ← 依赖 control/（需先 source control 的 install）
control/     ← 独立构建（纯算法，无硬件依赖）
```

构建顺序：
```bash
# 1. 先构建 control
cd control/
colcon build

# 2. 再构建 ai（依赖 manager_msgs）
cd ai/
source ../control/install/setup.bash
colcon build

# 3. navigation 可以独立构建（但需要 LiDAR/IMU 硬件或 mock）
cd navigation/
colcon build
```

## 许可

Apache License 2.0
