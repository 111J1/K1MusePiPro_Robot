# 上层控制子系统

> [!WARNING]
> ## ⚠️ 重要声明
> 本模块是对真实部署代码的重构版本。**不能直接部署。** 详见[仓库根目录 README](../../README.md)。

## 包含的 ROS2 包

| 包 | 语言 | 功能 |
|----|------|------|
| `k1muse_control_core` | C11 | 纯 C 库：串行帧协议、取放状态机 (22 状态)、放置规划器、抓取配置验证 |
| `k1muse_control_manager` | C++17 | ROS2 动作服务器：任务执行、抓取执行器、设备就绪管理、STM32 串口客户端 |
| `k1muse_task_manager` | C++17 | ROS2 节点：语音意图 → ExecuteTask 动作路由 |
| `k1muse_manager_msgs` | ROS2 IDL | 任务/动作消息定义 (ExecuteTask.action, TaskStatus.msg) |

## 取放任务流程

```
语音意图 → TaskManager → ExecuteTask Action → ControlManager
    → GraspTaskExecutor
        → stop_all()
        → ensure_arm_ready() / ensure_lift_ready()  [回零优化]
        → lift_move_z() → arm_gripper(open)
        → move_pose(pregrasp) → move_pose(grasp)
        → arm_gripper(close) → settle → carry
```

## 放置规划器

输入：目标在底盘坐标系中的 3D 位置 + 抓取配置
输出：底盘移动距离/方向、升降台 Z 目标、4 个手臂姿态 (grasp/pregrasp/lifted/retreat)

## 抓取配置

预标定 3 个物体 4 种抓取方式：
- bottle.side, box.side, box.top, umbrella.top

## Demo-Tolerant 模式

- 手臂姿态容差：±1.5cm
- 夹持器：无故障即假定成功
- 升降台：严格检查 ±2cm

## 许可

Apache License 2.0
