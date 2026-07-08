# AI 运行时子系统

> [!WARNING]
> ## ⚠️ 重要声明
> 本模块是对真实部署代码的重构版本。**不能直接部署。**
> 缺少运行时依赖：SpacemiT AI SDK (VAD/ASR/TTS/Vision)、sherpa-onnx (唤醒词)、llama-server (LLM)。
> 详见[仓库根目录 README](../../README.md)。

## 包含的 ROS2 包

| 包 | 功能 |
|----|------|
| `k1muse_ai_runtime` | AI 运行时核心（LifecycleNode, 调度器, 5 个模型运行时） |
| `k1muse_ai_runtime_msgs` | 运行时控制/状态/告警消息定义 |
| `k1muse_multimodal_supervisor` | 8 状态多模态交互 FSM |
| `k1muse_voice_audio` | 音频采集与播放（ALSA 后端） |
| `k1muse_voice_intent` | 14 条 FastIntent 规则 + LLM 回退 |
| `k1muse_voice_reminder` | 定时提醒节点 |
| `k1muse_voice_msgs` | 语音流水线消息定义 |
| `k1muse_audio_msgs` | 音频帧消息定义 |
| `k1muse_vision_3d` | 2D 检测 → 3D 坐标转换 |
| `k1muse_vision_msgs` | 视觉检测消息定义 |
| `k1muse_common` | 共享工具（QoS, NodeReady） |
| `k1muse_mock_devices` | 模拟音频/相机/深度发布者 |
| `k1muse_robot_bringup` | 启动文件 + YAML 配置 |

## 架构核心

### 8 状态交互 FSM

```
BOOT → IDLE → WAKE_ACK → LISTENING → INTENT_PROCESSING → TTS_RUNNING → IDLE
                ↑                        ↑
            TARGETING (视觉查找)    EMERGENCY_OR_FAULT
```

### VAD 分段器 (5 状态)

```
Armed → PreSpeech → InSpeech → EndingSilence → SegmentReady → Armed
```

### 意图路由

FastIntent (14 条正则规则) → LLM (llama.cpp HTTP) → 安全回退

### V2 视觉流水线

- 5 种运行时配置：NORMAL / GUARD / PATROL / VOICE_ONLY / STANDBY
- 多检测器轮询调度（yolov8n, yolov8n-pose, yolov8_fire）
- 语音优先权控制（VoiceExclusiveGuard）

## 依赖

- `k1muse-control` (manager_msgs) — 需先构建
- SpacemiT AI SDK — 运行时依赖（未包含，需从官方获取）

## 许可

Apache License 2.0
