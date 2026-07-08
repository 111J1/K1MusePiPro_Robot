# V1 Current System Specs

Last rebuilt: 2026-06-26

This is the source-backed V1 system specification. It describes what exists in the current repository and whether it is part of the V1 runtime chain.

## Current Stage

- Development stage: Windows-first static development and no-hardware validation.
- Primary V1 target: stable mock voice/vision/manager loop plus K1 board-port preparation.
- Primary mock launch: `k1muse_manager_ros/src/k1muse_robot_bringup/launch/robot.mock.launch.py`.
- Primary board preparation launch: `k1muse_manager_ros/src/k1muse_robot_bringup/launch/robot.real_k1.launch.py`.
- Full board deployment: not completed as a whole-system claim. Intent hardening was user-confirmed on board on 2026-06-26; other hardware paths still need their own smoke evidence.
- Current real audio config fact: `plughw:2,0` for both capture and playback in `k1muse_core_ros/src/k1muse_voice_audio/config/audio_io.real_k1.yaml`.
- Excluded from current V1 facts: `k1muse_slam_ros/**`, `k1muse_communicate_ros/src/k1muse_mcu_bridge/**`, OCR/RapidOCR integration, and full K1 deployment.

## V1 Intent Hardening Summary

Current status: completed for the V1 intent-hardening scope, with board/runtime success confirmed by the user on 2026-06-26.

| Item | Current fact | Evidence | Status |
|---|---|---|---|
| Intent node | `intent_node` consumes `/voice/listen/result` and publishes `/voice/intent`, `/voice/tts/text`, `/voice/intent/status`, and `/intent/state`. | `k1muse_voice_intent/src/intent_node.cpp` | Implemented |
| Fast intent layer | Deterministic stop, simple motion, lift, query/chat, find, reminder list/create extraction and empty-reminder protection. | `src/fast_intent.cpp`, `include/k1muse_voice_intent/fast_intent.hpp` | Implemented |
| Route policy | Fast candidates carry category, slot state and reject reason; long/complex relative motion is guarded before task execution. | `src/intent_router.cpp`, `include/k1muse_voice_intent/intent_router.hpp` | Implemented |
| LLM mapping | LLM output is still the flat four-field contract: `kind`, `direction`, `target`, `reply`; `lift` maps to `action=lift,target=lift`. | `src/llm_intent_mapper.cpp`, `src/llm_response_validator.cpp` | Implemented |
| Structured output | Code supports `response_format=none|json_object|json_schema`; current board usage is `json_object` according to user confirmation. | `src/llama_server_client.cpp`, `src/intent_node.cpp`, user-confirmed runtime fact | Implemented with `json_object`; `json_schema` future stabilization |
| Runtime observability | Intent logs include `route_source`, `fast_category`, `fast_slot`, `fast_reject_reason`, `llm_status`, `response_format`, final intent/action/target/value and latency. | `src/intent_node.cpp` | Implemented |

Design boundary: the LLM output intentionally stays simple and fast. Complex behavior should be derived by local post-processing in `IntentRouter`, `LlmIntentMapper`, task/reminder managers, or future V2 policy code rather than by asking the model to emit a nested JSON plan.

## Workspace Overview

| Module | Path | Current role | V1 runtime chain? | Evidence | Status |
|---|---|---|---|---|---|
| Core ROS workspace | `k1muse_core_ros/src` | Audio, voice, AI runtime, supervisor, vision, mock devices, core messages. | Yes | package directories and CMake files under `k1muse_core_ros/src/**` | Active |
| Manager ROS workspace | `k1muse_manager_ros/src` | Task manager, control manager, manager messages, bringup launch/config. | Yes | `k1muse_robot_bringup/launch/*.launch.py`, manager packages | Active |
| Communicate workspace | `k1muse_communicate_ros` | Future MCU bridge area. | No | only referenced by `robot.real_motion.launch.py` comments as external requirement | Future |
| SLAM workspace | `k1muse_slam_ros` | Future SLAM/Nav area. | No | not referenced by V1 main launch | Future |
| External SDK material | `AI_SDK` | Board SDK/model/tooling material. | Dependency only | referenced by CMake and board config paths, not a ROS package boundary | External |
| Python contract tests | `test` | Static/contract checks that do not require ROS2 runtime. | Test support | `test/test_interface_contracts.py`, `test/test_v2_architecture_contracts.py` | Keep |
| Manual board/test scripts | `run_test` | Manual pipeline/audio scripts and sample audio. | No launch reference | `run_test/README.md`, `run_one_round.sh`, `test_pipeline.sh`, audio WAVs | Keep as dev support |
| Scratch output | `tmp` | Duplicate scripts and doc scan facts. | No | `tmp/doc_rebuild_scan/**`, duplicate `run_test` scripts | Scratch |
| Developer tooling | `tools` | Board log helper. | Support only | `tools/k1logs.ps1` | Keep |

## Package Inventory

| Package | Path | Current role | V1 runtime chain? | Evidence | Status |
|---|---|---|---|---|---|
| `k1muse_common` | `k1muse_core_ros/src/k1muse_common` | Shared QoS helpers and `NodeReady.msg`. | Yes | `msg/NodeReady.msg`, `include/k1muse_common/qos_profiles.hpp` | Implemented |
| `k1muse_audio_msgs` | `k1muse_core_ros/src/k1muse_audio_msgs` | Audio frame interface. | Yes | `msg/AudioFrame.msg` | Implemented |
| `k1muse_voice_msgs` | `k1muse_core_ros/src/k1muse_voice_msgs` | Wake/listen/intent/TTS/audio playback messages. | Yes | `msg/WakewordEvent.msg`, `msg/ListenResult.msg`, `msg/IntentLite.msg`, `msg/TtsTextRequest.msg`, `msg/TtsPlayRequest.msg`, `msg/AudioPlaybackState.msg` | Implemented |
| `k1muse_voice_audio` | `k1muse_core_ros/src/k1muse_voice_audio` | Unified audio capture and playback lifecycle node. | Yes | `src/audio_io_node.cpp`, `config/audio.mock.yaml`, `config/audio_io.real_k1.yaml` | Partial: mock path exists; ALSA board validation pending |
| `k1muse_voice_intent` | `k1muse_core_ros/src/k1muse_voice_intent` | ASR text to structured intent plus TTS reply, with hardened FastIntent/LLM routing. | Yes | `src/intent_node.cpp`, `src/intent_router.cpp`, `src/fast_intent.cpp`, `src/llama_server_client.cpp`, `config/intent.real_k1.yaml` | Implemented for V1 intent hardening; current board mode uses `json_object` by user confirmation |
| `k1muse_voice_reminder` | `k1muse_core_ros/src/k1muse_voice_reminder` | Reminder intent handling and TTS request publishing. | Yes | `src/reminder_node.cpp`, `config/reminder.mock.yaml` | Implemented for V1 scope |
| `k1muse_ai_runtime_msgs` | `k1muse_core_ros/src/k1muse_ai_runtime_msgs` | Runtime state/control/mode/resource/alert/pose interfaces. | Interfaces yes; some are V2 reserved | `msg/AiRuntimeControl.msg`, `msg/AiRuntimeState.msg`, `msg/AiruntimeMode.msg`, `msg/Pose2DFrame.msg`, services | Mixed: current and V2-reserved interfaces |
| `k1muse_ai_runtime` | `k1muse_core_ros/src/k1muse_ai_runtime` | Lifecycle inference runtime for wake/VAD/ASR/vision/TTS scheduling. | Yes | `src/ai_runtime_node.cpp`, `src/runtime_scheduler.cpp`, `src/resource_guard.cpp`, `config/ai_runtime.mock.yaml`, `config/ai_runtime.real_k1.yaml` | Partial: V1 mock path and V2 skeleton exist; real K1 validation pending |
| `k1muse_multimodal_supervisor` | `k1muse_core_ros/src/k1muse_multimodal_supervisor` | Interaction state owner and runtime control publisher. | Yes | `src/multimodal_supervisor_node.cpp`, `src/interaction_state_machine.cpp`, `config/supervisor.mock.yaml` | Implemented for V1 control loop |
| `k1muse_vision_msgs` | `k1muse_core_ros/src/k1muse_vision_msgs` | 2D detection, target request/response, target 3D messages. | Yes | `msg/Detection2DFrame.msg`, `msg/TargetRequest.msg`, `msg/TargetResponse.msg`, `msg/Target3D.msg` | Implemented |
| `k1muse_vision_3d` | `k1muse_core_ros/src/k1muse_vision_3d` | Converts selected 2D detection plus registered depth and color camera info into `Target3D`. | Yes, but partial | `src/vision_3d_node.cpp`, `config/vision_3d.mock.yaml` | Partial: code topics and config root aligned; board camera runtime verification pending |
| `k1muse_mock_devices` | `k1muse_core_ros/src/k1muse_mock_devices` | Deterministic mock audio, RGB camera, depth camera publishers. | Yes in mock launch only | `audio_scenario_node`, `camera_scenario_node`, `depth_camera_scenario_node` in CMake and launch | Keep as mock/test |
| `k1muse_manager_msgs` | `k1muse_manager_ros/src/k1muse_manager_msgs` | Manager task/motion/status interfaces. | Yes | `action/ExecuteTask.action`, `action/ExecuteMotion.action`, `srv/StopAndLatch.srv`, `msg/TaskStatus.msg` | Implemented |
| `k1muse_task_manager` | `k1muse_manager_ros/src/k1muse_task_manager` | Routes `IntentLite` to task/motion action goals and stop/reset services. | Yes | `src/task_manager_node.cpp`, `config/task_manager.mock.yaml`, `config/task_manager.real.yaml` | Partial: mock path primary |
| `k1muse_control_manager` | `k1muse_manager_ros/src/k1muse_control_manager` | Hosts task/motion action servers, target request flow, optional motion executor. | Yes | `src/control_manager_node.cpp`, `src/relative_motion_executor.cpp`, `config/control_manager.mock.yaml`, `config/control_manager.real.yaml` | Partial: real motion disabled in V1 main launch |
| `k1muse_robot_bringup` | `k1muse_manager_ros/src/k1muse_robot_bringup` | Launch/config entry package. | Yes | `launch/robot.mock.launch.py`, `launch/robot.real_k1.launch.py` | Implemented, with real-motion launch risk |

## Current Executables And Nodes

| Executable | Runtime node name | Package | Launch evidence | Main source | Status |
|---|---|---|---|---|---|
| `ai_runtime_node` | `ai_runtime` | `k1muse_ai_runtime` | `ai_runtime.mock.launch.py`, `ai_runtime.real_k1.launch.py` | `src/main.cpp`, `src/ai_runtime_node.cpp` | Lifecycle node |
| `audio_io_node` | `audio_io` | `k1muse_voice_audio` | `robot.mock.launch.py`, `robot.real_k1.launch.py` | `src/main.cpp`, `src/audio_io_node.cpp` | Lifecycle node |
| `intent_node` | `intent_node` | `k1muse_voice_intent` | `robot.mock.launch.py`, `robot.real_k1.launch.py` | `src/main.cpp`, `src/intent_node.cpp` | Regular node |
| `supervisor_node` | `multimodal_supervisor` | `k1muse_multimodal_supervisor` | `robot.mock.launch.py`, `robot.real_k1.launch.py` | `src/main.cpp`, `src/multimodal_supervisor_node.cpp` | Regular node |
| `task_manager_node` | `task_manager` | `k1muse_task_manager` | `robot.mock.launch.py`, `robot.real_k1.launch.py` | `src/main.cpp`, `src/task_manager_node.cpp` | Regular node |
| `control_manager_node` | `control_manager` | `k1muse_control_manager` | `robot.mock.launch.py`, `robot.real_k1.launch.py` | `src/main.cpp`, `src/control_manager_node.cpp` | Regular node |
| `reminder_node` | `reminder_node` | `k1muse_voice_reminder` | `robot.mock.launch.py`, `robot.real_k1.launch.py` | `src/main.cpp`, `src/reminder_node.cpp` | Regular node |
| `vision_3d_node` | launch name `vision_3d`, internal constructor `vision_3d_node` | `k1muse_vision_3d` | `robot.mock.launch.py`, `targeting.mock.launch.py` | `src/main.cpp`, `src/vision_3d_node.cpp` | Regular node; YAML root now matches launch name `vision_3d` |
| `audio_scenario_node` | `audio_scenario` | `k1muse_mock_devices` | `robot.mock.launch.py`, `voice_flow.mock.launch.py` | `src/audio_scenario_main.cpp` | Mock only |
| `camera_scenario_node` | `camera_scenario` | `k1muse_mock_devices` | `robot.mock.launch.py`, `targeting.mock.launch.py` | `src/camera_scenario_node.cpp` | Mock only |
| `depth_camera_scenario_node` | `depth_camera_scenario` | `k1muse_mock_devices` | `robot.mock.launch.py`, `targeting.mock.launch.py` | `src/depth_camera_scenario_node.cpp` | Mock only |

## Interface Inventory

| Package | Interfaces | Current role |
|---|---|---|
| `k1muse_common` | `msg/NodeReady.msg` | Readiness from `audio_io` and `intent_node` to supervisor. |
| `k1muse_audio_msgs` | `msg/AudioFrame.msg` | `/audio/raw_pcm` payload. |
| `k1muse_voice_msgs` | `WakewordEvent`, `ListenEvent`, `ListenResult`, `IntentStatus`, `IntentLite`, `TtsTextRequest`, `TtsStatus`, `TtsPlayRequest`, `AudioPlayRequest`, `AudioPlaybackState` | Voice pipeline topics. |
| `k1muse_vision_msgs` | `Detection2D`, `Detection2DFrame`, `TargetRequest`, `TargetResponse`, `Target3D` | Vision detection and find/target flow. |
| `k1muse_ai_runtime_msgs` | `AiRuntimeControl`, `AiRuntimeState`, `AiruntimeMode`, `AiruntimeResource`, `AiruntimeAlert`, `Pose2D`, `Pose2DFrame`, `SetMode`, `GetMode`, `EnableDetector` | Current runtime state/control plus V2 mode/alert/pose reservations. |
| `k1muse_manager_msgs` | `TaskStatus`, `MotionState`, `InteractionState`, `ExecuteTask.action`, `ExecuteMotion.action`, `StopAndLatch.srv`, `ResetMotionLatch.srv` | Manager action/status/control interfaces. |

## Launch Entry Inventory

| Launch file | Purpose | Included nodes/configs | Status |
|---|---|---|---|
| `k1muse_robot_bringup/launch/robot.mock.launch.py` | Primary V1 mock launch. | Includes `ai_runtime.mock.launch.py`; starts audio, intent, supervisor, task/control manager, reminder, vision_3d, and mock devices. | Primary V1 entry |
| `k1muse_robot_bringup/launch/voice_flow.mock.launch.py` | Smaller voice-flow mock. | Includes AI runtime, `audio_io`, `intent_node`, supervisor, `audio_scenario_node`. | Dev/test support |
| `k1muse_robot_bringup/launch/targeting.mock.launch.py` | Smaller vision/targeting mock. | Includes AI runtime, supervisor, `vision_3d`, RGB/depth mock devices. | Dev/test support |
| `k1muse_robot_bringup/launch/robot.real_k1.launch.py` | Board no-motion preparation. | Includes `ai_runtime.real_k1.launch.py`, ALSA `audio_io`, real intent config, real supervisor config, mock/no-motion managers, reminder, vision_3d. | Board preparation, not proof |
| `k1muse_robot_bringup/launch/robot.real_motion.launch.py` | Future real motion experiment. | Includes `robot.mock.launch.py`, then starts another control manager and task manager with real configs. | Not V1 mainline; must be redesigned before use |
| `k1muse_ai_runtime/launch/ai_runtime.mock.launch.py` | Standalone mock AI runtime lifecycle launch. | `ai_runtime_node` with `ai_runtime.mock.yaml`; autostart configure/activate. | Implemented |
| `k1muse_ai_runtime/launch/ai_runtime.real_k1.launch.py` | Standalone real K1 AI runtime lifecycle launch. | `ai_runtime_node` with `ai_runtime.real_k1.yaml`; autostart configure/activate. | Board preparation |

## Config Inventory

| Config | Used by | Important facts | Status |
|---|---|---|---|
| `k1muse_ai_runtime/config/ai_runtime.mock.yaml` | `ai_runtime.mock.launch.py` | `backend_mode: mock`, mock ASR/detection/TTS values, provider parameters. | Current mock |
| `k1muse_ai_runtime/config/ai_runtime.real_k1.yaml` | `ai_runtime.real_k1.launch.py` | `backend_mode: real`, sherpa KWS path, SenseVoice path, YOLO path, Matcha TTS path, 60s job deadline. | Board prep |
| `k1muse_ai_runtime/config/profiles.yaml` | V2 profile candidate | NORMAL/GUARD/PATROL/VOICE_ONLY/STANDBY schedules. | V2 design/config, not fully V1 behavior |
| `k1muse_ai_runtime/config/detectors.yaml` | V2 detector candidate | yolov8n, pose, fire detector records; `backend: mock` currently. | V2 design/config |
| `k1muse_voice_audio/config/audio.mock.yaml` | `robot.mock.launch.py` | wildcard root, `device_type: mock`. | Current mock |
| `k1muse_voice_audio/config/audio_io.real_k1.yaml` | `robot.real_k1.launch.py` | `device_type: alsa`, `capture_device: plughw:2,0`, `playback_device: plughw:2,0`. | Board prep |
| `k1muse_voice_intent/config/intent.mock.yaml` | `robot.mock.launch.py` | fast intent enabled, LLM fallback enabled, route policy parameters and structured-output params defaulted off. | Current mock |
| `k1muse_voice_intent/config/intent.real_k1.yaml` | `robot.real_k1.launch.py` | `llm_backend: real`, API base `http://127.0.0.1:8080/v1`, deterministic sampling, route policy parameters, and structured-output params. Current board runtime uses `json_object` by user confirmation; future target is `json_schema`. | Board intent path implemented; whole-system board deployment still separate |
| `k1muse_voice_intent/config/llama_server.real_k1.yaml` | No current parser in `intent_node`; comment says intent node does not parse it. | llama-server process proposal. | Manual/board support only |
| `k1muse_multimodal_supervisor/config/supervisor.mock.yaml` | mock launch | `no_speech_timeout_sec`, `wake_ack_preset`, `target_cache_ttl_ms`. | Current |
| `k1muse_multimodal_supervisor/config/supervisor.real_k1.yaml` | real K1 launch | same key parameters as mock. | Board prep |
| `k1muse_vision_3d/config/vision_3d.mock.yaml` | mock and real K1 launches | YAML root is `vision_3d`, matching launch name `vision_3d`. | Static alignment done; runtime param dump still useful on ROS2 |
| `k1muse_control_manager/config/control_manager.mock.yaml` | mock and no-motion real launch | `mock_delay_ms`, `find_timeout_ms`; no motion enabled. | Current V1 |
| `k1muse_control_manager/config/control_manager.real.yaml` | real motion experiment | `motion_enabled: true`, speed limits, control rate. | Future/experimental |
| `k1muse_task_manager/config/task_manager.mock.yaml` | mock and real K1 no-motion launch | `default_timeout_ms: 5000`. | Current V1 |
| `k1muse_task_manager/config/task_manager.real.yaml` | real motion experiment | distance/angle bounds. | Future/experimental |
| `k1muse_voice_reminder/config/reminder.mock.yaml` | mock and real K1 launch | timezone, timer, SQLite path. | Current V1 |
| `k1muse_robot_bringup/config/robot.mock.yaml`, `robot.real_k1.yaml` | no inspected launch reads these | mode/no_motion metadata. | Low-value unless wired |

## External Dependencies

| Dependency | Current code evidence | Is it called by V1 runtime? | Status |
|---|---|---|---|
| SpacemiT AI SDK | `k1muse_ai_runtime/CMakeLists.txt` fixed paths `/home/bianbu/AI_SDK/ai-sdk`; real backend sources. | Only when real backends are built and launched on board. | Board dependency |
| sherpa-onnx | `real_sherpa_wakeword_backend.cpp`, `ai_runtime.real_k1.yaml` KWS paths. | Only real wakeword backend. | Board dependency |
| SenseVoice | `real_sensevoice_asr_backend.cpp`, `sensevoice_asr_model_path`. | Only real ASR backend. | Board dependency |
| llama-server | `intent_node.cpp`, `llama_server_client.cpp`, `intent.real_k1.yaml`. | Real intent path when compiled and server is available; supports OpenAI-compatible `response_format=json_object|json_schema`. Current board runtime uses `json_object` by user confirmation. | Board/external process |
| PortAudio | `k1muse_voice_audio/CMakeLists.txt`, `portaudio_backend.cpp`. | Possible audio backend; current real K1 YAML uses ALSA. | Optional |
| ALSA | `alsa_backend.cpp`, `audio_io.real_k1.yaml`. | Yes for K1 real audio config. | Board dependency |
| RapidOCR | current maintained source search found no V1 ROS2 call path. | No. | External/future only |

## Feature Status Matrix

| Area | Current code evidence | Status | Notes |
|---|---|---|---|
| Mock voice loop | `audio_scenario_node -> /audio/raw_pcm -> ai_runtime -> intent_node -> supervisor -> ai_runtime/audio_io` | Implemented/partial | Source and launch exist; runtime execution still requires ROS2 environment. |
| Real audio capture/playback | `audio_io.real_k1.yaml`, `audio_io_node.cpp`, `alsa_backend.cpp` | Partial | Uses `plughw:2,0`; board device and permissions not proven here. |
| Wake/VAD/ASR/TTS runtime | `WakewordRuntime`, `VadAsrRuntime`, `TTSRuntime`, `BackendFactory` | Partial | Mock path present; real backends depend on SDK/model paths. |
| Intent parsing | `fast_intent.cpp`, `intent_router.cpp`, `llm_intent_mapper.cpp`, `llm_response_validator.cpp`, `llama_server_client.cpp`, `intent_node.cpp` | Implemented for V1 intent hardening | Fast/LLM route policy, safe fallback, flat JSON object output and current `json_object` structured output are in place; `json_schema` remains future stabilization. |
| Supervisor state/control | `multimodal_supervisor_node.cpp`, `interaction_state_machine.cpp`, `runtime_control_builder.cpp` | Implemented | Publishes `/ai_runtime/control` and `/robot/interaction_state`. |
| Mock RGB vision input | `camera_scenario_node.cpp` publishes `/camera/main/color/image_raw`; `ai_runtime_node.cpp` subscribes `/camera/main/color/image_raw` | Partial | Static topic wiring is aligned; ROS2 runtime smoke still required. |
| 2D detection output | `ai_runtime_node.cpp` publishes `/vision/detection_2d` | Partial | RGB input topic is aligned; actual detection output still depends on runtime backend/model behavior. |
| 3D target flow | `vision_3d_node.cpp`, `TargetRequest/Response/Target3D` | Partial | Code subscribes `/camera/main/depth_registered/image_raw` and `/camera/main/color/camera_info`; runtime frame/timestamp validation remains. |
| Manager ExecuteTask | `task_manager_node.cpp`, `control_manager_node.cpp`, `ExecuteTask.action` | Implemented for mock | Real semantics need hardening. |
| Real motion | `ExecuteMotion.action`, `relative_motion_executor.cpp`, `control_manager.real.yaml` | Future/partial | Not part of V1 main launch. |
| V2 runtime modes | `profiles.yaml`, `RuntimeProfileManager`, `AiruntimeMode.msg` | Design/skeleton | Not current V1 behavior unless explicitly wired. |
| V2 pose/fire alert | `MockPoseBackend`, `MockFireBackend`, `AiruntimeAlert.msg`, `AlertEventPublisher` | Design/skeleton | No separate `k1muse_vision_alert` package. |
| SLAM/Nav2 | `k1muse_slam_ros/**` | Not V1 | No V1 main launch inclusion. |
| MCU bridge | `k1muse_communicate_ros/src/k1muse_mcu_bridge/**` | Not V1 | Only future real-motion comment reference. |
| OCR/RapidOCR | external material only | Not implemented | Current code found no ROS2 integration. |
| Full board deployment | board configs and launch exist | Not complete | Needs hardware smoke records. |

## Known Corrections

- Old docs that said current audio device is `plughw:1,0` are outdated. Current config says `plughw:2,0`.
- Old docs that include `slam`, `mcubridge`, OCR, or full board deployment as current V1 runtime are outdated unless revalidated.
- `AudioIoNode` currently owns both capture and playback paths, not playback only.
- `AiruntimeMode`, `AiruntimeAlert`, `Pose2D`, `Pose2DFrame`, `SetMode`, `GetMode`, and `EnableDetector` existing as interfaces does not prove V2 runtime behavior is active.
