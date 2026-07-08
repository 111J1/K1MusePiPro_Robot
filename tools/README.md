# 开发工具

> [!WARNING]
> ## ⚠️ 重要声明
> 本目录中的工具脚本是对真实部署版本的抽象。**可能需要根据实际环境调整路径和端口配置。**

## 工具列表

| 工具 | 功能 |
|------|------|
| `mcu_sim.py` | STM32 MCU 模拟器（通过 CH340 USB-UART），发送 ChassisStatus 帧并打印收到的命令 |
| `test_comms.py` | 通信测试脚本 |
| `full_chain_test.sh` | 全链路集成测试 |
| `demo_console.py` | 演示控制台（通过蓝牙发送演示命令） |
| `demo_tuning_profile.json` | 演示调试参数 |
| `bluetooth_pairing_agent.py` | BlueZ 自动配对代理 |

## 许可

Apache License 2.0
