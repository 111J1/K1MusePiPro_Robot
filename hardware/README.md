# 硬件设计文件

> [!WARNING]
> ## ⚠️ 重要声明
> 本目录包含的是设计参考文件，不构成可直接生产的完整制造包。
> 详见[仓库根目录 README](../README.md)。

## 许可

**CC-BY-SA 4.0** — 署名-相同方式共享 4.0 国际

您可以自由地：
- 分享 — 在任何媒介以任何形式复制、发行本作品
- 改编 — 修改、转换或以本作品为基础进行创作

惟须遵守下列条件：
- 署名 — 必须给出适当的署名
- 相同方式共享 — 如果您再混合、转换或者基于本作品进行创作，您必须基于与原先许可协议相同的许可协议分发您贡献的作品

完整许可文本: https://creativecommons.org/licenses/by-sa/4.0/legalcode.zh-hans

## 目录结构

```
hardware/
├── cad/         3D CAD 文件 (.step, .stl, .3mf)
│   ├── chassis/    底盘部件（麦克纳姆轮安装、电池支架等）
│   ├── arm/        SO101 5-DOF 机械臂零件
│   ├── lift/       升降机构（多层平台）
│   └── mounts/     传感器安装座（LiDAR、相机、红外等）
├── pcb/         PCB 设计文件（立创 EDA 格式）
│   ├── motor-driver/   电机驱动板
│   ├── stm32-core/     STM32G474 核心板
│   └── baseboard/      泰山派扩展底板
├── matlab/      MATLAB 运动学分析脚本
│   └── assets/         可视化用 STL 模型
└── urdf/        URDF 机器人描述文件
```

## 软件

- 3D CAD: Fusion 360 / SolidWorks (推荐)
- PCB: 立创 EDA
- MATLAB: R2023a+
