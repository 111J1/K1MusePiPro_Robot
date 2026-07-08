# 升降三传感器接线

## 用途

升降机构目前使用三个位置传感器：

- `bottom`：底部传感器，用于原点/回零
- `middle`：中部传感器，用于后续中间位置参考
- `top`：顶部传感器，用于后续顶部位置参考或限位保护

三个传感器的 `VCC` 和 `GND` 已经共接，三个传感器各自单独引出 `D0` 信号线。

## STM32 引脚对应

| 传感器 | CubeMX Label | STM32 引脚 | GPIO 模式 | 当前软件用途 |
| --- | --- | --- | --- | --- |
| 底部 bottom | `LIFT_BOTTOM_SENSOR` | `PD14` | Input + Pull-up | 回零/home |
| 中部 middle | `LIFT_MIDDLE_SENSOR` | `PD8` | Input + Pull-up | 状态采样/诊断 |
| 顶部 top | `LIFT_TOP_SENSOR` | `PB13` | Input + Pull-up | 状态采样/诊断 |

## 杜邦线颜色记录

| 信号 | 杜邦线颜色 | 备注 |
| --- | --- | --- |
| 公共 VCC | 红色 | 三个传感器共用 |
| 公共 GND | 棕色 | 三个传感器共用 |
| bottom D | 白色 | 接 `PD14` |
| middle D | 灰色 | 接 `PD8` |
| top D | 紫色 | 接 `PB13` |

## 电平约定

当前 GPIO 使用 `GPIO_PULLUP`。按现有软件约定：

| 状态 | 电平 |
| --- | --- |
| 未触发/未遮挡 | `1` |
| 触发/遮挡 | `0` |

对应配置位于：

```c
#define LIFT_HOME_SENSOR_BLOCKED_LEVEL (0U)
#define LIFT_HOME_SENSOR_CLEAR_LEVEL   (1U)
```

## 注意事项

