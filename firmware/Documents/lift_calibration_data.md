# 升降机构标定数据

本文档记录升降机构的坐标定义、传感器边沿含义、已测量标定值，以及建议写入
`Application/Module/mdl_lift_config.h` 的配置值。

## 坐标定义

- 本文所有 `z` 均表示：从底部参考平面到升降装置最下方的距离。
- `z = 0 m` 表示升降装置最下方与底部参考平面重合，或软件定义的等效零点。
- `z` 正方向为升降向上。
- 传感器电平定义：
  - 未遮挡：`1`
  - 遮挡：`0`
- 边沿定义：
  - `falling_edge`：`1 -> 0`，进入遮挡区。
  - `rising_edge`：`0 -> 1`，离开遮挡区。

## 软件限位与物理限位

`LIFT_Z_MAX_M` 是正常运动命令的软件上限，不是机械硬限位，也不是 top 传感器触发点。

当前实测：

| 项目 | z / m | 说明 |
|---|---:|---|
| 正常软件上限 | `0.500` | 当前 `LIFT_Z_MAX_M`，用于限制普通 `MOVE_Z` 目标 |
| top 上行触发点 | `0.493` | 向上运动进入 top 传感器遮挡区的位置 |
| 机械顶部余量 | 约 `0.010` | top 触发后到机械顶端仍有约 1 cm 余量 |

工程建议：

- `LIFT_Z_MAX_M` 作为正常工作区软件上限，由软件目标范围约束。
- top 传感器只作为位置参考点，用于修正编码器累计误差，不承担限位或停车职责。
- 如果未来需要扩大正常工作区，应先确认机械余量、制动距离和负载条件，再调整 `LIFT_Z_MAX_M`。

## 基础机械参数

| 数据项 | 当前配置 | 当前值 | 说明 |
|---|---|---:|---|
| 未归零默认值 | `LIFT_Z_UNHOMED_DEFAULT_M` | `0.000f` | home 完成前使用的默认 z，不代表 HOME 完成位置 |
| 最小允许 z | `LIFT_Z_MIN_M` | `0.000f` | 正常运动下限 |
| 最大允许 z | `LIFT_Z_MAX_M` | `0.500f` | 正常运动软件上限 |
| 每圈理论行程 | `LIFT_TRAVEL_PER_ROUND_M` | `0.040f` | 当前按 40 mm/round 计算 |
| 行程修正系数 | `LIFT_TRAVEL_CALIBRATION_SCALE` | `1.000f` | 后续可用于修正累计比例误差 |
| 编码器每圈计数 | `LIFT_ENCODER_COUNT_PER_ROUND` | `1584U` | 当前编码器换算基准 |

## Bottom Home 传感器

bottom 传感器用于上电 home。它位于软件零点上方，因此 home 完成后不是把 z 置为 0，而是按实际触发边沿写入对应 z 值。

| 测量项 | 运动方向 | 电平边沿 | 配置项 | 实测值 / m | 建议配置 |
|---|---|---|---|---:|---|
| bottom 离开遮挡区 | 向上 | rising | `LIFT_HOME_RISING_Z_M` | `0.080` | `0.080f` |
| bottom 进入遮挡区 | 向下 | falling | `LIFT_HOME_FALLING_Z_M` | `0.077` | `0.077f` |

## Middle 参考传感器

middle 传感器有一段遮挡区，上下两个方向都会经过进入/离开遮挡区的边沿。该传感器适合作为中段位置参考，用于修正编码器累计误差。

| 测量项 | 运动方向 | 物理触发说明 | 电平边沿 | 配置项 | 最终值 / m | 建议配置 |
|---|---|---|---|---|---:|---|
| middle 进入遮挡区 | 向上 | 升降挡片上边沿触发 | falling | `LIFT_MIDDLE_UP_FALLING_Z_M` | `0.246` | `0.246f` |
| middle 离开遮挡区 | 向上 | 升降挡片下边沿释放 | rising | `LIFT_MIDDLE_UP_RISING_Z_M` | `0.341` | `0.341f` |
| middle 进入遮挡区 | 向下 | 升降挡片下边沿触发 | falling | `LIFT_MIDDLE_DOWN_FALLING_Z_M` | `0.337` | `0.337f` |
| middle 离开遮挡区 | 向下 | 升降挡片上边沿释放 | rising | `LIFT_MIDDLE_DOWN_RISING_Z_M` | `0.246` | `0.246f` |

说明：

- 向上进入遮挡区和向下离开遮挡区都在约 `0.246 m`，对应遮挡区下边界附近。
- 向上离开遮挡区和向下进入遮挡区分别为 `0.341 m` 与 `0.337 m`，两者相差约 `4 mm`，可视为传感器迟滞、机械间隙、制动和人工读数误差的综合结果。
- 若后续用于在线校准，建议先保持 `LIFT_SENSOR_REF_MAX_ERROR_M = 0.010f`，确认稳定后再收紧到 `0.003f ~ 0.005f`。

## Top 参考传感器

top 传感器只用于位置校准。当前实际结构中，可靠使用的是上行进入遮挡区的触发点和下行离开遮挡区的释放点；真正的限位由 `LIFT_Z_MIN_M` / `LIFT_Z_MAX_M` 软件目标范围负责。

| 测量项 | 运动方向 | 物理触发说明 | 电平边沿 | 配置项 | 最终值 / m | 建议配置 |
|---|---|---|---|---|---:|---|
| top 进入遮挡区 | 向上 | 升降挡片上边沿触发 | falling | `LIFT_TOP_UP_FALLING_Z_M` | `0.493` | `0.493f` |
| top 离开遮挡区 | 向下 | 升降挡片上边沿释放 | rising | `LIFT_TOP_DOWN_RISING_Z_M` | `0.490` | `0.490f` |

## 建议回填配置

```c
#define LIFT_Z_MAX_M (0.50f)

#define LIFT_HOME_RISING_Z_M (0.080f)
#define LIFT_HOME_FALLING_Z_M (0.077f)

#define LIFT_MIDDLE_UP_RISING_Z_M (0.341f)
#define LIFT_MIDDLE_UP_FALLING_Z_M (0.246f)
#define LIFT_MIDDLE_DOWN_RISING_Z_M (0.246f)
#define LIFT_MIDDLE_DOWN_FALLING_Z_M (0.337f)

#define LIFT_TOP_UP_FALLING_Z_M (0.493f)
#define LIFT_TOP_DOWN_RISING_Z_M (0.490f)
```

说明：

- `LIFT_Z_MAX_M = 0.50f` 保留为正常软件上限。
- `LIFT_TOP_UP_FALLING_Z_M = 0.493f` 记录 top 上行位置校准点。
- top 参考点和软件上限相互独立，调整 `LIFT_Z_MAX_M` 不应改变 top 传感器标定值。

## 测试检查项

回填配置并烧录后，建议按以下顺序检查：

1. 上电 home，确认最终停在 bottom home 对应位置，`is_homed = 1`。
2. 小范围 `MOVE_Z`，确认 `current_z` 与尺子读数趋势一致。
3. 低速穿过 middle，确认 `last_ref_sensor = 1`，`last_ref_error` 小于 `LIFT_SENSOR_REF_MAX_ERROR_M`。
4. 慢速靠近 top，确认 top 触发后仅更新参考校准状态，不触发保护或停车。
5. 多次往返后观察 `current_z` 是否有明显累计漂移，必要时再调整 `LIFT_TRAVEL_CALIBRATION_SCALE`。

