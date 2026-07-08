# 坐标系与升降 Z 约定

## 底盘坐标系

底盘坐标系原点位于底盘中心在地面上的投影点。

```text
x: 向前
y: 向左
z: 从地面向上
origin: 底盘中心地面点
```

因此，底盘坐标系下的目标 z 应理解为目标 TCP 点相对于地面的高度。

## 升降与机械臂高度关系

升降命令 z 不等于物体目标 z。升降命令需要扣除升降零位时机械臂基座的离地高度，以及当前抓取姿态下机械臂 TCP 在机械臂基座坐标系中的 z。

约定公式：

```text
lift_z = target_tcp_z - arm_base_z_at_lift_zero - preferred_arm_tcp_z
```

其中：

- `target_tcp_z`：目标 TCP 在底盘坐标系下的高度，即离地高度。
- `arm_base_z_at_lift_zero`：升降处于零位或 home 位时，机械臂基座坐标原点的离地高度。
- `preferred_arm_tcp_z`：已验证抓取姿态中，TCP 在机械臂基座坐标系下的 z。
- `lift_z`：最终发送给下位机升降模块的位置命令。

反向校验公式：

```text
predicted_tcp_ground_z = arm_base_z_at_lift_zero + lift_z + preferred_arm_tcp_z
```

实际校验方法：给定一个已知 `lift_z`，让机械臂移动到已知抓取姿态，测量 TCP 实际离地高度。如果实测值接近 `predicted_tcp_ground_z`，说明坐标约定正确。

## 升降行程策略

当前建议暂时使用更安全的软件上限：

```text
lift_max_z = 0.45
```

0.50 m 触发冲顶错误的问题应单独作为升降标定和下位机限位问题排查。除非缺少这段行程会直接阻塞当前抓取验证，否则不建议让该问题打断 PC 侧完整流程验证。

后续需要对照 `https://github.com/2026EmbedGameYYC/EmbodiedAI_Robot` 检查：

- 下位机中升降零位的定义。
- 下位机软件最大高度限制。
- 物理上限传感器触发阈值。
- 编码器到米的换算关系。
- home 后是否存在零点偏移。
- 接近顶部时是否故意保留了安全余量。
