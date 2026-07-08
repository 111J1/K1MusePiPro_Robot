# MUSE Pi Pro 完整部署手册

目标：从刷机后的空白板卡开始，按本文逐步操作，恢复到可 SSH、可构建、可运行双机分布式 ROS2 SLAM/Nav/探索的完整状态。本文也适用于更换树莓派或重新部署计算端。

## 约定

| 项目 | 值 |
|---|---|
| 本机仓库 | `E:\Embed_Game\MusePiPro_prj` |
| MUSE 板端用户 | `bianbu` |
| MUSE 板端 IP（Wi-Fi 管理） | `192.168.137.194` |
| MUSE 板端 IP（有线 ROS2） | `10.10.10.2/24` |
| MUSE SSH 别名 | `musepi` |
| MUSE 通信工作区 | `/home/bianbu/k1muse_communicate_ros` |
| MUSE SLAM 工作区 | `/home/bianbu/k1muse_slam_ros` |
| 树莓派计算端用户 | `user` |
| 树莓派有线 IP（ROS2） | `10.10.10.1/24` |
| 树莓派通信工作区 | `/home/user/k1muse_communicate_ros` |
| 树莓派 SLAM 工作区 | `/home/user/k1muse_slam_ros` |
| 树莓派 MPPI overlay | `/home/user/mppi_overlay` |
| ROS_DOMAIN_ID | `42` |
| RMW | `rmw_cyclonedds_cpp` |
| MUSE 内核 | `6.6.63` |
| SSH 密钥 | `C:\Users\YJY\.ssh\codex_musepi_ed25519` |

---

## 第一部分：单板 MUSE Pi Pro 部署

### 0. 本机准备

确认 SSH key 和别名：

```powershell
Test-Path C:\Users\YJY\.ssh\codex_musepi_ed25519
Get-Content C:\Users\YJY\.ssh\config
ssh musepi "hostname && uname -a"
```

`C:\Users\YJY\.ssh\config` 需要包含：

```sshconfig
Host musepi
    HostName 192.168.137.194
    User bianbu
    IdentityFile C:\Users\YJY\.ssh\codex_musepi_ed25519
    IdentitiesOnly yes
```

首次刷机后如果还不能免密，先用板端密码登录一次，把公钥写入板端：

```powershell
$pub = Get-Content C:\Users\YJY\.ssh\codex_musepi_ed25519.pub
ssh bianbu@192.168.137.194 "mkdir -p ~/.ssh && chmod 700 ~/.ssh && grep -qxF '$pub' ~/.ssh/authorized_keys 2>/dev/null || echo '$pub' >> ~/.ssh/authorized_keys; chmod 600 ~/.ssh/authorized_keys"
```

当前公钥：

```text
ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIGwzSsLZgUuQaBi/0wqCRBoPCPdN1axxRAnCjR6djP9f codex-musepi
```

本机 Python 工具依赖：

```powershell
python -m pip install pyserial
```

### 1. 板端基础包

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git pkg-config \
  python3-pip python3-colcon-common-extensions \
  python3-serial python3-smbus2 \
  device-tree-compiler \
  openssh-server \
  wayvnc
```

ROS2 Humble 依赖：

```bash
sudo apt-get install -y \
  ros-humble-ament-cmake \
  ros-humble-rclcpp ros-humble-rclpy \
  ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime \
  ros-humble-sensor-msgs ros-humble-std-msgs \
  ros-humble-nav-msgs ros-humble-geometry-msgs \
  ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-tools \
  ros-humble-launch ros-humble-launch-ros ros-humble-ros2launch \
  ros-humble-xacro ros-humble-urdf ros-humble-robot-state-publisher \
  ros-humble-cartographer ros-humble-cartographer-ros ros-humble-cartographer-rviz \
  ros-humble-slam-toolbox \
  ros-humble-rviz2 ros-humble-rviz-imu-plugin \
  ros-humble-rmw-cyclonedds-cpp
```

> MUSE 上不需要 `ros-humble-navigation2` 和 `ros-humble-nav2-*`，Nav2 只在树莓派上运行。

Python 包兜底：

```bash
python3 -m pip install --user pyserial smbus2
```

确认 ROS setup 路径：

```bash
ls -l /opt/ros/humble/setup.bash /opt/bros/humble/setup.bash 2>/dev/null || true
```

本文默认使用 `/opt/ros/humble/setup.bash`。如果当前镜像只提供 `/opt/bros/humble/setup.bash`，后续命令中的 `/opt/ros/humble/setup.bash` 改为 `/opt/bros/humble/setup.bash`。

### 2. 用户环境

`.bashrc` 至少需要 ROS2 环境：

```bash
grep -nE 'ROS|RMW|QT_QPA_PLATFORM|k1muse' ~/.bashrc || true
cat >> ~/.bashrc <<'EOF'

# K1 MUSE Pi Pro ROS2 environment
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export QT_QPA_PLATFORM=xcb
EOF
```

有本机镜像时，优先用镜像恢复，但先确认文件存在并备份板端当前文件：

```powershell
cd E:\Embed_Game\MusePiPro_prj
Test-Path environment/.bashrc
ssh musepi "cp /home/bianbu/.bashrc /home/bianbu/.bashrc.pre-sync"
scp environment/.bashrc musepi:/home/bianbu/.bashrc
ssh musepi "bash -n /home/bianbu/.bashrc"
```

### 3. WayVNC 桌面

配置 LXQt Wayland 自动登录：

```bash
sudo mkdir -p /etc/sddm.conf.d
sudo tee /etc/sddm.conf.d/autologin.conf >/dev/null <<'EOF'
[Autologin]
User=bianbu
Session=lxqt-wayland.desktop
EOF
```

有本机镜像时恢复 `/home/bianbu/start_wayvnc.sh`：

```powershell
cd E:\Embed_Game\MusePiPro_prj
Test-Path environment/start_wayvnc.sh
scp environment/start_wayvnc.sh musepi:/home/bianbu/start_wayvnc.sh
ssh musepi "bash -n /home/bianbu/start_wayvnc.sh && chmod +x /home/bianbu/start_wayvnc.sh"
```

如果本机还没有镜像，先在板端创建最小脚本，之后再同步回本机镜像：

```bash
cat > /home/bianbu/start_wayvnc.sh <<'EOF'
#!/usr/bin/env bash
set -e
export XDG_RUNTIME_DIR=/run/user/$(id -u)
export WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-wayland-0}
exec wayvnc 0.0.0.0 5900
EOF
chmod +x /home/bianbu/start_wayvnc.sh
```

重启后检查：

```bash
sudo reboot
```

```bash
loginctl
ls -l /run/user/1000/wayland-*
/home/bianbu/start_wayvnc.sh
```

VNC Viewer 连接：`192.168.137.194:5900`。无显示器时需要 HDMI dummy plug。

### 4. 源码同步

```powershell
cd E:\Embed_Game\MusePiPro_prj
ssh musepi "rm -rf /home/bianbu/k1muse_communicate_ros/src /home/bianbu/k1muse_slam_ros/src; mkdir -p /home/bianbu/k1muse_communicate_ros /home/bianbu/k1muse_slam_ros"
scp -r k1muse_communicate_ros/src musepi:/home/bianbu/k1muse_communicate_ros/
scp -r k1muse_slam_ros/src musepi:/home/bianbu/k1muse_slam_ros/
```

不要同步 `build/`、`install/`、`log/`、`.vscode/`、缓存和 `*.pyc`。

如果使用 `scp -r` 后发现缓存被带到板端，可只在板端源码目录内清理：

```bash
find /home/bianbu/k1muse_communicate_ros/src /home/bianbu/k1muse_slam_ros/src \( -type d -name __pycache__ -o -type d -name .pytest_cache \) -prune -exec rm -rf {} +
find /home/bianbu/k1muse_communicate_ros/src /home/bianbu/k1muse_slam_ros/src -type f -name '*.pyc' -delete
```

### 5. udev 串口别名

```bash
sudo cp /home/bianbu/k1muse_communicate_ros/src/udev/99-ros2-communicate.rules /etc/udev/rules.d/
sudo cp /home/bianbu/k1muse_slam_ros/src/udev/99-ros2-slam.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/mymcu /dev/mylidar /dev/myimu
```

期望：

- `/dev/mymcu` -> UART4 `/dev/ttyS4`
- `/dev/mylidar` -> UART5 `/dev/ttyS5`
- `/dev/myimu` -> CH340 USB IMU

每条 udev 规则必须单行，不能在逗号后换行。

### 6. 设备树

目标：

- UART4：`/soc/uart@d4017300`，`status = okay`，`pinctrl-0 = <0x92>`，运行设备 `/dev/ttyS4`
- UART5：`/soc/uart@d4017400`，`status = okay`，`pinctrl-0 = <0x91>`，运行设备 `/dev/ttyS5`
- `uart4_3_grp`：`phandle = <0x92>`
- `uart5_3_grp`：`phandle = <0x91>`
- `/soc/pcie@ca800000`：UART4 复用引脚时设为 `disabled`

先备份板端当前文件：

```bash
sudo cp /boot/env_k1-x.txt /boot/env_k1-x.txt.pre-k1muse
sudo cp /boot/spacemit/6.6.63/k1-x_MUSE-Pi-Pro.dtb /boot/spacemit/6.6.63/k1-x_MUSE-Pi-Pro.dtb.pre-k1muse
```

如果是同一内核和同一镜像基线，可恢复本仓库镜像：

```powershell
cd E:\Embed_Game\MusePiPro_prj
scp device_tree/boot/env_k1-x.txt musepi:/tmp/env_k1-x.txt
scp device_tree/boot/spacemit/6.6.63/k1-x_MUSE-Pi-Pro.dtb musepi:/tmp/k1-x_MUSE-Pi-Pro.dtb
ssh musepi "sudo cp /tmp/env_k1-x.txt /boot/env_k1-x.txt && sudo cp /tmp/k1-x_MUSE-Pi-Pro.dtb /boot/spacemit/6.6.63/k1-x_MUSE-Pi-Pro.dtb && sync"
```

如果系统版本不确定，先基于板端当前 DTB 做最小修改，不直接覆盖旧 DTB：

```bash
dtc -I dtb -O dts -o /tmp/k1-x-current.dts /boot/spacemit/6.6.63/k1-x_MUSE-Pi-Pro.dtb
grep -nE 'uart@d4017300|uart@d4017400|uart4_3_grp|uart5_3_grp|pcie@ca800000' /tmp/k1-x-current.dts
```

修改 DTS 后编译并替换：

```bash
dtc -I dts -O dtb -o /tmp/k1-x_MUSE-Pi-Pro.dtb /tmp/k1-x-current.dts
sudo cp /tmp/k1-x_MUSE-Pi-Pro.dtb /boot/spacemit/6.6.63/k1-x_MUSE-Pi-Pro.dtb
sync
sudo reboot
```

重启后检查：

```bash
tr -d '\0' </proc/device-tree/soc/uart@d4017300/status
od -An -tx4 /proc/device-tree/soc/uart@d4017300/pinctrl-0
tr -d '\0' </proc/device-tree/soc/uart@d4017400/status
od -An -tx4 /proc/device-tree/soc/uart@d4017400/pinctrl-0
ls -l /dev/ttyS4 /dev/ttyS5 /dev/mymcu /dev/mylidar
```

期望：UART4 为 `okay` / `92000000`，UART5 为 `okay` / `91000000`。

### 7. 构建 MUSE

通信工作区先构建：

```bash
cd /home/bianbu/k1muse_communicate_ros
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

再构建 SLAM 工作区：

```bash
cd /home/bianbu/k1muse_slam_ros
source /opt/ros/humble/setup.bash
source /home/bianbu/k1muse_communicate_ros/install/setup.bash
colcon build
source install/setup.bash
```

### 8. MUSE 自启动服务

#### 8.1 蓝牙配对 agent

手机端点击 `bianbu` 后完成配对，不需 K1 桌面端确认。

```bash
cd /home/bianbu/k1muse_communicate_ros
sudo cp install/k1muse_mobile_bridge/share/k1muse_mobile_bridge/systemd/k1-bluetooth-pairing-agent.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now k1-bluetooth-pairing-agent.service
```

检查：

```bash
systemctl is-enabled k1-bluetooth-pairing-agent.service
systemctl is-active k1-bluetooth-pairing-agent.service
journalctl -u k1-bluetooth-pairing-agent.service -n 20 --no-pager
bluetoothctl show
bluetoothctl devices Trusted
```

期望：服务为 `enabled` / `active`，日志包含 `Bluetooth pairing agent ready`。

#### 8.2 App 蓝牙入口 supervisor

```bash
cd /home/bianbu/k1muse_communicate_ros
sudo cp install/k1muse_mobile_bridge/share/k1muse_mobile_bridge/systemd/k1-mobile-supervisor.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now k1-mobile-supervisor.service
```

检查：

```bash
systemctl is-enabled k1-mobile-supervisor.service
systemctl is-active k1-mobile-supervisor.service
journalctl -u k1-mobile-supervisor.service -n 80 --no-pager
ls -l /dev/rfcomm0 2>/dev/null || true
```

期望：服务为 `enabled` / `active`。手机未连接时日志停在等待连接。

故障恢复：

```bash
sudo systemctl restart k1-mobile-supervisor.service
sudo rfcomm release /dev/rfcomm0 || true
sudo systemctl status k1-mobile-supervisor.service --no-pager
```

刷机后恢复顺序：先恢复 pairing agent，再恢复 mobile supervisor。

### 9. 最小硬件检查

串口原始数据：

```bash
stty -F /dev/mylidar 230400 raw -echo
timeout 3 dd if=/dev/mylidar bs=1 count=32 2>/dev/null | od -An -tx1

stty -F /dev/mymcu 115200 raw -echo
timeout 3 dd if=/dev/mymcu bs=1 count=32 2>/dev/null | od -An -tx1

stty -F /dev/myimu 115200 raw -echo
timeout 3 dd if=/dev/myimu bs=1 count=32 2>/dev/null | od -An -tx1
```

MCU 桥接 smoke test：

```bash
cd /home/bianbu/k1muse_communicate_ros
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run k1muse_mcu_bridge mcu_bridge_node
```

另开终端：

```bash
source /opt/ros/humble/setup.bash
source /home/bianbu/k1muse_communicate_ros/install/setup.bash
ros2 topic echo --once /mcu/chassis/status
ros2 service list | grep /mcu/chassis
```

---

## 第二部分：树莓派 4B 计算端部署

### 10. 树莓派 OS 和基础环境

要求：Ubuntu 22.04 (Jammy) arm64，ROS2 Humble。

#### 10.1 网络配置

树莓派有线口 `eth0` 设为静态 IP `10.10.10.1/24`，用于 ROS2 数据面。Wi-Fi 连入局域网用于 SSH 管理。

netplan 示例 (`/etc/netplan/01-eth0.yaml`)：

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 10.10.10.1/24
```

```bash
sudo netplan apply
ip addr show eth0  # 确认 10.10.10.1/24
```

#### 10.2 MUSE 有线口配置

MUSE 有线口设为 `10.10.10.2/24`。MUSE 的 netplan 或 connman 配置取决于当前系统：

```bash
# 检查当前网络管理方式
ps aux | grep -E 'NetworkManager|connman|systemd-networkd'

# connman 方式
sudo connmanctl config ethernet_<mac>_cable --ipv4 manual 10.10.10.2 255.255.255.0

# 或用 ip 命令临时设置（重启失效）
sudo ip addr add 10.10.10.2/24 dev eth0
```

两端互 ping 验证：

```bash
# MUSE
ping -c 3 10.10.10.1

# 树莓派
ping -c 3 10.10.10.2
```

#### 10.3 MUSE 到树莓派 SSH 免密

```bash
# 在 MUSE 上生成密钥（如果没有）
ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519 -N ""

# 把 MUSE 公钥写入树莓派
ssh-copy-id -i ~/.ssh/id_ed25519.pub user@10.10.10.1

# 验证免密
ssh user@10.10.10.1 "hostname"
```

#### 10.4 树莓派 ROS2 依赖

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git pkg-config \
  python3-pip python3-colcon-common-extensions \
  ros-humble-cartographer ros-humble-cartographer-ros ros-humble-cartographer-rviz \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-mppi-controller \
  ros-humble-nav2-map-server ros-humble-cartographer-ros-msgs \
  ros-humble-rviz2 ros-humble-rviz-imu-plugin ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-tf2-ros \
  ros-humble-rmw-cyclonedds-cpp
```

#### 10.5 树莓派用户环境

```bash
grep -nE 'ROS|RMW|CYCLONEDDS' ~/.bashrc || true
cat >> ~/.bashrc <<'EOF'

# K1 remote compute ROS2 environment
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
EOF
```

#### 10.6 CycloneDDS 数据面配置

树莓派 `/home/user/.ros/cyclonedds_eth.xml`：

```bash
mkdir -p ~/.ros
cat > ~/.ros/cyclonedds_eth.xml <<'EOF'
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="eth0" priority="default" multicast="default" />
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer Address="10.10.10.2"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
```

MUSE `/home/bianbu/.ros/cyclonedds_end.xml`：

```bash
mkdir -p ~/.ros
cat > ~/.ros/cyclonedds_end.xml <<'EOF'
<CycloneDDS>
  <Domain>
    <General>
      <NetworkInterfaceAddress>10.10.10.2</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer Address="10.10.10.1"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
```

> 启动脚本会自动检测并 export `CYCLONEDDS_URI` 指向上述文件。

**验证 CycloneDDS 互通：**

```bash
# 树莓派
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$HOME/.ros/cyclonedds_eth.xml
ros2 run demo_nodes_cpp talker &
```

```bash
# MUSE
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$HOME/.ros/cyclonedds_end.xml
ros2 run demo_nodes_cpp listener
```

期望：MUSE 能收到树莓派发布的 `/chatter`。

### 11. 树莓派源码同步

```powershell
cd E:\Embed_Game\MusePiPro_prj
ssh -i C:\Users\YJY\.ssh\codex_musepi_ed25519 user@<树莓派Wi-Fi-IP> "rm -rf /home/user/k1muse_communicate_ros/src /home/user/k1muse_slam_ros/src; mkdir -p /home/user/k1muse_communicate_ros /home/user/k1muse_slam_ros"
scp -i C:\Users\YJY\.ssh\codex_musepi_ed25519 -r k1muse_communicate_ros/src user@<树莓派Wi-Fi-IP>:/home/user/k1muse_communicate_ros/
scp -i C:\Users\YJY\.ssh\codex_musepi_ed25519 -r k1muse_slam_ros/src user@<树莓派Wi-Fi-IP>:/home/user/k1muse_slam_ros/
```

如果树莓派 Wi-Fi IP 未知，也可通过 MUSE 有线中继：

```powershell
cd E:\Embed_Game\MusePiPro_prj
scp -r k1muse_communicate_ros/src musepi:/tmp/comm_src/
scp -r k1muse_slam_ros/src musepi:/tmp/slam_src/
ssh musepi "scp -r /tmp/comm_src user@10.10.10.1:/home/user/k1muse_communicate_ros/src && scp -r /tmp/slam_src user@10.10.10.1:/home/user/k1muse_slam_ros/src"
```

### 12. 树莓派构建

```bash
cd /home/user/k1muse_communicate_ros
source /opt/ros/humble/setup.bash
colcon build --packages-select k1muse_mcu_bridge
source install/setup.bash

cd /home/user/k1muse_slam_ros
source /opt/ros/humble/setup.bash
source /home/user/k1muse_communicate_ros/install/setup.bash
colcon build --packages-select imu_ros2_device ldlidar_stl_ros2 k1muse_description k1muse_slam_nav k1muse_exploration
source install/setup.bash
```

### 13. MPPI overlay 构建（树莓派）

> **必须执行**。树莓派上 ROS apt 包提供的 `libmppi_controller.so` 在 Cortex-A72 上触发 SIGILL (exit code -4)，会导致 `controller_server` 启动后立即崩溃，探索/导航无法工作。

#### 13.1 获取源码

```bash
# 本机
cd /tmp
git clone --depth 1 --branch humble https://github.com/ros-navigation/navigation2.git nav2_mppi_src
```

如果 GitHub 不可达，也可在树莓派上用 `apt source ros-humble-nav2-mppi-controller`（需配置 deb-src）。

#### 13.2 传到树莓派

```powershell
scp -r /tmp/nav2_mppi_src/nav2_mppi_controller musepi:/tmp/
```

```bash
# 在 MUSE 上
scp -r /tmp/nav2_mppi_controller user@10.10.10.1:/tmp/
```

#### 13.3 本机构建

```bash
# 在树莓派上
mkdir -p /tmp/mppi_ws/src
cp -r /tmp/nav2_mppi_controller /tmp/mppi_ws/src/
cd /tmp/mppi_ws
source /opt/ros/humble/setup.bash
colcon build --parallel-workers 1 --packages-select nav2_mppi_controller \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
```

> `--parallel-workers 1` 避免树莓派 4B (4GB RAM) OOM。构建耗时约 7 分钟。

#### 13.4 部署 overlay

```bash
rm -rf /home/user/mppi_overlay
cp -r /tmp/mppi_ws/install /home/user/mppi_overlay
ls /home/user/mppi_overlay/nav2_mppi_controller/lib/libmppi_controller.so
```

> MPPI overlay 由 `k1_start_map.sh` 和 `k1_start_exploration.sh` 的远程段自动 source。

#### 13.5 验证 MPPI 不会崩溃

```bash
source /opt/ros/humble/setup.bash
source /home/user/mppi_overlay/setup.bash
timeout 15 ros2 run nav2_controller controller_server --ros-args \
  --params-file /home/user/k1muse_slam_ros/src/k1muse_slam_nav/config/nav2_params.yaml
# 正常退出码 124 (timeout)，不应出现 exit code -4
```

### 14. 树莓派 swap（可选）

MPPI 构建和 Cartographer 运行时可能消耗较多内存。建议给树莓派 4B 添加 swap：

```bash
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

---

## 第三部分：全系统集成验证

### 15. 分布式建图 smoke test

清理上一轮进程：

```bash
# MUSE
/home/bianbu/k1muse_slam_ros/install/k1muse_exploration/lib/k1muse_exploration/k1_stop_exploration.sh fast || true
/home/bianbu/k1muse_slam_ros/install/k1muse_slam_nav/lib/k1muse_slam_nav/k1_stop_mapping.sh fast || true
/home/bianbu/k1muse_communicate_ros/install/k1muse_mcu_bridge/lib/k1muse_mcu_bridge/k1_stop_mcu_bridge.sh || true
```

启动建图：

```bash
source /opt/ros/humble/setup.bash
source /home/bianbu/k1muse_communicate_ros/install/setup.bash
source /home/bianbu/k1muse_slam_ros/install/setup.bash
k1_start_mapping.sh
```

验证数据流：

```bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0

# 数据速率
ros2 topic hz /scan        # 期望 ~10 Hz
ros2 topic hz /odom        # 期望 ~20 Hz
ros2 topic hz /imu/data    # 期望 ~10 Hz

# 地图
timeout 10 ros2 topic echo --once /map --field info

# TF
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_ros tf2_echo odom base_footprint

# 树莓派进程
ssh user@10.10.10.1 "pgrep -af 'cartographer|occupancy_grid'"
```

停止建图：

```bash
k1_save_map.sh manual
k1_stop_mapping.sh fast
```

### 16. 分布式探索 smoke test

```bash
k1_start_exploration.sh -2.3 3.0 -1.2 2.35
```

验证节点（期望所有 6 个关键节点可见）：

```bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0
ros2 node list | grep -E 'cartographer_node|cartographer_occupancy|controller_server|planner_server|bt_navigator|rrt_frontier'
```

检查树莓派 controller_server 进程（不应崩溃）：

```bash
ssh user@10.10.10.1 "pgrep -af controller_server"
# 期望：有 pid 输出，说明 MPPI 正常
```

检查探索日志：

```bash
ssh user@10.10.10.1 "tail -n 40 /home/user/.ros/k1muse_exploration/logs/exploration.log | grep -E 'controller_server|rrt_frontier|goal|error|ERROR'"
# 期望：有 "Sent exploration goal"，无 "process has died"
```

停止探索：

```bash
k1_stop_exploration.sh fast
```

---

## 第四部分：回退和故障排查

### 17. 回退到 MUSE 本地计算

如果树莓派不可用或需要测试 MUSE 单机模式：

```bash
K1_REMOTE_COMPUTE=0 k1_start_mapping.sh
K1_REMOTE_COMPUTE=0 K1_MAPPING_USE_RVIZ=true k1_start_mapping.sh
K1_REMOTE_COMPUTE=0 k1_save_map.sh manual
K1_REMOTE_COMPUTE=0 k1_stop_mapping.sh fast
K1_REMOTE_COMPUTE=0 k1_start_exploration.sh -2.3 3.0 -1.2 2.35
K1_REMOTE_COMPUTE=0 k1_stop_exploration.sh fast
```

注意：MUSE 本地计算性能远低于树莓派，Cartographer/Nav2 会加重 MUSE 负载。

### 18. 回退 MPPI overlay

如果 overlay 导致问题，恢复系统 MPPI（会再次 SIGILL）：

```bash
ssh user@10.10.10.1 "rm -rf /home/user/mppi_overlay"
```

然后从 `k1_start_map.sh` 和 `k1_start_exploration.sh` 中删除 `source /home/user/mppi_overlay/setup.bash` 行。

恢复后探索/导航不可用（controller 崩溃），但建图仍正常工作。

### 19. 常见故障排查

#### 19.1 ros2 node list/topic list 无输出或很慢

ROS2 CLI 在 RISC-V 上很慢。设置环境变量后再试：

```bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0
```

也可直接检查进程而非通过 ros2 CLI：

```bash
# MUSE
pgrep -af 'mcu_bridge|lidar|ldlidar|ybimu|robot_state|scan_self'

# 树莓派
ssh user@10.10.10.1 "pgrep -af 'cartographer|controller_server|planner|bt_navigator|rrt_frontier'"
```

#### 19.2 controller_server exit code -4

树莓派上 MPPI overlay 未构建或未正确 source。检查：

```bash
ssh user@10.10.10.1 "ls -la /home/user/mppi_overlay/nav2_mppi_controller/lib/libmppi_controller.so"
```

如果文件存在但问题仍存在，检查 `k1_start_exploration.sh` 远程段是否包含：

```bash
source /home/user/mppi_overlay/setup.bash 2>/dev/null || true
```

#### 19.3 建图/探索启动超时

查看对应日志获取具体错误：

```bash
# MUSE
tail -n 80 /home/bianbu/.ros/k1muse_slam_nav/logs/slam_mapping.log

# 树莓派
ssh user@10.10.10.1 "tail -n 80 /home/user/.ros/k1muse_slam_nav/logs/slam_mapping.log"
ssh user@10.10.10.1 "tail -n 80 /home/user/.ros/k1muse_exploration/logs/exploration.log"
```

常见原因：

- `/odom` 不可见 → 检查 MCU bridge 是否启动、`/dev/mymcu` 是否存在
- Cartographer 不启动 → 检查 `/scan`、`/odom`、`/imu/data` 是否都有数据
- 树莓派 SSH 不通 → 检查有线连接、IP 配置、SSH key
- ROS2 节点发现不到 → 检查 `ROS_DOMAIN_ID=42`、CycloneDDS 配置、网络互通

#### 19.4 树莓派有线 SSH 超时

可能原因：树莓派系统负载过高（编译或其他进程）。处理：

1. 等待 1-2 分钟让进程自然结束
2. 通过 Wi-Fi IP SSH 进去 `sudo reboot`
3. 检查 MUSE 有线口配置 `ip addr show eth0`

#### 19.5 蓝牙地图传输慢/遥控延迟

- 确认 `mobile_bridge_node` 使用了最新配置（`tile_size: 32` 等）
- 重启 supervisor：`sudo systemctl restart k1-mobile-supervisor.service`
- 检查 RFCOMM 设备：`ls -l /dev/rfcomm0`

#### 19.6 App 连接不上/MUSE 蓝牙不可见

```bash
systemctl status k1-bluetooth-pairing-agent.service
systemctl status k1-mobile-supervisor.service
journalctl -u k1-bluetooth-pairing-agent.service -n 30 --no-pager
bluetoothctl show  # 确认 Powered: yes, Discoverable: yes
```

如果 pairing agent 未运行：`sudo systemctl restart k1-bluetooth-pairing-agent.service`
如果 supervisor 未运行：`sudo systemctl restart k1-mobile-supervisor.service`

---

## 附录 A：全部文件路径速查

### MUSE (`/home/bianbu/`)

```
k1muse_communicate_ros/
├── src/           ← 同步自本机 k1muse_communicate_ros/src/
├── build/
├── install/
└── log/

k1muse_slam_ros/
├── src/           ← 同步自本机 k1muse_slam_ros/src/
├── build/
├── install/
└── log/

.ros/
├── cyclonedds_end.xml     ← CycloneDDS MUSE 配置
├── k1muse_slam_nav/run/   ← 建图 PID
├── k1muse_slam_nav/logs/  ← 建图日志
├── k1muse_exploration/run/  ← 探索 PID
└── k1muse_exploration/logs/ ← 探索日志

~/.bashrc
~/start_wayvnc.sh
```

### 树莓派 (`/home/user/`)

```
k1muse_communicate_ros/
├── src/           ← 同步自本机
├── build/
└── install/

k1muse_slam_ros/
├── src/           ← 同步自本机
├── build/
└── install/

mppi_overlay/           ← MPPI 本机构建 overlay
└── nav2_mppi_controller/lib/
    ├── libmppi_controller.so
    └── libmppi_critics.so

.ros/
├── cyclonedds_eth.xml     ← CycloneDDS 树莓派配置
├── k1muse_slam_nav/run/   ← 建图 PID
├── k1muse_slam_nav/logs/  ← 建图日志
├── k1muse_exploration/run/  ← 探索 PID
└── k1muse_exploration/logs/ ← 探索日志

~/.bashrc
/swapfile                ← 可选 2GB swap
```

## 附录 B：依赖版本锁定

| 包 | 版本 | 架构 | 说明 |
|---|---|---|---|
| ROS2 | Humble | riscv64 / aarch64 | 主线 ROS2 版本 |
| Cartographer | humble package | riscv64 / aarch64 | apt 安装 |
| Nav2 | humble package | aarch64 | 仅树莓派 |
| Nav2 MPPI | 1.1.20 (overlay rebuild) | aarch64 | 树莓派本机重编，替代 apt 包 |
| CycloneDDS | humble package | riscv64 / aarch64 | ROS2 RMW |
| 内核 (MUSE) | 6.6.63 | riscv64 | 固定版本，DTB 绑定 |
| 内核 (RPi) | Ubuntu Jammy 默认 | aarch64 | 不锁定 |
