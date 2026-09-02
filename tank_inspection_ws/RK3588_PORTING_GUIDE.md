# RK3588 移植、安装与运行指南

更新时间：2026-07-17

本文档说明如何把 `tank_inspection_ws` 从当前 x86_64 开发机迁移到 RK3588 arm64 平台，
完成 ROS 2、MCAP、D405、MID-360 和项目依赖的安装、构建与验收。

重要边界：当前工程已在 Ubuntu 24.04 x86_64 + ROS 2 Jazzy 上完成 mock 端到端测试、
D405 深度设备冒烟测试和 MID-360 点云/IMU 冒烟测试，但尚未在任何 RK3588 板卡上实际
构建或运行。因此本文档是迁移
实施方案，不是 RK3588 已通过报告。所有构建、吞吐、温度、USB、网络和 MCAP 结果都必须
在目标板上重新测量。

## 1. 推荐结论

优先使用：

```text
RK3588 arm64
Ubuntu 24.04 LTS 64-bit
ROS 2 Jazzy
系统 Python 3.12
NVMe SSD + ext4
D405 直连 USB 3.x
MID-360 使用独立千兆以太网口
```

理由：当前工作区是在 Ubuntu 24.04 + Jazzy 上开发和验证的；ROS 2 Jazzy 官方支持
Ubuntu Noble 24.04 的 64-bit ARM。这样可以最大限度减少 rosbag2 CLI、Python、Launch
和依赖版本差异。

兼容路线：

```text
Ubuntu 22.04 LTS 64-bit + ROS 2 Humble + 系统 Python 3.10
```

ROS 2 Humble 官方支持 Ubuntu 22.04 arm64，但当前工程没有在 Humble 上构建验证。
选择 Humble 后必须重新检查：

- rosbag2/MCAP CLI参数。
- `rosbag2_py` API兼容性。
- MCAP writer option字段。
- RealSense wrapper与librealsense版本。
- Livox驱动分支和构建脚本。
- 全部现有功能测试及完整端到端流程。

禁止组合：

- Ubuntu 22.04 + ROS 2 Jazzy二进制包。
- Ubuntu 24.04 + ROS 2 Humble二进制包。
- x86_64 的 `build/`、`install/` 或二进制文件复制到 arm64。
- Conda Python与系统ROS Python混用。
- ROS服务器版librealsense、RealSense上游deb和手工源码安装同时存在。

## 2. 官方资料

迁移时只以以下官方资料为准：

- [ROS 2 Jazzy Ubuntu安装](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)
- [ROS 2 Humble Ubuntu安装](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [ROS apt source](https://github.com/ros-infrastructure/ros-apt-source)
- [rosbag2和MCAP官方仓库](https://github.com/ros2/rosbag2)
- [RealSense ROS 2 Wrapper](https://github.com/realsenseai/realsense-ros)
- [librealsense Linux发行版安装](https://github.com/realsenseai/librealsense/blob/master/doc/distribution_linux.md)
- [librealsense Linux源码安装](https://github.com/realsenseai/librealsense/blob/master/doc/installation.md)
- [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2)
- [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2)

官方Livox仓库当前列出了 Ubuntu 24.04/Jazzy 和 Ubuntu 22.04/Humble；官方README也说明
该驱动主要用于调试和测试，正式产品使用前仍需完成针对本项目的持续负载与故障测试。

## 3. RK3588硬件建议

### 3.1 最低准备

- RK3588板卡，推荐至少8 GiB内存，16 GiB更适合长时多模态采集。
- 厂商明确支持的Ubuntu 24.04 arm64系统镜像。
- 稳定电源和主动散热；不能依赖无风扇裸板完成生产压力测试。
- NVMe SSD，容量按任务时长估算；不建议把原始MCAP长期写入microSD。
- D405使用质量可靠的USB 3.x数据线，避免低速Hub。
- MID-360使用独立千兆以太网链路，雷达按厂商要求独立供电。
- 如需跨设备高精度同步，准备设备支持的时间源、网络或硬件触发链路。

### 3.2 为什么优先NVMe

相机、点云、IMU和探头波形同时录制时，瞬时写入量会明显高于平均MCAP文件增长速度。
Zstd压缩率不能作为容量承诺。NVMe应提供：

- 足够的持续写入速度和IOPS。
- 比完整任务估算至少更大的安全余量。
- 可查询的温度和健康状态。
- `ext4`等本地Linux文件系统。

正式采集时先写本地NVMe，关闭、验证和哈希后再复制到NAS。不要直接把唯一原始数据写到
不稳定网络挂载，也不要自动删除本地原始MCAP。

### 3.3 NPU/GPU不是本阶段依赖

当前阶段只有采集、缓存、时间关联、MCAP和回放，不需要安装：

- RKNN Toolkit。
- NPU运行时。
- CUDA。
- TensorRT。
- PyTorch或其他神经网络框架。

这些组件不应加入采集进程，避免引入额外ABI、内存和温度风险。

## 4. 安装前只读检查

在RK3588上先执行并保存输出：

```bash
pwd
cat /etc/os-release
lsb_release -a
uname -a
uname -m
dpkg --print-architecture
python3 --version
echo "$ROS_DISTRO"
command -v ros2 || true
df -h
free -h
lsblk -o NAME,SIZE,FSTYPE,MOUNTPOINTS,MODEL
lsusb -t
ip -br link
ip -br addr
```

Jazzy推荐路线的预期关键值：

```text
Ubuntu 24.04 / noble
uname -m: aarch64
dpkg architecture: arm64
Python: 3.12.x
```

如果厂商镜像只是在说明中称为“Ubuntu”，但 `/etc/os-release` 不是标准Noble/Jammy，
不要强行添加ROS apt源。先确认该BSP的glibc、apt仓库和内核兼容性。

同时记录：

```bash
cat /proc/device-tree/model 2>/dev/null || true
cat /sys/firmware/devicetree/base/model 2>/dev/null || true
```

RK3588厂商内核可能是定制5.10、6.1或其他版本。不要因为Ubuntu用户空间版本正确就假设
RealSense DKMS、网卡驱动或USB控制器行为与标准PC内核一致。

## 5. 安装Ubuntu 24.04 + ROS 2 Jazzy

以下命令会修改系统并使用 `sudo`。先确认板卡系统可恢复、网络正常并得到授权。

### 5.1 Locale和基础工具

```bash
sudo apt update
sudo apt install -y locales software-properties-common curl gnupg lsb-release
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository universe
```

对于RK厂商BSP，不要未经厂商确认直接执行全系统 `dist-upgrade` 或更换内核。先查看：

```bash
apt list --upgradable
```

再根据板卡厂商升级策略处理。

### 5.2 添加ROS官方apt源

官方当前推荐使用 `ros2-apt-source` 包配置密钥和软件源：

```bash
export ROS_APT_SOURCE_VERSION="$(curl -s \
  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | awk -F'"' '/tag_name/ {print $4; exit}')"

curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
```

执行前可打开官方release页面核对版本和文件名。不要从非官方网盘或未知镜像下载密钥包。

### 5.3 安装ROS、构建工具和MCAP

无桌面采集设备优先安装 `ros-base`：

```bash
sudo apt install -y \
  ros-jazzy-ros-base \
  ros-dev-tools \
  ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-diagnostic-updater \
  python3-yaml \
  jq rsync sysstat usbutils ethtool smartmontools lm-sensors
```

需要在RK桌面直接查看图像或RViz时再安装：

```bash
sudo apt install -y ros-jazzy-rqt-image-view ros-jazzy-rviz2
```

不要为无显示器设备安装完整Desktop，除非确有需求。

### 5.4 初始化rosdep

```bash
sudo rosdep init
rosdep update
```

如果提示rosdep已经初始化，不要删除已有配置，直接执行 `rosdep update`。

### 5.5 验证基础ROS环境

```bash
source /opt/ros/jazzy/setup.bash
echo "$ROS_DISTRO"
command -v ros2
ros2 --help
ros2 pkg prefix rosbag2_storage_mcap
python3 --version
```

预期：

```text
ROS_DISTRO=jazzy
ros2位于/opt/ros/jazzy/bin/ros2
Python 3.12
能够找到rosbag2_storage_mcap
```

不要在 `(base)` Conda提示符下继续。若系统自动激活Conda：

```bash
conda deactivate
```

并关闭Conda自动激活base环境。

## 6. Ubuntu 22.04 + Humble兼容路线

只有板卡BSP无法稳定支持Ubuntu 24.04时才选此路线。使用ROS Humble官方安装文档配置源，
然后安装：

```bash
sudo apt install -y \
  ros-humble-ros-base \
  ros-dev-tools \
  ros-humble-rosbag2-storage-mcap \
  ros-humble-diagnostic-updater \
  python3-yaml \
  jq rsync sysstat usbutils ethtool smartmontools lm-sensors
```

检查插件候选：

```bash
apt-cache policy ros-humble-rosbag2-storage-mcap
```

如果没有候选包，停止并根据官方rosbag2 Humble分支制定源码安装方案；不要伪造MCAP插件。

Humble构建完成后必须运行本指南第11节的全部测试。现有Jazzy测试结果不能替代Humble结果。

## 7. 迁移工作区源码

### 7.1 不要复制架构相关产物

从x86开发机迁移时不要复制：

```text
build/
install/
log/
bags/
remote_store/
reports/ros_logs/
```

这些目录包含x86二进制、旧日志和原始数据，不应混入RK构建。

### 7.2 使用rsync复制源码

在x86开发机执行，替换 `<RK_USER>` 和 `<RK_HOST>`：

```bash
rsync -a --info=progress2 \
  --exclude build/ \
  --exclude install/ \
  --exclude log/ \
  --exclude bags/ \
  --exclude remote_store/ \
  --exclude reports/ros_logs/ \
  ~/physic_bev/tank_inspection_ws/ \
  <RK_USER>@<RK_HOST>:~/tank_inspection_ws/
```

也可以使用只包含源码和文档的离线压缩包。不要初始化Git，除非项目管理流程另有要求。

### 7.3 在RK上创建运行目录

```bash
cd ~/tank_inspection_ws
mkdir -p bags remote_store reports/ros_logs
```

建议计算迁移包或源码清单的SHA256，在两端比较，避免网络或U盘复制损坏。

## 8. 安装项目依赖并构建

### 8.1 使用rosdep安装依赖

Jazzy：

```bash
cd ~/tank_inspection_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
```

Humble则把两处 `jazzy` 替换为 `humble`。

注意：`inspection_bringup` 声明了RealSense运行依赖，因此rosdep可能安装RealSense ROS包。
在执行前查看rosdep计划；如果目标板暂不接D405，可先明确依赖策略，但不能伪造包。

### 8.2 原生arm64构建

推荐直接在RK3588上构建，不复制x86产物：

```bash
cd ~/tank_inspection_ws
source /opt/ros/jazzy/setup.bash
export ROS_LOG_DIR="$PWD/reports/ros_logs"

colcon build --symlink-install \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

当前工作区有8个包，其中 Livox 驱动和 PCL 依赖会显著增加编译内存；顺序构建能减少
峰值内存。确认内存和散热充足后，可改为：

```bash
colcon build --symlink-install \
  --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

不要一开始使用全部8核并行；源码编译、Zstd和系统后台任务可能造成温度和内存峰值。

构建后：

```bash
source install/setup.bash
ros2 pkg list | grep '^inspection_'
```

应看到以下工程包，并能单独找到 Livox 厂商包：

```text
inspection_interfaces
inspection_livox_adapter
inspection_sync
inspection_mock_sensors
inspection_bringup
inspection_tools
eddy_driver
livox_ros_driver2
```

## 9. RealSense D405在RK3588上的安装

RK3588通常使用厂商定制内核。RealSense安装必须选择且只选择一种路径。

### 9.1 路径A：ROS官方二进制包，优先尝试

先检查arm64候选：

```bash
apt-cache policy \
  ros-jazzy-realsense2-camera \
  ros-jazzy-realsense2-camera-msgs \
  ros-jazzy-librealsense2
```

三者都有正确Jazzy/arm64候选时：

```bash
sudo apt install -y ros-jazzy-realsense2-camera
```

安装后检查版本来自同一ROS发行源和相近构建批次：

```bash
dpkg-query -W -f='${Package}\t${Version}\n' \
  ros-jazzy-realsense2-camera \
  ros-jazzy-realsense2-camera-msgs \
  ros-jazzy-librealsense2 \
  ros-jazzy-diagnostic-updater
```

不要混用RealSense上游deb和ROS版 `ros-jazzy-librealsense2`。官方RealSense ROS文档明确
要求三种SDK安装方案只选一种，以免多版本和workspace冲突。

### 9.2 验证USB和设备权限

连接D405后：

```bash
source /opt/ros/jazzy/setup.bash
lsusb
lsusb -t
rs-enumerate-devices -s
```

必须确认设备实际连接为USB 3.x，而不是只看接口外观。若枚举失败，先检查：

- USB线和接口。
- 稳定供电。
- udev规则。
- 当前用户的设备访问权限。
- 是否有其他RealSense进程占用设备。
- RK3588 USB控制器或BSP内核日志。

必要时查看只读内核日志：

```bash
dmesg | tail -n 100
```

### 9.3 路径B：源码/RSUSB后端，仅在路径A失败时

RealSense官方说明，RSUSB后端可以绕过特定V4L2内核补丁，适合非标准ARM内核；但生产环境
仍需在RK3588上做充分稳定性测试。启用此路径前：

1. 不得保留另一套librealsense运行库。
2. 从所选 `realsense-ros` release确认兼容的librealsense tag。
3. 记录tag/commit，不使用无法复现的浮动master。
4. 获取系统安装授权。

依赖：

```bash
sudo apt install -y \
  git cmake build-essential pkg-config \
  libusb-1.0-0-dev libudev-dev libssl-dev
```

按官方源码仓库操作，版本用占位符表示：

```bash
git clone https://github.com/realsenseai/librealsense.git
cd librealsense
git checkout <SUPPORTED_LIBREALSENSE_TAG>
./scripts/setup_udev_rules.sh

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DFORCE_RSUSB_BACKEND=true \
  -DBUILD_EXAMPLES=false \
  -DBUILD_GRAPHICAL_EXAMPLES=false \
  -DBUILD_WITH_DDS=false

cmake --build build --parallel 2
sudo cmake --install build
sudo ldconfig
```

然后根据RealSense ROS官方文档从源码构建wrapper，并在rosdep时跳过 `librealsense2`：

```bash
mkdir -p ~/ws_realsense/src
git clone https://github.com/realsenseai/realsense-ros.git \
  -b ros2-master ~/ws_realsense/src/realsense-ros

cd ~/ws_realsense
source /opt/ros/jazzy/setup.bash
rosdep install -i --from-path src --rosdistro jazzy \
  --skip-keys=librealsense2 -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

在实际项目中应固定wrapper的release/tag，而不是长期使用浮动 `ros2-master`。上面的clone命令
对应官方说明，首次验证后要把选定版本记录进RK验收报告。

### 9.4 D405启动测试

```bash
cd ~/tank_inspection_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_LOG_DIR="$PWD/reports/ros_logs"

ros2 launch inspection_bringup hardware_pipeline.launch.py \
  start_d405:=true \
  d405_serial_no:=_YOUR_SERIAL \
  d405_depth_profile:=640x480x30 \
  start_sync:=false \
  record:=false
```

另一个终端：

```bash
source /opt/ros/jazzy/setup.bash
source ~/tank_inspection_ws/install/setup.bash
ros2 topic echo /camera/camera/depth/image_rect_raw \
  sensor_msgs/msg/Image --no-arr --once --timeout 10
ros2 topic hz /camera/camera/depth/image_rect_raw --window 200 --wall-time
```

不能沿用x86上的30 Hz结论。必须在RK的真实USB链路重新记录平均帧率、最大间隔、CPU、RSS和
丢帧诊断。

## 10. MID-360和Livox驱动安装

### 10.1 安装Livox SDK2

官方Livox SDK2支持ARM，安装会写入 `/usr/local`，必须获得授权：

```bash
sudo apt install -y git cmake build-essential

git clone https://github.com/Livox-SDK/Livox-SDK2.git ~/Livox-SDK2
cd ~/Livox-SDK2
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
sudo cmake --install build
sudo ldconfig
```

正式工程应固定SDK tag或commit并记录，不长期依赖浮动master。

### 10.2 构建livox_ros_driver2

当前工程已把 `livox_ros_driver2` 源码放在主工作区：

```bash
cd ~/tank_inspection_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to inspection_bringup \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
```

迁移包中若缺少 `src/livox_ros_driver2`，应先从官方仓库获取与 x86 基线相同的固定 tag/
commit 到该目录，再应用并审核本工程记录的兼容补丁；不能临时使用浮动 master。Humble
需要切换匹配分支并重新验证，不能直接复用当前 Jazzy 结果。

当前 x86 基线是 driver 1.2.6 + SDK2 1.2.5，普通 MID-360 已通过实机测试；源码对
SDK-1.2.6 才出现的 MID-360S 枚举做了版本保护。RK 正式部署优先把 SDK2 和 driver 固定到
同一已验证版本，再重新构建和实测。

不要在主工作区直接运行厂商 `build.sh`：该脚本会删除工作区级的 `../../build/` 和
`../../install/`，从而清除其他工程包的构建产物。本项目统一使用上面的 `colcon build`
命令，确保驱动、Launch、IMU 适配器和记录配置一起构建。

### 10.3 Overlay顺序

所有外部驱动都使用ROS二进制包时，运行工程只需：

```bash
source /opt/ros/jazzy/setup.bash
source ~/tank_inspection_ws/install/setup.bash
```

默认情况下 Livox 已在主工作区，不需要额外 `ws_livox` overlay。若 RealSense 使用独立
源码 workspace，每个新终端按固定顺序：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_realsense/install/setup.bash   # 仅源码RealSense路径需要
source ~/tank_inspection_ws/install/setup.bash
```

顺序含义是：ROS underlay -> 可选外部驱动 overlay -> 本项目 overlay。不要同时 source
旧的 Humble、Jazzy、x86 共享目录或其他不相关机器人 workspace。检查：

```bash
echo "$AMENT_PREFIX_PATH" | tr ':' '\n'
ros2 pkg prefix realsense2_camera
ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix inspection_bringup
```

如果使用二进制RealSense包，则不存在 `~/ws_realsense`，不要创建空目录或伪造setup文件。

### 10.4 网络配置原则

修改RK网口地址前先只读记录：

```bash
ip -br link
ip -br addr
ip route
ethtool <LIDAR_NIC>
```

然后依据MID-360当前官方手册和Livox JSON配置填写：

```text
lidar_ip
host_ip
cmd_data_port
push_msg_port
point_data_port
imu_data_port
log_data_port
```

本指南不提供虚构IP。主机和雷达必须同网段，且不能覆盖RK的管理网口、默认路由或其他设备
网络。任何Netplan/NetworkManager修改都应单独审核并保留恢复方法。

连通性检查：

```bash
ip -s link show <LIDAR_NIC>
ping -c 4 <MID360_IP>
```

### 10.5 PointCloud2优先

首次可参考官方 `rviz_MID360_launch.py` 验证PointCloud2。无显示器RK不应长期启动RViz，
应基于官方Launch保留驱动节点和PointCloud2输出、移除RViz进程。

启动后检查实际Topic：

```bash
ros2 topic list -t
ros2 topic info /livox/lidar -v
ros2 topic echo /livox/lidar sensor_msgs/msg/PointCloud2 \
  --no-arr --once --timeout 10
ros2 topic hz /livox/lidar --window 100 --wall-time
```

`livox_ros_driver2` 1.2.6 的 `xfer_format=0` PointXYZRTLT输出应包含：

```text
x, y, z, intensity, tag, line, timestamp
```

其中官方 `timestamp` 是每点 `float64` 时间戳。独立适配器也可提供已明确单位、符号和参考点
的 `time_offset`，验证器要求两者至少有一个。如果点时间字段缺失、所有帧长期为相同值，或
字段语义不明，必须写明确的适配器或调整接口方案，不能伪造字段。核心工程不依赖Livox
CustomMsg；只有确需CustomMsg时才在独立适配层使用。

MID-360 官方协议的角速度是 rad/s、加速度是 g。主工作区会保留厂商值到
`/livox/imu_raw`，并由 `inspection_livox_adapter` 把加速度乘以 9.80665 后发布标准
`/livox/imu`。RK 验收时必须同时检查两路约 200 Hz、时间戳一致、转换诊断
`rejected_count=0`、`non_monotonic_count=0`。

## 11. RK3588上的工程测试

### 11.1 全量构建和单元测试

```bash
cd ~/tank_inspection_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_LOG_DIR="$PWD/reports/ros_logs"

colcon test
colcon test-result --verbose
```

当前x86基线是：

```text
16 tests, 0 errors, 0 failures, 0 skipped
```

其中包含 14 个项目测试用例和两个 CTest wrapper 结果。Livox 厂商源码的格式 lint 默认
关闭；若显式开启，它会因厂商原始格式和内嵌 RapidJSON 产生既有失败，不应与项目功能
测试混为一谈。RK 必须独立达到功能测试同样结果，不能把 x86 报告复制成 RK 报告。

### 11.2 先跑mock端到端

即使最终使用真实硬件，也先用mock隔离ARM编译、DDS和MCAP问题：

```bash
ros2 launch inspection_bringup mock_pipeline.launch.py \
  record:=true \
  debug_mode:=true \
  output_directory:="$PWD/bags/rk_mock_001"
```

保持至少65秒，正常 `Ctrl+C`。然后：

```bash
ros2 bag info -s mcap bags/rk_mock_001
ros2 run inspection_tools validate_bag \
  bags/rk_mock_001 \
  --output reports/rk_mock_001_validation.json
```

要求：

- 至少两个自动分卷。
- 13个要求Topic存在。
- 验证 `valid=true`、`errors=[]`。
- 缓存长度不超过配置上限。
- 无无限增长、未关闭文件或时间戳倒退。

### 11.3 回放和重新生成FusionIndex

终端A：

```bash
ros2 run inspection_sync inspection_sync_node \
  --ros-args --params-file src/inspection_bringup/config/cache.yaml
```

终端B：

```bash
ros2 bag play bags/rk_mock_001 --rate 2.0 \
  --exclude-topics /derived/inspection/fusion_index /diagnostics
```

确认新的 `/derived/inspection/fusion_index` 被生成，且缺失/超时不会被标记为有效。

### 11.4 D405单设备录制

按照 [HARDWARE_INTEGRATION_GUIDE.md](HARDWARE_INTEGRATION_GUIDE.md) 执行D405 65秒录制，
使用 `--allow-partial` 验证。记录RK上的实际：

- USB类型。
- 图像帧率和最大间隔。
- MCAP平均与峰值写入。
- CPU、RSS、温度。
- 分卷耗时和是否丢帧。
- 正常关闭与回放结果。

### 11.5 完整多模态测试

D405、MID-360、IMU、Odometry和涡流都接通后：

- 设置 `start_sync:=true`。
- 默认以 `/inspection/eddy_current/raw` 为硬件融合锚点。
- 运行不带 `--allow-partial` 的完整验证。
- 至少进行5--15分钟短测和1小时生产配置压力测试。
- 断开/恢复一个设备，验证诊断和 `valid=false` 行为。

## 12. 性能、温度和存储监控

安装 `sysstat` 后可在采集期间使用：

```bash
pidstat -durh 1
iostat -dx 1
free -h
df -h
ip -s link show <LIDAR_NIC>
```

读取温度：

```bash
for zone in /sys/class/thermal/thermal_zone*; do
  printf '%s ' "$zone"
  cat "$zone/type" 2>/dev/null
  cat "$zone/temp" 2>/dev/null
done
```

查看CPU调频策略：

```bash
cat /sys/devices/system/cpu/cpufreq/policy*/scaling_governor
```

不要直接套用其他RK板卡的性能模式命令。调频、电源和NPU配置依赖板卡BSP，写操作必须按
厂商文档执行并记录。

检查NVMe挂载和健康：

```bash
findmnt -T <NVME_BAG_DIRECTORY>
df -h <NVME_BAG_DIRECTORY>
sudo smartctl -a <NVME_DEVICE>
```

容量估算：

```bash
ros2 run inspection_tools estimate_storage --json \
  --task-duration-minutes 60
```

估算不包含所有协议开销，也不保证Zstd压缩率。正式任务开始前应至少保留完整任务估算加
安全余量，并监控剩余空间；空间不足时停止新任务，不能自动删除原始MCAP。

## 13. 运行目录和生产操作

建议把源码与数据分开：

```text
~/tank_inspection_ws/             # 源码、build、install、log
<NVME_MOUNT>/tank_bags/           # 原始MCAP
<NVME_MOUNT>/tank_remote_store/   # 本机NAS模拟或暂存副本
<NVME_MOUNT>/tank_reports/        # 验证与资源报告
```

启动时通过参数传入新任务目录，不把挂载点写死进源代码：

```bash
ros2 launch inspection_bringup hardware_pipeline.launch.py \
  start_d405:=true \
  d405_serial_no:=_YOUR_SERIAL \
  start_sync:=false \
  record:=true \
  debug_mode:=true \
  output_directory:=<NVME_MOUNT>/tank_bags/<UNIQUE_TASK_DIRECTORY>
```

暂时不要把rosbag recorder做成开机自动启动的固定systemd服务，因为固定输出目录会产生
覆盖/复用风险。应先实现明确的任务ID、资产元数据、唯一目录、磁盘预检和正常停止流程，
再设计服务化。

## 14. 迁移后的验收门槛

### 系统

- [ ] `uname -m` 为 `aarch64`，`dpkg`架构为 `arm64`。
- [ ] OS与ROS发行版匹配。
- [ ] 未激活Conda/venv。
- [ ] ROS包来自一致的软件源，没有混装重复librealsense。
- [ ] NVMe、USB和网口拓扑已记录。
- [ ] 主动散热和稳定电源已确认。

### 构建和测试

- [ ] 不使用x86 `build/`、`install/`和`log/`。
- [ ] arm64原生 `colcon build`通过。
- [ ] `colcon test-result` 的16项结果全部通过（14个项目用例）。
- [ ] mock 65秒MCAP、分卷、验证和回放通过。
- [ ] `STATUS_RK3588.md`记录真实结果，不复制x86结论。

### D405

- [ ] `rs-enumerate-devices`识别真实设备。
- [ ] 实际USB链路为USB 3.x。
- [ ] 深度图尺寸、编码、step和data长度正确。
- [ ] RK实测帧率、CPU、RSS和温度满足要求。
- [ ] 深度+彩色同时启用时重新完成带宽测试。

### MID-360

- [ ] SDK2和driver版本/commit已固定并记录。
- [ ] 主机与雷达IP、端口和恢复配置已记录。
- [ ] PointCloud2字段、offset、datatype和`timestamp`/`time_offset`语义正确。
- [ ] IMU单位、协方差和时间戳正确。
- [ ] 断网/重连、丢包和网卡统计已验证。

### 数据链路

- [ ] 原始模态仍按独立Topic录制。
- [ ] FusionIndex不复制原始大数组。
- [ ] 完整验证不使用 `--allow-partial`。
- [ ] 调试和生产分卷均正常关闭。
- [ ] Manifest状态达到 `VERIFIED`。
- [ ] 本地与副本全部SHA256一致。
- [ ] 没有自动删除本地原始文件。
- [ ] 1小时压力测试无缓存无限增长、热降频或持续丢帧。

## 15. 建议生成RK专用状态文件

首次RK验证时新建：

```text
STATUS_RK3588.md
```

至少记录：

- 板卡品牌/型号、内存、BSP、OS、内核和架构。
- ROS发行版和关键包版本。
- NVMe型号、文件系统、挂载点和剩余空间。
- D405/MID-360/探头固件和驱动版本。
- 构建与测试结果。
- mock和真实硬件MCAP路径、大小、时长和SHA256。
- CPU、RSS、温度、磁盘写入和网卡丢包。
- USB与网络拓扑。
- 已知问题、临时规避和下一步。

在这些结果真实完成前，不要把现有 [STATUS.md](STATUS.md) 中的x86数据改写成RK数据，
也不要声称完成RK3588多模态硬件验证。


  先执行：

  cd /home/dts/physic_bev/tank_inspection_ws
  source /opt/ros/jazzy/setup.bash
  source .venv_rerun/bin/activate
  source install/setup.bash

  本机Viewer烟测：

  python scripts/rerun_viewer_smoke_test.py --output-mode spawn

  实时ROS话题显示：

  ros2 launch inspection_rerun rerun_live.launch.py output_mode:=spawn

  MCAP回放显示：

  ros2 launch inspection_rerun rerun_replay.launch.py \
    bag_path:=/home/dts/physic_bev/tank_inspection_ws/bags/physic_bev_0.mcap \
    output_mode:=spawn \
    play_duration:=10.0

