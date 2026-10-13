# Odin1 双 PC 无线验证环境

两端 Docker 使用原生 ROS 1 Noetic；主机的 ROS 2 环境不参与此链路。本机为 x86_64，另一端是 ARM64 Jetson AGX Thor，两端分别编译自己的镜像。现有 Thor `agps` 容器保持运行。

| 角色 | 本机 | Thor |
|---|---|---|
| LAN 地址 | 192.168.5.21 | 192.168.5.9 |
| 机器人名字 / ID | odin_a / 0 | odin_b / 1 |
| Odin USB 序列号 | 276fe96f99321c78 | 952033c87532e832 |
| 镜像 | kimera-odin1:wireless | kimera-odin1:wireless-arm64 |
| 代码路径 | /home/uros/kimeramulti | /home/agpc/mrjq/kimera/kimera_multi |

本机运行 ROS master。每台 PC 就地处理自己的 RGB、原始点云、IMU、SDK 位姿，生成关键帧、词袋和视觉几何；两端各有分布式回环与 DPGO 节点。ROS master 负责注册和发现，节点的 TCP 连接直接跨 Wi-Fi。算法没有把所有计算集中到 master 上。

当前方式依赖一个共用 master 和互通的局域网。允许两端 TCP 动态端口，关闭路由器 AP 客户端隔离。此模式适合实验室在线验证；独立 master、断网恢复和通信预算调度需要另外接入 RTM。

## 构建

从 Odin 仓库根目录进入 `kimera_multi`。首次构建需要下载固定提交的依赖和词典；`docker/odin1.repos`、`docker/wireless.repos` 记录提交。

```bash
python3 docker/prepare_odin1.py
DOCKER_BUILDKIT=0 docker build --network=host --build-arg BUILD_JOBS=2 \
  -f docker/Dockerfile.odin1 -t kimera-odin1:noetic artifacts/docker/context
python3 docker/prepare_wireless.py --odin-source ../odin_ros_driver-main
DOCKER_BUILDKIT=0 docker build --network=host --build-arg BUILD_JOBS=2 \
  --build-arg BASE_IMAGE=kimera-odin1:noetic \
  -f docker/Dockerfile.odin1-wireless -t kimera-odin1:wireless \
  artifacts/docker/wireless-context
```

Thor 可以设 `BUILD_JOBS=8`，基础镜像命名 `kimera-odin1:noetic-arm64`，扩展镜像设 `BASE_IMAGE=kimera-odin1:noetic-arm64` 并命名 `kimera-odin1:wireless-arm64`。不使用 x86 镜像仿真。

`prepare_wireless.py` 只复制 SDK 驱动代码、静态库和配置，不覆盖原仓库。构建时将其 ROS 2 manifest 转为 ROS 1，补齐私有配置和帧前缀参数，保留 SDK 实现。DPGO/ROPTLIB/JSON 使用固定源码，禁用跨架构 `-march=native` 和 CMake 临时下载。

## 启动

第一次分别生成配置；当前两设备标定来自已有实测归档。如果更换设备，必须重新导出它自己的标定并生成参数。

```bash
python3 docker/configure_wireless.py odin_a ../odin_ros_driver-main  # 本机
python3 docker/configure_wireless.py odin_b ../odin_ros_driver-main  # Thor
```

本机三个终端中的前两个：

```bash
export ROS_IP=192.168.5.21
export ROS_MASTER_URI=http://192.168.5.21:11311
docker/wireless_odin1.bash master
```

```bash
export ROS_IP=192.168.5.21
export ROS_MASTER_URI=http://192.168.5.21:11311
export ODIN_TARGET_USB_SERIAL=276fe96f99321c78
docker/wireless_odin1.bash robot odin_a 0
```

Thor：

```bash
cd /home/agpc/mrjq/kimera/kimera_multi
export ROS_IP=192.168.5.9
export ROS_MASTER_URI=http://192.168.5.21:11311
export ODIN_TARGET_USB_SERIAL=952033c87532e832
export ODIN_WIRELESS_IMAGE=kimera-odin1:wireless-arm64
docker/wireless_odin1.bash robot odin_b 1
```

启动时根据序列号重新解析 USB bus/address，拒绝找不到、重复和 USB2 设备。驱动使用本地设备时间；RGB、点云与 SDK 位姿在同一设备时间域做配对及去畸变。两台设备的独立 SDK 原点不表示它们已经处于同一个世界坐标系。只有经视觉几何验证的跨机回环和后端优化，才能确定两图相对位置；没有跨机约束时不展示“融合成功”。

## 验证和停止

`artifacts/wireless/network_probe.json` 记录 ROS 双向 64 KiB 数据传输测试。`docker/observe_wireless.py` 必须在对应机器人所在 PC 的容器内运行，避免将原始 RGB-D 跨无线传输。它记录输入频率、RGB-D 队列、实际词袋和关键帧、SDK 位姿逐项不变检查。

```bash
docker/wireless_odin1.bash shell
python3 /workspace/docker/observe_wireless.py --robot odin_a --seconds 40 \
  --output /workspace/artifacts/wireless/odin_a/live_validation.json
rosnode list
rostopic list
```

每端本地数据主题 `/odin_{a,b}/odin1/*` 和 `/odin_{a,b}/kimera/*`；SDK 图与视觉关键帧在 `/odin_{a,b}/kimera_vio_ros/*`；回环在 `/odin_{a,b}/kimera_distributed/*`；优化轨迹和图在 `/odin_{a,b}/dpgo_ros_node/*`。

在对应机器运行 `docker stop --time 20 odin-wireless-odin_a` / `odin-wireless-odin_b`，最后本机运行 `docker stop odin-wireless-master`。采集不自动加入系统启动。重新开始一次运动实验时，两端都重启算法，避免把新 SDK 原点拼接到旧图里。

SDK 前端使用整幅 RGB 的 ORB 特征做词袋检索，视觉几何消息仍只包含通过实测深度和置信度筛选的特征。早期前端把词袋也限定在稀疏深度点覆盖区域，重复场景的检索会受深度采样变化影响。`bow_use_all_rgb_features=false` 可用于与早期版本比较。

SDK 默认静止关键帧间隔为 2 秒，原后端每 100 个关键帧生成一个子图会导致短实验长时间只有一个子图。双机启动文件将 `max_submap_size` 设为 5，并保留启动参数以便之后按场景调整。分布式后端需要足够的跨机回环才能初始化第二台设备的全局坐标；仅有 Wi-Fi 通信、两个局部位姿图和相同房间，不代表融合已经完成。

已录制的原始传感器输入可以跨 PC 重放：`start_wireless_robot.py ... --replay` 关闭硬件驱动、启动 RGB-D 适配器；`replay_wireless_raw.py --robot odin_a --run <录包目录名>` 仅播放压缩图像、原始点云、IMU 和高频 SDK 位姿，不播放录包内的旧算法输出。两台均保留本地录包，通过局域网重新计算并交换算法消息。
