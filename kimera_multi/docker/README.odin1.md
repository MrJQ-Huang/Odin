# Odin1 的 Noetic / Kimera Docker 环境

宿主机保持 ROS 2 Humble，负责 Odin1 驱动、适配和 ROS 2 MCAP → ROS 1 bag 导出；
容器运行 ROS 1 Noetic。默认主路径使用 Odin SDK 本地轨迹，生成 Kimera-Multi
所需的相机位姿图、BoW、VLC frame 消息和关键帧查询接口，不运行 Kimera-VIO。
独立 Kimera RGB-D+IMU VIO / 单机回环保留为 `--frontend vio` 对照模式。
此镜像已验证单台及双台 Odin1 的独立前端接入；
分布式多机器人后端、语义建图和跨机通信仍需另外集成验证。

从仓库根目录执行：

```bash
docker/odin1.bash build
docker/odin1.bash check
docker/odin1.bash replay
```

默认回放已经适配的 `artifacts/odin1/motion_rgbd.bag`，参数为
`artifacts/odin1/params`，默认使用 SDK 前端，以 0.5 倍速回放，可指定 `--rate 1`。
SDK 默认输出目录为 `artifacts/odin1/docker_sdk`，VIO 为 `artifacts/odin1/docker_motion`。
输出包含节点日志、
`result.bag` 和 `summary.json`；脚本要求轨迹、位姿图、BoW、VLC 均有输出才报告成功。
SDK 模式还检查每条输出位姿和时间戳均与 SDK 输入一致、相机图外参变换正确、
描述子/bearing/depth 对齐，以及位姿图和关键帧查询服务一致。
回放脚本会在开始时保存执行源码快照及 SHA256。VIO 模式另外输出 VIO CSV。
输出目录必须不存在，避免覆盖之前的实验。

以下是改造前独立 VIO 对照路径的结果：本机已完成原生 Noetic 全消息读回、
运动录包和静态录包的 Kimera VIO/LCD 验证。
运动半速回放输出 154 个位姿、154 个 BoW 查询、153 个有效 VLC 关键帧；
运动 1 倍速回放输出数量相同，约 10.26 Hz 的输入在本机通过验证。
静态正常速度回放输出 107 个位姿，相对初始位置的最大变化约 1.62 mm。
这些证明输入与运行链路可用，不构成精度基准；默认外参和 IMU 噪声仍需独立标定。
完整证据在 `artifacts/odin1/validation.json` 及各次回放的 `summary.json`。
已用过的输出目录保持原样，再次运行请指定一个新目录，例如：

```bash
docker/odin1.bash replay --frontend sdk --rate 1 --output /workspace/artifacts/odin1/docker_sdk_new
```

静态录包或新数据可以显式指定：

```bash
docker/odin1.bash replay \
  --bag /workspace/artifacts/odin1/qos_rgbd.bag \
  --params /workspace/artifacts/odin1/params \
  --frontend sdk --output /workspace/artifacts/odin1/docker_sdk_static --rate 1
docker/odin1.bash shell
```

路径参数使用**容器路径**：仓库挂载为 `/workspace`。
容器使用当前用户的 UID/GID，实验文件归本机用户所有。
回放在容器内部启动独立 ROS master，不占用宿主机的 11311 端口；不需要 GPU、显示器或 USB 权限。
实时 ROS 2 → ROS 1 桥接尚未包含在此镜像中。

## 同一主机上的双 Odin1 验证

两台分别生成标定和适配配置，输入话题指向各自的驱动命名空间，
输出 `robot_name` 分别为 `odin_a`、`odin_b`。单机回放支持
`--robot-name` 和 `--robot-id`；回放脚本会核对位姿图、BoW、VLC 中的机器人编号。
`odin1.bash replay` 使用仓库里当前的回放脚本，并在结果中记录其 SHA256。

同时启动两套独立 ROS master 和 SDK 前端：

```bash
python3 docker/replay_dual_odin1.py \
  --frontend sdk \
  --bag-a artifacts/dual_odin1_20261013/odin_a/motion_usb128_rgbd.bag \
  --params-a artifacts/dual_odin1_20261013/odin_a/params \
  --bag-b artifacts/dual_odin1_20261013/odin_b/motion_usb128_rgbd.bag \
  --params-b artifacts/dual_odin1_20261013/odin_b/params \
  --rate 1 --output artifacts/dual_odin1_20261013/replay_new
```

输出目录必须不存在；全部路径必须在仓库内。总报告保留两个实例的启动时间、
退出状态和子报告。子报告记录请求回放倍速及 `/clock` 中间 80% 区间的实际倍速，
避免把运行结束误当作跟上正常速度。此入口不启动跨机器人回环或分布式优化。
两个容器拥有独立时钟；把它们接入共同通信环境之前，仍需处理设备时钟映射。
SDK 前端输出话题仍在 `/robot_name/kimera_vio_ros`，以匹配上游固定接口，
其中里程计是 SDK IMU 位姿，图节点是使用标定外参得到的相机位姿。
SDK 默认只对运动阈值或最大时间间隔选出的关键帧做 ORB / BoW，
使用有界图像队列和关键帧查询缓存，减少重复定位和静止场景处理。
SDK 功能回归测试入口为 `docker/odin1.bash test-sdk`；
VIO 对照回放显式传 `--frontend vio`。两种模式都不启动 DPGO 或跨机回环。

### SDK 前端改造验证（2026-10-13）

使用上述分别移动的约 103 秒录包，以 1 倍速同时回放两路，结果如下：

| 检查 | Odin A | Odin B |
|---|---:|---:|
| SDK 输入 / 原样转发位姿 | 1054 / 1054 | 1055 / 1055 |
| 相机图节点 / 里程计边 | 104 / 103 | 87 / 86 |
| 有效 VLC 关键帧 | 102 | 87 |
| BoW 查询条数 | 102 | 174 |
| RGB-D 总帧 / 选中关键帧 | 1054 / 104 | 1054 / 87 |

两路位置和四元数分量改变量均为 **0**，时间戳逐条一致；
相机图与 SDK 位姿及外参变换一致，图边与相邻相机节点的相对变换一致，
描述子 / bearing / 深度行数及数值有效性通过检查，首 / 中 / 尾关键帧查询成功。
A 有两个关键帧深度特征不足，只输出图节点，不伪造 VLC。
B 的 BoW 数量是视觉帧的两倍，因为按上游协议分别发往机器人 0 和 1。
没有 RGB-D 配对丢失或队列溢出，所有回放进程退出码为 0。

`/clock` 中间 80% 区间的实际倍速均约 1.000；播放器墙钟耗时分别为
106.72 / 109.52 秒，包含启动和磁盘读取，这不是完整实机在线延迟保证。
程序仅对约 9.9% / 8.3% 的 RGB-D 帧提取关键帧特征，并保留全部 SDK 位姿。
原生前端的链接检查未包含 Kimera-VIO 库，避免重复运行本地定位。

报告在 `artifacts/dual_odin1_20261013/docker_sdk_rigid_pair/summary.json`，
各子目录保存执行脚本快照、输入 / 参数校验和、实际镜像 ID、日志和结果 bag。
几何测试覆盖相机杆臂、相对运动、插值、舍入标定旋转校正及非法变换拒绝。
首轮严格校验发现原厂标定旋转的非正交舍入误差，修正为 SO(3) 后重新编译并通过；
未放宽位姿一致性检查。跨机器人回环、DPGO 和 ROS 2 → ROS 1 实时桥接未在此回放中启动。
同一镜像的独立 VIO 对照回归也通过：约 30 秒静态录包输出 103 个位姿、
103 个图节点和 102 个有效 VLC 帧，所有进程正常退出。
汇总、源码校验和及范围限制见
`artifacts/dual_odin1_20261013/sdk_frontend_validation.json`。

2026-10-13 本机验证：两台 Odin1 共用 USB 3 Hub，均以 5 Gbps 连接。
默认 16 MB USB 缓冲池出现 `LIBUSB_ERROR_NO_MEM` 和停流；临时调至 128 MB 后，
连续 6 分钟双机观察未复现该错误，图像和点云均约 10.26 Hz，
IMU 分别约 400.8 / 398.2 Hz，时间戳严格递增。
宿主机 USB 设置（不是容器设置）：

```bash
cat /sys/module/usbcore/parameters/usbfs_memory_mb
sudo sh -c 'printf 128 > /sys/module/usbcore/parameters/usbfs_memory_mb'
```

本次只改运行时数值，重启后需重新设置。长时间稳定性仍需扩大测试。
运动录包的静止首段（双路各 60 秒、615 帧）同时回放，均输出 205 个位姿，
实际回放倍速均约 1.000，图/BoW/VLC 编号分别为 0 / 1，全部进程正常退出。
两套 640×518 在线适配器同时运行时出现队列积压和丢帧；当前推荐同时采集原始数据，
再离线转换和运行双前端。完整证据见 `artifacts/dual_odin1_20261013/validation.json`。

分别移动两台设备的复测中，完整原始录制约 320 秒，未出现 USB 停流。
运动片段导出约 103 秒，每路 1054 对 RGB-D；B 有一帧因位姿插值跨越
15 ms 数据间隙而跳过，保留了原来的覆盖检查和传感器时间戳。
改造前的独立 VIO 路径修复空特征崩溃后，两路同时以请求倍速 1 回放，通过完整运动片段验证：
A / B 分别输出 349 / 350 个位姿，位姿图节点数同为 349 / 350，
BoW 查询 349 / 700 条，VLC 关键帧 349 / 350 个，机器人编号严格为 0 / 1，
全部回放进程正常退出。轨迹覆盖约 102.54 / 102.44 秒，
`/clock` 中间 80% 区间的倍速约 1.000；包含启动延迟和磁盘读取的播放器
墙钟耗时约 128.38 / 127.87 秒，因此该结果不构成实时延迟保证。
结果在 `artifacts/dual_odin1_20261013/docker_motion_usb128_fixed_pair/`；
设备自身里程计仅用于参考和深度去畸变，不是轨迹精度的真值。
厂商 SDK 在本次停止驱动时，B 出现 `malloc_consolidate(): invalid chunk size`；
原始录包在停止驱动前已正常落盘，此 SDK 退出问题仍待处理。

真实运动还触发了上游 Kimera 的空特征点问题：OpenCV 4.2 的
`undistortPoints` 对空点集抛异常。构建脚本现在补丁处理空去畸变输入和
无有效参考轨迹的光流输入，让前端进入原有的新特征检测流程。
可以在已构建的镜像中运行实际 C++ 库的回归测试：

```bash
docker/odin1.bash test-sparse \
  /workspace/artifacts/dual_odin1_20261013/odin_a/params/LeftCameraParams.yaml
```

测试覆盖空输入清理、正常投影计算、空参考帧和全部无效参考特征。

### 点云累积与离线配准预览

本次运动录包已另行生成几何预览，目录为
`artifacts/dual_odin1_20261013/cloud_fusion_preview/`。
打开 `viewer.html` 可以旋转、缩放、切换 SDK / VIO 轨迹累积图、隐藏任意设备，
或查看水平切片。无需网络资源；`fusion_comparison.png` / `.pdf` 是静态对比图，
`fused_vio_candidate.ply` 和 `fused_sdk_candidate.ply` 可用点云查看器打开。
蓝色和粉色标识 A / B 设备来源，未使用相机颜色。

每台选取 210 帧原始 DTOF 扫描，置信度 ≥35、距离 0.3–8 m；用原有高频
SDK 位姿按点时间去畸变，保留 10 ms 插值间隙检查。VIO 版通过约 3.4 Hz
输出位姿插值累积，允许最大 500 ms 间隔，并用起始时刻的一个固定变换
转到该设备的 SDK 局部坐标系。采用 5 cm 体素，剔除只有一次点测量的体素。
两路分别累积后，对每种轨迹来源独立执行多起点、粗到细的几何 ICP，
仅估计一个跨机刚体变换，不修改单机轨迹，不加入回环或分布式优化。

以 10 cm 邻近阈值评价这次候选配准，SDK 版 B→A / A→B 匹配比例约
89.4% / 60.7%，VIO 版约 68.6% / 50.4%；匹配点 B→A 距离 RMSE
分别约 4.0 / 5.0 cm。这是输入图上的拟合诊断，不能解释为地图真值精度。
水平切片中 VIO 版可见墙面重影；该预览不构成 Kimera-Multi 跨机融合验证。
详细参数、候选变换和指标保存在 `accumulation.json` / `registration.json`。
实验脚本在 `experiments/dual_odin1_20261013/cloud_fusion_preview.py` 和
`render_cloud_preview.py`，使用本机 `.venv-odin` 中的 rosbags、SciPy、Matplotlib。

构建时 `prepare_odin1.py` 按 `odin1.repos` 的确切 SHA 缓存源代码，校验词袋文件 SHA256，
再生成只包含源代码和词袋的构建上下文。原始录包、实验数据、虚拟环境和 Git 历史不会发送给 Docker。
它会拒绝覆盖手动修改过的依赖缓存。缓存和日志均在 `artifacts/docker/`。
基础镜像固定为 Noetic/Focal 的 digest；Ubuntu/ROS 的二进制软件包仍由 apt 获取。
初次编译和下载较慢，后续利用 Docker 缓存；修改本地 launch/params 可以直接回放，
修改 C++ 或依赖则重新构建。

若在最后一个包构建失败，可保留失败容器的编译结果继续构建：

```bash
docker commit <failed-build-container-id> kimera-odin1:compiled-cache
ODIN1_BASE_IMAGE=kimera-odin1:compiled-cache docker/odin1.bash build
```

这用于本机编译恢复；普通初次构建仍以文件中固定的 Noetic 基础镜像开始。

回放先用录包起始时间初始化 `/clock`，避免上游等待有效时间与脚本等待订阅者
相互阻塞。结束后短暂推进 `/clock`，使定时发布器完成收尾；传感器消息时间戳不改写。
导出 bag 默认无压缩，可选 BZ2；不要使用已保留的 `*.rosbags_lz4.bag`，
其 LZ4 流被此 Noetic 原生播放器拒绝。

为避免 ROS Noetic 的 `cv_bridge` 与 OpenCV 3 同进程混用，这里全部链接 Focal 的系统
OpenCV 4.2；`noetic_system_packages` 对原依赖名 `opencv3_catkin`、`gflags_catkin`
提供系统库导出，`opengv_catkin` 编译固定版本的上游 OpenGV，替代已删除的下载包装。
GTSAM 固定为 4.2a9，启用 manifold IMU preintegration，禁用 march-native，保留 unstable 库。
默认单包、两条编译任务适合本机 16 GB 内存，可通过 `BUILD_JOBS=1 docker/odin1.bash build` 降低内存占用。

本机安装 Docker 后，当前登录会话可能还没有新组权限；脚本会用 `sg docker` 启动新进程，
无需立刻注销登录，也不会在脚本中保存 sudo 密码。
原有六机器人数据集镜像仍在 `Dockerfile` / `build.bash` / `run.bash`，它使用上游移动分支，
不作为 Odin1 的构建或验证入口。
