# Odin1 输入适配

主路径复用 Odin SDK 的单机定位：SDK 位姿直接转发，图像和测量支持的深度
生成关键帧、ORB / BoW / VLC，供 Kimera-Multi 的分布式模块使用。
新前端不运行 Kimera-VIO，不把 SDK 轨迹当作额外定位约束重新估计。
原来的 RGB-D+IMU 独立 VIO 路径保留为对照。共享转换代码用于
ROS 2 实时节点和 ROS 2 MCAP → ROS 1 bag 导出。

数据路径：

```text
Odin compressed RGB ── FishPoly 去畸变 ──────────── RGB 640×518
Odin raw cloud ── confidence / range / 逐点去畸变 ─ optical-Z 深度 640×518
Odin 400 Hz IMU ── 保留传感器时间戳与原始测量 ──── IMU
                                                    ↓
Odin SDK odometry ── 原始时间戳和位姿 ─────────────── SDK 本地轨迹
                                                    ↓
                  ROS 1 bag 回放 / ros1_bridge → SDK frontend → Kimera-Multi 接口
```

## 安装

ROS 1 / Kimera 环境使用仓库内的 [Docker 入口](../../docker/README.odin1.md)：
从仓库根目录执行 `docker/odin1.bash build`、`docker/odin1.bash check`、
`docker/odin1.bash replay`。宿主机只负责 ROS 2 驱动和适配。

ROS 2 实时节点需要本机 ROS 2 Humble；离线导出不需要 ROS 1 或 ROS 2 运行环境。
OpenCV 使用 Ubuntu 的 `python3-opencv`，NumPy 固定在 2 以下，避免本机 OpenCV 的 ABI 冲突。
从仓库根目录执行：

```bash
python3 -m venv --system-site-packages .venv-odin
PYTHONNOUSERSITE=1 .venv-odin/bin/python -m pip install -e 'adapters/odin1[bag]'
```

新环境需要 `python3-venv` 和 `python3-opencv`。当前机器的 `.venv-odin` 已准备好。
设置 `PYTHONNOUSERSITE=1` 防止加载用户目录中的 NumPy 2；不要升级系统 NumPy。
离线 bag 读写使用 [rosbags 的 ROS 1/2 API](https://ternaris.gitlab.io/rosbags/topics/rosbag1.html)。
ROS 1 导出默认无压缩以保证原生 Noetic 播放速度，节省空间时可传 `--compression bz2`。
不使用 rosbags 的 LZ4 写出：它的自读回测试通过，但本机 Noetic 原生播放器拒绝该流。

## 为设备生成参数

使用该台设备驱动下载的 `calib.yaml`，每台设备分别生成：

```bash
PYTHONNOUSERSITE=1 .venv-odin/bin/odin1-prepare \
  --calib experiments/odin1_20261012/runtime/calib.yaml \
  --output artifacts/odin1
```

输出包含 `adapter.yaml`、原厂相机标定、完整 Kimera 参数目录和标定来源记录。
工具拒绝覆盖已有目录。修改设备名、输入话题、分辨率、外参或深度设置时，
先复制 `config/adapter.yaml`，修改后通过 `--config` 传入，再生成一个新目录。
生成后的实时节点和 bag 导出必须使用同一个 `adapter.yaml`。

所有矩阵遵循 `p_A = T_A_B p_B`：

- `T_camera_lidar` 来自该台设备的 `Tcl_0`。
- 默认 `T_imu_lidar` 使用设备文档第 5.2 节的固定外参，显式写在配置中。
- Kimera 相机 `T_BS = T_imu_camera = T_imu_lidar * inverse(T_camera_lidar)`，body 为 IMU。
- 本地 Odin 驱动不同模块对 WIWC 和 TF 的方向存在不一致；适配器不会自动采纳这些矩阵。
  有独立标定结果时，应显式覆盖 `T_imu_lidar`。默认外参仍需实机标定验证。

IMU 噪声参数继承现有 D455 参数作为调试初值，**不是 Odin 的 Allan 标定结果**。
修改噪声、时间偏移和重力参数应以独立测量为依据。

## 导出 ROS 1 回放包

```bash
PYTHONNOUSERSITE=1 .venv-odin/bin/odin1-export-bag \
  --input experiments/odin1_20261012/qos_validation_bag \
  --config artifacts/odin1/adapter.yaml \
  --output artifacts/odin1/qos_rgbd.bag
```

输出 `.bag`、转换统计 `.json` 和 RGB/深度 `.preview.png`。
`--start` 和 `--duration` 的单位是秒，起点以首张 RGB 的传感器时间戳为基准。
工具保留原始传感器时钟，将 bag 的事件时间也设为该时钟，供 `rosbag play --clock` 使用。
RGB、深度和 CameraInfo 使用同一个 RGB 时间戳及相机 optical frame。

转换先检查 IMU 的长缺口和传感器时钟倒退。IMU 缺口超过 20 ms 时默认拒绝导出；
`--allow-imu-gaps` 仅用于诊断，输出不应直接作为有效的 VIO 数据。
逐点补偿禁止外推，禁止跨越超过配置阈值的位姿缺口；无法补偿的帧计入 JSON。
首尾位姿覆盖不足也可能丢弃边界帧，应检查转换统计。

### SDK 主路径

默认 Docker 回放使用 `--frontend sdk`。在已编译本仓库的 ROS 1 Noetic 工作区中：

```bash
roslaunch kimera_multi kimera_sdk_odin1.launch \
  params_folder:=/absolute/path/to/artifacts/odin1/params robot_name:=odin1 robot_id:=0
rosbag play --clock /absolute/path/to/artifacts/odin1/qos_rgbd.bag
```

SDK 前端直接转发 `/reference/odometry`，不运行 Kimera-VIO，不修改 SDK 位姿或时间戳。
只在选中的关键帧提取 ORB 特征，使用测量掩码和有效深度过滤特征，
用与 Kimera 相同的固定 DBoW2 词典生成 BoW，并发布 LCD 所需的 VLC 数据。
默认关键帧条件为至少相隔 0.1 秒，且平移 ≥0.1 m、旋转 ≥10° 或间隔 ≥2 秒；
没有足够深度特征时仍保留 SDK 位姿图节点，但不伪造视觉关键帧。
图像/深度/掩码严格按时间戳配对，SDK 位姿禁止外推；不同流迟到会等待有界队列，
SDK 时钟倒退或位姿坐标框架变化会终止该图运行，需要开启新段。

输出保留 `/robot_name/kimera_vio_ros` 命名空间，以兼容上游分布式模块：

| 输出后缀 | 含义 |
|---|---|
| `/odometry` | SDK 原始 IMU 位姿，仅改输出 frame 名称 |
| `/keyframe_odometry` | `T_world_camera = T_world_imu * T_imu_camera` |
| `/pose_graph`、`/pose_graph_incremental` | 相机位姿图，SDK 相邻关键帧相对运动 |
| `/bow_query`、`/vlc_frames` | 与图节点编号一致的词袋、描述子、相机 bearing 和深度 |
| `/vlc_frame_query`、`/pose_graph_query` | 按机器人编号和关键帧编号查询缓存数据的服务 |
| `/sdk_diagnostics` | SDK 转发、关键帧、无深度特征、插值和有界队列计数 |

位姿图使用相机位姿，使节点坐标与 VLC 的视觉几何一致；不得再对这条图重复
应用相机外参。VLC 的 `submap_id = pose_id`、`T_submap_pose = identity` 表示初始
每个相机关键帧自成子图；上游分布式模块可随后重新划分子图。
相机图优化后转回 IMU 位姿应使用 `T_world_imu = T_world_camera * inverse(T_imu_camera)`。
对通过刚体校验但含舍入误差的标定旋转，SDK 节点一次性投影到 SO(3)，
保留平移和原始标定文件，输出数值校正量；这不是重新进行外参标定。
图边协方差来自显式初始模型：平移标准差 0.1 m、旋转标准差 0.1 rad，
不是 SDK 提供的已标定误差模型。关键帧查询缓存默认最多 2000 帧，
被淘汰帧的请求返回失败，不能将这份内存缓存当作永久归档。

这个节点提供本地前端接口，不启动跨机回环、DPGO、语义网格或地图校正。
实时输入可将 `sdk_odom_topic` 指向桥接后的 SDK 高频位姿，并关闭模拟时钟；
默认最大位姿插值间隔 0.12 秒用于兼容录包的普通位姿流，精确匹配时间戳无需插值。
逐点点云去畸变仍使用适配器原来的高频位姿和 10 ms 间隙检查。

### 独立 VIO 对照路径

在已安装 Kimera-VIO / Kimera-VIO-ROS 的 ROS 1 Noetic 工作区中：

```bash
roslaunch kimera_multi kimera_vio_odin1.launch \
  params_folder:=/absolute/path/to/artifacts/odin1/params robot_name:=odin1 robot_id:=0
# 另一个终端；先启动 VIO，再回放。
rosbag play --clock /absolute/path/to/artifacts/odin1/qos_rgbd.bag
```

启动文件选择 RGB-D frontend，开启 BoW / VLC 发布、关闭 GUI，
保留深度有效性检查；Odin 位姿仅导出为 reference，不作为外部 VIO 因子。
多设备分别设置 `robot_name` / `robot_id` 和设备标定目录。
此文件启动单机 VIO 输入链；分布式优化、通信和语义网格仍使用上游配置，需另行联调。

## ROS 2 实时模式

先用 Odin 驱动启动 RGB compressed、raw DTOF、IMU、普通及高频位姿；
无需启动原厂深度补全节点，也不需要原厂去畸变图。
驱动必须使用设备统一时间戳模式，不能分别给流写接收时间。

```bash
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 .venv-odin/bin/odin1-adapter --config artifacts/odin1/adapter.yaml
```

默认输出前缀 `/odin1/kimera`：

| 后缀 | 类型 / 含义 |
|---|---|
| `/rgb/image_raw` | `Image`, bgr8，去畸变 RGB |
| `/depth/image_raw` | `Image`, 32FC1，单位米，光轴 Z，未知值为 0 |
| `/rgb/camera_info` | `CameraInfo`，零畸变 pinhole，与 RGB/深度共用内参 |
| `/depth/measured_mask` | `Image`, mono8，直接投影有测量支持的像素，非补全覆盖率 |
| `/imu` | `Imu`，原始加速度/角速度及时间戳 |
| `/reference/odometry` | `Odometry`，SDK 前端的轨迹源；VIO 模式的对照位姿，不是真值 |
| `/diagnostics` | `String` JSON，配对、处理和丢帧计数 |

实时连接 ROS 1 Kimera 时，需要在兼容的 ROS 1/2 桥接环境中桥接标准消息话题；
宿主机保持 ROS 2，ROS 1 已在 Docker 中配置；完整实时桥接尚未完成。使用实时模式时将前端的
`should_use_sim_time:=false`，回放模式用 `true`。
发生设备时钟重置时，应重新启动适配器和估计器，分开录包。
实时入口默认把 BLAS / OpenMP 和 OpenCV 线程数限制为 1，给高频回调留出算力；
有更多算力且完成复测后，可通过 `ODIN1_BLAS_THREADS` 调整数学库线程数。

## 深度与录制约束

点云按 confidence ≥35 过滤，在目标相机中取最近光轴深度；
默认每个测量最多覆盖 3×3 像素，并剔除深度跳变边界，不做全图补洞。
这些覆盖像素不是独立测量，`splat_radius: 0` 可关闭小范围扩展。
`measured_mask` 标记直接投影落点，可能包含后来被边界过滤去除的深度；
要判断实际可用深度，还须检查 `depth > 0`。

原始点云逐点 `offset_time` 单位为秒，实测跨度约 94 ms。
默认使用高频 Odin 位姿的平移插值和 quaternion SLERP，把每个点变换到 RGB 时间。
这会使深度补偿依赖 Odin 位姿质量，应记录该处理条件。
`deskew: none` 可用于静态排查，但转动时会有扫描畸变。

录制原始流时，对 `/odin1/imu` 和 `/odin1/odometry_highfreq` 使用至少 4000 的可靠队列。
可用本次实验中的 `experiments/odin1_20261012/record_odin.sh`。
回放原始 ROS 2 包时，发布端也应增大高频队列：

```bash
ros2 bag play /path/to/raw_bag --clock \
  --qos-profile-overrides-path adapters/odin1/config/rosbag2_qos.yaml
```

该 QoS 文件匹配默认设备话题；改变输入命名空间时需相应修改键名。
原始 `/odin1/wiwc` 的 covariance 携带外参；不要当作位姿不确定度。
导出的 reference covariance 清零表示未知，并未将其当作真实噪声模型。

## 验证

```bash
PYTHONNOUSERSITE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  .venv-odin/bin/python -m pytest -q adapters/odin1/tests
```

测试包括遮挡 Z-buffer、无效点过滤、FishPoly 光轴映射、逐点时间补偿、
旋转插值、位姿缺口、PointCloud2 非对齐字段/大小端/行填充、
Kimera OpenCV YAML 和 ROS 2 → ROS 1 的序列化往返。
`tools/observe_ros2.py` 可独立订阅实时节点，核对输出频率、帧名及 RGB-D 时间戳。

当前验证覆盖传感器输入适配；这不等于已经验证 VIO 精度、跨机器人回环或语义网格。

本机录包验证结果：12 项测试通过；静态片段导出 322 对 RGB-D，
运动录包的连续 45 秒片段导出 461 对 RGB-D，并完成 ROS 1 全消息读回。
约 31.5 秒的原始 ROS 2 录包以原速回放时，实时节点输出全部 322 对可配对 RGB-D
（10.26 Hz）和全部 12,637 条 IMU（400.8 Hz），无处理队列溢出或时间戳倒退。
这是录包回放验证，未运行 Kimera VIO，也不代表精度或长时间实机稳定性验证。

双 PC 原生 ROS 1 在线路径使用 `odin1_ros1_adapter`，共享本适配器的标定、投影和去畸变代码。驱动在每台机器本地接入 USB，ROS master 用于局域网注册，分布式回环与 DPGO 在两端分别运行。具体环境、启动和原始数据回放见 [双 PC 无线验证](../../docker/README.wireless.md)。词袋使用完整 RGB 特征，VLC 3D 特征仍要求实测深度。
