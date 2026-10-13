# 在其他 PC 同步 Odin1 / Kimera 工作区

本次快照位于 `MrJQ-Huang/Odin` 仓库的 `kimera-odin1-sdk-20261013` 分支，
Kimera 项目在 `kimera_multi/` 子目录。仓库根目录原有 Odin 驱动、ROS 2
工作区和 Web UI 保留，Kimera 在独立 Docker 中运行 ROS 1 Noetic。

## 获取代码

新 PC：

```bash
git clone --branch kimera-odin1-sdk-20261013 --single-branch \
  https://github.com/MrJQ-Huang/Odin.git odin
cd odin/kimera_multi
```

已经克隆 `Odin` 的 PC，在没有需要保留的未提交修改时切换分支：

```bash
git fetch origin
git switch --track origin/kimera-odin1-sdk-20261013
cd kimera_multi
```

以后在该分支中执行 `git pull --ff-only` 更新。分支名不代表上传了 Docker 镜像；
其他 PC 需要本地构建，或另外传输镜像。

## 准备宿主机与 Docker

建议宿主机使用 Ubuntu 22.04 / ROS 2 Humble，与本次驱动验证环境一致。
ROS 2 安装和 Odin 驱动要求见仓库根目录及
[驱动说明](../odin_ros_driver-main/README.md)。
以下命令在 `odin/kimera_multi` 中执行：

```bash
sudo apt-get update
sudo apt-get install -y docker.io git python3-venv python3-opencv
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"

python3 -m venv --system-site-packages .venv-odin
PYTHONNOUSERSITE=1 .venv-odin/bin/python -m pip install -e 'adapters/odin1[bag,test]'

docker/odin1.bash build
docker/odin1.bash check
docker/odin1.bash test-sdk
```

Docker 初次构建会下载固定版本源码、词典和软件包，需要网络；默认两条编译任务。
Docker 脚本可用 `sg docker` 获取新组权限，无须把 sudo 密码写入脚本。
不要升级用户目录的 NumPy 到 2；`PYTHONNOUSERSITE=1` 防止污染系统 OpenCV。

详细构建、回放和故障说明见 [Docker 文档](docker/README.odin1.md)。
独立 VIO 对照模式显式传 `--frontend vio`，默认模式是 `--frontend sdk`。

## 标定、输入与回放

以下目录保留了本次两台设备的标定和参数，可查看或在同一设备上复用：

- `artifacts/dual_odin1_20261013/odin_a/`
- `artifacts/dual_odin1_20261013/odin_b/`
- `artifacts/odin1/`：此前单台测试配置。

其他 Odin1 必须使用该设备自己的标定。连接到另一台 PC 后，USB 总线和地址
可能变化，应通过 Odin 驱动 / Web UI 重新绑定设备。两台电脑各接一台时，
分别使用 `odin_a` / `odin_b` 的命名空间及机器人编号 0 / 1。
历史 `experiments/` 采集脚本可能包含原机路径或旧 USB 地址，不应直接启动。

生成新设备参数和转换原始录包的步骤见 [适配说明](adapters/odin1/README.md)。
原始 MCAP 和 ROS 1 bag 未进入 Git；复制到对应路径或显式指定新的路径后才能回放。
例如把 A 的已转换录包复制到本地后：

```bash
docker/odin1.bash replay --frontend sdk --robot-name odin_a --robot-id 0 \
  --bag /workspace/artifacts/dual_odin1_20261013/odin_a/motion_usb128_rgbd.bag \
  --params /workspace/artifacts/dual_odin1_20261013/odin_a/params \
  --rate 1 --output /workspace/artifacts/dual_odin1_20261013/replay_pc_a
```

路径为容器中的路径：`kimera_multi/` 挂载为 `/workspace`。
输出目录必须是新目录，避免覆盖本次验证记录。

## 已上传的验证内容

- [SDK 前端验证汇总](artifacts/dual_odin1_20261013/sdk_frontend_validation.json)
- [双机 SDK 回放](artifacts/dual_odin1_20261013/docker_sdk_rigid_pair/summary.json)
- [双机整体实验记录](artifacts/dual_odin1_20261013/validation.json)
- [点云对比图](artifacts/dual_odin1_20261013/cloud_fusion_preview/fusion_comparison.png)
- [点云交互预览](artifacts/dual_odin1_20261013/cloud_fusion_preview/viewer.html)

将 `viewer.html` 在本地浏览器中打开，可切换 SDK / VIO 累积地图；
对应 PLY 和 NPZ 文件也已保存。它们是离线几何配准候选，不能当作已完成的
Kimera-Multi 跨机回环和全局优化结果。

报告中的绝对路径、运行镜像 ID 和源码校验和记录的是原机实验，
保留其内容作为证据，不表示其他 PC 已安装同样环境。
`SYNC_MANIFEST.json` 记录本次复制的文件及校验和。
历史报告是已跟踪的快照；`artifacts/` 的新运行输出仍默认被 Git 忽略。

## 快照范围与当前状态

包含原 Kimera 项目、Odin1 适配代码、SDK C++ 前端、Docker 构建脚本、
测试、参数、实验脚本、标定、统计和小型样本，以及已有点云和轨迹预览。
原始 MCAP / bag / DB3 录包、虚拟环境、Docker 镜像、依赖源码缓存、
构建产物、进程 ID 和临时日志留在原机；不需要它们即可重新安装和构建代码。

已验证 SDK 本地前端、两路并发运动回放和独立 VIO 对照。
ROS 2→ROS 1 实时桥接、跨电脑通信、跨机回环、DPGO 与优化后的实时地图显示
尚未完成联调，当前分支不应被解释为完整的双机实时融合系统。
