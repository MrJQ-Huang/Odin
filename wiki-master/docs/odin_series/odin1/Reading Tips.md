---
layout: default
title: 阅读提示
parent: Odin1
nav_order: 2
---

# 阅读提示

本系列文档偏“上手实践”，建议你一边阅读一边在真实环境中操作验证。下面给出阅读方式与资源入口，帮助你更快建立完整路径。

## 符号说明

- ⚠️ 重要注意事项：影响安全、稳定性或会导致常见踩坑的内容
- 💡 操作提示：推荐做法、排障技巧、提升效率的小技巧
- 📖 术语解释：关键概念、缩写或名词的快速说明


## 文档指引与资源入口

### 开发者支持

* Odin ros驱动获取链接
  - Odin ROS Driver： [Odin\_ros\_driver](https://github.com/manifoldsdk/odin_ros_driver)
* Odin开源导航算法获取链接
  - Odin Navigation Stack： [Odin-Nav-Stack](https://github.com/ManifoldTechLtd/Odin-Nav-Stack)

## 常见排查思路（建议收藏）

- **先看日志与话题**
  先确认节点是否正常启动、关键话题是否有数据、驱动运行终端是否有错误信息。
- **再查硬件日志信息**
  - /src/odin_ros_driver/config/control_command.yaml文件中打开devstatussog: 1  && save_log: 1 ；
  - 运行驱动后查看/src/odin_ros_driver/log/Driver_**文件夹中的csv文件，查看是否有异常数据。
- **遇到问题并反馈问题**
  - 在遇到某些特殊场景无法正常工作时，建议录制/odin1/cloud_raw && /odin1/imu && /odin1/image/compressed三个原始数据的bag包及calib.yaml文件协助分析；
  - 提供详细的环境信息，包括但不限于：操作系统版本、ROS版本、驱动版本、供电信息等；
  - 提供/src/odin_ros_driver/log/Driver_**文件夹。


