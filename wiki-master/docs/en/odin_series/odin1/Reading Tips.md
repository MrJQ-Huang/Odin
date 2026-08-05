---
layout: default
title: Reading Tips
nav_exclude: true
search_exclude: true
---

# Reading Tips

This documentation is hands-on by design — we recommend reading it while you work through the steps on real hardware. Below is a quick guide to how to read the manual and where to find supporting resources, so you can build a complete picture as fast as possible.

## Symbol Conventions

- ⚠️ **Important note** — content that affects safety, stability, or commonly causes pitfalls
- 💡 **Tip** — recommended practices, troubleshooting hints, and small tricks to save time
- 📖 **Glossary** — quick explanation of key concepts, acronyms, or terms

## Documentation Index & Resources

### Developer Support

* Odin ROS driver
  - Odin ROS Driver: [Odin\_ros\_driver](https://github.com/manifoldsdk/odin_ros_driver)
* Odin open-source navigation stack
  - Odin Navigation Stack: [Odin-Nav-Stack](https://github.com/ManifoldTechLtd/Odin-Nav-Stack)

## General Troubleshooting Workflow (worth bookmarking)

- **First, check logs and topics**
  Confirm the node has launched correctly, key topics are publishing data, and the driver terminal shows no error messages.
- **Then, check hardware logs**
  - In `/src/odin_ros_driver/config/control_command.yaml`, set `devstatussog: 1` and `save_log: 1`.
  - After running the driver, inspect the CSV files inside `/src/odin_ros_driver/log/Driver_**` for any abnormal data.
- **Reporting an issue**
  - When you encounter scenarios where the device does not work as expected, please record a bag containing the three raw streams `/odin1/cloud_raw`, `/odin1/imu` and `/odin1/image/compressed`, together with `calib.yaml`, to help us analyze the problem.
  - Please also include detailed environment information: OS version, ROS version, driver version, power-supply details, etc.
  - Attach the `/src/odin_ros_driver/log/Driver_**` folder.

---

[← Back to Odin1 (English)](./)
