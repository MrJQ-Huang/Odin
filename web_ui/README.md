# Odin1 Web UI

本机双 Odin1 调试面板。它不依赖 Flask 或 npm，只使用 Python 标准库。

启动：

```bash
cd ~/odin/web_ui
python3 server.py --host 127.0.0.1 --port 8765
```

打开：

```text
http://127.0.0.1:8765
```

功能：

- 扫描 `2207:0019` Odin1 USB 设备
- 将当前 USB bus/address 分配给 Odin A / Odin B
- 编辑 `control_command_odin_a.yaml` / `control_command_odin_b.yaml`
- 编辑 `~/.ros/odin_a/calib.yaml` / `~/.ros/odin_b/calib.yaml`
- 启动、停止、重启 `dual_odin.launch.py`
- 写入 `/tmp/odin_a_command.txt` 和 `/tmp/odin_b_command.txt`

保存文件时会在原目录生成 `.bak-YYYYMMDD-HHMMSS` 备份。
