#!/usr/bin/env python3
import json
import os
import re
import shutil
import signal
import subprocess
import time
import shlex
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse


HOME = Path.home()
ODIN_ROOT = HOME / "odin"
ROS_WS = ODIN_ROOT / "ros2_ws"
DRIVER_DIR = ODIN_ROOT / "odin_ros_driver-main"
STATIC_DIR = Path(__file__).resolve().parent / "static"
STATE_FILE = Path(__file__).resolve().parent / "state.json"
LOG_FILE = Path(__file__).resolve().parent / "odin_web_launch.log"

DEFAULT_STATE = {
    "odin_a": {
        "usb_bus": "2",
        "usb_addr": "3",
        "serial": "952033c87532e832",
        "config": str(DRIVER_DIR / "config" / "control_command_odin_a.yaml"),
        "calib_dir": str(HOME / ".ros" / "odin_a"),
        "command_file": "/tmp/odin_a_command.txt",
        "frame_prefix": "odin_a",
    },
    "odin_b": {
        "usb_bus": "2",
        "usb_addr": "4",
        "serial": "276fe96f99321c78",
        "config": str(DRIVER_DIR / "config" / "control_command_odin_b.yaml"),
        "calib_dir": str(HOME / ".ros" / "odin_b"),
        "command_file": "/tmp/odin_b_command.txt",
        "frame_prefix": "odin_b",
    },
    "use_rviz": True,
}

QUICK_FIELDS = [
    {"key": "streamctrl", "type": "bool", "label": "数据流"},
    {"key": "sendcloudslam", "type": "bool", "label": "SLAM 点云"},
    {"key": "sendcloudrender", "type": "bool", "label": "彩色点云"},
    {"key": "senddtof", "type": "bool", "label": "Raw DTOF"},
    {"key": "sendimu", "type": "bool", "label": "IMU"},
    {"key": "sendodom", "type": "bool", "label": "Odometry"},
    {"key": "sendrgb", "type": "bool", "label": "RGB"},
    {"key": "sendrgbcompressed", "type": "bool", "label": "压缩 RGB"},
    {"key": "sendrgbundistort", "type": "bool", "label": "去畸变 RGB"},
    {"key": "senddepth", "type": "bool", "label": "Depth demo"},
    {"key": "sendreprojection", "type": "bool", "label": "重投影"},
    {"key": "sendoverlay", "type": "bool", "label": "Overlay"},
    {"key": "recorddata", "type": "bool", "label": "录制 OLX"},
    {"key": "devstatuslog", "type": "bool", "label": "设备状态日志"},
    {"key": "save_log", "type": "bool", "label": "SDK 日志"},
    {"key": "showpath", "type": "bool", "label": "轨迹"},
    {"key": "showcamerapose", "type": "bool", "label": "相机位姿"},
    {"key": "pubintensitygray", "type": "bool", "label": "强度灰度图"},
    {"key": "resetalgo", "type": "bool", "label": "启动时重置算法"},
    {"key": "sendimagemask", "type": "bool", "label": "启动时发送 mask"},
    {"key": "dtof_fps", "type": "select", "label": "DTOF FPS x10", "options": ["100", "145"]},
    {"key": "cloud_raw_confidence_threshold", "type": "number", "label": "Raw 点云置信阈值"},
    {"key": "use_host_ros_time", "type": "select", "label": "时间戳模式", "options": ["0", "1", "2"]},
    {"key": "tf_extra_publish_rate", "type": "number", "label": "额外 TF Hz"},
    {"key": "custom_map_mode", "type": "select", "label": "地图模式", "options": ["0", "1", "2"]},
    {"key": "custom_init_pos", "type": "text", "label": "初始位姿"},
    {"key": "custom_init_pose_search_radius", "type": "number", "label": "重定位搜索半径"},
    {"key": "custom_init_pose_max_rot_deg", "type": "number", "label": "最大旋转角"},
    {"key": "relocalization_map_abs_path", "type": "text", "label": "重定位地图路径"},
    {"key": "mapping_result_dest_dir", "type": "text", "label": "地图保存目录"},
    {"key": "mapping_result_file_name", "type": "text", "label": "地图文件名"},
    {"key": "image_mask_abs_path", "type": "text", "label": "Mask 图片路径"},
]

launch_process = None
launch_log_handle = None


def load_state():
    if not STATE_FILE.exists():
        save_state(DEFAULT_STATE)
        return json.loads(json.dumps(DEFAULT_STATE))
    try:
        with STATE_FILE.open("r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception:
        data = {}
    state = json.loads(json.dumps(DEFAULT_STATE))
    for key, value in data.items():
        if isinstance(value, dict) and isinstance(state.get(key), dict):
            state[key].update(value)
        else:
            state[key] = value
    return state


def save_state(state):
    STATE_FILE.write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding="utf-8")


def json_response(handler, payload, status=200):
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    handler.send_response(status)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Content-Length", str(len(body)))
    handler.end_headers()
    handler.wfile.write(body)


def read_json(handler):
    size = int(handler.headers.get("Content-Length", "0"))
    if size <= 0:
        return {}
    raw = handler.rfile.read(size).decode("utf-8")
    return json.loads(raw)


def allowed_path(path):
    real = Path(path).expanduser().resolve()
    allowed_roots = [
        (DRIVER_DIR / "config").resolve(),
        (HOME / ".ros" / "odin_a").resolve(),
        (HOME / ".ros" / "odin_b").resolve(),
    ]
    return any(real == root or root in real.parents for root in allowed_roots)


def read_text_file(path):
    real = Path(path).expanduser().resolve()
    if not allowed_path(real):
        raise PermissionError(f"path not allowed: {real}")
    return real.read_text(encoding="utf-8")


def backup_and_write(path, text):
    real = Path(path).expanduser().resolve()
    if not allowed_path(real):
        raise PermissionError(f"path not allowed: {real}")
    real.parent.mkdir(parents=True, exist_ok=True)
    if real.exists():
        stamp = time.strftime("%Y%m%d-%H%M%S")
        shutil.copy2(real, real.with_name(real.name + f".bak-{stamp}"))
    real.write_text(text, encoding="utf-8")


def calib_path(slot_state):
    return str(Path(slot_state["calib_dir"]).expanduser() / "calib.yaml")


def scan_odin_devices():
    devices = []
    for dev in sorted(Path("/sys/bus/usb/devices").glob("*")):
        vendor = dev / "idVendor"
        product = dev / "idProduct"
        if not vendor.exists() or not product.exists():
            continue
        try:
            if vendor.read_text().strip() != "2207" or product.read_text().strip() != "0019":
                continue
            item = {
                "sysfs": str(dev),
                "port": dev.name,
                "bus": safe_read(dev / "busnum"),
                "addr": safe_read(dev / "devnum"),
                "speed": safe_read(dev / "speed"),
                "serial": safe_read(dev / "serial"),
                "product": safe_read(dev / "product"),
                "manufacturer": safe_read(dev / "manufacturer"),
            }
            devices.append(item)
        except OSError:
            continue
    return devices


def safe_read(path):
    try:
        return path.read_text(errors="ignore").strip()
    except Exception:
        return ""


def parse_config_values(text):
    values = {}
    key_re = re.compile(r"^(\s*)([A-Za-z0-9_.]+):\s*(.*?)(\s+#.*)?$")
    for line in text.splitlines():
        match = key_re.match(line)
        if not match:
            continue
        key = match.group(2)
        raw = match.group(3).strip()
        if raw.startswith(("'", '"')) and raw.endswith(("'", '"')) and len(raw) >= 2:
            raw = raw[1:-1]
        values[key] = raw
    return values


def format_value(value, field_type):
    if field_type == "bool":
        return "1" if str(value).lower() in ("1", "true", "yes", "on") else "0"
    if field_type == "number":
        return str(value).strip() or "0"
    if field_type == "select":
        return str(value)
    text = str(value)
    if text.startswith("[") and text.endswith("]"):
        return text
    return json.dumps(text, ensure_ascii=False)


def update_config_fields(text, updates):
    field_types = {field["key"]: field["type"] for field in QUICK_FIELDS}
    lines = text.splitlines()
    found = set()
    key_re = re.compile(r"^(\s*)([A-Za-z0-9_.]+):\s*(.*?)(\s+#.*)?$")
    for i, line in enumerate(lines):
        match = key_re.match(line)
        if not match:
            continue
        indent, key, _old, comment = match.groups()
        if key not in updates:
            continue
        value = format_value(updates[key], field_types.get(key, "text"))
        lines[i] = f"{indent}{key}: {value}{comment or ''}"
        found.add(key)
    missing = [key for key in updates if key not in found]
    if missing:
        lines.append("")
        lines.append("  # Added by Odin Web UI")
        for key in missing:
            value = format_value(updates[key], field_types.get(key, "text"))
            lines.append(f"  {key}: {value}")
    return "\n".join(lines) + "\n"


def ros2_command_available(command):
    probe = (
        "source /opt/ros/humble/setup.bash >/dev/null 2>&1 && "
        f"ros2 {shlex.quote(command)} -h >/dev/null 2>&1"
    )
    return subprocess.run(["bash", "-lc", probe], cwd=str(HOME)).returncode == 0


def launch_status():
    global launch_process
    running = launch_process is not None and launch_process.poll() is None
    external = ""
    if not running:
        proc = subprocess.run(
            ["pgrep", "-af", "ros2 launch odin_ros_driver dual_odin.launch.py|host_sdk_sample"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
        external = proc.stdout.strip()
    return {
        "running": running,
        "pid": launch_process.pid if running else None,
        "returncode": None if launch_process is None else launch_process.poll(),
        "external": external,
        "log_file": str(LOG_FILE),
    }


def resolve_usb_bindings(state):
    devices = scan_odin_devices()
    for slot in ("odin_a", "odin_b"):
        serial = str(state[slot].get("serial", "")).strip()
        if not serial:
            continue
        match = next((dev for dev in devices if dev.get("serial") == serial), None)
        if match:
            state[slot]["usb_bus"] = match["bus"]
            state[slot]["usb_addr"] = match["addr"]
    return state


def start_launch():
    global launch_process, launch_log_handle
    if launch_process is not None and launch_process.poll() is None:
        raise RuntimeError("dual Odin launch is already running")

    state = resolve_usb_bindings(load_state())
    save_state(state)
    args = [
        "ros2",
        "launch",
        "odin_ros_driver",
        "dual_odin.launch.py",
        f"use_rviz:={'true' if state.get('use_rviz') else 'false'}",
        f"odin_a_usb_bus:={state['odin_a']['usb_bus']}",
        f"odin_a_usb_addr:={state['odin_a']['usb_addr']}",
        f"odin_b_usb_bus:={state['odin_b']['usb_bus']}",
        f"odin_b_usb_addr:={state['odin_b']['usb_addr']}",
        f"odin_a_config:={state['odin_a']['config']}",
        f"odin_b_config:={state['odin_b']['config']}",
        f"odin_a_calib_dir:={state['odin_a']['calib_dir']}",
        f"odin_b_calib_dir:={state['odin_b']['calib_dir']}",
    ]
    shell_cmd = (
        "source /opt/ros/humble/setup.bash && "
        f"source {shlex.quote(str(ROS_WS / 'install' / 'setup.bash'))} && "
        f"exec {shlex.join(args)}"
    )
    LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
    launch_log_handle = LOG_FILE.open("a", encoding="utf-8")
    launch_log_handle.write("\n\n==== Odin launch started " + time.strftime("%Y-%m-%d %H:%M:%S") + " ====\n")
    launch_log_handle.flush()
    launch_process = subprocess.Popen(
        ["bash", "-lc", shell_cmd],
        cwd=str(ROS_WS),
        stdout=launch_log_handle,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
        text=True,
    )
    return launch_status()


def stop_launch():
    global launch_process, launch_log_handle
    if launch_process is None or launch_process.poll() is not None:
        return launch_status()
    os.killpg(os.getpgid(launch_process.pid), signal.SIGINT)
    try:
        launch_process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(launch_process.pid), signal.SIGTERM)
        try:
            launch_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(launch_process.pid), signal.SIGKILL)
            launch_process.wait(timeout=5)
    if launch_log_handle:
        launch_log_handle.flush()
        launch_log_handle.close()
        launch_log_handle = None
    return launch_status()


def tail_log(lines=240):
    if not LOG_FILE.exists():
        return ""
    data = LOG_FILE.read_text(encoding="utf-8", errors="replace").splitlines()
    return "\n".join(data[-lines:])


def list_topics():
    if not ros2_command_available("topic"):
        return {"available": False, "topics": [], "error": "ros-humble-ros2topic is not installed"}
    cmd = (
        "source /opt/ros/humble/setup.bash && "
        f"source {shlex.quote(str(ROS_WS / 'install' / 'setup.bash'))} && "
        "timeout 5s ros2 topic list"
    )
    proc = subprocess.run(["bash", "-lc", cmd], cwd=str(ROS_WS), text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    topics = [line.strip() for line in proc.stdout.splitlines() if line.strip()]
    return {"available": True, "topics": topics, "error": proc.stderr.strip()}


class OdinHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(STATIC_DIR), **kwargs)

    def log_message(self, fmt, *args):
        print(f"[web] {self.address_string()} - {fmt % args}")

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        try:
            if path == "/api/status":
                state = load_state()
                json_response(self, {
                    "state": state,
                    "devices": scan_odin_devices(),
                    "launch": launch_status(),
                    "ros2topic_available": ros2_command_available("topic"),
                    "quick_fields": QUICK_FIELDS,
                })
                return
            if path == "/api/configs":
                state = load_state()
                payload = {}
                for slot in ("odin_a", "odin_b"):
                    cfg = state[slot]["config"]
                    calib = calib_path(state[slot])
                    cfg_text = read_text_file(cfg)
                    payload[slot] = {
                        "config_path": cfg,
                        "config_text": cfg_text,
                        "values": parse_config_values(cfg_text),
                        "calib_path": calib,
                        "calib_text": read_text_file(calib) if Path(calib).exists() else "",
                    }
                json_response(self, payload)
                return
            if path == "/api/log":
                params = parse_qs(parsed.query)
                lines = int(params.get("lines", ["240"])[0])
                json_response(self, {"log": tail_log(lines)})
                return
            if path == "/api/topics":
                json_response(self, list_topics())
                return
            super().do_GET()
        except Exception as exc:
            json_response(self, {"error": str(exc)}, status=500)

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        try:
            body = read_json(self)
            if path == "/api/state":
                state = load_state()
                for slot in ("odin_a", "odin_b"):
                    if slot in body:
                        state[slot].update({k: str(v) for k, v in body[slot].items() if k in state[slot]})
                state = resolve_usb_bindings(state)
                if "use_rviz" in body:
                    state["use_rviz"] = bool(body["use_rviz"])
                save_state(state)
                json_response(self, {"ok": True, "state": state})
                return
            if path == "/api/config-fields":
                slot = body.get("slot")
                if slot not in ("odin_a", "odin_b"):
                    raise ValueError("slot must be odin_a or odin_b")
                state = load_state()
                cfg = state[slot]["config"]
                text = read_text_file(cfg)
                backup_and_write(cfg, update_config_fields(text, body.get("updates", {})))
                json_response(self, {"ok": True})
                return
            if path == "/api/file":
                target = body.get("path")
                text = body.get("text", "")
                backup_and_write(target, text)
                json_response(self, {"ok": True})
                return
            if path == "/api/command":
                slot = body.get("slot")
                if slot not in ("odin_a", "odin_b"):
                    raise ValueError("slot must be odin_a or odin_b")
                key = str(body.get("key", "")).strip()
                value = str(body.get("value", "")).strip()
                if not re.match(r"^[A-Za-z0-9_.-]+$", key):
                    raise ValueError("invalid command key")
                command_file = Path(load_state()[slot]["command_file"])
                command_file.write_text(f"set {key} {value}\n", encoding="utf-8")
                json_response(self, {"ok": True, "written": str(command_file), "command": f"set {key} {value}"})
                return
            if path == "/api/launch/start":
                json_response(self, start_launch())
                return
            if path == "/api/launch/stop":
                json_response(self, stop_launch())
                return
            if path == "/api/launch/restart":
                stop_launch()
                json_response(self, start_launch())
                return
            json_response(self, {"error": "not found"}, status=404)
        except Exception as exc:
            json_response(self, {"error": str(exc)}, status=500)


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Local web UI for dual Odin1 ROS2 launch/configuration")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()

    load_state()
    server = ThreadingHTTPServer((args.host, args.port), OdinHandler)
    print(f"Odin Web UI: http://{args.host}:{args.port}")
    try:
        server.serve_forever()
    finally:
        stop_launch()


if __name__ == "__main__":
    main()
