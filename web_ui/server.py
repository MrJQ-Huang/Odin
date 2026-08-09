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
    "launch_target": "auto",
}

SLOTS = ("odin_a", "odin_b")
LAUNCH_TARGETS = {"auto", "odin_a", "odin_b", "dual"}

QUICK_FIELDS = [
    {"key": "streamctrl", "type": "bool", "label": "数据流"},
    {"key": "sendcloudrender", "type": "bool", "label": "RGB 着色点云"},
    {"key": "senddtof", "type": "bool", "label": "DTOF 数据源"},
    {"key": "sendcloudslam", "type": "bool", "label": "SLAM 点云"},
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

RUNTIME_COMMAND_KEYS = {
    "sendrgb",
    "sendrgbcompressed",
    "sendrgbundistort",
    "sendimu",
    "sendodom",
    "senddtof",
    "sendcloudslam",
    "sendcloudrender",
    "pubintensitygray",
    "showpath",
    "showcamerapose",
    "devstatuslog",
}

launch_process = None
launch_log_handle = None
launch_target_requested = None
launch_effective_slots = []


def odin_runtime_processes():
    proc = subprocess.run(
        ["ps", "-eo", "pid=,pgid=,args="],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )
    current = os.getpid()
    matches = []
    for line in proc.stdout.splitlines():
        parts = line.strip().split(None, 2)
        if len(parts) < 3:
            continue
        pid, pgid, cmd = int(parts[0]), int(parts[1]), parts[2]
        if pid == current:
            continue
        is_odin_driver = "host_sdk_sample" in cmd and "odin_ros_driver" in cmd
        is_dual_launch = "ros2 launch odin_ros_driver dual_odin.launch.py" in cmd
        is_dual_rviz = "rviz2" in cmd and "dual_odin_ros2.rviz" in cmd
        is_dual_tf = "static_transform_publisher" in cmd and ("odin_a/odom" in cmd or "odin_b/odom" in cmd)
        if is_odin_driver or is_dual_launch or is_dual_rviz or is_dual_tf:
            matches.append({"pid": pid, "pgid": pgid, "cmd": cmd})
    return matches


def wait_for_odin_processes_to_exit(timeout=8):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if not odin_runtime_processes():
            return True
        time.sleep(0.2)
    return not odin_runtime_processes()


def stop_external_odin_processes():
    processes = odin_runtime_processes()
    if not processes:
        return
    pgids = sorted({proc["pgid"] for proc in processes if proc["pgid"] != os.getpgrp()})
    for pgid in pgids:
        try:
            os.killpg(pgid, signal.SIGINT)
        except ProcessLookupError:
            pass
    if wait_for_odin_processes_to_exit(timeout=8):
        return
    for pgid in pgids:
        try:
            os.killpg(pgid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    if wait_for_odin_processes_to_exit(timeout=4):
        return
    for pgid in pgids:
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    wait_for_odin_processes_to_exit(timeout=2)


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
            if item["bus"] and item["addr"]:
                devnode = Path("/dev/bus/usb") / f"{int(item['bus']):03d}" / f"{int(item['addr']):03d}"
                item["devnode"] = str(devnode)
                item["can_read"] = os.access(devnode, os.R_OK)
                item["can_write"] = os.access(devnode, os.W_OK)
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


def config_bool(values, key):
    return str(values.get(key, "0")).strip().lower() in ("1", "true", "yes", "on")


def set_rviz_display(group, display_name, enabled):
    for display in group.get("Displays", []):
        if display.get("Name") == display_name:
            display["Enabled"] = bool(enabled)
            display["Value"] = bool(enabled)
            return


def sync_dual_rviz_config(active_slots=None):
    rviz_path = DRIVER_DIR / "config" / "dual_odin_ros2.rviz"
    if not rviz_path.exists():
        return
    import yaml

    active = set(active_slots or SLOTS)
    state = load_state()
    cfg = yaml.safe_load(rviz_path.read_text(encoding="utf-8"))
    displays = cfg.get("Visualization Manager", {}).get("Displays", [])
    slot_config = {
        "Odin A": parse_config_values(read_text_file(state["odin_a"]["config"])),
        "Odin B": parse_config_values(read_text_file(state["odin_b"]["config"])),
    }

    for group in displays:
        name = group.get("Name")
        if name not in slot_config:
            continue
        values = slot_config[name]
        slot = "odin_a" if name == "Odin A" else "odin_b"
        slot_active = slot in active
        render_on = config_bool(values, "sendcloudrender")
        dtof_on = config_bool(values, "senddtof")
        slam_on = config_bool(values, "sendcloudslam")

        prefix = "A" if name == "Odin A" else "B"
        group["Enabled"] = bool(slot_active)
        group["Value"] = bool(slot_active)
        set_rviz_display(group, f"{prefix} render", slot_active and render_on)
        set_rviz_display(group, f"{prefix} raw", slot_active and dtof_on and not render_on)
        set_rviz_display(group, f"{prefix} slam", slot_active and slam_on)

    rviz_path.write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False), encoding="utf-8")


def write_runtime_commands(slot, updates):
    state = load_state()
    commands = []
    field_types = {field["key"]: field["type"] for field in QUICK_FIELDS}
    for key, value in updates.items():
        if key in RUNTIME_COMMAND_KEYS:
            commands.append(f"set {key} {format_value(value, field_types.get(key, 'text'))}")
    if not commands:
        return []
    command_file = Path(state[slot]["command_file"])
    command_file.write_text("\n".join(commands) + "\n", encoding="utf-8")
    return commands


def ros2_command_available(command):
    probe = (
        "source /opt/ros/humble/setup.bash >/dev/null 2>&1 && "
        f"ros2 {shlex.quote(command)} -h >/dev/null 2>&1"
    )
    return subprocess.run(["bash", "-lc", probe], cwd=str(HOME)).returncode == 0


def launch_status():
    global launch_process
    owned_running = launch_process is not None and launch_process.poll() is None
    external_processes = odin_runtime_processes()
    running = owned_running or bool(external_processes)
    external = "\n".join(f"{proc['pid']} {proc['cmd']}" for proc in external_processes)
    return {
        "running": running,
        "pid": launch_process.pid if owned_running else (external_processes[0]["pid"] if external_processes else None),
        "returncode": None if launch_process is None else launch_process.poll(),
        "external": external,
        "log_file": str(LOG_FILE),
        "target": launch_target_requested,
        "effective_slots": launch_effective_slots,
    }


def resolve_usb_bindings(state):
    devices = scan_odin_devices()
    for slot in SLOTS:
        serial = str(state[slot].get("serial", "")).strip()
        if not serial:
            continue
        match = next((dev for dev in devices if dev.get("serial") == serial), None)
        if match:
            state[slot]["usb_bus"] = match["bus"]
            state[slot]["usb_addr"] = match["addr"]
    return state


def match_device_for_slot(state, devices, slot):
    slot_state = state.get(slot, {})
    serial = str(slot_state.get("serial", "")).strip()
    bus = str(slot_state.get("usb_bus", "")).strip()
    addr = str(slot_state.get("usb_addr", "")).strip()
    if serial:
        match = next((dev for dev in devices if dev.get("serial") == serial), None)
        if match:
            return match
    if bus and addr:
        return next((dev for dev in devices if dev.get("bus") == bus and dev.get("addr") == addr), None)
    return None


def slot_label(slot):
    return "Odin A" if slot == "odin_a" else "Odin B"


def launch_plan(state, devices, target=None):
    requested = str(target or state.get("launch_target") or "auto").strip()
    if requested not in LAUNCH_TARGETS:
        requested = "auto"

    matches = {slot: match_device_for_slot(state, devices, slot) for slot in SLOTS}
    duplicate_serial = (
        str(state.get("odin_a", {}).get("serial", "")).strip()
        and str(state.get("odin_a", {}).get("serial", "")).strip()
        == str(state.get("odin_b", {}).get("serial", "")).strip()
    )

    temporary_slot = None
    temporary_device = None
    if len(devices) == 1 and not matches["odin_a"] and not matches["odin_b"]:
        temporary_device = devices[0]
        temporary_slot = "odin_b" if requested == "odin_b" else "odin_a"
        matches[temporary_slot] = temporary_device

    if requested == "dual":
        slots = ["odin_a", "odin_b"]
    elif requested in SLOTS:
        slots = [requested]
    else:
        online_slots = [slot for slot in SLOTS if matches[slot]]
        if len(online_slots) >= 2 and not duplicate_serial:
            slots = ["odin_a", "odin_b"]
        elif len(online_slots) == 1:
            slots = online_slots
        else:
            slots = []

    missing_slots = [slot for slot in slots if not matches[slot]]
    selected_devices = [matches[slot] for slot in slots if matches[slot]]
    permission_slots = [
        slot
        for slot in slots
        if matches[slot] and not (matches[slot].get("can_read") and matches[slot].get("can_write"))
    ]
    ready = bool(slots) and not missing_slots and not permission_slots and not duplicate_serial

    if requested == "auto" and len(slots) == 2:
        label = "自动匹配：双机"
    elif requested == "auto" and len(slots) == 1:
        label = f"自动匹配：{slot_label(slots[0])} 单机"
    elif requested == "dual":
        label = "手动选择：双机"
    elif slots:
        label = f"手动选择：{slot_label(slots[0])} 单机"
    else:
        label = "自动匹配：等待设备"

    if duplicate_serial:
        reason = "A/B 绑定到了同一个 serial"
    elif not devices:
        reason = "还没有看到 Odin"
    elif missing_slots:
        reason = "、".join(slot_label(slot) for slot in missing_slots) + " 未在线"
    elif permission_slots:
        reason = "、".join(slot_label(slot) for slot in permission_slots) + " USB 权限不足"
    elif not slots:
        reason = "请选择设备身份或切换启动目标"
    else:
        reason = f"{label} 已就绪"

    return {
        "requested": requested,
        "effective_slots": slots,
        "ready": ready,
        "label": label,
        "reason": reason,
        "temporary_slot": temporary_slot,
        "temporary_serial": temporary_device.get("serial") if temporary_device else "",
        "permission_slots": permission_slots,
        "missing_slots": missing_slots,
        "matches": matches,
    }


def apply_plan_usb_addresses(state, plan):
    for slot in plan["effective_slots"]:
        dev = plan["matches"].get(slot)
        if not dev:
            continue
        state[slot]["usb_bus"] = dev["bus"]
        state[slot]["usb_addr"] = dev["addr"]
    return state


def summarize_state(state, devices, launch):
    matches = {}
    for slot in SLOTS:
        match = match_device_for_slot(state, devices, slot)
        matches[slot] = match

    serials = [
        str(state.get(slot, {}).get("serial", "")).strip()
        for slot in SLOTS
        if str(state.get(slot, {}).get("serial", "")).strip()
    ]
    duplicate_serial = len(serials) == 2 and serials[0] == serials[1]
    a_online = matches["odin_a"] is not None
    b_online = matches["odin_b"] is not None
    plan = launch_plan(state, devices, state.get("launch_target"))
    binding_ready = bool(plan["effective_slots"]) and not plan["missing_slots"] and not duplicate_serial
    usb_access_ready = all(bool(dev.get("can_read")) and bool(dev.get("can_write")) for dev in plan["matches"].values() if dev)
    launch_usb_access_ready = not plan["permission_slots"]
    running = bool(launch.get("running"))

    if running:
        status = "running"
        action = "monitor"
        running_slots = launch.get("effective_slots") or plan["effective_slots"]
        running_label = "双 Odin" if len(running_slots) == 2 else slot_label(running_slots[0]) if running_slots else "Odin"
        message = f"{running_label} 正在运行"
    elif len(devices) == 0:
        status = "no_device"
        action = "scan"
        message = "还没有看到 Odin"
    elif duplicate_serial:
        status = "duplicate_binding"
        action = "bind"
        message = "A/B 绑定到了同一个 serial"
    elif binding_ready and not launch_usb_access_ready:
        status = "permission_blocked"
        action = "fix_usb_permissions"
        message = plan["reason"]
    elif binding_ready:
        status = "ready"
        action = "start"
        message = plan["reason"]
    elif a_online or b_online:
        status = "partial_bound"
        action = "bind"
        message = plan["reason"]
    else:
        status = "unbound"
        action = "bind"
        message = plan["reason"]

    return {
        "status": status,
        "message": message,
        "recommended_action": action,
        "device_count": len(devices),
        "a_online": a_online,
        "b_online": b_online,
        "binding_ready": binding_ready,
        "usb_access_ready": usb_access_ready,
        "launch_usb_access_ready": launch_usb_access_ready,
        "launch_ready": plan["ready"] and not running,
        "duplicate_serial": duplicate_serial,
        "launch_plan": {k: v for k, v in plan.items() if k != "matches"},
        "matches": {
            "odin_a": matches["odin_a"],
            "odin_b": matches["odin_b"],
        },
    }


def start_launch(target=None):
    global launch_process, launch_log_handle, launch_target_requested, launch_effective_slots
    if launch_status().get("running"):
        raise RuntimeError("Odin launch is already running")

    state = resolve_usb_bindings(load_state())
    devices = scan_odin_devices()
    plan = launch_plan(state, devices, target or state.get("launch_target"))
    if not plan["ready"]:
        raise RuntimeError(plan["reason"])
    state["launch_target"] = plan["requested"]
    state = apply_plan_usb_addresses(state, plan)
    save_state(state)
    sync_dual_rviz_config(plan["effective_slots"])
    enable_a = "odin_a" in plan["effective_slots"]
    enable_b = "odin_b" in plan["effective_slots"]
    args = [
        "ros2",
        "launch",
        "odin_ros_driver",
        "dual_odin.launch.py",
        f"use_rviz:={'true' if state.get('use_rviz') else 'false'}",
        f"enable_odin_a:={'true' if enable_a else 'false'}",
        f"enable_odin_b:={'true' if enable_b else 'false'}",
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
    clean_env = os.environ.copy()
    for key in list(clean_env):
        if key.startswith("SNAP") or key in {
            "GTK_EXE_PREFIX",
            "GTK_PATH",
            "GTK_IM_MODULE_FILE",
            "GIO_MODULE_DIR",
            "GIO_EXTRA_MODULES",
            "LD_LIBRARY_PATH",
        }:
            clean_env.pop(key, None)
    LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
    launch_log_handle = LOG_FILE.open("a", encoding="utf-8")
    launch_target_requested = plan["requested"]
    launch_effective_slots = plan["effective_slots"]
    launch_log_handle.write(
        "\n\n==== Odin launch started "
        + time.strftime("%Y-%m-%d %H:%M:%S")
        + f" target={plan['requested']} slots={','.join(plan['effective_slots'])} ====\n"
    )
    launch_log_handle.flush()
    launch_process = subprocess.Popen(
        ["bash", "-lc", shell_cmd],
        cwd=str(ROS_WS),
        stdout=launch_log_handle,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
        text=True,
        env=clean_env,
    )
    return launch_status()


def stop_launch():
    global launch_process, launch_log_handle, launch_target_requested, launch_effective_slots
    if launch_process is not None and launch_process.poll() is None:
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
    stop_external_odin_processes()
    if launch_log_handle:
        launch_log_handle.flush()
        launch_log_handle.close()
        launch_log_handle = None
    launch_target_requested = None
    launch_effective_slots = []
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


def odin_usb_devnodes():
    devnodes = []
    for dev in scan_odin_devices():
        node = dev.get("devnode")
        if node and Path(node).exists():
            devnodes.append(node)
    return sorted(set(devnodes))


def fix_usb_permissions():
    devnodes = odin_usb_devnodes()
    if not devnodes:
        raise RuntimeError("没有找到 Odin USB 设备")
    cmd = ["pkexec", "/bin/chmod", "a+rw", *devnodes]
    proc = subprocess.run(
        cmd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=120,
    )
    if proc.returncode != 0:
        detail = proc.stderr.strip() or proc.stdout.strip() or f"pkexec exited with {proc.returncode}"
        raise RuntimeError(detail)
    return {"ok": True, "devnodes": devnodes}


class OdinHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(STATIC_DIR), **kwargs)

    def log_message(self, fmt, *args):
        print(f"[web] {self.address_string()} - {fmt % args}")

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        try:
            if path == "/api/status":
                state = load_state()
                devices = scan_odin_devices()
                launch = launch_status()
                json_response(self, {
                    "state": state,
                    "devices": devices,
                    "launch": launch,
                    "summary": summarize_state(state, devices, launch),
                    "ros2topic_available": ros2_command_available("topic"),
                    "quick_fields": QUICK_FIELDS,
                })
                return
            if path == "/api/configs":
                state = load_state()
                payload = {}
                for slot in SLOTS:
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
                for slot in SLOTS:
                    if slot in body:
                        state[slot].update({k: str(v) for k, v in body[slot].items() if k in state[slot]})
                state = resolve_usb_bindings(state)
                if "use_rviz" in body:
                    state["use_rviz"] = bool(body["use_rviz"])
                if body.get("launch_target") in LAUNCH_TARGETS:
                    state["launch_target"] = body["launch_target"]
                save_state(state)
                json_response(self, {"ok": True, "state": state})
                return
            if path == "/api/config-fields":
                slots = body.get("slots") or [body.get("slot")]
                slots = [slot for slot in slots if slot in SLOTS]
                if not slots:
                    raise ValueError("slot must be odin_a or odin_b")
                state = load_state()
                updates = body.get("updates", {})
                for slot in slots:
                    cfg = state[slot]["config"]
                    text = read_text_file(cfg)
                    backup_and_write(cfg, update_config_fields(text, updates))
                plan = launch_plan(state, scan_odin_devices(), state.get("launch_target"))
                sync_dual_rviz_config(plan["effective_slots"] or SLOTS)
                runtime_commands = []
                launch_action = "none"
                if body.get("apply_running", False):
                    status_before = launch_status()
                    if status_before.get("running"):
                        restart_target = status_before.get("target") or state.get("launch_target") or "auto"
                        stop_launch()
                        status_after = start_launch(restart_target)
                        launch_action = "restarted"
                    else:
                        status_after = launch_status()
                else:
                    status_after = launch_status()
                json_response(self, {
                    "ok": True,
                    "slots": slots,
                    "runtime_commands": runtime_commands,
                    "launch_action": launch_action,
                    "launch": status_after,
                })
                return
            if path == "/api/file":
                target = body.get("path")
                text = body.get("text", "")
                backup_and_write(target, text)
                json_response(self, {"ok": True})
                return
            if path == "/api/command":
                slot = body.get("slot")
                if slot not in SLOTS:
                    raise ValueError("slot must be odin_a or odin_b")
                key = str(body.get("key", "")).strip()
                value = str(body.get("value", "")).strip()
                if not re.match(r"^[A-Za-z0-9_.-]+$", key):
                    raise ValueError("invalid command key")
                command_file = Path(load_state()[slot]["command_file"])
                command_file.write_text(f"set {key} {value}\n", encoding="utf-8")
                json_response(self, {"ok": True, "written": str(command_file), "command": f"set {key} {value}"})
                return
            if path == "/api/usb-permissions/fix":
                json_response(self, fix_usb_permissions())
                return
            if path == "/api/launch/start":
                json_response(self, start_launch(body.get("target")))
                return
            if path == "/api/launch/stop":
                json_response(self, stop_launch())
                return
            if path == "/api/launch/restart":
                target = body.get("target")
                if not target:
                    status_before = launch_status()
                    target = status_before.get("target") or load_state().get("launch_target") or "auto"
                stop_launch()
                json_response(self, start_launch(target))
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
