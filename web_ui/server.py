#!/usr/bin/env python3
import json
import math
import os
import re
import shutil
import signal
import sqlite3
import subprocess
import select
import termios
import threading
import time
import shlex
import tty
from collections import deque
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse


HOME = Path.home()
ODIN_ROOT = HOME / "odin"
ROS_WS = ODIN_ROOT / "ros2_ws"
DRIVER_DIR = ODIN_ROOT / "odin_ros_driver-main"
STATIC_DIR = Path(__file__).resolve().parent / "static"
STATE_FILE = Path(__file__).resolve().parent / "state.json"
LOCAL_STATE_FILE = Path(__file__).resolve().parent / "local_state.json"
LOG_FILE = Path(__file__).resolve().parent / "odin_web_launch.log"
RTK_LOG_FILE = Path(__file__).resolve().parent / "rtk_ros_launch.log"
CAPTURE_LOG_FILE = Path(__file__).resolve().parent / "capture_rosbag.log"
CAPTURE_DIR = ODIN_ROOT / "captures"
CAPTURE_MANIFEST = "capture_manifest.json"
FUSION_OUTPUT_DIR = CAPTURE_DIR / "fusion_outputs"
FUSION_MAX_POINTS = 120000

DEFAULT_STATE = {
    "odin_a": {
        "usb_bus": "",
        "usb_addr": "",
        "serial": "",
        "config": str(DRIVER_DIR / "config" / "control_command_odin_a.yaml"),
        "calib_dir": str(HOME / ".ros" / "odin_a"),
        "command_file": "/tmp/odin_a_command.txt",
        "frame_prefix": "odin_a",
    },
    "odin_b": {
        "usb_bus": "",
        "usb_addr": "",
        "serial": "",
        "config": str(DRIVER_DIR / "config" / "control_command_odin_b.yaml"),
        "calib_dir": str(HOME / ".ros" / "odin_b"),
        "command_file": "/tmp/odin_b_command.txt",
        "frame_prefix": "odin_b",
    },
    "use_rviz": True,
    "launch_target": "auto",
    "rtk": {
        "port": "/dev/ttyACM0",
        "baudrate": "115200",
    },
    "capture": {
        "cloud_topic": "/odin_b/odin1/cloud_render",
        "output_dir": str(CAPTURE_DIR),
        "last_bag": "",
    },
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

ODIN_CAPTURE_SUFFIXES = [
    "odin1/cloud_render",
    "odin1/cloud_slam",
    "odin1/cloud_raw",
    "odin1/odometry",
    "odin1/odometry_highfreq",
    "odin1/wiwc",
    "odin1/imu",
    "odin1/path",
    "odin1/camera_pose_visual",
    "odin1/image",
    "odin1/image/compressed",
    "odin1/image/undistorted",
    "odin1/image/intensity_gray",
    "odin1/reprojected_image",
    "odin1/overlay_image",
]

RTK_CAPTURE_TOPICS = [
    "/gnss/fix",
    "/gnss/heading",
    "/gnss/heading_deg",
    "/gnss/status",
    "/gnss/raw_sentence",
]

CAPTURE_SUPPORT_TOPICS = [
    "/odin_rtk/bound_cloud",
    "/odin_rtk/bound_meta",
    "/tf",
    "/tf_static",
]

launch_process = None
launch_log_handle = None
launch_target_requested = None
launch_effective_slots = []
topic_cache = {"updated_at": 0.0, "data": None}
topic_cache_lock = threading.Lock()


def ros_shell_prefix():
    return (
        "source /opt/ros/humble/setup.bash && "
        f"source {shlex.quote(str(ROS_WS / 'install' / 'setup.bash'))} && "
    )


def clean_ros_env():
    env = os.environ.copy()
    for key in list(env):
        if key.startswith("SNAP") or key in {
            "GDK_PIXBUF_MODULEDIR",
            "GDK_PIXBUF_MODULE_FILE",
            "GSETTINGS_SCHEMA_DIR",
            "GTK_EXE_PREFIX",
            "GTK_IM_MODULE_FILE",
            "GTK_PATH",
            "GIO_EXTRA_MODULES",
            "GIO_MODULE_DIR",
            "LD_LIBRARY_PATH",
            "LOCPATH",
            "QT_PLUGIN_PATH",
            "XDG_DATA_HOME",
        }:
            env.pop(key, None)
    xdg_dirs = env.get("XDG_DATA_DIRS", "")
    if xdg_dirs:
        filtered = [item for item in xdg_dirs.split(":") if "/snap/" not in item and "/snapd/" not in item]
        env["XDG_DATA_DIRS"] = ":".join(filtered) or "/usr/local/share:/usr/share"
    path = env.get("PATH", "")
    if path:
        env["PATH"] = ":".join(item for item in path.split(":") if item and item != "/snap/bin")
    return env


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


def state_copy(value):
    return json.loads(json.dumps(value))


def read_state_file(path):
    if not path.exists():
        return {}
    try:
        with path.open("r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}


def merge_state(base, data):
    for key, value in data.items():
        if key not in base:
            continue
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            base[key].update(value)
        else:
            base[key] = value
    return base


def rehome_path(value, anchors):
    text = str(value or "")
    if not text:
        return text
    for pattern, replacement in anchors:
        updated = re.sub(pattern, replacement, text, count=1)
        if updated != text:
            return updated
    return text


def normalize_state_paths(state):
    changed = False
    anchors = [
        (r"^/home/[^/]+/odin", str(ODIN_ROOT)),
        (r"^/data/[^/]+/odin", str(ODIN_ROOT)),
    ]
    for slot in SLOTS:
        slot_state = state.get(slot, {})
        config = rehome_path(slot_state.get("config", ""), anchors)
        if config != slot_state.get("config", ""):
            slot_state["config"] = config
            changed = True
        calib_dir = rehome_path(
            slot_state.get("calib_dir", ""),
            [(r"^/home/[^/]+/\.ros", str(HOME / ".ros"))],
        )
        if calib_dir != slot_state.get("calib_dir", ""):
            slot_state["calib_dir"] = calib_dir
            changed = True
    capture = state.get("capture", {})
    for key in ("output_dir", "last_bag"):
        updated = rehome_path(capture.get(key, ""), anchors)
        if updated != capture.get(key, ""):
            capture[key] = updated
            changed = True
    return changed


def load_state():
    state = state_copy(DEFAULT_STATE)
    merge_state(state, read_state_file(STATE_FILE))
    has_local_state = LOCAL_STATE_FILE.exists()
    merge_state(state, read_state_file(LOCAL_STATE_FILE))
    changed = normalize_state_paths(state)
    before_dedupe = json.dumps(state.get("odin_a", {}), sort_keys=True) + json.dumps(state.get("odin_b", {}), sort_keys=True)
    state = dedupe_odin_bindings(state)
    after_dedupe = json.dumps(state.get("odin_a", {}), sort_keys=True) + json.dumps(state.get("odin_b", {}), sort_keys=True)
    if before_dedupe != after_dedupe:
        changed = True
    if changed and has_local_state:
        save_state(state)
    return state


def save_state(state):
    LOCAL_STATE_FILE.write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding="utf-8")


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


def udev_properties(devnode):
    try:
        proc = subprocess.run(
            ["udevadm", "info", "--query=property", f"--name={devnode}"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=3,
        )
    except Exception:
        return {}
    props = {}
    for line in proc.stdout.splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        props[key] = value
    return props


def scan_serial_devices():
    devices = []
    for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        for path in sorted(Path("/dev").glob(Path(pattern).name)):
            devnode = str(path)
            props = udev_properties(devnode)
            model = props.get("ID_MODEL", "")
            vendor = props.get("ID_VENDOR", "")
            serial = props.get("ID_SERIAL", "")
            by_id = ""
            links = props.get("DEVLINKS", "")
            for link in links.split():
                if "/dev/serial/by-id/" in link:
                    by_id = link
                    break
            stable_path = by_id or devnode
            label_bits = [part.replace("_", " ") for part in (vendor, model) if part]
            label = " ".join(label_bits) or path.name
            is_rtk_candidate = any(
                token in " ".join([model, vendor, serial]).upper()
                for token in ("GNSS", "RTK", "GPS", "UART", "HOLTEK", "UM982", "BEITIAN")
            )
            devices.append({
                "devnode": devnode,
                "name": path.name,
                "label": label,
                "vendor": vendor,
                "model": model,
                "serial": serial,
                "by_id": by_id,
                "stable_path": stable_path,
                "path": props.get("ID_PATH", ""),
                "bus": props.get("ID_BUS", ""),
                "can_read": os.access(devnode, os.R_OK),
                "can_write": os.access(devnode, os.W_OK),
                "rtk_candidate": is_rtk_candidate,
            })
    return devices


def resolved_port(path):
    try:
        return str(Path(path).resolve())
    except Exception:
        return str(path)


def port_matches_device(port, dev):
    if not port or not dev:
        return False
    aliases = {dev.get("devnode", ""), dev.get("by_id", ""), dev.get("stable_path", "")}
    if port in aliases:
        return True
    devnode = dev.get("devnode")
    return bool(devnode and resolved_port(port) == resolved_port(devnode))


def find_serial_device_for_port(port, serial_devices):
    return next((dev for dev in serial_devices if port_matches_device(port, dev)), None)


def select_default_rtk_port(state, serial_devices):
    configured = str(state.get("rtk", {}).get("port", "")).strip()
    if configured and Path(configured).exists():
        selected = find_serial_device_for_port(configured, serial_devices)
        return selected.get("stable_path") if selected else configured
    candidate = next((dev for dev in serial_devices if dev.get("rtk_candidate")), None)
    if candidate:
        return candidate.get("stable_path") or candidate["devnode"]
    if serial_devices:
        return serial_devices[0].get("stable_path") or serial_devices[0]["devnode"]
    return configured or "/dev/ttyACM0"


def same_serial_port(left, right):
    if not left or not right:
        return False
    if left == right:
        return True
    return resolved_port(left) == resolved_port(right)


def service_state(name):
    try:
        proc = subprocess.run(
            ["systemctl", "is-active", name],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=2,
        )
        active = proc.stdout.strip()
    except Exception:
        active = "unknown"
    try:
        proc = subprocess.run(
            ["systemctl", "is-enabled", name],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=2,
        )
        enabled = proc.stdout.strip()
    except Exception:
        enabled = "unknown"
    return {"active": active, "enabled": enabled}


def safe_read(path):
    try:
        return path.read_text(errors="ignore").strip()
    except Exception:
        return ""


FIX_QUALITY_TEXT = {
    "0": "无效",
    "1": "单点定位",
    "2": "DGPS/SBAS",
    "4": "RTK 固定解",
    "5": "RTK 浮点解",
    "6": "惯导定位",
    "7": "固定坐标",
}

RMC_MODE_TEXT = {
    "N": "无效",
    "A": "自主定位",
    "D": "差分定位",
    "E": "估算",
    "R": "RTK 固定解",
    "F": "RTK 浮点解",
}

GSA_FIX_TYPE_TEXT = {
    "1": "无定位",
    "2": "2D",
    "3": "3D",
}

NMEA_TALKER_TEXT = {
    "GN": "多星座联合",
    "GP": "GPS",
    "GL": "GLONASS",
    "GA": "Galileo",
    "BD": "北斗",
    "GB": "北斗",
}

HEADING_STATUS_TEXT = {
    "SOL_COMPUTED": "定向已解算",
    "INSUFFICIENT_OBS": "观测不足",
    "NONE": "无解",
}


def parse_float(value, default=None):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def parse_int(value, default=0):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def dm_to_deg(value, hemi):
    if not value:
        return None
    raw = parse_float(value)
    if raw is None:
        return None
    degrees = int(raw // 100)
    minutes = raw - degrees * 100
    result = degrees + minutes / 60.0
    if hemi in ("S", "W"):
        result = -result
    return result


def nmea_checksum_ok(sentence):
    if not sentence.startswith("$") or "*" not in sentence:
        return False
    body, checksum = sentence[1:].split("*", 1)
    value = 0
    for char in body:
        value ^= ord(char)
    try:
        return value == int(checksum[:2], 16)
    except ValueError:
        return False


def nmea_type(sentence):
    return sentence[3:6] if len(sentence) >= 6 else ""


def nmea_talker(sentence):
    return sentence[1:3] if len(sentence) >= 3 else ""


def format_utc_time(value):
    if not value or len(value) < 6:
        return ""
    return f"{value[0:2]}:{value[2:4]}:{value[4:]}"


def format_nmea_date(value):
    if not value or len(value) != 6:
        return ""
    day = value[0:2]
    month = value[2:4]
    year = int(value[4:6])
    year += 2000 if year < 80 else 1900
    return f"{year:04d}-{month}-{day}"


def nmea_datetime_utc(time_value, date_value):
    date_text = format_nmea_date(date_value)
    time_text = format_utc_time(time_value)
    if not date_text or not time_text:
        return ""
    return f"{date_text} {time_text} UTC"


def parse_rtk_sentence(line):
    if line.startswith("$"):
        kind = nmea_type(line)
        talker = nmea_talker(line)
        checksum_ok = nmea_checksum_ok(line)
        record = {
            "raw": line,
            "family": "NMEA",
            "talker": talker,
            "talker_text": NMEA_TALKER_TEXT.get(talker, talker or "未知"),
            "type": kind,
            "checksum_ok": checksum_ok,
        }
        fields = line.split("*", 1)[0].split(",")
        if kind == "GGA":
            quality = fields[6] if len(fields) > 6 else "0"
            record.update({
                "label": "定位结果 GGA",
                "utc": fields[1] if len(fields) > 1 else "",
                "utc_time": format_utc_time(fields[1]) if len(fields) > 1 else "",
                "latitude": dm_to_deg(fields[2], fields[3]) if len(fields) > 3 else None,
                "longitude": dm_to_deg(fields[4], fields[5]) if len(fields) > 5 else None,
                "fix_quality": quality,
                "fix_quality_text": FIX_QUALITY_TEXT.get(quality, "未知"),
                "satellites": parse_int(fields[7]) if len(fields) > 7 else 0,
                "hdop": parse_float(fields[8]) if len(fields) > 8 else None,
                "altitude_m": parse_float(fields[9]) if len(fields) > 9 else None,
                "geoid_sep_m": parse_float(fields[11]) if len(fields) > 11 else None,
                "diff_age_s": fields[13] if len(fields) > 13 else "",
                "station_id": fields[14] if len(fields) > 14 else "",
            })
        elif kind == "RMC":
            mode = fields[12] if len(fields) > 12 else ""
            date_value = fields[9] if len(fields) > 9 else ""
            utc_value = fields[1] if len(fields) > 1 else ""
            record.update({
                "label": "推荐导航 RMC",
                "utc": utc_value,
                "utc_time": format_utc_time(utc_value),
                "valid": len(fields) > 2 and fields[2] == "A",
                "latitude": dm_to_deg(fields[3], fields[4]) if len(fields) > 4 else None,
                "longitude": dm_to_deg(fields[5], fields[6]) if len(fields) > 6 else None,
                "speed_knots": parse_float(fields[7]) if len(fields) > 7 else None,
                "speed_mps": (parse_float(fields[7], 0.0) or 0.0) * 0.514444 if len(fields) > 7 else None,
                "course_deg": parse_float(fields[8]) if len(fields) > 8 else None,
                "date_ddmmyy": date_value,
                "date": format_nmea_date(date_value),
                "datetime_utc": nmea_datetime_utc(utc_value, date_value),
                "mode": mode,
                "mode_text": RMC_MODE_TEXT.get(mode, "未知"),
                "nav_status": fields[13] if len(fields) > 13 else "",
            })
        elif kind == "GSA":
            fix_type = fields[2] if len(fields) > 2 else ""
            prns = [value for value in fields[3:15] if value]
            record.update({
                "label": "DOP 与解算卫星 GSA",
                "selection_mode": fields[1] if len(fields) > 1 else "",
                "selection_mode_text": "自动" if len(fields) > 1 and fields[1] == "A" else "手动",
                "fix_type": fix_type,
                "fix_type_text": GSA_FIX_TYPE_TEXT.get(fix_type, "未知"),
                "satellites_used_prn": prns,
                "pdop": parse_float(fields[15]) if len(fields) > 15 else None,
                "hdop": parse_float(fields[16]) if len(fields) > 16 else None,
                "vdop": parse_float(fields[17]) if len(fields) > 17 else None,
                "system_id": fields[18] if len(fields) > 18 else "",
            })
        elif kind == "GSV":
            satellites = []
            body_fields = fields[4:]
            for idx in range(0, len(body_fields), 4):
                chunk = body_fields[idx:idx + 4]
                if len(chunk) < 4 or not chunk[0]:
                    continue
                satellites.append({
                    "prn": chunk[0],
                    "elevation_deg": parse_float(chunk[1]),
                    "azimuth_deg": parse_float(chunk[2]),
                    "snr_dbhz": parse_float(chunk[3]),
                })
            record.update({
                "label": "可见卫星 GSV",
                "total_messages": parse_int(fields[1]) if len(fields) > 1 else 0,
                "message_number": parse_int(fields[2]) if len(fields) > 2 else 0,
                "satellites_in_view": parse_int(fields[3]) if len(fields) > 3 else 0,
                "satellites": satellites,
                "signal_id": fields[-1] if len(fields) > 8 and len(fields[4:]) % 4 == 1 else "",
            })
        elif kind == "GST":
            record.update({
                "label": "伪距噪声 GST",
                "utc": fields[1] if len(fields) > 1 else "",
                "utc_time": format_utc_time(fields[1]) if len(fields) > 1 else "",
                "rms_m": parse_float(fields[2]) if len(fields) > 2 else None,
                "semi_major_std_m": parse_float(fields[3]) if len(fields) > 3 else None,
                "semi_minor_std_m": parse_float(fields[4]) if len(fields) > 4 else None,
                "orientation_deg": parse_float(fields[5]) if len(fields) > 5 else None,
                "lat_std_m": parse_float(fields[6]) if len(fields) > 6 else None,
                "lon_std_m": parse_float(fields[7]) if len(fields) > 7 else None,
                "alt_std_m": parse_float(fields[8]) if len(fields) > 8 else None,
            })
        elif kind == "ZDA":
            utc_value = fields[1] if len(fields) > 1 else ""
            day = fields[2] if len(fields) > 2 else ""
            month = fields[3] if len(fields) > 3 else ""
            year = fields[4] if len(fields) > 4 else ""
            date_text = f"{year}-{month.zfill(2)}-{day.zfill(2)}" if year and month and day else ""
            record.update({
                "label": "UTC 日期时间 ZDA",
                "utc": utc_value,
                "utc_time": format_utc_time(utc_value),
                "day": day,
                "month": month,
                "year": year,
                "date": date_text,
                "datetime_utc": f"{date_text} {format_utc_time(utc_value)} UTC" if date_text and utc_value else "",
                "local_zone_hours": fields[5] if len(fields) > 5 else "",
                "local_zone_minutes": fields[6] if len(fields) > 6 else "",
            })
        elif kind == "VTG":
            record.update({
                "label": "地速航向 VTG",
                "course_true_deg": parse_float(fields[1]) if len(fields) > 1 else None,
                "course_magnetic_deg": parse_float(fields[3]) if len(fields) > 3 else None,
                "speed_knots": parse_float(fields[5]) if len(fields) > 5 else None,
                "speed_kmh": parse_float(fields[7]) if len(fields) > 7 else None,
                "mode": fields[9] if len(fields) > 9 else "",
            })
        elif kind == "GLL":
            mode = fields[7] if len(fields) > 7 else ""
            record.update({
                "label": "地理位置 GLL",
                "latitude": dm_to_deg(fields[1], fields[2]) if len(fields) > 2 else None,
                "longitude": dm_to_deg(fields[3], fields[4]) if len(fields) > 4 else None,
                "utc": fields[5] if len(fields) > 5 else "",
                "utc_time": format_utc_time(fields[5]) if len(fields) > 5 else "",
                "status": fields[6] if len(fields) > 6 else "",
                "valid": len(fields) > 6 and fields[6] == "A",
                "mode": mode,
                "mode_text": RMC_MODE_TEXT.get(mode, "未知"),
            })
        elif kind == "HDT":
            record.update({
                "label": "真北航向 HDT",
                "heading_deg": parse_float(fields[1]) if len(fields) > 1 else None,
            })
        elif kind == "THS":
            record.update({
                "label": "真北航向状态 THS",
                "heading_deg": parse_float(fields[1]) if len(fields) > 1 else None,
                "status": fields[2] if len(fields) > 2 else "",
            })
        elif kind == "TRA":
            record.update({
                "label": "姿态角 TRA",
                "utc": fields[1] if len(fields) > 1 else "",
                "utc_time": format_utc_time(fields[1]) if len(fields) > 1 else "",
                "heading_deg": parse_float(fields[2]) if len(fields) > 2 else None,
                "pitch_deg": parse_float(fields[3]) if len(fields) > 3 else None,
                "roll_deg": parse_float(fields[4]) if len(fields) > 4 else None,
                "quality": fields[5] if len(fields) > 5 else "",
                "satellites": parse_int(fields[6]) if len(fields) > 6 else 0,
            })
        else:
            record["label"] = f"NMEA {kind or '未知'}"
        return record

    if line.startswith("#"):
        name = line.split(",", 1)[0].lstrip("#")
        record = {"raw": line, "family": "Unicore", "type": name, "label": name}
        if name == "HEADINGA" and ";" in line:
            data = line.split("*", 1)[0].split(";", 1)[1].split(",")
            if len(data) >= 5:
                status = data[0]
                record.update({
                    "label": "双天线定向 HEADINGA",
                    "solution_status": status,
                    "solution_status_text": HEADING_STATUS_TEXT.get(status, status),
                    "position_type": data[1],
                    "baseline_m": parse_float(data[2]),
                    "heading_deg": parse_float(data[3]),
                    "pitch_deg": parse_float(data[4]),
                    "heading_std_deg": parse_float(data[6]) if len(data) > 6 else None,
                    "pitch_std_deg": parse_float(data[7]) if len(data) > 7 else None,
                    "station_id": data[8].strip('"') if len(data) > 8 else "",
                    "satellites_tracked": parse_int(data[9]) if len(data) > 9 else 0,
                    "satellites_used": parse_int(data[10]) if len(data) > 10 else 0,
                    "valid": status == "SOL_COMPUTED",
                })
        return record

    return {"raw": line, "family": "Unknown", "type": "UNKNOWN", "label": "无法识别"}


def compact_gsv(records):
    latest_parts = {}
    for item in records:
        if item.get("type") != "GSV":
            continue
        key = (item.get("talker", ""), item.get("message_number", 0))
        latest_parts[key] = item

    by_talker = {}
    satellites = []
    for item in latest_parts.values():
        talker = item.get("talker", "")
        bucket = by_talker.setdefault(talker, {
            "talker": talker,
            "talker_text": item.get("talker_text", talker),
            "satellites_in_view": item.get("satellites_in_view", 0),
            "satellites": [],
        })
        bucket["satellites_in_view"] = max(bucket["satellites_in_view"], item.get("satellites_in_view", 0) or 0)
        for sat in item.get("satellites", []):
            entry = {
                **sat,
                "talker": talker,
                "talker_text": item.get("talker_text", talker),
            }
            bucket["satellites"].append(entry)
            satellites.append(entry)

    satellites.sort(key=lambda sat: (
        sat.get("talker_text", ""),
        parse_int(sat.get("prn"), 999),
    ))
    return {
        "groups": list(by_talker.values()),
        "satellites": satellites,
        "reported_in_view": sum(group.get("satellites_in_view", 0) for group in by_talker.values()),
        "decoded_count": len(satellites),
    }


def compact_gsa(records):
    latest_by_talker = {}
    for item in records:
        if item.get("type") == "GSA":
            latest_by_talker[item.get("talker", "")] = item
    return list(latest_by_talker.values())


def compact_time(latest):
    zda = latest.get("ZDA") or {}
    rmc = latest.get("RMC") or {}
    gga = latest.get("GGA") or {}
    source = zda or rmc or gga
    return {
        "source": source.get("type", ""),
        "utc": source.get("utc", ""),
        "utc_time": source.get("utc_time", ""),
        "date": source.get("date", ""),
        "datetime_utc": source.get("datetime_utc", ""),
    }


def compact_rtk_summary(records, connected=False, error=""):
    latest = {}
    counts = {}
    for item in records:
        typ = item.get("type", "UNKNOWN")
        counts[typ] = counts.get(typ, 0) + 1
        latest[typ] = item

    fix = latest.get("GGA") or {}
    rmc = latest.get("RMC") or {}
    heading = latest.get("HEADINGA") or latest.get("THS") or latest.get("HDT") or latest.get("TRA") or {}
    gsa = compact_gsa(records)
    gsv = compact_gsv(records)
    gst = latest.get("GST") or {}
    vtg = latest.get("VTG") or {}
    zda = latest.get("ZDA") or {}
    gll = latest.get("GLL") or {}
    time_info = compact_time(latest)
    quality = str(fix.get("fix_quality", ""))
    rtk_fixed = quality == "4" or rmc.get("mode") == "R"
    rtk_float = quality == "5" or rmc.get("mode") == "F"
    position_source = fix or rmc or gll
    position_valid = bool(position_source.get("latitude") is not None and position_source.get("longitude") is not None)
    heading_valid = bool(heading.get("valid") or (heading.get("heading_deg") is not None and heading.get("type") in ("HDT", "TRA", "THS")))

    if rtk_fixed:
        state = "RTK 固定解"
        kind = ""
    elif rtk_float:
        state = "RTK 浮点解"
        kind = "warn"
    elif position_valid:
        state = fix.get("fix_quality_text") or rmc.get("mode_text") or "已定位"
        kind = "warn" if quality == "1" else ""
    elif connected and not records:
        state = "串口已打开，等待数据"
        kind = "warn"
    elif error:
        state = "连接异常"
        kind = "bad"
    else:
        state = "未连接"
        kind = "neutral"

    return {
        "state": state,
        "kind": kind,
        "connected": connected,
        "error": error,
        "counts": counts,
        "fix": fix,
        "rmc": rmc,
        "heading": heading,
        "gsa": gsa,
        "gsv": gsv,
        "gst": gst,
        "vtg": vtg,
        "zda": zda,
        "gll": gll,
        "time": time_info,
        "position_valid": position_valid,
        "rtk_fixed": rtk_fixed,
        "rtk_float": rtk_float,
        "heading_valid": heading_valid,
    }


def configure_serial_fd(fd, baudrate):
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[0] &= ~(termios.IXON | termios.IXOFF | termios.IXANY)
    attrs[1] = 0
    attrs[2] |= termios.CLOCAL | termios.CREAD
    attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
    attrs[2] |= termios.CS8
    if hasattr(termios, "CRTSCTS"):
        attrs[2] &= ~termios.CRTSCTS
    attrs[3] = 0
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 1
    speed = getattr(termios, f"B{baudrate}", termios.B115200)
    attrs[4] = speed
    attrs[5] = speed
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


class RtkMonitor:
    def __init__(self):
        self.lock = threading.Lock()
        self.thread = None
        self.stop_event = threading.Event()
        self.launch_process = None
        self.bridge_process = None
        self.log_handle = None
        self.port = ""
        self.baudrate = 115200
        self.connected = False
        self.error = ""
        self.node_status = {}
        self.raw_lines = deque(maxlen=80)
        self.records = deque(maxlen=80)
        self.last_sentence_at = None

    def start(self, port, baudrate):
        self.stop()
        with self.lock:
            self.port = port
            self.baudrate = int(baudrate)
            self.connected = False
            self.error = ""
            self.node_status = {}
            self.raw_lines.clear()
            self.records.clear()
            self.last_sentence_at = None
        self.stop_event.clear()
        self.launch_ros_node()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        self.stop_process(self.bridge_process)
        self.stop_process(self.launch_process)
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)
        self.thread = None
        self.bridge_process = None
        self.launch_process = None
        if self.log_handle:
            self.log_handle.flush()
            self.log_handle.close()
            self.log_handle = None
        with self.lock:
            self.connected = False

    def stop_process(self, proc):
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=4)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=2)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass

    def launch_ros_node(self):
        RTK_LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
        self.log_handle = RTK_LOG_FILE.open("a", encoding="utf-8")
        self.log_handle.write(
            "\n\n==== RTK ROS launch started "
            + time.strftime("%Y-%m-%d %H:%M:%S")
            + f" port={self.port} baudrate={self.baudrate} ====\n"
        )
        self.log_handle.flush()
        args = [
            "ros2",
            "launch",
            "odin_ros_driver",
            "gnss_rtk.launch.py",
            f"port:={self.port}",
            f"baudrate:={self.baudrate}",
            "namespace:=gnss",
        ]
        shell_cmd = ros_shell_prefix() + f"exec {shlex.join(args)}"
        self.launch_process = subprocess.Popen(
            ["bash", "-c", shell_cmd],
            cwd=str(ROS_WS),
            stdout=self.log_handle,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
            text=True,
            env=clean_ros_env(),
        )

    def start_bridge(self):
        bridge_code = r'''
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Bridge(Node):
    def __init__(self):
        super().__init__("odin_web_rtk_bridge")
        self.create_subscription(String, "/gnss/raw_sentence", self.raw_cb, 100)
        self.create_subscription(String, "/gnss/status", self.status_cb, 10)

    def emit(self, kind, data):
        print(json.dumps({"kind": kind, "data": data}, ensure_ascii=False), flush=True)

    def raw_cb(self, msg):
        self.emit("raw", msg.data)

    def status_cb(self, msg):
        self.emit("status", msg.data)


rclpy.init()
node = Bridge()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
'''
        shell_cmd = ros_shell_prefix() + "exec python3 -u -c " + shlex.quote(bridge_code)
        self.bridge_process = subprocess.Popen(
            ["bash", "-c", shell_cmd],
            cwd=str(ROS_WS),
            stdout=subprocess.PIPE,
            stderr=self.log_handle,
            preexec_fn=os.setsid,
            text=True,
            bufsize=1,
            env=clean_ros_env(),
        )

    def ros_process_running(self):
        return self.launch_process is not None and self.launch_process.poll() is None

    def run(self):
        while not self.stop_event.is_set():
            if not self.ros_process_running():
                code = None if self.launch_process is None else self.launch_process.poll()
                with self.lock:
                    self.connected = False
                    self.error = f"RTK ROS launch 已退出: {code}" if code is not None else "RTK ROS launch 未启动"
                time.sleep(0.5)
                continue
            if self.bridge_process is None or self.bridge_process.poll() is not None:
                self.start_bridge()
                time.sleep(0.2)
                continue
            line = self.bridge_process.stdout.readline() if self.bridge_process.stdout else ""
            if not line:
                time.sleep(0.05)
                continue
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            kind = event.get("kind")
            data = str(event.get("data", "")).strip()
            if kind == "raw" and data:
                record = parse_rtk_sentence(data)
                with self.lock:
                    self.raw_lines.append(data)
                    self.records.append(record)
                    self.last_sentence_at = time.time()
            elif kind == "status" and data:
                try:
                    status = json.loads(data)
                except json.JSONDecodeError:
                    status = {}
                with self.lock:
                    self.node_status = status
                    self.connected = bool(status.get("connected"))
                    self.error = str(status.get("error") or "")
        self.stop_process(self.bridge_process)
        self.bridge_process = None

    def snapshot(self):
        with self.lock:
            records = list(self.records)
            raw_lines = list(self.raw_lines)
            connected = self.connected
            error = self.error
            node_status = dict(self.node_status)
            last_sentence_at = self.last_sentence_at
            port = self.port
            baudrate = self.baudrate
            thread_running = bool(self.thread and self.thread.is_alive())
            launch_running = self.ros_process_running()
            launch_pid = self.launch_process.pid if launch_running else None
            bridge_pid = self.bridge_process.pid if self.bridge_process and self.bridge_process.poll() is None else None
        if launch_running and node_status and not error:
            error = str(node_status.get("error") or "")
        summary = compact_rtk_summary(records, connected=connected, error=error)
        return {
            "running": thread_running and launch_running,
            "source": "ros2",
            "topics": {
                "fix": "/gnss/fix",
                "heading": "/gnss/heading",
                "heading_deg": "/gnss/heading_deg",
                "status": "/gnss/status",
                "raw_sentence": "/gnss/raw_sentence",
            },
            "pid": launch_pid,
            "bridge_pid": bridge_pid,
            "log_file": str(RTK_LOG_FILE),
            "port": port,
            "baudrate": baudrate,
            "connected": connected,
            "last_sentence_age": None if last_sentence_at is None else max(0, time.time() - last_sentence_at),
            "node_status": node_status,
            "summary": summary,
            "records": records[-30:],
            "raw_lines": raw_lines[-40:],
        }


rtk_monitor = RtkMonitor()


def process_running(proc):
    return proc is not None and proc.poll() is None


def stop_process_group(proc, timeout=8):
    if not process_running(proc):
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=timeout)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=3)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass


def normalize_capture_dir(path):
    base = Path(path or CAPTURE_DIR).expanduser().resolve()
    base.mkdir(parents=True, exist_ok=True)
    return base


def bag_size_bytes(path):
    total = 0
    try:
        for item in Path(path).rglob("*"):
            if item.is_file():
                total += item.stat().st_size
    except OSError:
        return 0
    return total


def format_bytes(value):
    units = ["B", "KB", "MB", "GB", "TB"]
    size = float(value)
    for unit in units:
        if size < 1024 or unit == units[-1]:
            return f"{size:.1f} {unit}" if unit != "B" else f"{int(size)} B"
        size /= 1024
    return f"{int(value)} B"


def list_capture_bags(output_dir=None):
    base = normalize_capture_dir(output_dir or load_state().get("capture", {}).get("output_dir"))
    bags = []
    for metadata in sorted(base.glob("*/metadata.yaml"), key=lambda p: p.stat().st_mtime, reverse=True):
        bag_dir = metadata.parent
        try:
            stat = bag_dir.stat()
        except OSError:
            continue
        size_bytes = bag_size_bytes(bag_dir)
        topic_counts = bag_topic_counts(bag_dir)
        manifest = bag_dir / CAPTURE_MANIFEST
        bags.append({
            "name": bag_dir.name,
            "path": str(bag_dir),
            "mtime": stat.st_mtime,
            "size_bytes": size_bytes,
            "size": format_bytes(size_bytes),
            "topic_counts": topic_counts,
            "has_manifest": manifest.exists(),
            "cloud_messages": sum(
                count for topic, count in topic_counts.items()
                if "cloud" in topic.lower()
            ),
            "meta_messages": topic_counts.get("/odin_rtk/bound_meta", 0),
            "odom_messages": sum(
                count for topic, count in topic_counts.items()
                if "odometry" in topic or topic.endswith("/wiwc")
            ),
            "imu_messages": sum(
                count for topic, count in topic_counts.items()
                if topic.endswith("/imu")
            ),
            "image_messages": sum(
                count for topic, count in topic_counts.items()
                if "/image" in topic
            ),
            "rtk_messages": sum(
                count for topic, count in topic_counts.items()
                if topic.startswith("/gnss/")
            ),
        })
    return bags[:20]


def bag_topic_counts(bag_dir):
    metadata = Path(bag_dir) / "metadata.yaml"
    if not metadata.exists():
        return {}
    try:
        import yaml
        data = yaml.safe_load(metadata.read_text(encoding="utf-8")) or {}
    except Exception:
        return {}
    info = data.get("rosbag2_bagfile_information", {})
    counts = {}
    for item in info.get("topics_with_message_count", []) or []:
        topic = item.get("topic_metadata", {}).get("name")
        if topic:
            counts[topic] = int(item.get("message_count", 0) or 0)
    return counts


def bag_db_files(bag_dir):
    return sorted(Path(bag_dir).glob("*.db3"))


def db3_topic_summary(db_path):
    con = None
    try:
        con = sqlite3.connect(f"file:{db_path}?mode=ro&immutable=1", uri=True)
        topics = {
            int(row[0]): {"name": row[1], "type": row[2], "count": 0}
            for row in con.execute("select id, name, type from topics order by id")
        }
        for topic_id, count in con.execute("select topic_id, count(*) from messages group by topic_id"):
            if int(topic_id) in topics:
                topics[int(topic_id)]["count"] = int(count or 0)
        time_row = con.execute("select min(timestamp), max(timestamp), count(*) from messages").fetchone()
        return {
            "ok": True,
            "topics": list(topics.values()),
            "start_ns": int(time_row[0] or 0) if time_row else 0,
            "end_ns": int(time_row[1] or 0) if time_row else 0,
            "messages": int(time_row[2] or 0) if time_row else 0,
            "error": "",
        }
    except Exception as exc:
        return {
            "ok": False,
            "topics": [],
            "start_ns": 0,
            "end_ns": 0,
            "messages": 0,
            "error": str(exc),
        }
    finally:
        if con is not None:
            con.close()


def scan_fusion_bags(output_dir=None):
    base = normalize_capture_dir(output_dir or load_state().get("capture", {}).get("output_dir"))
    bags = []
    for bag_dir in sorted([item for item in base.iterdir() if item.is_dir()], key=lambda p: p.stat().st_mtime, reverse=True):
        if bag_dir.name == FUSION_OUTPUT_DIR.name:
            continue
        db_files = bag_db_files(bag_dir)
        if not db_files:
            continue
        topic_map = {}
        start_ns = 0
        end_ns = 0
        messages = 0
        errors = []
        readable = False
        for db_file in db_files:
            summary = db3_topic_summary(db_file)
            if not summary["ok"]:
                errors.append(f"{db_file.name}: {summary['error']}")
                continue
            readable = True
            messages += summary["messages"]
            if summary["start_ns"]:
                start_ns = summary["start_ns"] if not start_ns else min(start_ns, summary["start_ns"])
            if summary["end_ns"]:
                end_ns = max(end_ns, summary["end_ns"])
            for topic in summary["topics"]:
                item = topic_map.setdefault(topic["name"], {"name": topic["name"], "type": topic["type"], "count": 0})
                item["count"] += int(topic.get("count") or 0)
        pointcloud_topics = [
            topic for topic in topic_map.values()
            if topic["type"] == "sensor_msgs/msg/PointCloud2" and topic["count"] > 0
        ]
        pointcloud_topics.sort(key=lambda topic: (
            0 if topic["name"].endswith("/cloud_render") else 1 if topic["name"].endswith("/bound_cloud") else 2,
            topic["name"],
        ))
        bags.append({
            "name": bag_dir.name,
            "path": str(bag_dir),
            "size": format_bytes(bag_size_bytes(bag_dir)),
            "size_bytes": bag_size_bytes(bag_dir),
            "has_metadata": (bag_dir / "metadata.yaml").exists(),
            "has_manifest": (bag_dir / CAPTURE_MANIFEST).exists(),
            "readable": readable,
            "error": "; ".join(errors),
            "start_ns": start_ns,
            "end_ns": end_ns,
            "duration_sec": (end_ns - start_ns) / 1e9 if start_ns and end_ns and end_ns >= start_ns else 0,
            "messages": messages,
            "topics": sorted(topic_map.values(), key=lambda topic: topic["name"]),
            "pointcloud_topics": pointcloud_topics,
        })
    return bags


def fusion_bag_status(output_dir=None):
    bags = scan_fusion_bags(output_dir)
    outputs = []
    FUSION_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    for item in sorted(FUSION_OUTPUT_DIR.glob("*.ply"), key=lambda p: p.stat().st_mtime, reverse=True)[:20]:
        outputs.append({
            "name": item.name,
            "path": str(item),
            "size": format_bytes(item.stat().st_size),
            "mtime": item.stat().st_mtime,
        })
    return {
        "bags": bags,
        "outputs": outputs,
        "default_max_points": 60000,
        "max_points_limit": FUSION_MAX_POINTS,
    }


def resolve_fusion_bag_path(bag_path):
    base = normalize_capture_dir(load_state().get("capture", {}).get("output_dir"))
    bag = Path(str(bag_path or "")).expanduser().resolve()
    if not bag_path:
        raise RuntimeError("没有选择点云 bag")
    if not (bag == base or base in bag.parents):
        raise RuntimeError("只能读取采集目录内的 bag")
    if not bag.is_dir() or not bag_db_files(bag):
        raise RuntimeError(f"不是有效的 rosbag 数据目录: {bag}")
    return bag


def safe_fusion_topic(bag_dir, topic):
    topic = safe_capture_topic(topic)
    for db_file in bag_db_files(bag_dir):
        summary = db3_topic_summary(db_file)
        if not summary["ok"]:
            continue
        for item in summary["topics"]:
            if item["name"] == topic and item["type"] == "sensor_msgs/msg/PointCloud2" and item["count"] > 0:
                return topic
    raise RuntimeError(f"{bag_dir.name} 中没有可用点云 topic: {topic}")


def pointcloud_message_rows(bag_dir, topic):
    rows = []
    for db_file in bag_db_files(bag_dir):
        con = None
        try:
            con = sqlite3.connect(f"file:{db_file}?mode=ro&immutable=1", uri=True)
            topic_row = con.execute(
                "select id, type from topics where name = ?",
                (topic,),
            ).fetchone()
            if not topic_row or topic_row[1] != "sensor_msgs/msg/PointCloud2":
                continue
            topic_id = int(topic_row[0])
            for timestamp, data in con.execute(
                "select timestamp, data from messages where topic_id = ? order by timestamp",
                (topic_id,),
            ):
                rows.append((int(timestamp), bytes(data), db_file.name))
        finally:
            if con is not None:
                con.close()
    rows.sort(key=lambda item: item[0])
    return rows


def choose_cloud_row(rows, target_ns=None, fraction=0.5):
    if not rows:
        raise RuntimeError("所选 topic 没有点云消息")
    if target_ns:
        return min(rows, key=lambda item: abs(item[0] - target_ns))
    index = int(max(0, min(len(rows) - 1, round((len(rows) - 1) * float(fraction)))))
    return rows[index]


def pointcloud_to_arrays(serialized, max_points):
    import numpy as np
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2

    msg = deserialize_message(serialized, PointCloud2)
    field_names = [field.name for field in msg.fields]
    wanted = ["x", "y", "z"]
    has_rgb = "rgb" in field_names
    if has_rgb:
        wanted.append("rgb")
    arr = point_cloud2.read_points(msg, field_names=wanted, skip_nans=True)
    count = int(arr.shape[0])
    if count <= 0:
        raise RuntimeError("点云消息为空")
    sample_count = max(1, min(int(max_points), count))
    if count > sample_count:
        indices = np.linspace(0, count - 1, sample_count, dtype=np.int64)
        arr = arr[indices]
    xyz = np.column_stack((arr["x"], arr["y"], arr["z"])).astype(np.float32)
    colors = np.full((xyz.shape[0], 3), 188, dtype=np.uint8)
    if has_rgb:
        packed = arr["rgb"].copy().view(np.uint32)
        colors[:, 0] = ((packed >> 16) & 255).astype(np.uint8)
        colors[:, 1] = ((packed >> 8) & 255).astype(np.uint8)
        colors[:, 2] = (packed & 255).astype(np.uint8)
    return msg, xyz, colors, count


def euler_transform_matrix(transform):
    import numpy as np

    tx = float(transform.get("x", 0) or 0)
    ty = float(transform.get("y", 0) or 0)
    tz = float(transform.get("z", 0) or 0)
    roll = math.radians(float(transform.get("roll", 0) or 0))
    pitch = math.radians(float(transform.get("pitch", 0) or 0))
    yaw = math.radians(float(transform.get("yaw", 0) or 0))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float32)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float32)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float32)
    rotation = rz @ ry @ rx
    translation = np.array([tx, ty, tz], dtype=np.float32)
    return rotation, translation


def transform_points(points, transform):
    rotation, translation = euler_transform_matrix(transform)
    return (points @ rotation.T) + translation


def cloud_bounds(points):
    import numpy as np

    if points.size == 0:
        return {"min": [0, 0, 0], "max": [0, 0, 0], "center": [0, 0, 0], "extent": 1}
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    center = (mins + maxs) * 0.5
    extent = float(max((maxs - mins).max(), 0.001))
    return {
        "min": [round(float(v), 4) for v in mins],
        "max": [round(float(v), 4) for v in maxs],
        "center": [round(float(v), 4) for v in center],
        "extent": round(extent, 4),
    }


def write_ascii_ply(path, points, colors):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii") as handle:
        handle.write("ply\nformat ascii 1.0\n")
        handle.write(f"element vertex {len(points)}\n")
        handle.write("property float x\nproperty float y\nproperty float z\n")
        handle.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        handle.write("end_header\n")
        for point, color in zip(points, colors):
            handle.write(
                f"{float(point[0]):.5f} {float(point[1]):.5f} {float(point[2]):.5f} "
                f"{int(color[0])} {int(color[1])} {int(color[2])}\n"
            )


def fusion_preview_payload(body):
    import numpy as np

    bag_a = resolve_fusion_bag_path(body.get("bag_a"))
    bag_b = resolve_fusion_bag_path(body.get("bag_b"))
    if bag_a == bag_b:
        raise RuntimeError("A/B 不能选择同一个采集包，请从 captures 中选择两个不同数据源")
    topic_a = safe_fusion_topic(bag_a, body.get("topic_a"))
    topic_b = safe_fusion_topic(bag_b, body.get("topic_b"))
    max_points = max(1000, min(FUSION_MAX_POINTS, int(body.get("max_points") or 60000)))
    rows_a = pointcloud_message_rows(bag_a, topic_a)
    rows_b = pointcloud_message_rows(bag_b, topic_b)
    start = max(rows_a[0][0], rows_b[0][0])
    end = min(rows_a[-1][0], rows_b[-1][0])
    sync_mode = str(body.get("sync_mode") or "overlap_mid")
    target_ns = None
    sync_note = ""
    if sync_mode == "overlap_mid" and start <= end:
        target_ns = (start + end) // 2
        sync_note = "使用两段 bag 时间重叠区中点"
    elif sync_mode == "start":
        target_ns = max(rows_a[0][0], rows_b[0][0])
        sync_note = "使用较晚开始时间附近的帧"
    elif sync_mode == "end":
        target_ns = min(rows_a[-1][0], rows_b[-1][0])
        sync_note = "使用较早结束时间附近的帧"
    else:
        sync_note = "两段 bag 没有重叠时间，已各取中间帧"
    row_a = choose_cloud_row(rows_a, target_ns)
    row_b = choose_cloud_row(rows_b, target_ns)
    per_cloud_limit = max(500, max_points // 2)
    msg_a, points_a, colors_a, original_a = pointcloud_to_arrays(row_a[1], per_cloud_limit)
    msg_b, points_b, colors_b, original_b = pointcloud_to_arrays(row_b[1], per_cloud_limit)
    transform = body.get("transform") if isinstance(body.get("transform"), dict) else {}
    points_b_world = transform_points(points_b, transform)
    points = np.vstack((points_a, points_b_world)).astype(np.float32)
    colors = np.vstack((colors_a, colors_b)).astype(np.uint8)
    bounds = cloud_bounds(points)
    output_path = ""
    if body.get("save_ply"):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        output_path = str(FUSION_OUTPUT_DIR / f"fusion_preview_{stamp}.ply")
        write_ascii_ply(Path(output_path), points, colors)
    return {
        "ok": True,
        "sync_note": sync_note,
        "target_ns": target_ns,
        "bag_a": str(bag_a),
        "bag_b": str(bag_b),
        "topic_a": topic_a,
        "topic_b": topic_b,
        "selected_a_ns": row_a[0],
        "selected_b_ns": row_b[0],
        "delta_ms": round(abs(row_a[0] - row_b[0]) / 1e6, 3),
        "frame_a": msg_a.header.frame_id,
        "frame_b": msg_b.header.frame_id,
        "original_points_a": original_a,
        "original_points_b": original_b,
        "points_a": len(points_a),
        "points_b": len(points_b),
        "points_total": len(points),
        "bounds": bounds,
        "output_path": output_path,
        "points": np.round(points, 4).reshape(-1).tolist(),
        "colors": colors.reshape(-1).tolist(),
    }


def safe_capture_topic(topic):
    topic = str(topic or "").strip()
    if not topic.startswith("/") or not re.match(r"^[A-Za-z0-9_/-]+$", topic):
        raise ValueError("无效的 ROS topic")
    return topic


def resolve_capture_bag_path(bag_path, output_dir=None):
    base = normalize_capture_dir(output_dir or load_state().get("capture", {}).get("output_dir"))
    selected = str(bag_path or "").strip()
    if not selected:
        raise RuntimeError("没有选择要操作的 bag")
    bag = Path(selected).expanduser().resolve()
    if not (bag == base or base in bag.parents):
        raise RuntimeError("只能管理采集目录内的 bag")
    if not (bag / "metadata.yaml").exists():
        raise RuntimeError(f"不是有效的 rosbag 目录: {bag}")
    return bag


def slot_from_odin_topic(topic):
    match = re.match(r"^/(odin_[ab])/", str(topic or ""))
    return match.group(1) if match else ""


def capture_slots_for_context(cloud_topic, state=None, available_topics=None):
    state = state or load_state()
    topics = set(available_topics if available_topics is not None else [])
    observed_slots = []
    for topic in sorted(topics):
        slot = slot_from_odin_topic(topic)
        if slot:
            observed_slots.append(slot)
    slots = list(observed_slots)
    if not slots:
        try:
            plan = launch_plan(state, scan_odin_devices(), state.get("launch_target"))
            slots.extend(plan["effective_slots"])
        except Exception:
            pass
    try:
        selected_slot = slot_from_odin_topic(cloud_topic)
        if selected_slot:
            slots.append(selected_slot)
    except Exception:
        pass
    return [slot for slot in SLOTS if slot in slots]


def odin_capture_topics_for_slots(slots):
    topics = []
    for slot in slots:
        topics.extend(f"/{slot}/{suffix}" for suffix in ODIN_CAPTURE_SUFFIXES)
    return topics


def default_capture_topics(cloud_topic, state=None, available_topics=None):
    slots = capture_slots_for_context(cloud_topic, state, available_topics)
    topics = []
    if cloud_topic and cloud_topic.startswith("/"):
        topics.append(cloud_topic)
    topics.extend(odin_capture_topics_for_slots(slots))
    topics.extend([*CAPTURE_SUPPORT_TOPICS, *RTK_CAPTURE_TOPICS])
    return list(dict.fromkeys(topics))


def copy_if_exists(src, dest):
    src = Path(src).expanduser()
    if not src.exists() or not src.is_file():
        return False
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dest)
    return True


def write_capture_sidecars(bag_path, state, cloud_topic, topics, available_topics):
    bag = Path(bag_path)
    deadline = time.time() + 4
    while time.time() < deadline and not bag.exists():
        time.sleep(0.1)
    if not bag.exists():
        return
    slots = capture_slots_for_context(cloud_topic, state, available_topics)
    sidecar_dir = bag / "odin_metadata"
    sidecar_dir.mkdir(parents=True, exist_ok=True)
    copied = []
    for slot in slots:
        slot_state = state.get(slot, {})
        cfg = slot_state.get("config")
        if cfg and copy_if_exists(cfg, sidecar_dir / f"{slot}_control_command.yaml"):
            copied.append(str(sidecar_dir / f"{slot}_control_command.yaml"))
        calib = Path(slot_state.get("calib_dir", "")) / "calib.yaml"
        if copy_if_exists(calib, sidecar_dir / f"{slot}_calib.yaml"):
            copied.append(str(sidecar_dir / f"{slot}_calib.yaml"))
    common_calib = DRIVER_DIR / "config" / "calib.yaml"
    if copy_if_exists(common_calib, sidecar_dir / "driver_calib.yaml"):
        copied.append(str(sidecar_dir / "driver_calib.yaml"))
    manifest = {
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "profile": "full_odin_fusion_capture",
        "primary_cloud_topic": cloud_topic,
        "slots": slots,
        "record_topics": topics,
        "available_topics_at_start": sorted(available_topics),
        "sidecar_files": copied,
        "notes": [
            "This capture mirrors the useful parts of Odin OLX recording in ROS bag form.",
            "It records point clouds, odometry, IMU, images, TF, RTK/GNSS topics, and binding metadata when available.",
        ],
    }
    (bag / CAPTURE_MANIFEST).write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")


class CaptureManager:
    def __init__(self):
        self.lock = threading.Lock()
        self.bind_process = None
        self.record_process = None
        self.play_process = None
        self.rviz_process = None
        self.log_handle = None
        self.started_at = None
        self.play_started_at = None
        self.bag_path = ""
        self.play_bag_path = ""
        self.cloud_topic = ""
        self.topics = []

    def start(self, cloud_topic, output_dir):
        with self.lock:
            if process_running(self.record_process):
                raise RuntimeError("采集已经在运行")
        state = load_state()
        available_topics = current_topic_names()
        cloud_topic = resolve_capture_cloud_topic(cloud_topic, state, available_topics)
        if cloud_topic not in available_topics:
            cloud_candidates = sorted(topic for topic in available_topics if "cloud" in topic.lower())
            hint = "，当前可用点云: " + " ".join(cloud_candidates) if cloud_candidates else "，当前没有检测到点云 topic"
            raise RuntimeError(f"采集前未发现 {cloud_topic}{hint}")
        base = normalize_capture_dir(output_dir)
        bag_path = base / ("odin_rtk_" + time.strftime("%Y%m%d_%H%M%S"))
        topics = default_capture_topics(cloud_topic, state, available_topics)
        CAPTURE_LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
        log_handle = CAPTURE_LOG_FILE.open("a", encoding="utf-8")
        log_handle.write(
            "\n\n==== Odin RTK capture started "
            + time.strftime("%Y-%m-%d %H:%M:%S")
            + f" cloud_topic={cloud_topic} bag={bag_path} ====\n"
        )
        log_handle.flush()

        bind_args = [
            "ros2",
            "launch",
            "odin_ros_driver",
            "odin_rtk_capture.launch.py",
            f"cloud_topic:={cloud_topic}",
            "meta_topic:=/odin_rtk/bound_meta",
            "bound_cloud_topic:=/odin_rtk/bound_cloud",
        ]
        record_args = ["ros2", "bag", "record", "-o", str(bag_path), *topics]
        env = clean_ros_env()
        bind_proc = subprocess.Popen(
            ["bash", "-c", ros_shell_prefix() + f"exec {shlex.join(bind_args)}"],
            cwd=str(ROS_WS),
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
            text=True,
            env=env,
        )
        time.sleep(0.5)
        record_proc = subprocess.Popen(
            ["bash", "-c", ros_shell_prefix() + f"exec {shlex.join(record_args)}"],
            cwd=str(ROS_WS),
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
            text=True,
            env=env,
        )
        try:
            write_capture_sidecars(bag_path, state, cloud_topic, topics, available_topics)
        except Exception as exc:
            log_handle.write(f"capture sidecar write failed: {exc}\n")
            log_handle.flush()

        with self.lock:
            self.stop_owned_locked(close_log=False)
            self.bind_process = bind_proc
            self.record_process = record_proc
            self.log_handle = log_handle
            self.started_at = time.time()
            self.bag_path = str(bag_path)
            self.cloud_topic = cloud_topic
            self.topics = topics
        state.setdefault("capture", {})["cloud_topic"] = cloud_topic
        state["capture"]["output_dir"] = str(base)
        state["capture"]["last_bag"] = str(bag_path)
        save_state(state)
        return self.status()

    def stop_owned_locked(self, close_log=True):
        stop_process_group(self.record_process)
        stop_process_group(self.bind_process)
        self.record_process = None
        self.bind_process = None
        self.started_at = None
        if close_log and self.log_handle:
            self.log_handle.flush()
            self.log_handle.close()
            self.log_handle = None

    def stop(self):
        with self.lock:
            self.stop_owned_locked()
        return self.status()

    def start_playback(self, bag_path=None, loop=False, with_rviz=True):
        with self.lock:
            if process_running(self.play_process):
                raise RuntimeError("回放已经在运行")
        state = load_state()
        selected = str(bag_path or state.get("capture", {}).get("last_bag") or "").strip()
        if not selected:
            bags = list_capture_bags(state.get("capture", {}).get("output_dir"))
            selected = bags[0]["path"] if bags else ""
        bag = Path(selected).expanduser().resolve()
        if not (bag / "metadata.yaml").exists():
            raise RuntimeError(f"找不到可回放的 rosbag: {bag}")
        args = ["ros2", "bag", "play", str(bag)]
        if loop:
            args.append("--loop")
        rviz_args = [
            "rviz2",
            "-d",
            str(DRIVER_DIR / "config" / "dual_odin_ros2.rviz"),
        ]
        CAPTURE_LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
        if self.log_handle is None:
            self.log_handle = CAPTURE_LOG_FILE.open("a", encoding="utf-8")
        self.log_handle.write(
            "\n\n==== Odin RTK playback started "
            + time.strftime("%Y-%m-%d %H:%M:%S")
            + f" bag={bag} ====\n"
        )
        self.log_handle.flush()
        env = clean_ros_env()
        play_proc = subprocess.Popen(
            ["bash", "-c", ros_shell_prefix() + f"exec {shlex.join(args)}"],
            cwd=str(ROS_WS),
            stdout=self.log_handle,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
            text=True,
            env=env,
        )
        rviz_proc = None
        if with_rviz:
            rviz_proc = subprocess.Popen(
                ["bash", "-c", ros_shell_prefix() + f"exec {shlex.join(rviz_args)}"],
                cwd=str(ROS_WS),
                stdout=self.log_handle,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
                text=True,
                env=env,
            )
        with self.lock:
            self.play_process = play_proc
            self.rviz_process = rviz_proc
            self.play_started_at = time.time()
            self.play_bag_path = str(bag)
        return self.status()

    def stop_playback(self):
        with self.lock:
            stop_process_group(self.play_process)
            stop_process_group(self.rviz_process)
            self.play_process = None
            self.rviz_process = None
            self.play_started_at = None
            self.play_bag_path = ""
        return self.status()

    def delete_bag(self, bag_path):
        bag = resolve_capture_bag_path(bag_path)
        with self.lock:
            if process_running(self.record_process) and Path(self.bag_path).resolve() == bag:
                raise RuntimeError("当前 bag 正在采集，先停止采集")
            if process_running(self.play_process) and Path(self.play_bag_path).resolve() == bag:
                raise RuntimeError("当前 bag 正在回放，先停止回放")
            if self.bag_path and Path(self.bag_path).expanduser().resolve() == bag:
                self.bag_path = ""
            if self.play_bag_path and Path(self.play_bag_path).expanduser().resolve() == bag:
                self.play_bag_path = ""
        shutil.rmtree(bag)
        state = load_state()
        capture = state.setdefault("capture", {})
        if Path(str(capture.get("last_bag", "") or "/")).expanduser().resolve() == bag:
            capture["last_bag"] = ""
            save_state(state)
        return self.status()

    def status(self):
        state = load_state()
        available_cloud_topics = []
        auto_cloud_topic = ""
        topic_error = ""
        available_topics = set()
        try:
            available_topics = current_topic_names()
            available_cloud_topics = capture_cloud_candidates(available_topics, state)
            auto_cloud_topic = resolve_capture_cloud_topic("__auto__", state, available_topics)
        except Exception as exc:
            topic_error = str(exc)
        with self.lock:
            recording = process_running(self.record_process)
            binding = process_running(self.bind_process)
            playing = process_running(self.play_process)
            rviz_running = process_running(self.rviz_process)
            started_at = self.started_at
            play_started_at = self.play_started_at
            bag_path = self.bag_path or state.get("capture", {}).get("last_bag", "")
            play_bag_path = self.play_bag_path
            cloud_topic = self.cloud_topic or state.get("capture", {}).get("cloud_topic", "__auto__")
            if cloud_topic == "__auto__" and auto_cloud_topic:
                cloud_topic = auto_cloud_topic
            topics = list(self.topics or default_capture_topics(cloud_topic, state, available_topics)) if cloud_topic.startswith("/") else []
            live_record_topics = [topic for topic in topics if topic in available_topics]
            missing_record_topics = [topic for topic in topics if topic not in available_topics]
            record_pid = self.record_process.pid if recording else None
            bind_pid = self.bind_process.pid if binding else None
            play_pid = self.play_process.pid if playing else None
            rviz_pid = self.rviz_process.pid if rviz_running else None
        output_dir = state.get("capture", {}).get("output_dir", str(CAPTURE_DIR))
        return {
            "recording": recording,
            "binding": binding,
            "playing": playing,
            "record_pid": record_pid,
            "bind_pid": bind_pid,
            "play_pid": play_pid,
            "rviz_running": rviz_running,
            "rviz_pid": rviz_pid,
            "elapsed_sec": None if not started_at or not recording else max(0, time.time() - started_at),
            "play_elapsed_sec": None if not play_started_at or not playing else max(0, time.time() - play_started_at),
            "bag_path": bag_path,
            "play_bag_path": play_bag_path,
            "cloud_topic": cloud_topic,
            "available_cloud_topics": available_cloud_topics,
            "auto_cloud_topic": auto_cloud_topic,
            "topic_error": topic_error,
            "topics": topics,
            "live_record_topics": live_record_topics,
            "missing_record_topics": missing_record_topics,
            "output_dir": output_dir,
            "bags": list_capture_bags(output_dir),
            "log_file": str(CAPTURE_LOG_FILE),
        }

    def shutdown(self):
        self.stop()
        self.stop_playback()
        with self.lock:
            if self.log_handle:
                self.log_handle.flush()
                self.log_handle.close()
                self.log_handle = None


capture_manager = CaptureManager()


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
    return subprocess.run(["bash", "-c", probe], cwd=str(HOME), env=clean_ros_env()).returncode == 0


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
    return dedupe_odin_bindings(state)


def clear_odin_binding(state, slot):
    state[slot]["usb_bus"] = ""
    state[slot]["usb_addr"] = ""
    state[slot]["serial"] = ""
    return state


def preferred_slot_for_duplicate(state):
    a_serial = str(state.get("odin_a", {}).get("serial", "")).strip()
    b_serial = str(state.get("odin_b", {}).get("serial", "")).strip()
    if b_serial and b_serial == DEFAULT_STATE["odin_b"]["serial"]:
        return "odin_b"
    if a_serial and a_serial == DEFAULT_STATE["odin_a"]["serial"]:
        return "odin_a"
    return "odin_a"


def dedupe_odin_bindings(state):
    a = state.get("odin_a", {})
    b = state.get("odin_b", {})
    duplicate_serial = (
        str(a.get("serial", "")).strip()
        and str(a.get("serial", "")).strip() == str(b.get("serial", "")).strip()
    )
    duplicate_usb = (
        str(a.get("usb_bus", "")).strip()
        and str(a.get("usb_addr", "")).strip()
        and str(a.get("usb_bus", "")).strip() == str(b.get("usb_bus", "")).strip()
        and str(a.get("usb_addr", "")).strip() == str(b.get("usb_addr", "")).strip()
    )
    if duplicate_serial or duplicate_usb:
        keep = preferred_slot_for_duplicate(state)
        clear_odin_binding(state, "odin_b" if keep == "odin_a" else "odin_a")
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
    serials = [
        str(state.get(slot, {}).get("serial", "")).strip()
        for slot in SLOTS
        if str(state.get(slot, {}).get("serial", "")).strip()
    ]
    duplicate_serial = len(serials) == 2 and serials[0] == serials[1]
    plan = launch_plan(state, devices, state.get("launch_target"))
    matches = plan["matches"]
    a_online = matches["odin_a"] is not None
    b_online = matches["odin_b"] is not None
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
        f"odin_a_config:={state['odin_a']['config']}",
        f"odin_b_config:={state['odin_b']['config']}",
        f"odin_a_calib_dir:={state['odin_a']['calib_dir']}",
        f"odin_b_calib_dir:={state['odin_b']['calib_dir']}",
    ]
    if enable_a:
        args.extend([
            f"odin_a_usb_bus:={state['odin_a']['usb_bus']}",
            f"odin_a_usb_addr:={state['odin_a']['usb_addr']}",
        ])
    if enable_b:
        args.extend([
            f"odin_b_usb_bus:={state['odin_b']['usb_bus']}",
            f"odin_b_usb_addr:={state['odin_b']['usb_addr']}",
        ])
    shell_cmd = (
        "source /opt/ros/humble/setup.bash && "
        f"source {shlex.quote(str(ROS_WS / 'install' / 'setup.bash'))} && "
        f"exec {shlex.join(args)}"
    )
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
        ["bash", "-c", shell_cmd],
        cwd=str(ROS_WS),
        stdout=launch_log_handle,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
        text=True,
        env=clean_ros_env(),
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


def list_topics(max_age=1.5):
    with topic_cache_lock:
        cached = topic_cache["data"]
        if cached and time.time() - topic_cache["updated_at"] <= max_age:
            return dict(cached)
    if not ros2_command_available("topic"):
        data = {"available": False, "topics": [], "error": "ros-humble-ros2topic is not installed"}
        with topic_cache_lock:
            topic_cache["updated_at"] = time.time()
            topic_cache["data"] = data
        return dict(data)
    cmd = ros_shell_prefix() + "timeout 5s ros2 topic list"
    proc = subprocess.run(
        ["bash", "-c", cmd],
        cwd=str(ROS_WS),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        env=clean_ros_env(),
    )
    topics = [line.strip() for line in proc.stdout.splitlines() if line.strip()]
    data = {"available": True, "topics": topics, "error": proc.stderr.strip()}
    with topic_cache_lock:
        topic_cache["updated_at"] = time.time()
        topic_cache["data"] = data
    return dict(data)


def current_topic_names():
    data = list_topics()
    if not data.get("available"):
        raise RuntimeError(data.get("error") or "ros2 topic 不可用")
    return set(data.get("topics") or [])


def preferred_capture_topics_for_slots(slots):
    topics = []
    for slot in slots:
        if slot not in SLOTS:
            continue
        prefix = "/" + slot
        topics.extend([
            f"{prefix}/odin1/cloud_render",
            f"{prefix}/odin1/cloud_slam",
            f"{prefix}/odin1/cloud_raw",
        ])
    return topics


def capture_cloud_candidates(available_topics=None, state=None):
    topics = set(available_topics if available_topics is not None else current_topic_names())
    state = state or load_state()
    plan = launch_plan(state, scan_odin_devices(), state.get("launch_target"))
    preferred = preferred_capture_topics_for_slots(plan["effective_slots"])
    fallback = [
        "/odin_a/odin1/cloud_render",
        "/odin_b/odin1/cloud_render",
        "/odin_a/odin1/cloud_slam",
        "/odin_b/odin1/cloud_slam",
        "/odin_a/odin1/cloud_raw",
        "/odin_b/odin1/cloud_raw",
    ]
    discovered = sorted(
        topic for topic in topics
        if re.match(r"^/odin_[ab]/", topic) and "cloud" in topic.lower()
    )
    ordered = []
    for topic in [*preferred, *fallback, *discovered]:
        if topic in topics and topic not in ordered:
            ordered.append(topic)
    return ordered


def resolve_capture_cloud_topic(requested_topic, state=None, available_topics=None):
    state = state or load_state()
    topics = set(available_topics if available_topics is not None else current_topic_names())
    requested = str(requested_topic or "").strip()
    candidates = capture_cloud_candidates(topics, state)
    if requested and requested != "__auto__":
        requested = safe_capture_topic(requested)
        if requested in topics:
            return requested
    if candidates:
        return candidates[0]
    raise RuntimeError("采集前没有检测到 Odin 点云 topic，请先启动 Odin 并等待点云发布")


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


def rtk_status_payload():
    state = load_state()
    serial_devices = scan_serial_devices()
    port = select_default_rtk_port(state, serial_devices)
    baudrate = str(state.get("rtk", {}).get("baudrate", "115200"))
    selected = find_serial_device_for_port(port, serial_devices)
    if state.setdefault("rtk", {}).get("port") != port:
        state["rtk"]["port"] = port
        save_state(state)
    monitor = rtk_monitor.snapshot()
    monitor_port_missing = bool(monitor.get("port")) and not Path(monitor["port"]).exists()
    monitor_port_changed = bool(port) and not same_serial_port(monitor.get("port", ""), port)
    if monitor.get("running") and selected and (monitor_port_missing or monitor_port_changed):
        rtk_monitor.start(port, baudrate)
        time.sleep(0.1)
        monitor = rtk_monitor.snapshot()
    return {
        "state": {
            "port": port,
            "baudrate": baudrate,
        },
        "devices": serial_devices,
        "selected": selected,
        "ready": bool(selected and selected.get("can_read") and selected.get("can_write")),
        "permission_blocked": bool(selected and not (selected.get("can_read") and selected.get("can_write"))),
        "system": {
            "modemmanager": service_state("ModemManager"),
            "modemmanager_candidate": bool(selected and udev_properties(selected["devnode"]).get("ID_MM_CANDIDATE") == "1"),
        },
        "monitor": monitor,
    }


RTK_OUTPUT_COMMANDS = [
    "#AGNGGA COM1 1",
    "#AGNRMC COM1 1",
    "#AGPGSA COM1 1",
    "#AGPGSV COM1 1",
    "#AGPGST COM1 1",
    "#AGPZDA COM1 1",
    "#AGPVTG COM1 1",
    "#AGPGLL COM1 1",
    "#AGPGGA COM1 1",
    "#AGPRMC COM1 1",
    "#AGPHDT COM1 1",
    "#AGPTHS COM1 1",
    "#AGPTRA COM1 1",
]


def configure_rtk_output(port, baudrate):
    real = Path(port).resolve()
    if not str(real).startswith("/dev/tty"):
        raise RuntimeError("不是有效的串口设备")
    if not real.exists():
        raise RuntimeError(f"串口不存在: {real}")

    fd = os.open(str(real), os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    preview = []
    try:
        configure_serial_fd(fd, int(baudrate))
        payload = ("\r\n".join(RTK_OUTPUT_COMMANDS) + "\r\n").encode("ascii")
        os.write(fd, payload)
        deadline = time.time() + 2.0
        buffer = bytearray()
        while time.time() < deadline and len(preview) < 12:
            readable, _, _ = select.select([fd], [], [], 0.2)
            if fd not in readable:
                continue
            data = os.read(fd, 4096)
            if not data:
                continue
            buffer.extend(data)
            while b"\n" in buffer and len(preview) < 12:
                raw_line, _, rest = buffer.partition(b"\n")
                buffer = bytearray(rest)
                line = raw_line.decode("ascii", errors="replace").strip()
                if line:
                    preview.append(line)
    finally:
        os.close(fd)

    return {
        "ok": True,
        "devnode": str(real),
        "commands": RTK_OUTPUT_COMMANDS,
        "preview": preview,
    }


def install_rtk_probe_guard():
    rule = (
        'SUBSYSTEM=="tty", ATTRS{idVendor}=="04d9", ATTRS{idProduct}=="b534", '
        'ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_MM_PORT_IGNORE}="1", '
        'MODE="0666", GROUP="dialout"\\n'
    )
    script = (
        "set -e\n"
        "printf '%s' " + shlex.quote(rule) + " > /etc/udev/rules.d/99-odin-rtk-holtek.rules\n"
        "udevadm control --reload-rules\n"
        "udevadm trigger --subsystem-match=tty || true\n"
        "systemctl restart ModemManager || true\n"
    )
    proc = subprocess.run(
        ["pkexec", "/bin/sh", "-c", script],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=120,
    )
    if proc.returncode != 0:
        detail = proc.stderr.strip() or proc.stdout.strip() or f"pkexec exited with {proc.returncode}"
        raise RuntimeError(detail)
    return {"ok": True, "rule": "/etc/udev/rules.d/99-odin-rtk-holtek.rules"}


def fix_rtk_permissions(port):
    real = Path(port).resolve()
    if not str(real).startswith("/dev/tty"):
        raise RuntimeError("不是有效的串口设备")
    if not real.exists():
        raise RuntimeError(f"串口不存在: {real}")
    cmd = ["pkexec", "/bin/chmod", "a+rw", str(real)]
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
    return {"ok": True, "devnode": str(real)}


def rtk_usb_device_name(port):
    real = Path(port).resolve()
    if not str(real).startswith("/dev/tty"):
        raise RuntimeError("不是有效的串口设备")
    if not real.exists():
        raise RuntimeError(f"串口不存在: {real}")
    tty_sys = Path("/sys/class/tty") / real.name / "device"
    if not tty_sys.exists():
        raise RuntimeError(f"找不到串口 sysfs 节点: {real.name}")
    target = tty_sys.resolve()
    for parent in [target, *target.parents]:
        vendor = parent / "idVendor"
        product = parent / "idProduct"
        if vendor.exists() and product.exists():
            return parent.name
    raise RuntimeError(f"找不到 {real.name} 对应的 USB 设备")


def reset_rtk_usb(port):
    device_name = rtk_usb_device_name(port)
    script = (
        "set -e\n"
        "systemctl stop ModemManager 2>/dev/null || true\n"
        f"echo -n {shlex.quote(device_name)} > /sys/bus/usb/drivers/usb/unbind\n"
        "sleep 1\n"
        f"echo -n {shlex.quote(device_name)} > /sys/bus/usb/drivers/usb/bind\n"
        "udevadm settle || true\n"
    )
    proc = subprocess.run(
        ["pkexec", "/bin/sh", "-c", script],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=120,
    )
    if proc.returncode != 0:
        detail = proc.stderr.strip() or proc.stdout.strip() or f"pkexec exited with {proc.returncode}"
        raise RuntimeError(detail)
    return {"ok": True, "usb_device": device_name}


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
                    "rtk": rtk_status_payload(),
                    "capture": capture_manager.status(),
                    "launch": launch,
                    "summary": summarize_state(state, devices, launch),
                    "ros2topic_available": ros2_command_available("topic"),
                    "quick_fields": QUICK_FIELDS,
                })
                return
            if path == "/api/capture/status":
                json_response(self, capture_manager.status())
                return
            if path == "/api/fusion/status":
                json_response(self, fusion_bag_status())
                return
            if path == "/api/rtk/status":
                json_response(self, rtk_status_payload())
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
                json_response(self, list_topics(max_age=0))
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
                if isinstance(body.get("rtk"), dict):
                    state.setdefault("rtk", {}).update({
                        k: str(v)
                        for k, v in body["rtk"].items()
                        if k in ("port", "baudrate")
                    })
                if isinstance(body.get("capture"), dict):
                    state.setdefault("capture", {}).update({
                        k: str(v)
                        for k, v in body["capture"].items()
                        if k in ("cloud_topic", "output_dir", "last_bag")
                    })
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
            if path == "/api/rtk/start":
                state = load_state()
                rtk = state.setdefault("rtk", {})
                port = str(body.get("port") or rtk.get("port") or "/dev/ttyACM0")
                baudrate = str(body.get("baudrate") or rtk.get("baudrate") or "115200")
                rtk["port"] = port
                rtk["baudrate"] = baudrate
                save_state(state)
                rtk_monitor.start(port, baudrate)
                time.sleep(0.2)
                json_response(self, rtk_status_payload())
                return
            if path == "/api/rtk/stop":
                rtk_monitor.stop()
                json_response(self, rtk_status_payload())
                return
            if path == "/api/rtk/configure-output":
                state = load_state()
                rtk = state.setdefault("rtk", {})
                port = str(body.get("port") or rtk.get("port") or "/dev/ttyACM0")
                baudrate = str(body.get("baudrate") or rtk.get("baudrate") or "115200")
                rtk["port"] = port
                rtk["baudrate"] = baudrate
                save_state(state)
                rtk_monitor.stop()
                result = configure_rtk_output(port, baudrate)
                rtk_monitor.start(port, baudrate)
                time.sleep(0.3)
                payload = rtk_status_payload()
                payload["configure_result"] = result
                json_response(self, payload)
                return
            if path == "/api/rtk-permissions/fix":
                state = load_state()
                port = str(body.get("port") or state.get("rtk", {}).get("port") or "/dev/ttyACM0")
                json_response(self, fix_rtk_permissions(port))
                return
            if path == "/api/rtk-system/probe-guard":
                json_response(self, install_rtk_probe_guard())
                return
            if path == "/api/rtk-system/usb-reset":
                state = load_state()
                rtk = state.setdefault("rtk", {})
                port = str(body.get("port") or rtk.get("port") or "/dev/ttyACM0")
                baudrate = str(body.get("baudrate") or rtk.get("baudrate") or "115200")
                rtk_monitor.stop()
                result = reset_rtk_usb(port)
                time.sleep(2)
                serial_devices = scan_serial_devices()
                port = select_default_rtk_port(state, serial_devices)
                rtk["port"] = port
                rtk["baudrate"] = baudrate
                save_state(state)
                rtk_monitor.start(port, baudrate)
                time.sleep(0.4)
                payload = rtk_status_payload()
                payload["reset_result"] = result
                json_response(self, payload)
                return
            if path == "/api/capture/start":
                state = load_state()
                capture = state.setdefault("capture", {})
                cloud_topic = str(body.get("cloud_topic") or capture.get("cloud_topic") or "/odin_b/odin1/cloud_render")
                output_dir = str(body.get("output_dir") or capture.get("output_dir") or CAPTURE_DIR)
                json_response(self, capture_manager.start(cloud_topic, output_dir))
                return
            if path == "/api/capture/stop":
                json_response(self, capture_manager.stop())
                return
            if path == "/api/capture/play":
                bag_path = str(body.get("bag_path") or "").strip()
                loop = bool(body.get("loop", False))
                with_rviz = bool(body.get("with_rviz", True))
                json_response(self, capture_manager.start_playback(bag_path, loop, with_rviz))
                return
            if path == "/api/capture/stop-play":
                json_response(self, capture_manager.stop_playback())
                return
            if path == "/api/capture/delete":
                bag_path = str(body.get("bag_path") or "").strip()
                json_response(self, capture_manager.delete_bag(bag_path))
                return
            if path == "/api/fusion/preview":
                json_response(self, fusion_preview_payload(body))
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
        capture_manager.shutdown()
        rtk_monitor.stop()
        stop_launch()


if __name__ == "__main__":
    main()
