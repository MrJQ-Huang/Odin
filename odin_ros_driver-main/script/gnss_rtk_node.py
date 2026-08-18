#!/usr/bin/env python3
import json
import math
import os
import select
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String


FIX_QUALITY_TEXT = {
    "0": "invalid",
    "1": "single",
    "2": "dgps_sbas",
    "4": "rtk_fixed",
    "5": "rtk_float",
    "6": "ins",
    "7": "fixed_position",
}

RMC_MODE_TEXT = {
    "N": "invalid",
    "A": "autonomous",
    "D": "differential",
    "E": "estimated",
    "R": "rtk_fixed",
    "F": "rtk_float",
}


def as_float(value, default=math.nan):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def as_int(value, default=0):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def dm_to_deg(value, hemi):
    if not value:
        return math.nan
    raw = float(value)
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


def nmea_kind(sentence):
    if len(sentence) < 6:
        return ""
    return sentence[3:6]


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_safe(item) for item in value]
    return value


class SerialPort:
    def __init__(self, path, baudrate):
        self.path = path
        self.baudrate = baudrate
        self.fd = None
        self.old_attrs = None

    def open(self):
        self.fd = os.open(self.path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.old_attrs = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        attrs = termios.tcgetattr(self.fd)
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
        speed = getattr(termios, f"B{self.baudrate}", termios.B115200)
        attrs[4] = speed
        attrs[5] = speed
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)

    def close(self):
        if self.fd is None:
            return
        try:
            if self.old_attrs is not None:
                termios.tcsetattr(self.fd, termios.TCSANOW, self.old_attrs)
        finally:
            os.close(self.fd)
            self.fd = None
            self.old_attrs = None

    def read_available(self, timeout):
        if self.fd is None:
            return b""
        readable, _, _ = select.select([self.fd], [], [], timeout)
        if self.fd not in readable:
            return b""
        try:
            return os.read(self.fd, 4096)
        except BlockingIOError:
            return b""


class GnssRtkNode(Node):
    def __init__(self):
        super().__init__("gnss_rtk_node")
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("frame_id", "gnss_link")
        self.declare_parameter("publish_raw", True)
        self.declare_parameter("read_timeout_sec", 0.05)
        self.declare_parameter("reconnect_delay_sec", 2.0)
        self.declare_parameter("status_period_sec", 1.0)

        self.port_path = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.publish_raw = bool(self.get_parameter("publish_raw").value)
        self.read_timeout = float(self.get_parameter("read_timeout_sec").value)
        self.reconnect_delay = float(self.get_parameter("reconnect_delay_sec").value)
        self.status_period = float(self.get_parameter("status_period_sec").value)

        self.fix_pub = self.create_publisher(NavSatFix, "fix", 10)
        self.heading_pub = self.create_publisher(Vector3Stamped, "heading", 10)
        self.heading_deg_pub = self.create_publisher(Float64, "heading_deg", 10)
        self.status_pub = self.create_publisher(String, "status", 10)
        self.raw_pub = self.create_publisher(String, "raw_sentence", 50)

        self.serial = None
        self.buffer = bytearray()
        self.counts = {}
        self.last_fix = {}
        self.last_rmc = {}
        self.last_heading = {}
        self.last_sentence_time = None
        self.next_reconnect_time = 0.0
        self.serial_error = ""

        self.timer = self.create_timer(0.02, self.poll)
        self.status_timer = self.create_timer(self.status_period, self.publish_status)
        self.get_logger().info(
            f"GNSS RTK serial node ready: port={self.port_path} baudrate={self.baudrate}"
        )

    def destroy_node(self):
        if self.serial:
            self.serial.close()
        super().destroy_node()

    def ensure_serial(self):
        if self.serial and self.serial.fd is not None:
            return True
        now = time.monotonic()
        if now < self.next_reconnect_time:
            return False
        self.serial = SerialPort(self.port_path, self.baudrate)
        try:
            self.serial.open()
            self.serial_error = ""
            self.get_logger().info(f"Opened GNSS serial port {self.port_path} at {self.baudrate}")
            return True
        except OSError as exc:
            self.serial = None
            self.next_reconnect_time = now + self.reconnect_delay
            self.serial_error = str(exc)
            self.get_logger().warn(f"Cannot open {self.port_path}: {exc}")
            return False

    def poll(self):
        if not self.ensure_serial():
            return
        try:
            data = self.serial.read_available(self.read_timeout)
        except OSError as exc:
            self.get_logger().warn(f"Serial read failed: {exc}")
            self.serial_error = str(exc)
            self.serial.close()
            self.serial = None
            self.next_reconnect_time = time.monotonic() + self.reconnect_delay
            return
        if not data:
            return
        self.buffer.extend(data)
        while b"\n" in self.buffer:
            raw_line, _, rest = self.buffer.partition(b"\n")
            self.buffer = bytearray(rest)
            line = raw_line.decode("ascii", errors="replace").strip()
            if line:
                self.handle_sentence(line)

    def handle_sentence(self, line):
        self.last_sentence_time = time.monotonic()
        if self.publish_raw:
            self.raw_pub.publish(String(data=line))

        if line.startswith("$"):
            kind = nmea_kind(line)
            self.counts[kind] = self.counts.get(kind, 0) + 1
            if not nmea_checksum_ok(line):
                self.get_logger().warn(f"Bad NMEA checksum: {line[:96]}")
                return
            fields = line.split("*", 1)[0].split(",")
            if kind == "GGA":
                self.handle_gga(fields)
            elif kind == "RMC":
                self.handle_rmc(fields)
            elif kind == "HDT":
                self.handle_hdt(fields)
            elif kind == "TRA":
                self.handle_tra(fields)
            elif kind == "VTG":
                self.handle_vtg(fields)
        elif line.startswith("#"):
            name = line.split(",", 1)[0].lstrip("#")
            self.counts[name] = self.counts.get(name, 0) + 1
            if name == "HEADINGA":
                self.handle_headinga(line)
        else:
            self.counts["partial_or_unknown"] = self.counts.get("partial_or_unknown", 0) + 1

    def handle_gga(self, fields):
        quality = fields[6] if len(fields) > 6 else "0"
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self.frame_id
        fix.latitude = dm_to_deg(fields[2], fields[3]) if len(fields) > 3 else math.nan
        fix.longitude = dm_to_deg(fields[4], fields[5]) if len(fields) > 5 else math.nan
        fix.altitude = as_float(fields[9]) if len(fields) > 9 else math.nan
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        if quality == "0":
            fix.status.status = NavSatStatus.STATUS_NO_FIX
        elif quality == "2":
            fix.status.status = NavSatStatus.STATUS_SBAS_FIX
        elif quality in ("4", "5"):
            fix.status.status = NavSatStatus.STATUS_GBAS_FIX
        else:
            fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = (
            NavSatStatus.SERVICE_GPS
            | NavSatStatus.SERVICE_GLONASS
            | NavSatStatus.SERVICE_COMPASS
            | NavSatStatus.SERVICE_GALILEO
        )
        self.fix_pub.publish(fix)
        self.last_fix = {
            "utc": fields[1] if len(fields) > 1 else "",
            "quality": quality,
            "quality_text": FIX_QUALITY_TEXT.get(quality, "unknown"),
            "satellites": as_int(fields[7]) if len(fields) > 7 else 0,
            "hdop": as_float(fields[8]) if len(fields) > 8 else math.nan,
            "latitude": fix.latitude,
            "longitude": fix.longitude,
            "altitude": fix.altitude,
            "age": fields[13] if len(fields) > 13 else "",
            "station_id": fields[14] if len(fields) > 14 else "",
        }

    def handle_rmc(self, fields):
        mode = fields[12] if len(fields) > 12 else ""
        self.last_rmc = {
            "utc": fields[1] if len(fields) > 1 else "",
            "valid": len(fields) > 2 and fields[2] == "A",
            "speed_mps": as_float(fields[7], 0.0) * 0.514444 if len(fields) > 7 else math.nan,
            "course_deg": as_float(fields[8]) if len(fields) > 8 else math.nan,
            "date_ddmmyy": fields[9] if len(fields) > 9 else "",
            "mode": mode,
            "mode_text": RMC_MODE_TEXT.get(mode, "unknown"),
        }

    def handle_vtg(self, fields):
        self.last_rmc["vtg_course_deg"] = as_float(fields[1]) if len(fields) > 1 else math.nan
        self.last_rmc["vtg_speed_kmh"] = as_float(fields[7]) if len(fields) > 7 else math.nan

    def handle_hdt(self, fields):
        heading = as_float(fields[1]) if len(fields) > 1 else math.nan
        if math.isfinite(heading):
            self.publish_heading(heading, math.nan, math.nan, "HDT", "SOL_COMPUTED")

    def handle_tra(self, fields):
        heading = as_float(fields[2]) if len(fields) > 2 else math.nan
        pitch = as_float(fields[3]) if len(fields) > 3 else math.nan
        if math.isfinite(heading):
            self.publish_heading(heading, pitch, math.nan, "TRA", "SOL_COMPUTED")

    def handle_headinga(self, line):
        body = line.split("*", 1)[0]
        if ";" not in body:
            return
        data = body.split(";", 1)[1].split(",")
        if len(data) < 5:
            return
        solution_status = data[0]
        position_type = data[1]
        baseline_m = as_float(data[2])
        heading_deg = as_float(data[3])
        pitch_deg = as_float(data[4])
        heading_std = as_float(data[6]) if len(data) > 6 else math.nan
        pitch_std = as_float(data[7]) if len(data) > 7 else math.nan
        tracked = as_int(data[9]) if len(data) > 9 else 0
        used = as_int(data[10]) if len(data) > 10 else 0
        self.last_heading = {
            "source": "HEADINGA",
            "solution_status": solution_status,
            "position_type": position_type,
            "baseline_m": baseline_m,
            "heading_deg": heading_deg,
            "pitch_deg": pitch_deg,
            "heading_std_deg": heading_std,
            "pitch_std_deg": pitch_std,
            "satellites_tracked": tracked,
            "satellites_used": used,
            "valid": solution_status == "SOL_COMPUTED" and math.isfinite(heading_deg),
        }
        if self.last_heading["valid"]:
            self.publish_heading(heading_deg, pitch_deg, baseline_m, "HEADINGA", solution_status)

    def publish_heading(self, heading_deg, pitch_deg, baseline_m, source, solution_status):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.vector.x = heading_deg
        msg.vector.y = pitch_deg
        msg.vector.z = baseline_m
        self.heading_pub.publish(msg)
        self.heading_deg_pub.publish(Float64(data=heading_deg))
        self.last_heading.update({
            "source": source,
            "solution_status": solution_status,
            "heading_deg": heading_deg,
            "pitch_deg": pitch_deg,
            "baseline_m": baseline_m,
            "valid": True,
        })

    def publish_status(self):
        age = None
        if self.last_sentence_time is not None:
            age = time.monotonic() - self.last_sentence_time
        payload = {
            "port": self.port_path,
            "baudrate": self.baudrate,
            "connected": bool(self.serial and self.serial.fd is not None),
            "error": self.serial_error,
            "seconds_since_sentence": age,
            "counts": self.counts,
            "fix": self.last_fix,
            "rmc": self.last_rmc,
            "heading": self.last_heading,
        }
        self.status_pub.publish(String(data=json.dumps(json_safe(payload), ensure_ascii=False, allow_nan=False)))


def main():
    rclpy.init()
    node = GnssRtkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
