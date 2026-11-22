"""Serve the firmware course simulation and HC-06 telemetry on localhost."""

import argparse
import csv
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import re
import socket
import subprocess
from tempfile import TemporaryDirectory
import threading
import time
from urllib.parse import urlsplit
from route import course_with_curves, shortest_route
from stlink_feed import StlinkReader
from teach import HELD, MOTION_LABELS, TeachingRun


HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[1]
CHANNELS = ("HT", "TL", "G", "TR", "HP")
RANGES = ",".join(name + r"=(---|\d{1,4})" for name in CHANNELS)
FRAME = re.compile(
    r"(Goc:(---|-?\d{1,6}(?:\.\d+)?) do \| ToF:" + RANGES
    + r" mm \| Lenh:([A-Z0-9_*-]+(?::[A-Z0-9_*-]+)*?)"
    + r"(?: \| E1:(-?\d{1,5}),E2:(-?\d{1,5}))?)"
    + r"(?:\*([0-9A-F]{2})(?=Goc:|\s|$)|(?=Goc:|[\r\n]|$))",
    re.ASCII,
)
STATUS_LABELS = {"---", "XOAY-T45", "XOAY-P45"}
for source in ("main.c", "navigation.c", "motion.c"):
    STATUS_LABELS.update(re.findall(r'"([A-Z][A-Z0-9_*-]*)"',
                                    (PROJECT / "Src" / source).read_text(encoding="utf-8-sig")))


def course():
    with TemporaryDirectory(prefix="robot-map-") as scratch:
        executable = Path(scratch) / "navigation.exe"
        subprocess.run([
            "gcc", "-std=c11", "-O2", "-DNAV_NEW_COURSE_TEST", "-ITests/fakes", "-IInc",
            "Tests/test_navigation.c", "Src/navigation.c", "Src/grid_map.c",
            "-lm", "-o", str(executable),
        ], cwd=PROJECT, check=True)
        scene = json.loads(subprocess.check_output([str(executable), "--trace"]))
        start = scene.pop("frames")[0][:2]
        vertices = shortest_route(scene["width"], scene["height"], scene["obstacles"], start)
        reference = [point[:2] for point in course_with_curves(vertices, scene["obstacles"])]
        scene["reference"] = reference
        scene["referenceMm"] = round(sum(math.dist(a, b) for a, b in zip(reference, reference[1:])), 1)
        simulator = Path(scratch) / "sim_course.exe"
        subprocess.run([
            "gcc", "-std=c11", "-O2", "-ITests/fakes", "-IInc",
            "Tests/sim_course.c", "Src/grid_map.c", "-lm", "-o", str(simulator),
        ], cwd=PROJECT, check=True)
        scene["stepMs"] = 50
        scene["cases"] = {}
        for name in ("nominal", "first_bend", "first_bend_old", "slip", "extra", "south", "join",
                     "t9", "t11", "t14", "curve_lag", "t14_lag", "edge",
                     "north_short", "t9_short", "start_yaw_error", "arc_before", "arc_before_lag",
                     "fast_first_curve"):
            output = subprocess.check_output([str(simulator), "trace", name], text=True)
            *frames, summary = list(csv.reader(output.splitlines()))
            scene["cases"][name] = {
                "frames": [[float(value) for value in row] for row in frames],
                "outcome": summary[1],
                "middleGapMm": float(summary[-1]),
            }
        for name in ("first_bend", "first_bend_old"):
            case = scene["cases"][name]
            bend = next((frame for frame in case["frames"] if frame[2] < 86.0), None)
            if bend is not None:
                case["firstBend"] = bend[:2]
        scene["cases"]["extra"]["obstacle"] = [990, 800, 1090, 900]
        scene["cases"]["south"]["obstacle"] = [1540, 280, 1640, 380]
        scene["cases"]["join"]["obstacle"] = [1320, 525, 1360, 565]
        scene["cases"]["t9"]["obstacle"] = [530, 815, 570, 855]
        scene["cases"]["t11"]["obstacle"] = [800, 925, 900, 970]
        scene["cases"]["t14"]["obstacle"] = [1200, 750, 1300, 815]
        scene["cases"]["t14_lag"]["obstacle"] = scene["cases"]["t14"]["obstacle"]
        scene["cases"]["arc_before"]["obstacle"] = [960, 860, 995, 890]
        scene["cases"]["arc_before_lag"]["obstacle"] = scene["cases"]["arc_before"]["obstacle"]
        return json.dumps(scene, separators=(",", ":")).encode()


def crc8(payload):
    crc = 0
    for value in payload:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ (0x07 if crc & 0x80 else 0)) & 0xFF
    return crc


def parse_frame(line):
    for match in reversed(list(FRAME.finditer(line))):
        payload, angle, *values, label, e1, e2, checksum = match.groups()
        encoders = None
        if e1 is not None or checksum is not None:
            if e1 is None or checksum is None or crc8(payload.encode("ascii")) != int(checksum, 16):
                continue
            encoders = [int(e1), int(e2)]
            if any(not -32768 <= count <= 32767 for count in encoders):
                continue
        if all(part in STATUS_LABELS for part in label.split(":")):
            return {
                "angle": None if angle == "---" else float(angle),
                "ranges": dict(zip(CHANNELS, (None if v == "---" else int(v) for v in values))),
                "label": label,
                "encoders": encoders,
            }
    return None


class SerialFeed:
    def __init__(self, port, record=None, checkpoint=None, hold_drive=False, manual_trim=False):
        self.port = port
        self.phase = "connecting" if port else "disabled"
        self.error = None
        self.frame = None
        self.received_at = None
        self.dropped = 0
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.process = None
        self.teaching = TeachingRun.restore(checkpoint) if checkpoint else TeachingRun()
        self.hold_drive = hold_drive
        self.manual_trim = manual_trim
        self.record_file = record.open("x", encoding="utf-8", newline="") if record else None
        self.record = csv.writer(self.record_file) if self.record_file else None
        if self.record:
            self.record.writerow(["timestamp", "yaw_deg", *CHANNELS, "state", "encoder1", "encoder2"])
            self.record_file.flush()
        if port:
            threading.Thread(target=self.read, daemon=True).start()

    def read(self):
        while not self.stop.is_set():
            with self.lock:
                self.phase = "connecting"
                self.error = None
                self.received_at = None
            try:
                with subprocess.Popen([
                    "powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass",
                    "-File", str(HERE / "read_com.ps1"), "-Port", self.port,
                    "-OwnerPid", str(os.getpid()),
                ], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                    errors="replace", creationflags=subprocess.CREATE_NO_WINDOW) as process:
                    with self.lock:
                        self.process = process
                        if self.stop.is_set():
                            process.terminate()
                    for line in process.stdout:
                        self.accept(line.strip())
                    with self.lock:
                        self.process = None
                        if self.teaching.pending:
                            self.teaching.pending = None
                            self.teaching.problem = "Kết nối mất giữa bước điều khiển."
                        if self.phase != "error":
                            self.phase = "error"
                            self.error = "Luồng COM đã đóng."
            except OSError as error:
                with self.lock:
                    self.phase = "error"
                    self.error = str(error)
            self.stop.wait(4)

    def accept(self, line):
        with self.lock:
            if line == "@OPEN":
                self.phase = "open"
            elif line.startswith("@ERROR "):
                self.phase = "error"
                self.error = line[7:]
            elif line.startswith("@SENT "):
                return
            elif self.phase == "open" and line:
                frame = parse_frame(line)
                if frame:
                    self.frame = frame
                    self.received_at = time.monotonic()
                    self.teaching.sample(frame, self.received_at)
                    if self.record and frame["encoders"] is not None:
                        self.record.writerow([datetime.now().astimezone().isoformat(timespec="milliseconds"),
                                              frame["angle"], *(frame["ranges"][key] for key in CHANNELS),
                                              frame["label"], *frame["encoders"]])
                        self.record_file.flush()
                else:
                    self.dropped += 1

    def control(self, action):
        with self.lock:
            process = self.process
            if self.phase != "open" or process is None or process.poll() is not None:
                raise ValueError("COM Bluetooth chưa mở.")
            now = time.monotonic()
            self.teaching.expire(now)
            fresh = self.received_at is not None and now - self.received_at < 2.5
            if action == "save":
                self.teaching.save(PROJECT / "build")
            elif action == "record":
                self.teaching.start(self.frame if fresh else None)
            elif action == "x" or action in MOTION_LABELS:
                if action in HELD and not self.hold_drive:
                    raise ValueError("Chế độ giữ phím cần firmware hỗ trợ.")
                continuing = (action in HELD and self.teaching.pending
                              and self.teaching.pending["command"] == action)
                if action != "x" and not fresh and not continuing:
                    raise ValueError("Chờ dữ liệu mới từ xe trước khi lái.")
                self.teaching.begin(action, self.frame, now)
                try:
                    process.stdin.write(action)
                    process.stdin.flush()
                except OSError:
                    self.teaching.pending = None
                    self.teaching.problem = "Lệnh chưa gửi được qua Bluetooth."
                    raise
            else:
                raise ValueError("Lệnh điều khiển không hợp lệ.")
            return self.teaching.snapshot()

    def snapshot(self):
        with self.lock:
            self.teaching.expire(time.monotonic())
            return {
                "port": self.port, "baud": 9600, "phase": self.phase,
                "error": self.error, "frame": self.frame, "dropped": self.dropped,
                "ageMs": None if self.received_at is None
                else round(1000 * (time.monotonic() - self.received_at)),
                "canDrive": self.port not in (None, "ST-LINK") and self.phase == "open",
                "holdDrive": self.hold_drive,
                "manualTrim": self.manual_trim,
                "teaching": self.teaching.snapshot(),
            }

    def close(self):
        self.stop.set()
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                self.process.terminate()
            if self.record_file is not None:
                self.record = None
                self.record_file.close()


class LocalServer(ThreadingHTTPServer):
    allow_reuse_address = False

    def server_bind(self):
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


class StlinkFeed(SerialFeed):
    def __init__(self, reader, record):
        super().__init__(None)
        self.port, self.phase, self.reader = "ST-LINK", "connecting", reader
        self.probe_lock = threading.Lock()
        self.fields = ("tick_ms", "rx_count", "command", "nav_state", "label", "x_mm", "y_mm",
                       "heading_deg", "target_deg", "near_mask", "fault_state", "fault_tick_ms")
        if record:
            self.record_file = record.open("x", encoding="utf-8", newline="")
            self.record = csv.writer(self.record_file)
            self.record.writerow(["timestamp", *self.fields, *CHANNELS, "encoder1", "encoder2",
                                  "pwm1", "pwm2"])
        threading.Thread(target=self.read, daemon=True).start()

    def read(self):
        verified = False
        while not self.stop.is_set():
            try:
                with self.probe_lock:
                    if not verified:
                        self.reader.verify()
                        verified = True
                    frame = self.reader.frame()
                with self.lock:
                    self.phase, self.error = "open", None
                    self.frame = frame
                    self.received_at = time.monotonic()
                    if self.record:
                        self.record.writerow([datetime.now().astimezone().isoformat(timespec="milliseconds"),
                                              *(frame[key] for key in self.fields),
                                              *(frame["ranges"][key] for key in CHANNELS),
                                              *frame["encoders"], *frame["pwm"]])
                        self.record_file.flush()
            except (OSError, KeyError, ValueError, subprocess.TimeoutExpired) as error:
                with self.lock:
                    self.phase, self.error = "error", str(error)
                verified = False
            self.stop.wait(0.25 if verified else 2.0)

    def run_trace(self):
        with self.probe_lock:
            self.reader.verify()
            return self.reader.run_trace()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--com", default="COM16", help="Windows Bluetooth serial port")
    parser.add_argument("--no-com", action="store_true", help="serve simulation only")
    parser.add_argument("--record", type=Path, help="write verified encoder telemetry into a new CSV")
    parser.add_argument("--teach-checkpoint", type=Path, help="resume a paused Bluetooth teaching session")
    parser.add_argument("--hold-drive", action="store_true", help="enable W/A/D for compatible firmware")
    parser.add_argument("--manual-trim", action="store_true", help="show 10-degree turns for compatible firmware")
    parser.add_argument("--stlink", nargs=3, type=Path, metavar=("PROGRAMMER", "NM", "ELF"),
                        help="read a matching STM32 over ST-LINK")
    parser.add_argument("--port", type=int, default=8765, help="local HTTP port")
    args = parser.parse_args()
    if not re.fullmatch(r"COM[1-9]\d*", args.com.upper()):
        parser.error("--com must be a Windows COM name, e.g. COM16")
    scene = course()
    assets = {"/": ("index.html", "text/html; charset=utf-8"),
              "/map.js": ("map.js", "text/javascript; charset=utf-8"),
              "/map.css": ("map.css", "text/css; charset=utf-8")}

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            host = f"127.0.0.1:{args.port}"
            names = (host, f"localhost:{args.port}")
            if (self.path != "/api/drive" or self.headers.get("Host") not in names
                    or self.headers.get("Origin") not in (None, *(f"http://{name}" for name in names))
                    or self.headers.get_content_type() != "application/json"):
                self.send_error(403)
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 1 <= length <= 128:
                    raise ValueError("Yêu cầu không hợp lệ.")
                action = json.loads(self.rfile.read(length))["action"]
                if not isinstance(action, str):
                    raise ValueError("Lệnh điều khiển không hợp lệ.")
                response, status = {"teaching": feed.control(action)}, 200
            except (ValueError, KeyError, TypeError, OSError) as error:
                response, status = {"error": str(error)}, 409
            content = json.dumps(response, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()
            self.wfile.write(content)

        def do_GET(self):
            path = urlsplit(self.path).path
            content_type = "application/json; charset=utf-8"
            if path == "/api/course":
                content = scene
            elif path == "/api/status":
                content = json.dumps(feed.snapshot(), ensure_ascii=False).encode()
            elif path == "/api/run":
                try:
                    if not isinstance(feed, StlinkFeed):
                        raise ValueError("Kết nối ST-LINK để đọc bộ nhớ hành trình.")
                    content = json.dumps({"frames": feed.run_trace()}).encode()
                except (OSError, KeyError, ValueError, subprocess.TimeoutExpired) as error:
                    content = json.dumps({"error": str(error)}, ensure_ascii=False).encode()
            elif path in assets:
                filename, content_type = assets[path]
                content = (HERE / filename).read_bytes()
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()
            self.wfile.write(content)

        def log_message(self, *_args):
            pass

    with LocalServer(("127.0.0.1", args.port), Handler) as server:
        feed = (StlinkFeed(StlinkReader(*args.stlink), args.record) if args.stlink else
                SerialFeed(None if args.no_com else args.com.upper(), args.record, args.teach_checkpoint,
                           args.hold_drive, args.manual_trim))
        try:
            print(f"http://localhost:{server.server_port}", flush=True)
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            feed.close()


if __name__ == "__main__":
    main()
