"""Read-only STM32 telemetry for a locally built, matching firmware image."""
import re
import struct
import subprocess
from tempfile import TemporaryDirectory
from pathlib import Path


NAV_LABELS = (
    "Chưa cấu hình", "Đang dừng", "Sẵn sàng", "Tìm đường", "Đang xoay", "Đang tiến",
    "Chờ ToF", "Về đích", "Không có đường", "Mất vị trí", "Lỗi MPU", "Lỗi chuyển động",
    "Lỗi ToF", "Vào sân", "Lối vào bị chặn", "Phanh tránh va chạm", "Qua đích",
    "Lối ra bị chặn", "Đang lùi", "Dò lối", "Phanh ở viền",
)
PWM_ADDR = 0x40012C34
ENCODER_ADDRS = (0x40000424, 0x40000824)


class StlinkReader:
    def __init__(self, programmer, nm, elf):
        self.command = [str(programmer), "-c", "port=SWD", "mode=HOTPLUG", "freq=1000"]
        self.image = elf.with_suffix(".bin").read_bytes()
        output = self.run([str(nm), "-S", "-a", str(elf)])
        names = ("nav_diag", "control_diag", "motion_diag", "cb", "uwTick", "next_heading")
        self.addresses = {}
        self.trace_size = 0
        for row in output.splitlines():
            fields = row.split()
            if len(fields) != 4:
                continue
            name = fields[-1].split(".lto_priv.", 1)[0]
            if name in (*names, "nav_trace", "nav_trace_count"):
                if name in self.addresses:
                    raise ValueError(f"ELF có nhiều địa chỉ cho {name}.")
                self.addresses[name] = int(fields[0], 16)
            if name == "nav_trace":
                self.trace_size = int(fields[1], 16)
        if not self.addresses.keys() >= set(names):
            raise ValueError("ELF thiếu địa chỉ chẩn đoán ST-LINK.")

    @staticmethod
    def run(command):
        result = subprocess.run(command, capture_output=True, text=True, errors="replace",
                                timeout=8, creationflags=subprocess.CREATE_NO_WINDOW)
        if result.returncode:
            message = next((line.strip() for line in result.stdout.splitlines()
                            if line.strip().startswith("Error:")), "ST-LINK chưa kết nối.")
            raise OSError(message)
        return result.stdout

    def verify(self):
        with TemporaryDirectory(prefix="robot-stlink-") as folder:
            path = Path(folder) / "board.bin"
            self.run([*self.command, "-u", "0x08000000", str(len(self.image)), str(path)])
            if path.read_bytes() != self.image:
                raise OSError("Firmware trên bo khác bản build dùng để đọc dữ liệu.")

    def read_blocks(self, blocks):
        args = [arg for address, size in blocks for arg in ("-r32", hex(address), str(size))]
        output = self.run(self.command + args)
        words = {}
        for base, line in re.findall(r"(?m)^0x([0-9a-fA-F]{8}) : ([0-9a-fA-F ]+)", output):
            for offset, word in enumerate(line.split()):
                words[int(base, 16) + offset * 4] = int(word, 16)

        return [b"".join(struct.pack("<I", words[offset])
                         for offset in range(address, address + size, 4))
                for address, size in blocks]

    def frame(self):
        blocks = [(self.addresses[name], size) for name, size in (
            ("nav_diag", 80), ("control_diag", 12), ("motion_diag", 24),
            ("cb", 140), ("uwTick", 4), ("next_heading", 4))]
        blocks.extend((address, 4) for address in ENCODER_ADDRS)
        blocks.append((PWM_ADDR, 8))
        nav, control, motion, sensors, tick, target, e1, e2, pwm = self.read_blocks(blocks)
        nav = struct.unpack("<20i", nav)
        tick = struct.unpack("<I", tick)[0]
        angle = struct.unpack_from("<i", motion, 16)[0] / 1000
        target = struct.unpack("<f", target)[0]
        ranges = {}
        for index, name in enumerate(("HT", "TL", "G", "TR", "HP")):
            dist = struct.unpack_from("<H", sensors, index * 28 + 18)[0]
            sample = struct.unpack_from("<I", sensors, index * 28 + 24)[0]
            ranges[name] = dist if sensors[index * 28 + 17] and dist < 8000 and (
                (tick - sample) & 0xFFFFFFFF) <= 200 else None
        rx, command, _ = struct.unpack("<3I", control)
        state = nav[0]
        return {
            "angle": angle, "label": NAV_LABELS[state] if 0 <= state < len(NAV_LABELS) else str(state),
            "ranges": ranges, "encoders": [struct.unpack_from("<h", e)[0] for e in (e1, e2)],
            "x_mm": nav[4], "y_mm": nav[5], "heading_deg": nav[6] / 1000, "target_deg": round(target, 2),
            "pwm": list(struct.unpack("<2I", pwm)),
            "tick_ms": tick, "nav_state": state, "near_mask": nav[14],
            "fault_state": nav[17], "fault_tick_ms": nav[18],
            "rx_count": rx, "command": chr(command) if 0 < command < 128 else "",
        }

    def run_trace(self):
        if not self.trace_size:
            raise ValueError("Firmware chưa có bộ nhớ hành trình.")
        record = struct.Struct("<HhhhhHHHBx")
        capacity = self.trace_size // record.size
        nav, count, data, pwm = self.read_blocks([
            (self.addresses["nav_diag"], 4),
            (self.addresses["nav_trace_count"], 4),
            (self.addresses["nav_trace"], self.trace_size),
            (PWM_ADDR, 8),
        ])
        if struct.unpack("<I", nav)[0] in (2, 3, 4, 5, 13, 16, 18) or any(struct.unpack("<2I", pwm)):
            raise ValueError("Đọc bộ nhớ hành trình sau khi xe dừng.")
        count = struct.unpack("<I", count)[0]
        rows = []
        for i in range(max(0, count - capacity), count):
            t, x, y, yaw, target, left, center, right, state = record.unpack_from(
                data, i % capacity * record.size)
            rows.append({"ms": t * 10, "x_mm": x, "y_mm": y, "yaw_deg": yaw / 100,
                         "target_deg": target / 100, "state": state,
                         "tof_mm": [None if mm == 65535 else mm for mm in (left, center, right)]})
        return rows
