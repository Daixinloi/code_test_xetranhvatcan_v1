"""Record measured poses from supervised, finite Bluetooth commands."""

from datetime import datetime
import json
import math
from pathlib import Path


MOTION_LABELS = dict(w="TIEN", s="LUI", a="RE-T90", d="RE-P90",
                     u=("XOAY-T10", "XOAY-T45"), p=("XOAY-P10", "XOAY-P45"))
MOTION_LABELS.update(W="TIEN", A="RE-T90", D="RE-P90")
HELD = "WAD"
# Trial scale for the Bluetooth teaching track, estimated from measured displacement.
TEACH_MM_PER_COUNT = 0.0934


class TeachingRun:
    def __init__(self):
        self.active = False
        self.origin = None
        self.rebase_heading = False
        self.pending = None
        self.points = []
        self.saved = None
        self.problem = None

    @classmethod
    def restore(cls, checkpoint: Path):
        document = json.loads(checkpoint.read_text(encoding="utf-8-sig"))
        run = cls()
        run.active = True
        run.origin = document["origin"]
        run.points = document["points"]
        run.problem = document["problem"]
        run.rebase_heading = document.get("rebase_heading", False)
        return run

    def snapshot(self):
        return {"active": self.active, "busy": self.pending is not None, "points": list(self.points),
                "problem": self.problem, "saved": self.saved}

    def start(self, frame):
        if self.pending or not frame or frame["label"] != "DUNG" or frame["angle"] is None:
            raise ValueError("Chờ xe dừng và có số đo góc mới.")
        self.active = True
        self.origin = frame["angle"]
        self.rebase_heading = False
        self.points = [{"x": 420, "y": -45, "yaw": 90}]
        self.problem = self.saved = None

    def begin(self, command, frame, now):
        if command == "x":
            if self.pending and self.pending["command"] in HELD:
                self.pending["released"] = True
                self.pending["sent"] = now
                return
            if self.pending and self.active:
                self.problem = "Đã ngắt một bước. Tuyến ghi có thể thiếu đoạn đó."
            self.pending = None
            return
        if (self.pending and command in HELD and self.pending["command"] == command
                and not self.pending.get("released")):
            self.pending["sent"] = now
            return
        if self.pending or frame["label"] != "DUNG" or frame["angle"] is None:
            raise ValueError("Chờ bước trước hoàn tất.")
        if self.active and frame["encoders"] is None:
            raise ValueError("Khung Bluetooth chưa có encoder để ghi đường.")
        self.pending = {"command": command, "before": frame, "sent": now, "moving": False,
                        "stopped": 0, "count": 0, "last_angle": frame["angle"]}

    def expire(self, now):
        if self.pending and now - self.pending["sent"] > 10.0:
            self.pending = None
            self.problem = "Chưa xác nhận được bước chạy. Kiểm tra xe trước khi gửi lệnh tiếp."

    def sample(self, frame, now):
        if self.rebase_heading and frame["label"] == "DUNG" and frame["angle"] is not None:
            self.origin = 90 + frame["angle"] - self.points[-1]["yaw"]
            self.rebase_heading = False
        self.expire(now)
        if not self.pending:
            return
        step = self.pending
        command = step["command"]
        if frame["label"].startswith(MOTION_LABELS[command]):
            step["moving"] = True
            step["stopped"] = 0
            if self.active and command == "W":
                self.sample_held_step(step, frame)
            return
        if frame["label"].startswith("DUNG:"):
            self.pending = None
            self.problem = f"Xe báo {frame['label']}. Bước này chưa ghi được."
            return
        if frame["label"].split(":", 1)[0] == "LUI-KHAN":
            self.pending = None
            self.problem = "Xe tự lùi vì vật cản. Bước này chưa ghi được."
            return
        if frame["label"] != "DUNG" or (now - step["sent"] < 0.35 and command not in HELD):
            return
        if command in HELD and not step.get("released") and now - step["sent"] < 0.6:
            return
        step["stopped"] += 1
        if step["stopped"] < 2:
            return
        self.pending = None
        if not self.active:
            return
        before = step["before"]
        yaw = frame["angle"]
        counts = frame["encoders"]
        if yaw is None or counts is None:
            self.problem = "Thiếu encoder hoặc góc MPU ở cuối bước."
            return
        if command in HELD:
            if step["moving"] or counts != before["encoders"]:
                self.sample_held_step(step, frame)
            else:
                self.problem = "Chưa xác nhận bước giữ phím qua Bluetooth."
            return
        turned = abs(math.remainder(yaw - before["angle"], 360))
        if command in "ws":
            confirmed = step["moving"] or counts != before["encoders"]
            if command == "w" and counts[1] > 0:
                self.problem = "Encoder 2 báo lùi sau lệnh tiến. Bước này chưa ghi được."
                return
        else:
            confirmed = step["moving"] or turned >= (5 if command in "up" else 10)
        if not confirmed:
            self.problem = "Có bước không xác nhận được qua Bluetooth. Quỹ đạo chỉ để tham khảo."

        distance = abs(counts[1]) * TEACH_MM_PER_COUNT if command in "ws" else 0.0
        if distance > 150:
            self.problem = "Encoder bước vượt giới hạn đo. Quỹ đạo chỉ để tham khảo."
            return
        if command == "s":
            distance = -distance
        self.append_pose(command, yaw, before["angle"], counts, distance, confirmed)

    def sample_held_step(self, step, frame):
        yaw, counts = frame["angle"], frame["encoders"]
        if yaw is None or counts is None:
            return
        count = abs(counts[1])
        distance = max(0, count - step["count"]) * TEACH_MM_PER_COUNT if step["command"] == "W" else 0
        if distance > 0 or step["command"] != "W":
            self.append_pose(step["command"], yaw, step["last_angle"], counts, distance, True)
        step["count"] = count
        step["last_angle"] = yaw

    def append_pose(self, command, yaw, previous_yaw, counts, distance, confirmed):
        previous = self.points[-1]
        heading = 90 + math.remainder(yaw - self.origin, 360)
        midpoint = math.radians(heading - math.remainder(yaw - previous_yaw, 360) / 2)
        self.points.append({"x": round(previous["x"] + distance * math.cos(midpoint), 1),
                            "y": round(previous["y"] + distance * math.sin(midpoint), 1),
                            "yaw": round(math.remainder(heading, 360), 2), "command": command,
                            "observed": confirmed, "encoders": counts})

    def save(self, directory: Path):
        if not self.active or self.pending or len(self.points) < 2:
            raise ValueError("Cần một lượt ghi đã có bước chạy và xe đã dừng.")
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        path = directory / f"taught_course_{stamp}.json"
        document = {"units": "mm", "start": [420, -45, 90], "calibration": TEACH_MM_PER_COUNT,
                    "problem": self.problem, "poses": self.points}
        with path.open("x", encoding="utf-8") as output:
            json.dump(document, output, ensure_ascii=False, indent=2)
        self.active = False
        self.saved = path.name
