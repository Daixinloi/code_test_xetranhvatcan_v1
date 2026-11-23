"""So sánh chuỗi ghi thanh ghi VL53L0X trước/sau khi chuyển sang bảng.

Chạy: python runs/20261124-flash-trim/verify_init_table.py
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OLD = ROOT / "runs/20261124-flash-trim/backup/vl53l0x.c"
NEW = ROOT / "code_test_xetranhvatcan/Src/vl53l0x.c"

MACROS = {"REG_SYSTEM_INTERRUPT_CONFIG_GPIO": "0x0A",
          "REG_SYSTEM_SEQUENCE_CONFIG": "0x01",
          "REG_SYSRANGE_START": "0x00",
          "REG_SYSTEM_INTERRUPT_CLEAR": "0x0B"}


def norm(token):
    token = token.strip()
    token = MACROS.get(token, token)
    try:
        return int(token, 16)
    except ValueError:
        return None


def init_body(path):
    text = path.read_text(encoding="utf-8")
    start = text.index("bool VL53L0X_Init(")
    body = text[start:]
    # Bỏ phần sau bảng tuning (Reference Calibration và nhánh lỗi), vì phần đó
    # không đổi giữa hai bản.
    body = body.split("/* 4. Thực hiện Reference Calibration")[0]
    return body


def old_sequence():
    seq = []
    for line in init_body(OLD).splitlines():
        line = line.strip()
        m = re.match(r"writeReg\(dev,\s*([^,]+),\s*([^)]+)\);", line)
        if m:
            reg, value = norm(m.group(1)), norm(m.group(2))
            if reg is not None and value is not None:
                seq.append(("w", reg, value))
        m = re.match(r"readReg\(dev,\s*0x91,\s*&dev->stop_variable\);", line)
        if m:
            seq.append(("r", 0x91))
    return seq


def new_sequence():
    body = init_body(NEW)
    seq = []
    # Đi theo thứ tự xuất hiện: các bảng và lệnh đọc stop_variable.
    for m in re.finditer(r"static const Vl53l0xRegValue (\w+)\[\] = \{(.*?)\};|"
                         r"readReg\(dev,\s*0x91,\s*&dev->stop_variable\);", body, re.S):
        if m.group(1):
            table = m.group(2)
            for pair in re.finditer(r"\{([^,{}]+),\s*([^,{}]+)\}", table):
                seq.append(("w", norm(pair.group(1)), norm(pair.group(2))))
        else:
            seq.append(("r", 0x91))
    return seq


def main():
    old, new = old_sequence(), new_sequence()
    print(f"old entries: {len(old)}, new entries: {len(new)}")
    if old == new:
        print("MATCH: register write sequence is identical")
        return 0
    for i, (a, b) in enumerate(zip(old, new)):
        if a != b:
            print(f"first difference at {i}: old={a} new={b}")
            break
    else:
        print(f"length differs: old {len(old)} vs new {len(new)}")
        print("tail old:", old[len(new):] if len(old) > len(new) else new[len(old):])
        print("last 6 old:", old[-6:])
        print("last 6 new:", new[-6:])
    return 1


if __name__ == "__main__":
    sys.exit(main())
