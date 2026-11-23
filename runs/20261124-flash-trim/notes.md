# 2026-11-24 — Giải phóng flash, build Release hết tràn

## Hiện tượng

`cmake --build --preset Release` thất bại:

```text
region `FLASH' overflowed by 16 bytes
RAM:   17640 B / 20 KB (86.13%)
FLASH: 65552 B / 64 KB (100.02%)
```

Người dùng yêu cầu giữ nguyên thuật toán, chỉ bỏ phần không dùng đến mà tốn
RAM/flash. Đã kiểm tra trước khi sửa: `nav_trace`, `nav_diag`, `motion_diag`,
`i2c_diag`, `mpu_diag`, `tof_diag` đều được
`Tools/map_monitor/stlink_feed.py` đọc qua SWD nên **không** phải dữ liệu chết;
`grid_map.c` (A* + né tham chiếu) và toàn bộ `navigation.c` được
`Tests/test_navigation.c` và `Sim_course` dùng nên giữ nguyên. Vì
`--gc-sections` + LTO đã bỏ các hàm không tham chiếu, phần dư thực tế nằm ở
newlib stdio và ở các chuỗi `writeReg()` lặp trong khởi tạo VL53L0X.

## Thay đổi

1. `Src/vl53l0x.c` — chuỗi nạp thanh ghi VL53L0X chuyển sang 3 bảng
   `static const Vl53l0xRegValue[]` + `WriteRegTable()`.
   `runs/20261124-flash-trim/verify_init_table.py` so từng cặp (thanh ghi, giá
   trị) theo thứ tự giữa bản backup và bản mới: **92/92 MATCH**. Vị trí đọc
   `0x91 -> stop_variable` và thứ tự đọc/ghi được giữ nguyên.
2. `Inc/text_format.h` (mới) — bộ ghép chuỗi nhỏ (`Text_String`, `Text_UInt`,
   `Text_Int`, `Text_HexByte`) thay `snprintf` cho telemetry.
   `Inc/bluetooth_status.h` dùng bộ này; `Src/main.c` dùng cho `"ToF: %u/5"` và
   5 ô `ranges[]`. Khung telemetry, CRC và quy tắc bỏ khung khi thiếu chỗ giữ
   nguyên từng byte.
3. `Src/teach_drive.c` — thêm `#include <string.h>` (trước đây được lấy gián
   tiếp qua `bluetooth_status.h`; header này không còn include stdio/string).
4. `Tests/test_text_format.c` (mới) + một ca trong `Tests/run_checks.ps1` —
   so khung telemetry với `snprintf` trên PC cho nhiều giá trị (yaw âm/0/lớn,
   ToF `8190/8191`, notice dài, `E1/E2` cực trị, buffer quá nhỏ, đúng ngưỡng
   `length + 5 == size`). Đây là phép thử thay thế cho việc "đổi snprintf".

## Kết quả build (cùng toolchain, cùng preset)

| Bản | FLASH | RAM | Ghi chú |
| --- | --- | --- | --- |
| Release trước | 65.552/65.536 B (100,02 %) | 17.640/20.480 B | **tràn 16 B, không link được** |
| Release sau bảng VL53L0X | 64.900/65.536 B (99,03 %) | 17.640/20.480 B | −652 B |
| Release sau bỏ snprintf | **62.900/65.536 B (95,98 %)** | **17.616/20.480 B** | −2.000 B nữa, tổng −2.652 B |
| Debug | 63.620/65.536 B (97,08 %) | 17.632/20.480 B | trước đây Debug vượt flash |
| Teach | 30 KB/62 KB (48,39 %) | 3.576/20.480 B | build OK |

HEX Release: `code_test_xetranhvatcan/build/Release/code_test_xetranhvatcan.hex`,
177.006 byte, SHA-256
`455971CAA3AA4417828F2B7A72641870AE906105B0AA23D93AE5DF6F6468B158`.

`Tests/run_checks.ps1`: **10/10 nhóm PASS** (thêm nhóm `text_format`).
Bản đồ link không còn `snprintf`/`_svfprintf` (các mục `__sfp`, `_free_r`,
`_sbrk` chỉ còn ở danh sách section bị loại).

## Chưa làm

- **Chưa nạp**, chưa chạy xe thật với HEX này.
- Không đổi `course_route.h`, `navigation.c`, PID, hệ số encoder hay bất kỳ
  tham số điều khiển nào.
- Nếu sau này cần thêm telemetry dạng số thực (`%f`) thì phải nạp lại newlib
  printf hoặc tự viết bộ chia, vì stdio đã bị bỏ khỏi firmware.
