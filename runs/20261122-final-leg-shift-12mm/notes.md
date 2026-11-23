# 2026-11-22 — Dịch chặng cuối sang phải 12 mm + công cụ tìm đường ngắn nhất

## Yêu cầu
- Chặng cuối bị chạm line → dịch sang phải thêm ~12 mm.
- Thêm thuật toán né vật cản và tìm đường đi ngắn nhất tới đích (kèm ảnh map 120×120 cm
  với T1..T17, Ô1..Ô5, khối chữ U ở giữa).

## Đã làm
1. Đồng bộ mã nguồn dự án chính với đúng bản đang chạy trên xe
   (`runs/20261122-restore-session/firmware/`):
   - `Inc/course_route.h` — tuyến cong 73 điểm (bản 38C477ED…).
   - `Src/navigation.c` — `FixedLineMove()` dùng `Motion_RetargetForward` khi đang tiến.
   Trước đó dự án chính còn tuyến 72 điểm + `FixedLineMove()` dừng–quay cũ, không khớp bản đã nạp.
2. Dịch 5 điểm cuối sang phải 12 mm (X 1300 → 1312), giữ nguyên Y:
   `(1304,658) (1308,645) (1311,631) (1312,616) (1312,-65)`.
   Các điểm trước đó giữ nguyên, nên quỹ đạo chỉ đổi ở chặng hạ xuống đích.
3. Thêm `Tools/plan_route.py` — A* + né vật cản + tuyến ngắn nhất, dùng đúng
   `grid_static_obstacles[18]` của firmware.
4. Sinh `Inc/course_route_planned.h` (94 điểm, A* inflate 52 mm) và thêm công tắc
   `COURSE_ROUTE_USE_PLANNED` trong `course_route.h` (mặc định 0).
   Bảng này không được biên dịch khi công tắc = 0 nên không tốn flash.

## Kết quả đo bằng `Tools/plan_route.py`
Đo lại sau khi sửa bảng so sánh trong tool (bản trước còn giữ 5 điểm cuối cũ X=1300,
nên số của tuyến hiện tại bị thấp hơn thực tế). Số dưới đây là số đúng của bản đã nạp.

- Tuyến đang nạp (73 điểm, tính cả đoạn vào từ mũi xe): dài 2.535,4 mm.
  - Ở inflate 52 mm: 0 điểm/0 đoạn cắt vật cản, khoảng hở nhỏ nhất 2,8 mm.
  - Ở inflate 64 mm (nửa đường chéo thân xe): 3 điểm / 5 đoạn nằm trong vùng nới.
    Đây là lý do hình học giải thích việc chạm nhẹ góc T11/T14, và cũng là lý do
    không thể dùng 64 mm làm bộ lọc duy nhất cho tuyến tay.
- Bản A* (inflate 52 mm, sinh lại): 94 điểm, dài 2.393,8 mm → **ngắn hơn 141,6 mm
  (5,6 %)**, khoảng hở nhỏ nhất 1,4 mm, 0 va chạm.
- Bản A* (inflate 64 mm): 2.409 mm, 0 va chạm, nhưng 158 điểm sau khi làm dày
  → 948 byte bảng, vượt 224 byte flash còn trống nên không dùng được ở Release.

## Build
- Release: Flash 65.312 / 65.536 byte (99,66 %, còn 224 byte), RAM 17.184 / 20.480 byte.
- HEX `build/Release/code_test_xetranhvatcan.hex`, SHA-256
  `3DB9DBED378B821D05CC65B2545B694060FDBB9C048EF7C1F6964E84311D239E`, 183.780 byte.
- `Tests/run_checks.ps1`: 9/9 nhóm PASS.

## Nạp
- SWD Under Reset 100 kHz, nguồn 3,27 V, STM32F103 medium-density 64 KB.
- Download 100 % sau 13,76 s, verify toàn file đạt, `Download verified successfully`, reset phần mềm.
- Vector sau nạp `0x08000000 = 20005000 0800CD65`.
- So với HEX cũ `38C477ED…` chỉ 4 dòng khác nhau → thay đổi gọn trong phần dữ liệu tuyến.
- Log: `flash.txt`, `readback.txt`.

## Chưa làm
- Chưa chạy xe thật với bản mới.
- Chưa xác nhận 12 mm là đủ (có thể cần 15–20 mm nếu vẫn chạm).
- Chưa thử tuyến A* (`COURSE_ROUTE_USE_PLANNED=1`) trên xe.
