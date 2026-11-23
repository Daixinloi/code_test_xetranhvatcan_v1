# 2026-11-22 — chặng cuối dịch trái 27 mm (X=1302 → 1275), nới T11/T14

## Yêu cầu của người dùng

Chạy lại bản đã nạp (chặng cuối X=1302 mm): T9, T11, T14 đã qua nhưng còn sát;
T15 và T16 vẫn bị đâm. Đề nghị giảm tọa độ X chặng cuối khoảng 7 cm và hỏi đang
áp dụng thuật toán né vật cản nào.

## Vì sao không dịch được 7 cm

Bảng vật cản `grid_static_obstacles[18]` (dùng chung cho firmware và
`Tools/plan_route.py`):

| Vật cản | Hình chữ nhật (mm) |
| --- | --- |
| U (khối giữa) | (600, 0) – (1200, 600) |
| T15 | (1400, 400) – (1500, 500) |
| T16 | (1400, 100) – (1500, 200) |

Hành lang về đích rộng 1200…1400 = 200 mm. Thân xe 90 × 90 mm (nửa rộng 45 mm) nên
tâm xe chỉ được nằm trong 1245…1355 mm. Dịch 70 mm → X = 1232 mm → mép trái thân
1187 mm, lấn 13 mm vào khối U. Vì vậy chọn 27 mm (X = 1275 mm):

| X chặng cuối | Mép thân trái | Khe tới U | Mép thân phải | Khe tới T15 |
| --- | --- | --- | --- | --- |
| 1302 (bản đang chạy) | 1257 | 57 mm | 1347 | 53 mm |
| 1275 (bản mới) | 1230 | 30 mm | 1320 | 80 mm |
| 1232 (yêu cầu 7 cm) | 1187 | **−13 mm** | 1277 | 123 mm |

## Thay đổi

`Inc/course_route.h` + `Tools/plan_route.py` `CURRENT_ROUTE` (73 điểm, khớp 100%):

- Chặng cuối: `(1304,645) (1302,631) (1302,616) (1302,-65)` →
  `(1240,690) (1258,680) (1264,670) (1269,661) (1272,652) (1275,643) (1275,616) (1275,590) (1275,-65)`.
  Cua một chiều, bỏ gù phải tại (1258,668) của bản cũ.
- Nới T11: hạ 4–5 mm các điểm X = 734–742 mm (`(734,892) (742,885)`).
- Nới T14: hạ 5–12 mm vùng 1112–1190 mm và làm mềm khúc gối (bỏ điểm gãy 68,7° ở
  `(1132,712)`).
- `FINISH_ENTRY` / `FINISH_NOSE` trong công cụ: 1302 → 1275 mm.

## Đo lại

`python Tools/plan_route.py validate --inflate N` (log `validate_45.txt`,
`validate_52.txt`, `validate_64.txt`):

| Nới biên | Điểm trong vật cản | Đoạn cắt vật cản | Hở tới mép nới |
| --- | --- | --- | --- |
| 45 mm (nửa rộng thân) | 0 | 0 | 23,7 mm |
| 52 mm | 0 | 0 | 13,8 mm |
| 64 mm (nửa chéo thân) | 0 | 1 | −1,0 mm |

Đoạn cắt ở 64 mm là `(734,892) → (742,885)` cắt góc nới T11: khe T9/T11 chỉ 180 mm
theo đường chéo nên không đủ cho thân 90 mm xoay mọi hướng. Ở 45 mm (đúng nửa rộng
thân) tuyến sạch.

Khoảng hở thật tới từng vật cản (chưa nới biên):

| Vật cản | Bản đang chạy | Bản mới | Ghi chú |
| --- | --- | --- | --- |
| T11 | 83,5 mm | 87,1 mm | điểm bám `(742,885)` |
| T14 | 97,1 mm | 108,2 mm | điểm bám `(1104,750)` |
| T9 | 98,5 mm | 94,8 mm | đổi lấy T11, hai khối hai phía |
| T15 | 174,9 mm | 154,0 mm | góc; trong dải Y 400–500 còn 80 mm từ mép thân |
| T16 | — | 80 mm | cùng cột X với T15 |
| U | 103,2 mm | 75,0 mm | 30 mm từ mép thân |

## Build và test

- `runs/build_release.ps1`: EXIT 0. FLASH 65.312 / 65.536 byte (còn 224 B),
  RAM 17.184 / 20.480 byte. Bảng tuyến vẫn 73 điểm nên kích thước không đổi.
- HEX `build/Release/course_test_xetranhvatcan.hex`, 183.780 byte,
  SHA-256 `1B7E33A1E6F19E3872A3ABEFADAB6FD24FA2EB9EF805DD1849BF4611ABFFFC`.
- `Tests/run_checks.ps1`: 9/9 PASS (log `runs/tests.log`).
- `Inc/course_route_planned.h` sinh lại ở nới biên 52 mm: 94 điểm, 2.392,3 mm,
  hở nhỏ nhất 1,4 mm, 0 va chạm (vẫn sau công tắc `COURSE_ROUTE_USE_PLANNED 0`).

## Chưa kiểm chứng

- Chưa nạp lên xe và chưa chạy thật.
- `Tests/sim_course.exe` không dùng được để kiểm chứng tuyến cố định: nó dừng ở
  assertion `NAV_WAIT_CLEARANCE` vì fixed-line mode chủ động bỏ qua ToF. Đây là
  hành vi có sẵn, không phải hồi quy của lượt này.

## Bước tiếp theo

1. Nạp HEX và chạy thật; quan sát T15, T16 và chặng xuống đích.
2. Nếu vẫn chạm T15/T16: đo khoảng lệch phải thực tế tại điểm chạm. Nếu lệch
   ≤ 27 mm thì tuyến mới đã đủ; nếu lệch lớn hơn thì phải xử lý bám tuyến hoặc
   bật né bằng ToF, không dịch tuyến thêm được (đã sát khối U 30 mm).
3. Nếu T11 còn sát: khe T9/T11 chỉ cho tối đa ~90 mm, hiện 87,1 mm, gần giới hạn.
