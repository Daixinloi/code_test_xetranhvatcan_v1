# Nhật ký lỗi và cách sửa

## 2026-11-23 — Xe đổi hướng giật khi bám waypoint dày

- **Triệu chứng:** người dùng báo xe vẫn chạy zic-zac và hỏi có thể bẻ góc khi đang tiến không. Chưa có log telemetry hoặc lượt chạy xe thật.
- **Giả thuyết:** bộ fixed-line đã retarget khi đang chạy, nhưng hướng ngắm vào waypoint hiện tại làm hướng đổi theo các điểm lấy mẫu liên tiếp.
- **Thử không đạt:** lookahead 45 mm khiến trace T9 thất bại; ép giới hạn hướng ±12° khiến xe lệch ở connector đầu; lookahead 20 mm cộng dò theo hướng mới làm trace `front_open` thất bại. Đã bỏ các biến thể này. Đây là bằng chứng simulator, không phải hành vi xe thật.
- **Sửa hiện tại:** `Src/navigation.c` ngắm waypoint lấy mẫu kế tiếp khi chạy tuyến bình thường; giữ waypoint dịch cho detour ToF và dùng hướng waypoint gốc khi bắt đầu dò. Không đổi bảng route, tốc độ cruise, hay ngưỡng ToF.
- **Kiểm chứng:** `Tests/run_checks.ps1` đạt 10 nhóm unit và 7 trace. Ba trace `nominal`, `t9`, `front_open` về đích với khoảng hở mô phỏng 34,8/35,2/38,2 mm. Release và Debug đều build thành công. Sau đó đã nạp Release SHA `9056360DCFABACEE69039BEBDE127322EBBE8C6301E69502223FF055895DCA42`; CubeProgrammer verify thành công, readback vector `20005000 0800D495`, CCR1/CCR2=0. Chưa thử hành vi xe thật; lỗi còn trạng thái **đã nạp, chờ thử xe**.
- Bàn giao: [`runs/20261123-waypoint-smoothing/notes.md`](../runs/20261123-waypoint-smoothing/notes.md).

## 2026-11-23 — Bản làm mượt waypoint lỗi, đã khôi phục bản trước

- **Triệu chứng:** người dùng phản hồi firmware waypoint smoothing lỗi và yêu cầu cập nhật lại code bản trước. Chưa có mô tả cụ thể kiểu lỗi hoặc telemetry xe thật.
- **Sửa:** trong `Src/navigation.c`, hoàn tác việc ngắm waypoint kế tiếp và khôi phục hướng bắt đầu dò theo điểm ngắm waypoint hiện tại. Không đổi route, ToF hoặc speed profile.
- **Kiểm chứng PC:** `Tests/run_checks.ps1` đạt 10 nhóm unit + 7 trace; Release và Debug build PASS.
- **Nạp:** Release HEX tái build SHA `2ED13149325596D6D45004D3B196FC5D58957F82A7416C18AEBE9ECC5E1F2128`; CubeProgrammer verify thành công. Readback vector `20005000 0800D425`, CCR1/CCR2=0; probe báo 3,27 V. SHA khác artifact cũ `CBB76A40…`; artifact cũ không còn ở workspace, vì vậy đây là bản tái build từ nguồn đã phục hồi, không khẳng định byte-identical. Chưa thử xe thật sau nạp.
- **Trạng thái:** nguồn và firmware logic đã trở lại bản trước smoothing; chờ xác nhận xe thật. Người dùng chưa cung cấp triệu chứng cụ thể của lỗi mới.
- Ghi chú: [`runs/20261123-waypoint-smoothing/notes.md`](../runs/20261123-waypoint-smoothing/notes.md).

## 2026-11-24 — Build Release tràn flash 16 B

- **Triệu chứng** — `cmake --build --preset Release` báo
  `region FLASH overflowed by 16 bytes` với `FLASH: 65552 B / 65536 B`
  (RAM 17.640/20.480). HEX không được tạo; Debug cũng từng vượt flash.
- **Nguyên nhân** — Không phải mã điều hướng: `--gc-sections` + LTO đã bỏ các
  hàm không tham chiếu. Flash bị chiếm bởi (1) chuỗi khởi tạo VL53L0X viết
  từng thanh ghi bằng ~92 lời gọi `writeReg()` liên tiếp, mỗi lời gọi tốn
  nhiều lệnh nạp tham số hơn một mục bảng; (2) toàn bộ newlib stdio
  (`snprintf`, `_svfprintf`, `__sfp`, `malloc`/`_sbrk`) chỉ để in khung
  telemetry có định dạng cố định. Các vùng nhớ debug (`nav_trace`, `nav_diag`,
  `motion_diag`, `tof_diag`, `i2c_diag`, `mpu_diag`) **không** phải dữ liệu
  chết: `Tools/map_monitor/stlink_feed.py` đọc chúng qua SWD.
- **Sửa** — `Src/vl53l0x.c`: gom chuỗi nạp vào 3 bảng `static const` +
  `WriteRegTable()`, giữ nguyên thứ tự/giá trị và vị trí đọc `0x91`
  (`runs/20261124-flash-trim/verify_init_table.py`: 92/92 MATCH) → −652 B.
  Thêm `Inc/text_format.h` và thay `snprintf` trong `Inc/bluetooth_status.h`,
  `Src/main.c` bằng bộ ghép chuỗi nhỏ → −2.000 B flash, −24 B RAM.
  `Src/teach_drive.c` thêm `#include <string.h>` vì header telemetry không còn
  include stdio/string gián tiếp.
- **Kiểm chứng** — `Tests/run_checks.ps1` **10/10 PASS**, gồm ca mới
  `Tests/test_text_format.c` so khung telemetry với `snprintf` trên PC từng
  byte (CRC, buffer quá nhỏ, ngưỡng `length+5 == size`, `E1/E2` cực trị).
  Release **62.900/65.536 B (còn 2.636 B)**, RAM 17.616/20.480; Debug
  63.620/65.536 B; Teach build OK. HEX SHA-256 `455971CA…68B158`.
- **Trạng thái** — Đã sửa và build xanh; **chưa nạp, chưa chạy xe thật**.
  Không đổi tuyến, PID, hệ số encoder hay tham số điều khiển. Ghi chú:
  `runs/20261124-flash-trim/notes.md`.

## 2026-11-23 — Bảng tuyến tràn int8 và lệnh chạy bị phát lại mỗi vòng

- **Triệu chứng** — `Tests/sim_course.c` chạy `trace nominal` dừng ở assertion
  `state != NAV_MOVING || distance_commands <= 1`; xe không tiến và lệnh chuyển
  động bị phát lại liên tục.
- **Nguyên nhân 1** — `Inc/course_route.h` nén bảng tuyến vào
  `int8_t course_route_cm[][2]`, nhưng X lên tới 132 cm. Giá trị 128…132 vượt
  `int8_t` (max 127) nên bị quấn thành số âm; sáu điểm cuối của tuyến trỏ ra
  ngoài sân và chặng về đích không thể tới.
- **Nguyên nhân 2** — `FixedLineMove()` trong `Src/navigation.c`: khi
  `state == NAV_MOVING` mã cũ không thoát sớm mà rơi xuống cuối hàm và gọi
  `Motion_StartDistanceHeading()` lần nữa mỗi 10 ms. Lệnh chạy bị khởi động lại
  liên tục, `distance_commands` tăng vô hạn.
- **Sửa** — (1) đổi bảng sang `struct { uint8_t x_cm; int8_t y_cm; }` để X không
  dấu; (2) khôi phục nhánh chạy liên tục: đang `NAV_MOVING` thì
  `Motion_RetargetForward()` giữ bánh đang lăn, chỉ quay tại chỗ khi lệch hướng
  quá `MOTION_ANGLE_TOLERANCE_DEG`.
- **Kiểm chứng** — `trace nominal` đạt: `finish`, 14,37 s, sai số ngang lớn nhất
  1,9 mm, khoảng hở nhỏ nhất 40,3 mm. `Tests/run_checks.ps1` 9/9 PASS. Release
  build FLASH 64.944/65.536 B (còn 592 B), HEX SHA-256 `906BF4C3…114905`.
  **Chưa nạp.**
- **Còn lại** — `trace t9` vẫn chạm vật cản phụ ở (482, 785): vết chạm là sượt
  ngang chứ không chắn trước mặt, nên tia giữa không thấy. Một bản né dùng tia
  giữa đã thử và đã bỏ vì làm đường trống chậm 14,37 → 19,4 s (tia giữa thấy
  tường khi vào cua), tức vi phạm yêu cầu "không đổi đường khi không có vật cản".

## 2026-11-23 — Giữ tuyến waypoint, né ToF chưa xác nhận

- Người dùng xác nhận waypoint đã tới đích; yêu cầu giữ nguyên tuyến chuẩn và
  chỉ dùng né cục bộ khi thật sự có vật cản.
- `COURSE_FIXED_LINE_TEST=1` đi vào `FixedLineMove()` rồi thoát sớm, nên bỏ qua
  phần né ToF tổng quát. Đường lùi an toàn trong `main.c` cũng có thể tranh
  quyền điều khiển khi Navigation đang chạy.
- Phiên này chọn lại chế độ waypoint, dùng bảng trong
  `runs/orig_route_head.h`, và thêm guard để `Service_Front_Clearance()` không
  hủy pose khi Navigation đang hoạt động.
- `Tests/run_checks.ps1`: 9/9 nhóm PASS. Ca simulator `trace nominal` và `trace
  t9` không đạt với mã/harness hiện tại; thử chuyển hẳn sang reactive cũng không
  về đích nên đã bỏ. Chưa build firmware, nạp hoặc chạy xe thật.
- Trạng thái: chưa tích hợp né ToF cục bộ mà vẫn bảo toàn waypoint. Không nạp
  bản hiện tại. Bước kế là ghép detour/rejoin vào nhánh waypoint rồi đạt ca mô
  phỏng đường trống và có vật cản trước khi build.
## 2026-11-22 — Sau khi nạp Release, không có dữ liệu Bluetooth và xe không chạy

- **Trạng thái** — Đã sửa nhánh phục hồi sau lỗi khởi tạo MPU và nạp bản mới; xác nhận MPU/ToF trở lại READY qua SWD, chưa xác nhận dữ liệu Bluetooth trên điện thoại hoặc chạy xe.
- **Môi trường và quan sát** — Người dùng báo ngay sau khi nạp HEX Release `C96F0A770F499655402A908E79BBC97DEDF1DE8BADC1899922931D38CE9DF2AF`, ứng dụng kết nối nhưng không thấy dữ liệu và xe không chạy. ST-LINK Hot Plug 100 kHz báo 3,27 V. Không gửi lệnh chuyển động.
- **Bằng chứng SWD** — Với ELF cùng build, `mpu_diag.stage=1`, `ack_mask=0`, `hal_error=0x20` (timeout), `motion_diag.state=1`; cả năm ToF đều `state=5` và chưa có mẫu. I2C2 PB10/SCL đọc thấp, PB11/SDA cao; SR2 BUSY và SR1 STOPF/BERR. Điều này phù hợp với bus I2C bị giữ lỗi; chưa xác định cảm biến/dây gây ra. Firmware khóa chuyển động nếu MPU chưa sẵn sàng. TIM1 CCR1/CCR2 sau reset đều 0.
- **Đường Bluetooth** — RAM cho thấy `auto_stream=true`; `control_diag` ghi hai byte nhận gần nhất `FC FF`, không có lệnh được chấp nhận. Firmware dùng software UART 9600 baud trên PA12 TX / PA11 RX. Chưa có waveform hoặc log phía điện thoại nên nguyên nhân mất dữ liệu chưa xác nhận; kiểm tra dây, baud và GND chung.
- **Bước tiếp** — Tắt nguồn trước khi tháo/cắm cảm biến; kiểm tra PB10/SCL được nhả cao, tách từng nhánh I2C để tìm nhánh giữ bus. Chỉ thử chạy khi MPU báo READY và bus ToF hoạt động. Đối chiếu đường HC-06 bằng logic analyzer/USB-UART nếu vẫn im lặng. Ghi chi tiết ở [runs/20261122-postflash-no-output-no-motion/notes.md](../runs/20261122-postflash-no-output-no-motion/notes.md).
- **Nguyên nhân phần mềm đã xác nhận cho việc xe bị khóa** — `Motion_Init()` chỉ chạy lúc boot. Nếu không ACK MPU, `mpu_ok` sai và `mpu_diag.stage=1`; nhánh phục hồi cũ chỉ chạy khi lỗi đọc runtime chuyển state sang 5. Vì vậy lỗi ACK ngay lúc boot không được thử lại và mọi lệnh chuyển động bị từ chối.
- **Sửa và kiểm chứng** — Sau `Run_Full_Initialization()`, `main.c` nay yêu cầu `Request_I2C_Recovery()` nếu MPU chưa sẵn sàng. `Tests/run_checks.ps1`: 9/9 nhóm PASS. Release build FLASH 65.312/65.536 B, RAM 17.616/20.480 B; HEX SHA-256 `131218ECF927D1ABB10F2564FF5EB55A135BBCA1E0DEA793B1FEE46205795746`. Đã nạp Under Reset 100 kHz, verify thành công, reset. Sau 5 giây Hot Plug: MPU `stage=4`, addr `0x68`, WHO_AM_I `0x70`; cả 5 ToF `state=1`, mỗi mắt 146 mẫu hợp lệ; TIM1 CCR1/CCR2 = 0. Lượt này phục hồi được bus/MPU; chưa biết nguyên nhân vật lý ban đầu khiến SCL thấp.
- **Bluetooth còn mở** — `auto_stream=true`, phần mềm phát 9600 baud qua PA12 và nhận ở PA11; sau nạp `control_diag.rx_count=0`. Chưa có waveform hoặc xác nhận trên điện thoại, nên chưa kết luận lỗi in dữ liệu đã được sửa.
- **Nạp lại theo yêu cầu** — Cùng HEX và SN `37FF71064E573436E9171343` được nạp/verify lại thành công ở Under Reset 100 kHz, 3,27 V. Sau ~5 giây, `mpu_diag.stage=1`, `ack_mask=0`, `hal_error=0x20`; `i2c_diag.state=2` sau 16 lần thử; năm ToF `state=5`, PWM=0. Lượt nạp đầu cùng HEX từng đọc được MPU READY và 5/5 ToF, nên boot/recovery không ổn định. Không gửi lệnh chuyển động. Chi tiết ở notes lượt.

## 2026-11-22 — Xe thật đâm T15/T16 ở chặng cuối, xin dịch trái 7 cm

- **Trạng thái** — Đã sửa tuyến và build; chờ nạp và chạy lại trên xe thật.
- **Môi trường và quan sát** — Xe thật, bản đã nạp có chặng cuối ở X=1302 mm. Người
  dùng báo T9, T11, T14 đã qua nhưng còn sát; T15 và T16 vẫn bị đâm; đề nghị giảm
  tọa độ X chặng cuối khoảng 7 cm. Không có log tọa độ hay ảnh điểm chạm.
- **Ràng buộc hình học** — Theo `grid_static_obstacles[18]`, khối U là
  `{600,0}`–`{1200,600}` và T15/T16 là `{1400,400}`–`{1500,500}` /
  `{1400,100}`–`{1500,200}`. Hành lang về đích rộng 200 mm; thân xe 90×90 mm nên tâm
  xe chỉ được nằm trong 1245…1355 mm. Dịch 70 mm (X=1232) làm thân xe lấn 13 mm vào
  khối U, nên không dùng được.
- **Nguyên nhân (giả thuyết có căn cứ)** — Bảng vật cản cho 53 mm từ mép thân tới
  T15 ở X=1302 mm, tức xe phải lệch phải khoảng 53 mm mới chạm. Xe vẫn chạm, nên
  nghi do bám tuyến lệch phải ở chặng xuống (sai số odometry/hướng) và/hoặc do khúc
  gối ở T14 và gù phải tại (1258,668) đẩy xe sang phải. Chưa có log để xác nhận.
- **Thay đổi** — `Inc/course_route.h` và `Tools/plan_route.py` `CURRENT_ROUTE`: chặng
  cuối X=1302 → 1275 mm (trái 27 mm) với cua lượn một chiều thay cho gù phải; hạ
  4–5 mm các điểm X=734–742 để nới T11; hạ 5–12 mm vùng T14 và làm mềm khúc gối.
- **Kiểm chứng** — `Tools/plan_route.py validate --inflate 45` và `--inflate 52`:
  0 điểm và 0 đoạn cắt vật cản; `--inflate 64` còn 1 đoạn cắt góc nới T11 ở khe T9/T11
  (khe 180 mm không đủ cho thân 90 mm xoay mọi hướng). Khoảng hở thật: T11 87,1 mm,
  T14 108,2 mm, T9 94,8 mm, T15 154,0 mm tại góc (80 mm từ mép thân trong dải Y
  400–500), U 75,0 mm. Release build FLASH 65.312/65.536 byte, HEX SHA-256
  `1B7E33A1E6F19E3872A3ABEFADAB6FD24FA2EB9EF805DD1849BF4611ABFFFC`.
  `Tests/run_checks.ps1` 9/9 PASS. Lượt `sim_course.exe` không dùng được để kiểm
  chứng tuyến cố định: nó dừng ở assertion `NAV_WAIT_CLEARANCE` vì fixed-line mode
  chủ động bỏ qua ToF (lỗi có sẵn, không phải hồi quy của lượt này).
- **Chưa xác nhận** — Chưa nạp và chưa chạy xe thật. Nếu vẫn chạm T15/T16 thì bước
  tiếp là đo khoảng lệch thật tại điểm chạm; nếu lệch phải > 27 mm thì phải xử lý
  bám tuyến hoặc bật né bằng ToF, không dịch tuyến thêm được vì đã sát khối U.
- **Cập nhật 2026-11-22 — lớp né mép trước bằng ToF** — Trạng thái: đã sửa mã và
  build Release, **chưa nạp, chưa xác nhận trên xe**. Lý do: người dùng báo xe
  thật xuống ở X≈1390 trong khi bảng tuyến hạ ở X=1275, nên tin encoder không đủ;
  lớp này giữ làn bằng `front[2]` (Truoc-Phai, −25°). Trong `FixedLineMove()`, khi
  `route_leg == U_SOUTH` và `y < 620 mm`, aim = `x + (118 − front[2].mm) × sin25°`,
  giữ aim cuối khi mất beam; cửa sổ hợp lệ 40…480 mm. Chốt biên
  `U_RIGHT_LANE_MM + pad` được bỏ qua khi lớp bật, và điều kiện về đích so với
  điểm bảng tuyến thay vì `remaining`. Bảng tuyến bỏ hai điểm `{1275,616,3}`,
  `{1275,590,3}`. Kiểm chứng PC: `runs/20261122-front-edge-avoid/validate_beam.py`
  → hội tụ X=1274,9 (29,9 mm tới U, 80,1 mm tới T15) với drift 0/88/120 mm; beam
  thấy khối U trong dải Y 153…659. Release build FLASH 65.476/65.536 byte (60 B
  trống), HEX SHA-256 `C2B063DC2BF8139CC134CAB15D4EFD5D231ABA40B8EAACA9B8BF66CEB4BD8E26`,
  `Tests/run_checks.ps1` 9/9 PASS. **Chưa xác nhận** — cần nạp và xem trace chặng
  cuối; trace hiện chưa có cột ToF nên chưa đối chiếu được dữ liệu cảm biến.
- **Cập nhật theo sơ đồ sân, 2026-11-22** — Đã giới hạn mục tiêu ngang hiệu chỉnh
  ToF vào X=1255…1345 mm (thân 90 mm và lề 10 mm tới U/T15). Mô hình PC
  hội tụ tại X=1274,9 mm kể cả khi drift đầu vào 120 mm; kiểm tra mục tiêu cho
  X=1200/1230/1275/1390/1450 đều nằm trong khoảng giới hạn. `Tests/run_checks.ps1`
  đạt 9/9 nhóm. Release build đạt: FLASH 65.504/65.536 B, RAM 17.192/20.480 B;
  HEX SHA-256 `778040597E0D4589400CA0F3ED53C7981A14CADD240E3D94CD5606B97B311CBD`.
  Chưa nạp hoặc chạy xe thật; 32 B flash còn lại khiến telemetry cần đưa qua
  UART/ST-LINK hoặc phải giảm footprint trước khi thêm vào firmware.
- **Cập nhật 2026-11-22 — tối ưu flash cho bảng tuyến** — Bảng tuyến tay được
  nén từ 6 B/điểm (int16 mm × 3) xuống 2 B/điểm (uint8 cm x, int8 cm y) và bung
  ra mm trong RAM lúc `Navigation_Init()` qua `CourseRoute_Init()`. Release
  build: FLASH 65.288/65.536 B (còn **248 B**, trước là 60 B), RAM
  17.616/20.480 B; HEX SHA-256
  `C96F0A770F499655402A908E79BBC97DEDF1DE8BADC1899922931D38CE9DF2AF`;
  `Tests/run_checks.ps1` 9/9 PASS. Điểm kết thúc làm tròn thành (1280, −70) nên
  bán kính về đích của điểm cuối được nới 8 → 12 mm. Đính chính dòng trên: trace
  **đã có** `front_mm[3]` và `stlink_feed.py` đã giải mã thành `tof_mm`, nên
  không cần thêm cột mới chỉ để đọc ToF. Vẫn **chưa nạp**.

## 2026-11-22 — `Tools/plan_route.py` đo sai "tuyến hiện tại" (bảng cũ X=1300)

- **Môi trường** — Windows 11, Python 3 (PATH), dự án `code_test_xetranhvatcan_v1`.
- **Bước tái hiện** — `python Tools/plan_route.py validate` sau khi `Inc/course_route.h`
  đã dịch 5 điểm cuối lên X=1304…1312.
- **Hiện tượng** — Tool báo tuyến hiện tại dài 2.526,8 mm và hở nhỏ nhất 5,7 mm ở inflate 50,
  tức là số của tuyến **trước** khi dịch 12 mm, không phải bản đang nằm trên xe.
- **Nguyên nhân** — Bảng `CURRENT_ROUTE` trong tool là bản sao tay của `course_route.h` và
  không được cập nhật theo; `FINISH_ENTRY`/`FINISH_NOSE` vẫn để 1300. Bảng sao chép tay
  không có cơ chế phát hiện lệch.
- **Sửa** — Cập nhật 5 điểm cuối thành `(1304,658) (1308,645) (1311,631) (1312,616) (1312,-65)`
  và `FINISH_ENTRY`/`FINISH_NOSE` = 1312. Sinh lại `Inc/course_route_planned.h` ở inflate 52 mm.
- **Kết quả** — `validate` giờ báo 74 điểm, 2.535,4 mm, hở nhỏ nhất 2,8 mm, 0 va chạm ở
  inflate 52 mm. `plan` báo 94 điểm, 2.393,4 mm, hở 1,4 mm, ngắn hơn 141,7 mm (5,6 %).
  `Tests/run_checks.ps1` 9/9 PASS, EXIT=0.
- **Phòng ngừa** — Nếu sửa tay `course_route.h`, phải sửa `CURRENT_ROUTE` trong tool cùng lúc;
  hoặc chạy `python Tools/plan_route.py validate` sau mỗi lần đổi tuyến để đối chiếu số điểm.
- **Chưa xác nhận** — Chưa chạy xe thật với tuyến A* (công tắc `COURSE_ROUTE_USE_PLANNED` vẫn 0).

## 2026-11-22 — Chặng cuối chạm line, cần dịch phải 12 mm

- **Trạng thái** — Đã sửa và đã nạp; chờ xác nhận trên xe thật.
- **Quan sát** — Trên xe thật, người dùng báo lượt chạy đã “khá ok” nhưng chặng cuối hạ xuống đích
  bị chạm line, cần dịch sang phải khoảng 12 mm. Đây là quan sát trên xe thật, không có log tọa độ.
- **Nguyên nhân (giả thuyết có căn cứ)** — Chặng cuối trong `Inc/course_route.h` hạ ở X=1300 mm.
  Khối chữ U giữa sân là `{600,0}`–`{1200,600}`; `U_CENTERED_MIDDLE_CLEARANCE_MM = 15` và vùng nới
  của planner là 115 mm, nên X=1300 chỉ còn 25 mm tính từ mép nới 1275 mm. Dịch phải 12 mm đưa về
  X=1312, tăng khoảng hở lên 37 mm.
- **Thay đổi** — 5 điểm cuối của tuyến dịch X 1300 → 1312 mm (giữ nguyên Y):
  `(1304,658) (1308,645) (1311,631) (1312,616) (1312,-65)`. Đồng thời đồng bộ
  `Inc/course_route.h` và `Src/navigation.c` của dự án chính về đúng bản đang chạy trên xe
  (tuyến cong 73 điểm + `FixedLineMove()` dùng `Motion_RetargetForward`), vì trước đó dự án chính
  còn tuyến 72 điểm và `FixedLineMove()` dừng–quay cũ.
- **Kiểm chứng** — `Tests/run_checks.ps1`: 9/9 nhóm PASS. Release build: Flash 65.312/65.536 byte
  (99,66 %), RAM 17.184/20.480 byte. HEX SHA-256
  `3DB9DBED378B821D05CC65B2545B694060FDBB9C048EF7C1F6964E84311D239E`, 183.780 byte; so với
  HEX cũ `38C477ED…` chỉ 4 dòng khác nhau. Nạp qua SWD Under Reset 100 kHz, nguồn 3,27 V:
  download 100 % sau 13,76 s, verify đạt, reset phần mềm, vector `20005000 0800CD65`.
- **Chưa xác nhận** — Chưa chạy xe thật với bản mới. Nếu vẫn chạm, tăng tiếp lên 15–20 mm.


## 2026-11-22 — Nạp bản cuối chat tuyến cong bị gián đoạn, ST-LINK mất kết nối USB

- **Trạng thái** — Đã khắc phục: sau khi rút/cắm lại USB ST-LINK và nguồn bo, nạp lại toàn bộ thành công và verify đạt. Chưa chạy xe thật.
- **Yêu cầu** — Người dùng gửi link chat chia sẻ `https://chatgpt.com/s/cx_6b02b1103a1c81918793f33249e58a36` và yêu cầu nạp đúng đoạn code cuối của chat đó vào mạch.
- **Nguồn code** — Link trả “Shared chat not found” (trang không còn nội dung, bản cache r.jina.ai cũng vậy), nên không đọc được bản gốc. Dùng bản đã khôi phục từ chính chat đó ở `runs/20261122-restore-session/firmware/`: `Inc/course_route.h` là tuyến cong Bezier qua T9 Y≈900 mm, hạ về Y=875 mm trước T11, vòng qua vùng (1130,750) mm rồi bo về X=1300 mm và xuống đích Y=−65 mm; `Src/navigation.c` `FixedLineMove()` giữ `Motion_RetargetForward` khi đang tiến với tầm nhìn 45 mm. HEX Release dựng lại khớp hoàn toàn SHA-256 `38C477EDC02F35B89C6B822356BEE126028F1A566B25A5EFD5DBECA422DBC928`, đúng hash bàn giao của bản cuối chat.
- **Quan sát** — Kết nối SWD Under Reset 100 kHz lúc nguồn 3,27 V thành công (STM32F103 medium-density, ST-LINK SN 37FF71064E573436E9171343). Lệnh ghi bắt đầu: xóa sector 0–63, download dừng ở khoảng 26% khi tiến trình CLI bị đóng cùng terminal (log `runs/20261122-restore-session/flash2.txt`). Sau đó mọi lần kết nối trả `DEV_CONNECT_ERR` rồi `DEV_USB_COMM_ERR` (4 lần thử lại 16:25–16:26, log `retry_probe.log`). Thiết bị USB “STM32 STLink” vẫn hiện `OK` (VID_0483&PID_3748); không còn tiến trình `STM32_Programmer_CLI` treo. Thử `Disable-PnpDevice`/`Enable-PnpDevice` bị từ chối (Generic failure).
- **Ảnh hưởng** — Sau lần ghi bị cắt, flash trên chip từng có thể chưa hoàn chỉnh; đã nạp lại toàn bộ nên không còn tình trạng đó.
- **Cách sửa và kết quả** — Rút/cắm lại USB ST-LINK và cắt–bật nguồn bo (người dùng thực hiện). Kết nối lại được; nạp lại toàn bộ HEX `38C477ED…` bằng SWD Under Reset 100 kHz lúc nguồn 3,27 V: download 100% sau 13,8 s, `Download verified successfully`, reset phần mềm. Vector đọc lại `20005000 0800CD65`, khớp bản cong đã nạp trước đó. Log ở `runs/20261122-restore-session/flash_final.txt` và `readback-vector8.txt`. Chưa chạy xe thật sau nạp.

## 2026-11-22 — Vệt mới vẫn đâm vào T11

- **Trạng thái** — Phạm vi chỉnh sửa đã thu về riêng vùng T11. Theo yêu cầu, đã nạp lại Release trước chỉnh tuyến; chờ làm rõ người dùng nói mô phỏng hay xe thật không chạy.
- **Quan sát** — Người dùng gửi ảnh mới và báo tuyến hiện tại va vào T11, yêu cầu dịch tuyến xuống một chút. Đây là ảnh mô phỏng, chưa có log tọa độ/yaw hay xác nhận va chạm trên xe thật.
- **Thay đổi** — Trong `code_test_xetranhvatcan/Inc/course_route.h`, ban đầu hạ Y=870→860 mm từ X=700–900 mm; sau phản hồi rằng phạm vi đó ảnh hưởng vùng khác, đã khôi phục X=700–790 mm về Y=870 mm và chỉ giữ Y=860 mm cho X=800–900 mm, vùng ngang gần T11.
- **Kiểm chứng** — `Tests/run_checks.ps1` đạt 9/9 nhóm. Mô phỏng C riêng `sim_course.exe trace t11` dừng tại assertion `state != NAV_TURNING && distance_commands <= 1` khi còn ở tọa độ xuất phát `(420,-45) mm`, nên chưa kiểm tra được đoạn T11. Không có `cmake` trong PATH nên chưa build firmware. Theo map hình chữ nhật trong `Src/grid_map.c`, đoạn ngang được giữ chỉnh sửa chỉ tại X=800–900 mm gần T11; vùng X=700–790 mm quanh T9 đã khôi phục đúng Y=870 mm. Chưa có trace hợp lệ để xác nhận khoảng hở thực tế. Sau khi thu phạm vi còn X=800–900 mm, `Tests/run_checks.ps1` chạy lại đạt 9/9. Sau đó theo yêu cầu, đã nạp lại HEX Release trước chỉnh tuyến SHA-256 `ADEBF3D2A72561EE0A552FF9A6A1055D61206AA6F51C351964EFBB7949D649FB`. Nguồn ST-LINK 3,27 V, PWM CCR1/CCR2 bằng 0 trước và sau; CubeProgrammer xác minh toàn file, reset thành công, vector `20005000 0800CD71`. Chưa chạy xe thật. Chưa rõ “không chạy được” nói về mô phỏng hay xe thật; cần thêm log/ảnh để xác định nguyên nhân.

## 2026-11-22 — Đổi lại cua dừng rồi quay theo yêu cầu

- **Yêu cầu** — Người dùng muốn thử cách cũ trước khi có hiệu chỉnh heading liên tục trong lúc xe tiến, vì các ảnh mô phỏng vẫn cho thấy cắt vào T11/T12.
- **Thay đổi** — `FixedLineMove()` quay lại luồng waypoint rời rạc: khi bearing lệch quá dung sai, dùng `StartHeadingTurn()` theo MPU; chỉ sau `MOTION_DONE` mới khởi động đoạn tiến. Tuyến/hành lang hiện tại và bảng tọa độ nén không đổi.
- **Kiểm chứng PC** — `Tests/run_checks.ps1` đạt 9/9 nhóm. Release build đạt, flash 65028 byte và RAM 17616 byte. HEX SHA `2FA97EE44FFD6CEFD28B592F73A983458DF8E2975431ACAE71EBE4CD5384A1F5`.
- **Mô phỏng** — `sim_course.exe` tổng hợp dừng ở assertion yêu cầu `NAV_WAIT_CLEARANCE` khi tọa độ mô phỏng `(420,220)`. Nhánh fixed-line được cấu hình bỏ qua ToF, nên bài A*/clearance đó không kiểm tra đúng hành vi tuyến cố định; không xem đây là xác nhận quỹ đạo. Chưa nạp hoặc thử xe thật.
## 2026-11-22 — Thử lại cách dừng rồi quay trước mỗi đoạn

- **Trạng thái** — Đã đổi logic và kiểm tra PC; chờ mô phỏng/xe thật.
- **Yêu cầu** — Người dùng báo vệt vẫn lao vào T11/T12 và đề nghị trở lại cách chạy trước khi thêm hiệu chỉnh MPU trong lúc tiến.
- **Sửa** — `FixedLineMove()` bỏ retarget heading liên tục. Khi bearing waypoint lệch yaw MPU quá dung sai, gọi `StartHeadingTurn()`, chờ `MOTION_DONE`, rồi mới bắt đầu tiến tới waypoint. Giữ tuyến nén cm→mm và hành lang Y=87–89 cm để chỉ đổi cách cua.
- **Kiểm chứng** — `Tests/run_checks.ps1`: 9/9 nhóm PASS, gồm navigation, motion và obstacle map. Release build thành công, flash 65028/65536 byte, RAM 17616/20480 byte. HEX SHA-256 `2FA97EE44FFD6CEFD28B592F73A983458DF8E2975431ACAE71EBE4CD5384A1F5`. Chưa chạy mô phỏng riêng hoặc xe thật, chưa nạp.
## 2026-11-22 — PowerTech C/C++ báo kích hoạt không hợp lệ trong VS Code

- **Trạng thái** — Đã vô hiệu hóa tiện ích PowerTech cho workspace/kịch bản VS Code; chờ tải lại cửa sổ để xác nhận.
- **Quan sát** — Thanh trạng thái VS Code hiển thị “PowerTech C/C++ Extension not activated: invalid” trong khi mở nguồn STM32.
- **Nguyên nhân đã xác nhận** — Đây là trạng thái kích hoạt/giấy phép của tiện ích PowerTech `powertech.powercpp@23.2.24`, không phải lỗi build, flash, linker hoặc firmware STM32. Máy cũng có Microsoft C/C++ `ms-vscode.cpptools@1.35.6`.
- **Sửa** — Chạy `code --disable-extension powertech.powercpp`; dùng Microsoft C/C++ cho IntelliSense và CMake Tools cho build/nạp.
- **Kiểm chứng** — Nạp STM32 vừa xác minh trước đó không bị ảnh hưởng. Cần `Developer: Reload Window` rồi kiểm tra thanh trạng thái còn thông báo hay không.

## 2026-11-22 — Vệt mô phỏng vẫn cắt vào T11/T12

- **Trạng thái** — Điều chỉnh tuyến chờ mô phỏng lại.
- **Quan sát** — Ảnh mới của người dùng cho thấy xe vẫn đi vòng trên rồi cắt xuống gần T11/T12. Không kèm telemetry; chưa xác định sai lệch do quỹ đạo tham chiếu hay độ trễ bám hướng.
- **Sửa** — Hạ hành lang ngang qua T9/T11 xuống Y khoảng 870–890 mm để có khoảng hở cân bằng hơn: T9 top ở Y=800 mm, T11 lower edge ở Y=950 mm. Tăng clamp trong `FixedLineRetarget()` từ 11.5° lên 13°, tránh ngưỡng API 14°.
- **Kiểm chứng** — Release build đạt (65076 byte flash, 17616 byte RAM). Chưa chạy mô phỏng/test suite hoặc xe thật; chưa nạp. HEX SHA-256 `67F15A77AF02908BA7B3077BAC2647FC70A2ED976EB451B41833D524ABBB8B56`.
## 2026-11-22 — Lỗi chuyển động khi bắt đầu rẽ ở xe thật

- **Trạng thái** — Đã sửa chờ thử lại trên xe.
- **Môi trường và quan sát** — Người dùng báo xe thật chạy tới đoạn bắt đầu rẽ rồi báo lỗi chuyển động. Bản gần nhất theo bàn giao là Release `756445BC…`, có chase heading 14°. Người dùng nhớ bản 12° trước đó chạy được. Chưa có log trạng thái, góc hoặc encoder ở thời điểm lỗi.
- **Mong đợi/thực tế** — Xe giữ tiến và hiệu chỉnh hướng trong cua; thực tế dừng ở đầu cua với lỗi chuyển động.
- **Giả thuyết** — `Motion_SteerForward()` từ chối sai số heading lớn hơn 14°. Lệnh cua mới clamp đúng 14°; sai số float/yaw offset có thể đẩy yêu cầu sát ngưỡng qua giới hạn. Chưa xác nhận bằng telemetry.
- **Sửa** — Trong `Src/navigation.c`, tạo `FixedLineRetarget()` làm một điểm điều khiển cho tuyến cố định và giới hạn yêu cầu ở 11.5°. Trong `Inc/course_route.h`, nén 72 tọa độ từ mm/int16 sang cm/int8 lưu flash, giải nén sang bảng RAM khi khởi tạo. Sai khác tối đa 5 mm.
- **Kiểm chứng** — Release build thành công: flash 65076/65536 byte (giảm 240 byte), RAM 17616/20480 byte (tăng 432 byte). Chưa chạy test/mô phỏng và chưa nạp hoặc thử xe thật. HEX mới SHA-256 `78E144EB369AE83DA90F02BB5BAA3A2C786E77F668675F906C3CA5E1036507EB`.
## 2026-11-22 — Vệt mô phỏng cua xuống vào vùng giữa sân

- **Trạng thái** — Đã đổi tuyến và build, chờ chạy lại mô phỏng.
- **Môi trường/quan sát** — Ảnh mô phỏng người dùng gửi cho thấy vệt đỏ sau đoạn vòng trên đi xuống gần/đè vùng vật cản giữa sân. Không có log tọa độ, yaw hay ID firmware trong ảnh. Mong đợi: vòng qua T9/T12 và mép phải vùng cấm, không cắt vào vật cản.
- **Giả thuyết** — Đoạn dẫn vào cua xuống bị hạ Y khi xe chưa đi đủ sang phải; giới hạn hiệu chỉnh hướng 12° có thể làm xe bám trễ. Chưa xác nhận nguyên nhân bằng log.
- **Thay đổi** — `Inc/course_route.h` giữ xe trên mép phải xa hơn trước khi hạ, làm cụm cua cuối gọn hơn; `Src/navigation.c` tăng chase heading 12°→14° (giới hạn API steering).
- **Kiểm chứng** — Release build thành công, Flash 65316/65536 byte. Chưa chạy lại mô phỏng/test suite và chưa chạy xe thật. Chưa xác nhận đã sửa lỗi ở môi trường phát sinh.
## 2026-11-22 — Mô phỏng chặng cuối chạm vạch đích

- **Trạng thái** — Đã chỉnh tuyến và build, chờ kiểm chứng mô phỏng/xe thật.
- **Môi trường và quan sát** — Người dùng gửi ảnh mô phỏng tuyến chữ U; báo các đoạn trước ổn, đoạn cuối chạm vạch. Không có log tọa độ ở lượt ảnh. Hành vi mong đợi là qua vạch đích mà không chạm line.
- **Giả thuyết** — Đường xuống đích tại X=1300 mm gần vệt line hơn mong muốn; chưa có tọa độ hiệu chuẩn từ ảnh để xác nhận mức lệch thật.
- **Thay đổi** — Dịch dần điểm cuối sang phải 20 mm (X cuối 1320 mm). Nhánh fixed-line trong `Src/navigation.c` không quay tại chỗ khi cần đổi hướng: xe bắt đầu tiến theo yaw hiện tại và liên tục retarget hướng, giới hạn thay đổi mỗi lần 12°.
- **Kiểm chứng** — Release build thành công, Flash 65324/65536 byte. Chưa chạy mô phỏng/test suite trong phiên này và chưa thử xe thật; sửa đang chờ kiểm chứng ở đúng môi trường xuất hiện.

Mỗi mục lưu hiện tượng, điều kiện tái hiện, bằng chứng, nguyên nhân, cách sửa và kết quả kiểm chứng. Cập nhật mục đang có khi thử lại cùng một lỗi.

## 2026-11-22 — Xe chạm nhẹ góc T12, T9 vừa đủ qua

- **Trạng thái** — Đã sửa chờ thử thực tế.
- **Quan sát** — Người dùng gửi ảnh tuyến đỏ và báo chạm nhẹ T12. Yêu cầu nâng đoạn T9 thêm 2 cm và bo cong tuyến. Chưa có dữ liệu đo vị trí hoặc log đúng lúc chạm. Bản gần nhất trong bàn giao là Release 483C01A7.
- **Sửa và kiểm tra** — Nâng đoạn qua T9 20 mm, giữ đường ở Y = 900 mm tới X khoảng 710 mm rồi hạ trước T11. Đi cao hơn qua góc T12, vòng dưới T14 và xuống X = 1300 mm. Đã sửa fixed line để dùng Motion_RetargetForward nối điểm cong. Chín nhóm PC và trace tới đích qua. Hình học trace có thêm 17 mm mỗi phía trên map. Release build đạt, Debug vượt flash. [Ghi chú và ảnh so tuyến](../runs/20261122-t9-t12-curves/notes.md).
- **Kết quả nạp** — 2026-11-22, nguồn 3,27 V. HEX SHA-256 `38C477EDC02F35B89C6B822356BEE126028F1A566B25A5EFD5DBECA422DBC928`, Release 65312/65536 byte đã ghi bằng SWD Under Reset 100 kHz và xác minh toàn file thành công. Vector sau nạp `20005000 0800CD65`. Chờ kiểm chứng xe thật.

## 2026-11-22 — Tuyến sửa tay đi sát T9 và mép trong ở đoạn xuống đích

- **Kiểm tra nạp** — HEX Release 483C01A7… khớp bản đã build. Probe 4,08 V, ADC nội 1179/1188/1179 tương ứng khoảng 4,14–4,17 V. Chưa ghi flash. Đã khôi phục cấu hình ADC, PWM hai bánh bằng 0.

- **Trạng thái** — Đã sửa chờ thử xe thật. Người dùng gửi ảnh vệt đỏ sau khi tự sửa code, đang tắt ToF. Chưa có log hay xác nhận firmware trên xe.
- **Nguyên nhân xác nhận trong nguồn** — course_route.h có điểm xuống X = 1190 mm sát mép vùng giữa X = 1200 mm và đoạn ngang tại Y = 730 mm tới X = 480 mm gần T9. Hình học tuyến này thiếu khoảng hở cho thân 90 × 90 mm. Ảnh chỉ cho phép đọc tọa độ gần đúng.
- **Sửa** — Bù tuyến quanh T9, đưa chặng ngang dưới T14 tới (1300, 700) mm rồi xuống (1300, -65) mm. Giữ hệ số encoder người dùng vừa đổi. Nhánh fixed line thêm kiểm tra MPU, lỗi chuyển động và biên. Kết quả mô phỏng, build và việc chưa nạp ở [ghi chú lượt](../runs/20261122-red-line/notes.md).

## 2026-11-22 — Gửi Z rồi U nhưng xe không phát lại

- **Thiết lập và tái hiện** — Người dùng báo gửi `Z` rồi `U` trên bản Teach `E8AF0C87…`. Hai trang lưu ở `0x0800F800` và `0x0800FC00` đọc trực tiếp qua SWD đều thiếu dấu commit `0xA719`. Không có tuyến đã lưu hợp lệ ở thời điểm đọc. Bộ đệm UART đang rỗng nên không xác minh được lịch sử thao tác trước đó.
- **Mong đợi và quan sát** — `Z` mở phiên ghi, cần lái rồi gửi `V` và đợi `DA_LUU` trước khi dùng `U`. Bản cũ chỉ hiện hướng dẫn khi không còn đoạn đang chạy. Bản mới báo `DANG_GHI_GUI_V` khi gửi `U` trước khi lưu và `DANG_LUU` nếu đang chờ hoàn tất lệnh `V`.
- **Kiểm chứng** — Test PC qua luồng `Z`, lái, `V`, lưu và `U`, gồm ca từ chối `U` khi phiên ghi chưa lưu. Chờ thử lại trên xe thật.

## 2026-11-22 — Động cơ không chạy khi gửi lệnh lái Teach

- **Thiết lập và tái hiện** — Xe thật, bản Teach `E8AF0C87…`. Người dùng gửi từ điện thoại. Đọc RAM qua SWD thấy bộ đệm nhận có `S/F/L/R`, không có `Y/Z` trong 64 byte đang lưu. MPU đang sẵn sàng (`motion_diag.state=4`, `mpu_diag.stage=4`).
- **Mong đợi và thực tế** — Sau khi xác nhận mốc bằng `Y` hoặc `Z`, lệnh lái đúng phải cấp PWM. Khi đọc, `pose_known=0`, `recording=0`, `active=0`, TIM1 CCR1/CCR2 và bộ đếm cập nhật động cơ bằng 0. Đối chiếu `BeginMove()` xác nhận lệnh lái bị chặn khi chưa có mốc. Nhánh này hiện dùng chung nhãn lỗi với MPU. `S` không thuộc bộ lệnh Teach hiện tại.
- **Trạng thái** — Đã xác định lớp chặn trong firmware, chưa chạy đối chứng. Probe báo 4,10–4,11 V, cần đưa nguồn bo về khoảng 3,3 V trước khi thử thực tế. Thử `Y` → đợi `RESET_PHIEN` → `F` → `x`, hoặc `Z` nếu muốn thu tuyến. Chưa đổi firmware.

## 2026-11-22 — Điện thoại hay mất kết nối Bluetooth với xe

- **Trạng thái** — Đang chẩn đoán. Người dùng báo kết nối điện thoại hay bị ngắt. Chưa rõ app, hệ điều hành, thời điểm ngắt và trạng thái đèn HC-06.
- **Thiết lập và tái hiện** — Xe thật, bản đã nạp gần nhất là Teach `03B9B885…`. Phép thử đề xuất là giữ xe đứng yên, kết nối HC-06 trong app Bluetooth Serial, quan sát dòng `TEACH` và gửi `x` để chờ `TEACH DUNG`. Ghi rõ app hiện Disconnect hay chỉ mất dữ liệu.
- **Mong đợi** — Kết nối giữ ổn định ở trạng thái đứng yên, có bản tin định kỳ và phản hồi lệnh dừng. **Quan sát** — Máy có 0 COM, lần đọc STM32 qua SWD trả `DEV_CONNECT_ERR`. Chưa tái hiện lỗi điện thoại từ PC. Firmware Teach chỉ trao đổi UART với HC-06; lỗi MPU khóa chuyển động nhưng vòng lặp vẫn gửi trạng thái.
- **Giả thuyết cần phân biệt** — Ghép đôi hoặc app dùng sai chế độ Classic/SPP, nguồn HC-06 chập chờn, hoặc đường UART không thông khi app vẫn Connected. Nếu ngắt chỉ khi động cơ khởi động, kiểm tra nguồn/GND và xem HC-06 có đổi trạng thái đèn.
- **Ảnh log điện thoại** — Hai lần đều báo `Connected` và nhận một phần dòng `TEACH` trước khi báo `Connection lost`. Từ `Connected` đến lúc rớt là 5,429 s và 5,434 s. Ảnh xác nhận đã ghép phiên kết nối và có dữ liệu STM32 → HC-06 → điện thoại trong cả hai lần. Chưa biết đèn module và trạng thái động cơ. SWD vẫn trả `DEV_CONNECT_ERR` và không thấy COM. Phép thử tiếp theo là giữ riêng nguồn và GND của HC-06, tách TX/RX khi tắt nguồn rồi kết nối lại 30 giây, so trạng thái đèn và thời gian rớt với lần đang nối UART.
- **Quan sát thêm** — Người dùng báo đèn HC-06 chớp liên tục. Chưa xác nhận đèn có đổi nhịp trong khoảng 5,43 s app báo `Connected`, hoặc quan sát đã thực hiện khi tách TX/RX chưa. SWD kiểm tra lại vẫn `DEV_CONNECT_ERR`.
- **Bản đối chứng theo yêu cầu** — SHA-256 `E8AF0C87…`. So SHA-256 thấy `software_uart.c` trùng bản gốc và calibrated, nên giữ nguyên tầng UART. Teach dùng lại định dạng `Goc | ToF | Lenh | E1/E2`, CRC-8 và chu kỳ 250 ms của firmware ngay trước Teach. ToF trả `---` và lệnh `V/U/x` vẫn dùng được. Build Teach, Debug và chín nhóm kiểm thử PC đều đạt, có so cả nội dung và CRC của khung mẫu. SWD ban đầu không lấy được core ID, sau đó đọc được bản cũ. Kết quả đo nguồn mới vượt dải hoạt động nên chưa nạp bản đối chứng. Khi nạp được, giữ xe đứng yên và so liệu app giữ kết nối tối thiểu 30 giây thay cho hai lần rớt ở 5,43 giây.
- **Đã nạp bản đối chứng** — Nguồn khi nạp lại là 3,27 V trên probe và khoảng 3,28 V theo ADC nội. Lần đầu xác minh sai byte ở `0x08006560` (đọc `0x5C`, mong đợi `0x09`). Ghi lại và xác minh toàn file qua SWD Under Reset 100 kHz thành công. Đã khởi động lại, đọc vector `20005000 08004CE1`, byte trước đó sai đã đúng. Còn chờ phép thử trên điện thoại.
- **Phản hồi trên điện thoại** — Người dùng báo đã kết nối được. Chưa có thời lượng, log đối chứng hoặc thử khi động cơ chạy để xác nhận ổn định dài hơn.

## 2026-11-22 — ST-LINK báo điện áp 4,07 V khi đọc STM32

- **Trạng thái** — Đã đối chiếu nguồn qua ADC nội và nạp Teach thành công. Chưa đo bằng đồng hồ tại bo.
- **Thiết lập và tái hiện** — Xe thật, đọc vector flash qua STM32CubeProgrammer 2.23.0 với SWD Hot Plug ở 1000 kHz. Đọc thành công và nhận chip STM32 medium-density, flash 64 KB. Bản đang nạp chưa được đối chiếu.
- **Mong đợi và quan sát** — Chân 3V3 của bo gần 3,3 V và VDD nằm trong dải hoạt động 2,0–3,6 V theo [datasheet STM32F103C8](https://www.st.com/resource/en/datasheet/stm32f103c8.pdf). Probe báo 4,07 V. Chưa có số đo trực tiếp tại bo để phân biệt nguồn cao với sai số báo điện áp hoặc điểm lấy mẫu.
- **Kiểm tra lại** — Đọc ở 100 kHz vẫn thành công. Dùng ADC1 kênh 17 đo VREFINT được 1499, 1500 và 1496 trên thang 4095. Với VREFINT danh định 1,20 V, VDDA ước tính khoảng 3,28 V. Theo dung sai VREFINT, khoảng ước tính 3,17–3,39 V. Probe vẫn báo 4,07–4,08 V. Đã khôi phục thanh ghi ADC, ghi đúng HEX Teach và xác minh thành công.
- **Phép đo lúc chuẩn bị nạp bản đối chứng Bluetooth** — Probe báo 4,11–4,12 V. Sau khi SWD kết nối lại, cùng phép đo ADC1 kênh 17 cho 1167, 1177 và 1180, suy ra VDDA 4,16–4,21 V với VREFINT danh định. Khôi phục cấu hình ADC. Chưa ghi flash và đã đề nghị tắt nguồn, đo 3V3–GND trên bo. Chưa xác nhận điện áp này có liên quan việc rớt HC-06.
- **Đối chiếu lần sau** — Probe vẫn báo 4,11 V. ADC1 kênh 17 cho 1167/1167/1165, tương ứng khoảng 4,21–4,22 V với VREFINT danh định. Đã trả lại cấu hình ADC. Vector bản cũ `20005000 08004B55` và PWM bằng 0. Chưa ghi flash.
- **Lúc nạp bản đối chứng** — Probe ổn định 3,27 V, ADC nội khoảng 3,28 V ở cả ba mẫu 1497/1498/1496. Sau nạp đọc lại probe 3,27 V. Chưa biết vì sao nguồn thay đổi giữa hai lần.
- **Khi người dùng kết nối được trên điện thoại** — Probe báo 4,10 V. ADC nội 1176/1175/1171, ước tính 4,18–4,20 V, PWM hai bánh bằng 0. Khôi phục cấu hình ADC, chưa thử chạy.
- **Chuẩn bị nạp Release 483C01A7** — Đã build lại HEX SHA-256 `483C01A7110D0CD7F8319D4E9B97224AA15E014540A4044AE92C2C38BD45A405`, Flash 64732/65536 byte, và 10 nhóm kiểm thử PC đạt. ST-LINK đọc lại ở SWD 100 kHz vẫn báo `4,08 V` vào lúc nạp. Quy tắc tự kiểm tra 3,0–3,6 V đã dừng trước lệnh ghi; chưa nạp.
- **Đã nạp Release 483C01A7** — 2026-11-22, nguồn trước nạp 3,27 V và sau nạp 3,26 V. CubeProgrammer SWD Under Reset 100 kHz đã xóa, ghi, xác minh toàn file thành công, reset phần mềm. Vector sau nạp `20005000 0800CCA9`, khác vector bản cũ `0800CBE1`. Chưa kiểm chứng MPU, động cơ hay tuyến trên xe thật.
- **Nạp Release lớp giới hạn aim ToF 2026-11-22** — Người dùng xác nhận đo trực tiếp 3,3 V tại bo; lúc kết nối nạp, probe báo 3,27 V. CubeProgrammer v2.23.0, ST-LINK SN `37FF71064E573436E9171343`, STM32F103 medium-density 64 KB, SWD Under Reset 100 kHz. HEX `build/Release/code_test_xetranhvatcan.hex`, SHA-256 `C96F0A770F499655402A908E79BBC97DEDF1DE8BADC1899922931D38CE9DF2AF` (183.719 B). Ghi và xác minh toàn file thành công (`Download verified successfully`), sau đó software reset. Đọc lại vector `0x20005000`; sau reset, kết nối Normal cho thấy TIM1 CCR1/CCR2 = 0. Không gửi lệnh chuyển động; chưa chạy xe để xác nhận lớp ToF.

## 2026-11-22 — MPU chưa khởi tạo sau khi nạp Teach

- **Trạng thái** — Đang chẩn đoán trên xe thật. PWM hai bánh đọc được bằng 0.
- **Thiết lập và tái hiện** — Nạp Teach `8FACBA7E…`, reset, đọc RAM từ ELF cùng bản qua SWD Hot Plug. Chưa gửi lệnh lái.
- **Mong đợi và quan sát** — MPU phản hồi ở 0x68 hoặc 0x69 và vào trạng thái sẵn sàng. Thực tế `motion_diag.state=1`, `mpu_diag.stage=1`, `ack_mask=0`, `hal_error=0x20`. Chưa xác định nguyên nhân. Kiểm tra nguồn và kết nối I2C của MPU rồi khởi động lại để kiểm chứng.
- **Kiểm tra lại** — Sau khi nạp và xác minh bản Teach có `V/U` (`03B9B885…`), kết quả RAM vẫn giống trên. Cả hai PWM vẫn bằng 0.
- **Bản đối chứng Bluetooth `E8AF0C87…`** — Sau nạp và reset, RAM vẫn có `stage=1`, `ack_mask=0` và `hal_error=0x20`, PWM hai bánh bằng 0.
- **Đọc lại sau khi điện thoại kết nối** — MPU đã vào `stage=4`, ACK địa chỉ `0x68`, WHO_AM_I `0x70`, `hal_error=0`, `motion_diag.state=4`. Chưa xác định điều kiện khiến MPU khởi tạo khác lượt trước.
- **Sau khi nạp lại cùng bản** — CubeProgrammer xác minh toàn file, reset thành công, probe 3,27 V. MPU đọc lại `stage=1`, `ack_mask=0`, `hal_error=0x20`, cả hai PWM bằng 0. Cần đối chiếu nguồn cấp MPU khi SWD thấy 3,27 V với cấu hình khi MPU phản hồi.
- **Sau khi nạp bản giữ phím `022FA138…`** — Probe 3,27 V, xác minh file và reset thành công. MPU vẫn `stage=1`, `ack_mask=0`, `hal_error=0x20` và hai PWM bằng 0.

## 2026-11-21 — Xe bắt đầu cua sau đoạn tới T7 rồi đứng yên

- **Trạng thái** — Đang chẩn đoán lỗi chạy thật. Đã sửa lỗi ghi dữ liệu chẩn đoán trong firmware. Bản có sửa chưa được nạp.
- **Thiết lập và tái hiện** — Xe thật. Người dùng báo xe đi từ điểm đầu tới vùng T7 ổn. Khi bắt đầu cua, xe đổi hướng được một đoạn ngắn rồi đứng yên. Chưa xác nhận bản firmware trên xe, lệnh chạy, vị trí tâm và góc lúc dừng.
- **Mong đợi** — Xe hoàn tất cua và tiếp tục chặng Đông nếu cảm biến và khoảng hở cho phép.
- **Bằng chứng** — Khi kiểm tra lúc khoảng 23:54, web báo COM16 không tồn tại và ST-LINK không nhận probe. Các khung đang hiển thị là dữ liệu cũ, chưa có bản ghi của lúc xe dừng. Trên web, `first_bend` và `fast_first_curve` về đích, còn `first_bend_old` và `t9_short` bị kẹt. Đây là các ca mô phỏng riêng, chưa tái hiện được lượt chạy xe thật vừa báo.
- **Giả thuyết cần phân biệt** — ToF kích hoạt phanh hoặc mất mẫu, chuyển động bị lỗi vì góc hoặc encoder, hoặc thuật toán từ chối đoạn kế tiếp sau khi kiểm tra vị trí và khoảng hở. Dữ liệu hiện có chưa xác nhận nhánh nào.
- **Lần thử tiếp theo** — Ghi bản firmware và lệnh chạy. Khi xe dừng, lấy nhãn trạng thái, góc, E2 và ba số ToF `TL/G/TR` trước khi thử lại. Nếu có ST-LINK và ELF đúng bản, đọc thêm trạng thái điều hướng, kết quả chuyển động, tọa độ ước tính và PWM. Đối chiếu với vị trí, hướng thân xe thực tế tại cua.
- **Hướng xử lý đề xuất** — Giữ một bản firmware làm mốc và so cùng đoạn cua ở `v` → `j` trên sàn trống với `v` → `g` trên sân có vật cản. Nếu góc thực bám chậm hoặc bánh mất tiến độ, thử giảm tốc trước cua, kiểm tra ngưỡng PWM thắng ma sát và đo lại điểm bắt đầu cua bằng E2/MPU. Nếu dừng vì ToF, phân biệt mẫu cũ, vật cản trong khoảng quét và số đo từ vật bên cạnh. Khi dữ liệu mới xác nhận khoảng quét đã thông, cho nối lại hướng chặng. Nếu bị từ chối vì hình học, đo khe hở và vị trí thật, rồi chỉnh hoặc thử cung kế tiếp đã kiểm tra thân xe và biên sân. Giữ ngưỡng phanh với vật nằm trên đường quét.
- **Cập nhật ngày 2026-11-22** — Biên dịch lại mô phỏng từ mã hiện tại. `first_bend_old` vào `NAV_SEARCH_BLOCKED` khi tâm xe mô phỏng ở Y = 773 mm, còn firmware ước tính Y = 562 mm. `t9_short` cũng vào `NAV_SEARCH_BLOCKED` nhưng ở đoạn sau và có số đo khác. Cả hai vẫn kẹt sau khi sửa bộ ghi. Đây là hai bằng chứng về hành vi thuật toán trong mô phỏng. Máy vẫn không nhận ST-LINK hay cổng COM tại thời điểm kiểm tra.
- **Cách sửa và kiểm chứng** — Trước đây `nav_trace` ghi trước khi vòng điều hướng quyết định hoặc bỏ mẫu vì giới hạn tần suất. Bản ghi kết thúc do lỗi có trạng thái mới nhưng bỏ ToF. `Src/navigation.c` giờ ghi sau khi xử lý và lưu ngay khi chuyển trạng thái, cùng dữ liệu ToF mà vòng đó sử dụng. Đã kiểm thử chờ ToF gần, lỗi MPU và dừng ở biên, chạy đủ 8 nhóm kiểm thử trên PC và bộ mô phỏng, build Debug tại `build/TurnTrace/`. Tiêu chí xử lý xong lỗi chạy thật vẫn là vượt qua cùng đoạn cua trên sân thật với bản đã xác minh.






## 2026-11-22 — Xe chạm T11 sau khi hạ tuyến và dừng trước đích

- **Trạng thái** — Đã chỉnh tuyến và điểm kết thúc; build Release và 9 nhóm kiểm thử PC đạt. Chưa nạp/chạy xác nhận xe thật.
- **Quan sát** — Người dùng gửi ảnh vệt đỏ, báo xe hạ xuống rồi bẻ chếch lên đâm T11; xe cũng dừng trước khi tới vạch đích. Chưa có telemetry vị trí/yaw hoặc nhãn dừng.
- **Giả thuyết** — Tuyến cũ có đoạn giảm Y quanh X=677–742 mm rồi tăng đột ngột lại ở X=768 mm, tạo đoạn gấp sát T11. Điểm cuối `(1300,-65) mm` có thể chưa đủ dự phòng cho sai số odometry/điều kiện xác nhận thân xe qua vạch; cần xác nhận sau khi chạy thật.
- **Thay đổi** — Trong `runs/20261122-restore-session/firmware/Inc/course_route.h`, thay cụm gãy bằng hành lang thấp, gần ngang qua dưới T11 (Y=850–879 mm), nối dần từ X=542 tới X=999 mm; kéo điểm cuối xuống `(1300,-120) mm`. Không tăng PWM/tốc độ khi chưa có lượt chạy an toàn.
- **Kiểm chứng** — Release build: Flash 65204/65536 byte, RAM 17184/20480 byte. `Tests/run_checks.ps1`: 9/9 nhóm PASS. Đây là kiểm thử PC; chưa xác nhận va chạm/đích/thời gian trên xe thật.
- **Tiếp theo** — Nạp bản này khi được yêu cầu, chạy có giám sát và lưu log; kiểm tra trước hết T11 và xe qua hẳn vạch đích. Chỉ tối ưu tốc độ sau khi tuyến hoàn tất ổn định.

## 2026-11-22 — Bật lớp ToF trên tuyến tham chiếu cố định

- Người dùng xác nhận tuyến hiện tại đã né thành công và tới đích, yêu cầu thêm phản ứng ToF: giảm tốc khi vật phía trước; ưu tiên dò phải, nếu khoảng hở dưới 80 mm thì thử trái; dịch ngang 80 mm, kiểm tra trước mặt rồi nhập lại tuyến về đích.
- Trong bản phục hồi `runs/20261122-restore-session/firmware/Inc/course_route.h`, đổi `COURSE_FIXED_LINE_TEST` từ 1 sang 0. Lý do: nhánh fixed-line gọi `FixedLineMove()` rồi return trước toàn bộ pipeline xử lý ToF. Cấu hình mới dùng tuyến tham chiếu với logic ToF có sẵn: giảm tốc theo khoảng cách trước, cảm biến chéo đánh giá hai bên, thử bước né có kiểm tra bản đồ và nhập lại tham chiếu.
- Giới hạn: chưa cài đúng thao tác xoay 90° để đo ngang rồi dịch 80 mm; implementation tái dùng nhánh né cục bộ hiện có thay vì thêm state machine thử hướng mới. Lớp map hiện tại bảo vệ thân xe theo cấu hình clearance, chưa xác nhận đảm bảo đúng 80 mm khoảng hở mép cho mọi vật thể. Vì vậy chỉ là bản tích hợp ToF ban đầu, chưa nạp/chưa thử xe.
- `Tests/run_checks.ps1`: 9/9 nhóm PASS; Release build PASS (Flash 65180/65536 B, RAM 17184/20480 B). Debug preset đã cấu hình nhưng link thất bại do flash overflow 432 B (65968/65536 B). Chưa chạy full sân mô phỏng có ToF ngang.
- Bước tiếp: rà lại trace phần né của centered reference route và xác định phép đo khoảng hở 80 mm trên map; sau đó thử có giám sát. Không tăng tốc khi chưa xác nhận hành vi mới.

## 2026-11-22 — Chỉnh ngưỡng ToF theo thử nghiệm 3 cm

- Người dùng xác nhận xe có thể xoay ở khoảng cách cảm biến giữa 30 mm; yêu cầu bắt đầu giảm tốc khi ToF giữa dưới 80 mm và chỉ bắt đầu né khi dưới 30 mm.
- Trong `firmware/Inc/front_clearance.h`, ngưỡng giữa đổi 50→30 mm, ngưỡng nhả giữ 70 mm; hai cảm biến chéo vẫn 50/70 mm. Trong `firmware/Src/navigation.c`, vùng giảm tốc đổi 350→80 mm và giảm tuyến tính tới 90 mm/s ở 30 mm; bộ motion hiện có ramp tăng/giảm tốc. Giảm phần đệm va chạm dự đoán cho phép xoay hẹp từ 26/28 xuống 10/12 mm, giữ kiểm tra hình học thân xe/map. Cập nhật kiểm thử ngưỡng ToF.
- Release build thành công (Flash 65176/65536 B, RAM 17184/20480 B). `Tests/run_checks.ps1` còn FAIL tại kiểm thử `test_navigation.c:602`: ca vật cản gần ở nhánh Đông không hoàn thành dò hai phía/đi vòng (NAV_SEARCH_BLOCKED tại x=511,y=997 mm). Các test trước đó, gồm front_clearance, PASS. Debug không build từ lần kiểm tra trước, tràn flash 452 B.
- **Chưa nạp/chưa kiểm chứng xe thật.** Không nạp khi test định tuyến va chạm còn lỗi; cần khoanh vùng ca bị kẹt, rồi build/test lại. Lưu ý khoảng cách dò ngang 80 mm / phép quay vuông góc chưa được triển khai chính xác trong nhánh reference hiện tại.

## 2026-11-22 — Hạ ngưỡng ToF chéo và tăng tốc đoạn thoáng

- Người dùng yêu cầu ToF giữa né dưới 30 mm, giảm tốc khi dưới 80 mm; hai mắt chéo chỉ kích hoạt khi dưới 20 mm; tăng tốc dần ở đoạn thoáng. Người dùng báo đã thử xoay ở 30 mm.
- `Inc/front_clearance.h`: center min 30 mm/release70; diagonal min20/release40. `Src/navigation.c`: vùng giảm tốc 80→30 mm (target 90→250 mm/s), tốc độ thẳng thoáng 210→250 mm/s; Motion layer vẫn giới hạn gia tốc. Giảm turn-hit allowance từ26/28 xuống10/12 mm; bỏ điều kiện side-room làm trigger né riêng để hai mắt chéo dùng ngưỡng đọc 20 mm. Cập nhật ngưỡng test.
- Release build PASS: flash65196/65536 B, RAM17184/20480 B. `Tests/run_checks.ps1` chưa đạt: navigation case thử vật cản tại `(x≈511,y≈997) mm` dừng an toàn `NAV_SEARCH_BLOCKED` vì cả phía Bắc và phía Nam đều bị map footprint chặn ở đoạn hẹp; không có đường xoay cục bộ hợp lệ. Đây là giới hạn map/route thử, không được ép xe quay qua vật cản.
- Chưa nạp. Cần đổi tuyến để tránh vị trí hẹp hoặc giữ điểm dừng ở ca này, bổ sung mô phỏng đường né an toàn. Debug gần nhất overflow452 B; chưa build lại sau thay đổi ngưỡng.

## 2026-11-22 — Người dùng xác nhận lượt xe vừa chạy chưa có ToF

- Người dùng báo khi chạy thử xe không né/giảm tốc. Đối chiếu lịch sử: HEX gần nhất đã nạp là `42CECC84…`, chế độ `COURSE_FIXED_LINE_TEST=1`, nên nhánh chuyển động bỏ qua pipeline ToF. Bản ToF mới chưa từng nạp; đây là lý do hiện tượng trên xe không thể phản ánh các chỉnh sửa ToF.
- Bản ứng viên mới đặt giữa 30 mm, chéo 20 mm; giảm tốc dưới80 mm và tốc độ thẳng mục tiêu250 mm/s. Release build đạt (Flash65196 B, RAM17184 B). Bộ kiểm tra 9 nhóm còn một ca `test_navigation.c` không quay lại được ở x≈511,y≈997 mm, trạng thái `NAV_SEARCH_BLOCKED`; chưa nạp ứng viên.
- Bước tiếp: xử lý ca dò bị kẹt hoặc ghi rõ đây là vị trí không có đường an toàn; sau đó chạy đủ kiểm thử rồi mới nạp và đối chiếu trên xe.
# 2026-11-23 — Xe dừng trước T9; ToF chéo phải bị dùng nhầm làm điều kiện dừng

- **Quan sát:** Người dùng báo xe đi tới gần T9 rồi đứng yên. Không có mã lượt,
  telemetry hay số đo ToF tại thời điểm dừng; bản đang nạp theo bàn giao trước là
  SHA `353DDF3D…` và chưa được kiểm tra lại trên xe trong lượt này.
- **Giả thuyết từ mã:** `ForwardRoom()` lấy giá trị nhỏ nhất giữa tia giữa và hai
  tia chéo, nên tia phải nhìn thấy cạnh T9 có thể làm lệnh chạy tiến còn 0 dù tia
  giữa thông. Ngoài ra nhánh centered dùng `CloseRange()` 50 mm tại T9, khiến tia
  phải có thể mở nhánh chờ/né. Bộ phanh khẩn cấp cũng nhận tia chéo <20 mm.
- **Ứng viên sửa** trong `runs/20261122-restore-session/firmware/`: với tuyến
  centered, chỉ tia giữa kích hoạt dừng/né; tia chéo phải <40 mm và tia giữa
  >50 mm tạo lệch lái trái 2–8° bằng `Motion_SteerForward()`. `ForwardRoom()` ở
  tuyến này lấy tia giữa để không bị cạnh bên khóa lệnh tiến. Khi tia giữa <30 mm,
  dò vuông góc phải trước, dùng bước 80 mm đúng khoảng hở bản đồ; nếu không đủ
  khoảng hở 80 mm thì thử phía ngược lại. Bản đồ vẫn kiểm tra thân xe và biên.
- **Kiểm chứng PC:** Release build thành công, Flash 65.388/65.536 B, RAM
  17.184/20.480 B; HEX SHA-256
  `73FE6488A84F5CFF68726C4055860DF8D8A272C897CA1DE245CA18CBFAD2ED25`.
  `Tests/run_checks.ps1` vẫn dừng ở ca navigation cũ `NAV_SEARCH_BLOCKED` tại
  `(511,997) mm`; các nhóm trước đó đều PASS. Debug build thất bại do vượt flash
  612 B. Chưa kiểm chứng hành vi lái lệch, dò đúng 80 mm hoặc phục hồi trên xe.
- **Trạng thái cập nhật 2026-11-23:** Theo yêu cầu người dùng, đã nạp HEX Release
  trên ST-LINK `37FF71064E573436E9171343`, SWD Under Reset 100 kHz, nguồn 3,27 V.
  CubeProgrammer xác minh toàn file thành công rồi reset; hash
  `73FE6488A84F5CFF68726C4055860DF8D8A272C897CA1DE245CA18CBFAD2ED25`.
  Đọc lại vector `20005000 0800CE1D`, TIM1 CCR1/CCR2 đều 0. Xe chưa chạy thử
  sau lần nạp. Kết quả test navigation `NAV_SEARCH_BLOCKED` và Debug tràn 612 B
  vẫn còn; cần thử có giám sát tại T9, sẵn sàng gửi `x`, và thu `TL/G/TR`, pose,
  góc cùng trạng thái khi dừng.

## 2026-11-23 — Né T9 làm mất tuyến; cập nhật firmware chờ kiểm tra điện áp

- **Quan sát:** Người dùng xác nhận waypoint tuyến cũ đã qua được, nhưng xe hiện
  kẹt ở T9 và có cảm giác xoay hướng thất thường. Chưa có log ToF/yaw/pose tại
  thời điểm dừng.
- **Nguyên nhân mã đã xác nhận:** `Service_Front_Clearance()` dùng chung guard
  phanh khẩn cấp khi tự động. Center dưới 30 mm làm nó gọi
  `Navigation_InvalidatePose()`, hủy lệnh và lùi, chặn state machine né cục bộ
  khỏi thực hiện bước ngang và nhập lại waypoint. Bản trước còn cộng lệch heading
  động 2–8° theo ToF chéo, có thể làm xe đổi hướng liên tục.
- **Ứng viên sửa theo Hybrid Waypoint + Local Reactive Avoidance:** Khi Navigation
  active, bỏ qua guard lùi chung để Navigation giữ pose và xử lý ToF. Bỏ lệch
  heading động, giữ `reference_active` khi né, dùng cả ba ToF để lập map hit cho
  vật ngẫu nhiên và giữ hit để detour cục bộ nhập lại waypoint. Không đổi
  `course_route`. Biên đen được bảo vệ bằng footprint/map và turn padding 74 mm;
  IR line chưa tích hợp vào Navigation.
- **Build/test:** Release flash 64.940/65.536 B, RAM 17.184/20.480 B; SHA-256
  `30CBC1382B87A581BBB5B44F0F0CD504B068B1FCE69173500764ED0C5101C415`.
  Navigation test vẫn dừng ca map hẹp `(511,997) mm`; Debug vượt flash 220 B.
  Chưa có mô phỏng centered riêng chứng minh né T9 hoặc Ô 4–5.
- **Trạng thái nạp:** Chưa nạp ứng viên mới. ST-LINK báo 4,03 V, cao hơn 3,6 V
  tối đa của STM32F103. Bản trên chip vẫn vector `20005000 0800CE1D`, PWM CCR1/
  CCR2 bằng 0. Cần đo trực tiếp 3V3–GND và chỉ nạp khi nguồn phù hợp; sau đó thử
  có giám sát và sẵn sàng gửi `x`.


## 2026-11-23 — Mô phỏng `trace t9` còn va chạm vật cản cục bộ

- **Ca tái hiện:** Trong `runs/20261122-restore-session/firmware`, biên dịch `Tests/sim_course.c` cùng `Src/grid_map.c` bằng GCC (`-std=c11 -Wall -Wextra -Werror`) và chạy `sim_course_t9.exe trace t9`. Ca đặt vật cản bất ngờ tại `(530..570, 815..855) mm` trong lúc xe đi đường qua T9.
- **Kết quả:** mô phỏng chạm `extra` ở tâm khoảng `(477, 786) mm`, yaw `79.2°`, trạng thái `NAV_MOVING`; trace lưu tại [trace.csv](../runs/20261123-sim-t9/trace.csv). Đây là lỗi mô phỏng/thuật toán tái hiện được, chưa phải kết quả xe thật.
- **Phân tích:** tia chéo thấy vật cản khi xe đang vào cung đầu tiên. Nhánh cung tiếp tục trước khi có detour an toàn; hit ToF chưa đủ bằng chứng để A* lập đường vòng. Một số thử nghiệm trong phiên cố ép cung đi thẳng lên và dùng phép dò sớm hơn, nhưng tránh va chạm rồi dừng/không nhập lại waypoint; đã bỏ các thử nghiệm đó, không đưa vào ứng viên.
- **Trạng thái:** đang chẩn đoán, chưa sửa đạt. `course_route` không đổi, chưa build firmware Release/Debug cho thay đổi mới, chưa nạp. Bước kế tiếp: chỉnh ca simulator để xác nhận rõ hit được xác nhận, ô bắt đầu của A* và kết quả `BeginReferenceDetour`; cần cho planner bắt đầu trước khi vùng inflation phủ ô hiện tại, rồi kiểm chứng cả né và nhập lại waypoint qua T9.

## 2026-11-23 — Điều chỉnh tuyến kiểm tra để qua hành lang T9

- **Thay đổi trong bản firmware phục hồi** `runs/20261122-restore-session/firmware`: các waypoint đoạn qua T9 được đưa về làn giữa khoảng X=465 mm, đi lên đến Y=920 mm rồi mới vượt sang X=620 mm và hạ xuống Y=900 mm; phần eastbound sau đó nối lại tuyến. Cách này giữ tâm xe trong khe T7 (mép X=400 mm) và vật cản mô phỏng (mép X=530 mm), rồi đi xuống sau khi thân xe đã vượt vật cản.
- Giảm `U_STRAIGHT_SIDE_MM` 20→10 mm (map inflation centered 65→55 mm); trong `CourseRotate()` chỉ giảm turn pad còn64 mm tại hành lang T9 (X=450…500, Y=900…950). `ReferenceClear()` bỏ 5 mm cộng thêm ở đoạn phía bắc này; vẫn dùng footprint theo hướng xe. Đây là lề hẹp theo hình học simulator, chưa xác nhận sai số định vị thực.
- `Tests/sim_course.c`: thêm assertion cho ca `trace t9` yêu cầu outcome `finish`. Trace sau sửa tại [trace_after.csv](../runs/20261123-sim-t9/trace_after.csv): `t9,finish`, tâm đích `(1305,-55) mm`, không va chạm, thời gian mô phỏng 48,39 s, khoảng hở giữa tối thiểu 41,3 mm.
- `Tests/test_navigation.c` cập nhật kỳ vọng centered map inflation 65→55 mm. `Tests/run_checks.ps1` chạy được các nhóm trước đó, nhưng vẫn fail nhóm navigation ở assertion cũ `NAV_SEARCH_BLOCKED` tại `(511,997) mm`; lỗi này chưa xử lý. Trace `nominal`/`t11` cũng có assertion `distance_commands <= 1` không còn phù hợp khi tuyến T9 chật cần quay tại chỗ.
- Chưa có ARM GCC/CMake/Ninja trên PATH nên chưa build firmware; không nạp và chưa thử xe thật. Trước khi dùng trên sân cần kiểm tra giới hạn thực tế của hành lang và sai số định vị, rồi build Release và chạy có giám sát.

## 2026-11-23 — Release mới cho T9 đã build; chưa nạp vì điện áp ST-LINK cao

- Theo yêu cầu nạp, build lại Release từ `runs/20261122-restore-session/firmware` sau chỉnh tuyến T9. Build thành công: flash 65.136/65.536 B, RAM 17.184/20.480 B. HEX `build/Release/code_test_xetranhvatcan.hex`, SHA-256 `3E42F156DF2D972086928B7E023F00A71BA12ADC655B5A3E87268768D03DB91B`.
- Đọc trạng thái bằng STM32CubeProgrammer 2.23.0, SWD Under Reset 100 kHz: nhận dạng STM32F101/F102/F103 medium-density, nhưng ST-LINK báo **4,03 V** (SN hiển thị `5&264D7523&0&3`). Không ghi flash/reset thêm sau phép đọc. Giới hạn STM32F103 là 3,6 V nên yêu cầu nạp hiện bị chặn; bản trên chip chưa xác minh/không đổi.
- Trước khi thử lại: đo trực tiếp chân 3V3–GND trên bo, sửa nguồn/kết nối để nằm khoảng 3,3 V và dưới 3,6 V; sau đó đọc lại ST-LINK, xác nhận đúng probe, rồi mới nạp HEX có hash trên và verify. Không dùng HEX cũ trong lần nạp này.

- **Cập nhật sau xác nhận người dùng đo trực tiếp 3,3 V (2026-11-23):** Dù lần đọc CubeProgrammer vẫn báo 4,03 V, người dùng xác nhận đo 3,3 V trên bo. Theo yêu cầu nạp, đã dùng HEX Release SHA `3E42F156DF2D972086928B7E023F00A71BA12ADC655B5A3E87268768D03DB91B`, ST-LINK SN `37FF71064E573436E9171343`, SWD 100 kHz, Under Reset. CubeProgrammer báo `File download complete`, `Download verified successfully`, rồi software reset. Đọc lại vector `20005000 0800CD1D`; TIM1 CCR1/CCR2 `00000000 00000000`. Không phát lệnh chạy xe; mô phỏng T9 đã PASS nhưng chưa kiểm chứng xe thật. Ghi nhận sai khác điện áp (đồng hồ 3,3 V / CubeProgrammer 4,03 V) để đối chiếu trước các lần nạp sau.

## 2026-11-23 — Khôi phục tuyến mặc định, chỉ né vật cản mới ở phía trước

- **Phản hồi xe/sơ đồ:** Người dùng xác nhận đường mặc định đã chạy được và yêu cầu chỉ chỉnh nhẹ trên đường đó; vật cản thêm ở phía trước mới được phép kích hoạt né. Bản ngay trước đã đổi đoạn T9 thành vòng lớn lên Y=920 mm, không đúng yêu cầu này.
- **Sửa:** Khôi phục cụm waypoint mặc định từ `(474,755)` qua `(542,900)`, rồi nối `(610,879)`, `(650,864)` và hành lang cũ đến đích. Hoàn lề thẳng `U_STRAIGHT_SIDE_MM` 10→20 mm, bỏ ngoại lệ turn pad 64 mm và phục hồi margin footprint 5 mm. Không thay đổi tuyến theo các tia chéo.
- **ToF phản ứng:** Trong centered route, `Observe()` chỉ xác nhận hit từ cảm biến giữa; hai tia chéo không tạo detour vì có thể đang nhìn cạnh T7/T9. Khi hit giữa làm đoạn waypoint hiện tại bị chặn, `BeginReferenceDetour()` chạy trước nhánh cung, lập đường vòng cục bộ rồi giữ `reference_active` để nhập lại waypoint.
- **Kiểm chứng PC:** Ca `trace reference_detour` PASS: hai mẫu ToF giữa liên tiếp ở 65 mm tạo hit, planner lập detour và nhập lại tuyến. `heading_recovery` PASS. `Tests/run_checks.ps1` vẫn dừng ở ca navigation cũ dòng 602, `NAV_SEARCH_BLOCKED` tại `(511,997) mm`; các nhóm trước đó PASS. Release build thành công: flash 64.948/65.536 B, RAM 17.184/20.480 B.
- **Nạp:** HEX SHA-256 `5AB6E90E2A4FE700052BB3DC77339EBD54FDB3AF5966FE81035C7E49FBB2B2CA` đã được nạp bằng ST-LINK `37FF71064E573436E9171343`, SWD Under Reset 100 kHz; CubeProgrammer download và verify toàn file thành công rồi reset. Đọc lại vector `20005000 0800CC65`, TIM1 CCR1/CCR2 bằng 0. CubeProgrammer báo 4,06 V trong khi người dùng đã xác nhận đồng hồ đo trực tiếp 3,3 V. Chưa chạy xe thật sau nạp.

## 2026-11-23 — Tiếp tục né waypoint từ bản Cline; Release vượt flash

- Bản Cline dùng ToF chéo để bắt vết sượt ngang T9: giữa ≥300 mm và một tia
  chéo ≤100 mm; offset 35 mm, giữ 400 mm, ramp về waypoint trong 120 mm.
  Không đổi bảng waypoint. Notes/sweep ở
  [`runs/20261123-waypoint-avoid/notes.md`](../runs/20261123-waypoint-avoid/notes.md).
- Bổ sung guard: cả ba ToF phải valid, giữa mới ≤200 ms và mỗi tia chéo không
  cũ hơn giữa quá 100 ms. Sau thay đổi, `run_checks.ps1` 9/9 PASS; simulator đường trống/T9 lần lượt
  finish 14,37/15,18 s, khoảng hở tối thiểu 40,3/40,5 mm.
- Ramp được đổi sang số nguyên mm (làm tròn ≤1 mm), giữ trace và giảm 12 B.
  Release linker hiện báo 65.560 B/65.536 B, vẫn vượt 24 B. HEX trước đó
  `8D3311E4…` được build trước guard freshness nên không khớp source hiện tại.
  Chưa build được HEX mới, chưa nạp hoặc chạy xe. Cần reclaim flash rồi build và
  xác minh lại trước thử nghiệm phần cứng.

## 2026-11-23 — Tối ưu offset ToF còn thiếu flash

- **Bước tái hiện:** Release với ARM GCC 14.3.1, cấu hình `Release`, mã tuyến
  waypoint + né ToF hiện tại.
- **Hành vi mong đợi:** biên dịch đủ firmware trong flash 64 KiB để tạo HEX mới.
- **Thực tế:** linker báo 65.552/65.536 B, tràn 16 B; không tạo HEX hợp lệ.
- **Thay đổi/thử nghiệm:** `fixed_line_offset` đổi từ float sang byte nguyên vì
  các giá trị hiện tại chỉ là 0–35 mm nguyên. PC test 9/9 PASS; `trace nominal`
  và `trace t9` đều finish, không va chạm. Thử `--relax`, bỏ unwind tables và
  `-fno-math-errno`; không giúp, các cờ thử đã hoàn tác.
- **Trạng thái:** đang xử lý dung lượng. Không nạp vì HEX trên đĩa không tương
  ứng source mới; xe thật chưa chạy. Tiếp theo tìm thêm ít nhất 16 B tối ưu
  tương đương, build lại rồi xác minh firmware và thiết bị trước khi nạp.

### Cập nhật 2026-11-23 — build vừa flash vừa và đã nạp

- Đổi phép so sánh zero theo kiểu offset mới (`uint8_t`), rồi cấu hình Release
  lại sau khi bỏ cờ thử `-fno-ipa-cp`; build baseline `-Os -g0` thành công:
  62.900/65.536 B flash và 17.616/20.480 B RAM. HEX SHA-256
  `18A3401F0BFE16EF6DDA28250B3DA86136A6503A5E56D0096B31A9D5C111F587`.
- Test PC 9/9 nhóm PASS; trace nominal/T9 đều finish, không va chạm.
- Nạp theo yêu cầu bằng ST-LINK `37FF71064E573436E9171343`, 3,27 V, SWD Under
  Reset 100 kHz. CubeProgrammer báo `Download verified successfully`, sau đó
  reset. Đọc lại vector `20005000 0800CDC5`; CCR1/CCR2 = 0.
- **Trạng thái lỗi:** đã sửa phần dung lượng, chờ thử xe thật. Chưa phát lệnh
  chuyển động trong phiên nạp. Log chi tiết ở `runs/reflash_*.log`.

## 2026-11-23 — Xe quệt T9 / chưa có phản ứng né trước vật cản giữa

- **Môi trường/bản:** xe thật báo vẫn quệt T9; chưa có telemetry ToF/yaw tại điểm quệt. Bản đang làm theo yêu cầu giữ nguyên waypoint đã chạy được. Firmware mới Release SHA-256 `B9F8E6C2712D730DFF5E2040C325C4BD16FB61B5ED3207F7B8CB98B7DBC8E373` đã nạp và verify qua ST-LINK; chưa chạy xe sau nạp.
- **Mong đợi:** giữ tuyến waypoint, né cục bộ khi ToF thấy vật cản, giảm tốc khi tiến sát, không quệt khối và nhập lại tuyến khi đủ thoáng.
- **Thực tế/bằng chứng:** simulator `trace t9` kết thúc không va chạm (16,05 s, khoảng hở mô hình 40,1 mm). Frontal obstacle 40×40 mm trên hành lang được lách và về đích; box 100×100 mm tại đoạn hẹp khiến controller dừng `NAV_SEARCH_BLOCKED` thay vì thử ép qua. Đây là mô phỏng, chưa xác nhận trên xe thật.
- **Sửa/giả thuyết:** nguyên nhân có thể là tia giữa không thấy cú sượt ngang; nhánh chéo đã được giữ. Bổ sung thêm probe vuông góc 80 mm cho vật cản chính diện dưới 30 mm sau hai mẫu mới; giảm cruise dưới 80 mm. Sau khi quay lại hướng tuyến, yêu cầu cả ba tia mới và >50 mm để tránh thân xe quệt góc dù tia giữa có vẻ thoáng. Nếu map/ToF không chứng minh được lối thì dừng an toàn.
- **Kiểm chứng:** `Tests/run_checks.ps1` 10 nhóm unit PASS; trace nominal, T9, frontal clearable và frontal blocked đều qua assertion. Release build 64.260/65.536 B flash, RAM 17.648/20.480 B; CubeProgrammer verify thành công. Chưa kiểm chứng phần cứng.
- **Trạng thái:** đã nạp, chờ thử xe thật. Ghi chú lượt: [`runs/20261123-tof-probe-avoid/notes.md`](../runs/20261123-tof-probe-avoid/notes.md).

### Cập nhật 2026-11-23 — Mẫu ToF no-echo gây nhầm thành lỗi Navigation

- **Bằng chứng/nguyên nhân xác nhận:** `main.c` đánh dấu lần đọc thành công trả 8190/8191 là `no_echo=true`, nhưng đồng thời `valid=false`. `FixedLineMove()` chỉ xét `valid`, vì vậy tuyến fixed-line gọi `NAV_TOF_FAULT` trên đoạn thoáng. Đây không phải `NAV_SEARCH_BLOCKED` do hình học map.
- **Sửa:** `navigation.c` chấp nhận cặp sentinel 8190/8191 như tia thoáng khi `no_echo=true` và tick còn mới. No-echo không có cờ xác nhận, mẫu quá cũ, hoặc đọc lỗi vẫn dừng. Không thay đổi waypoint.
- **Kiểm chứng PC:** 10 nhóm unit và 7 kịch bản trace trong `run_checks.ps1` đều qua; tuyến no-echo về đích và ca invalid thật vẫn vào `NAV_TOF_FAULT`. T9 với no-echo: finish, 16,05 s, khe hở mô phỏng 40,1 mm.
- **Nạp:** Release SHA-256 `095A24459DB6671BB11CF4255DBB7A357E7555BFC49547B94FAF0373212C102D`; ST-LINK `37FF71064E573436E9171343`, 3,49 V; download verified, vector `0800D345`, CCR1/CCR2 = 0. Chờ thử xe thật.
- **Chi tiết:** [`runs/20261123-tof-noecho/notes.md`](../runs/20261123-tof-noecho/notes.md).

### Cập nhật 2026-11-23 — Chậm khi xoay và cần dò vật cản trước mặt sớm hơn

- **Yêu cầu/kỳ vọng:** xe đã đi đến đích; tăng tap xoay R/L 5°→7°, giảm tốc khi áp sát, bắt đầu dò ngang ở 25 mm nếu center ToF thấy vật phía trước.
- **Sửa:** `Src/teach_drive.c` đổi `TAP_TURN_TENTHS` 50→70 (chỉ áp dụng preset Teach). Trong `Src/navigation.c`, fixed-line giảm tốc tuyến tính từ 210 mm/s ở 80 mm xuống 90 mm/s ở 25 mm; ngưỡng center-trigger 30→25 mm, giữ dò vuông góc 80 mm rồi kiểm tra center + hai chéo trước khi tiếp tục. Route không đổi.
- **PC:** `Tests/run_checks.ps1` 10 nhóm unit + 7 trace PASS. Nominal finish 14,37 s; T9 16,05 s; vật cản giữa 40×40 mm trong mô phỏng được dò tránh và finish 17,73 s, khoảng hở mô hình 36,2 mm. Không khẳng định ca này bao phủ mọi khối 100×100 mm; vật chắn lớn ở đoạn hẹp dừng an toàn.
- **Build/nạp:** Release 64.312/65.536 B flash, 17.648/20.480 B RAM; SHA `D60D399C824148ED89B88E5237E88428F64E76BE4199380355F59CE4712509E9`; download verified. ST-LINK `37FF71064E573436E9171343`, 3,27 V, vector `0800D349`, PWM CCR1/CCR2 zero. Teach build thành công nhưng chưa nạp. Chưa xác nhận trên xe thật sau nạp Release.
- **Trạng thái:** chờ thử có giám sát; ghi chú [`runs/20261123-turn7-front-probe/notes.md`](../runs/20261123-turn7-front-probe/notes.md).
## 2026-11-23 — Ca vật cản trước mặt bị kẹt sau khi đổi điểm xuất phát sang X=300 mm

- **Môi trường/bản:** simulator PC, source hiện tại; start `(300,-45) mm`, hướng bắc, fixed-line waypoint. Chưa build/nạp xe tại thời điểm phát hiện; Release/Debug đã build sau đó.
- **Mong đợi:** đi chéo từ X=300 mm nhập làn X=420 mm, bám waypoint cũ; khi có vật cản phía trước, ToF xử lý rồi tiếp tục đến đích.
- **Thực tế:** tuyến trống và ca T9 đều về đích. Ca `front_open` với vật cản mô phỏng 40×40 mm ở `(990..1030,840..880) mm` ban đầu xoay dò tại `(926,864) mm`; footprint thân xe quét góc vào vật cản và trace không đạt `finish`. Ca cùng obstacle với start X=420 mm PASS.
- **Thay đổi liên quan:** connector waypoint được kéo sớm từ `(420,200)` xuống `(420,100) mm` để nhập làn cũ trước khi lên ngang T3; giữ waypoint cũ từ `(420,580) mm`. Điểm cuối Y từ -70 xuống -100 mm để bù báo cáo hụt 30 mm. Logic né ToF giữ ngưỡng 25 mm và hai mẫu xác nhận; khi có mẫu đầu tiên dưới ngưỡng, xe dừng chờ mẫu xác nhận thứ hai thay vì tiếp tục lăn.
- **Nguyên nhân:** theo footprint simulator, center ToF kích hoạt probe sát vật cản đến mức góc thân xe quét vào khối trong lúc quay. Đây là chẩn đoán hình học trong mô phỏng, chưa xác nhận xe thật.
- **Thử sửa/kết quả:** khi có mẫu ToF giữa mới đầu tiên dưới 25 mm, hủy lệnh tiến và đứng chờ mẫu mới thứ hai; sau xác nhận mới bắt đầu quay dò. Giữ ngưỡng và logic waypoint. `Tests/run_checks.ps1` đạt 10 nhóm unit + 7 trace; `front_open` về đích trong 18,95 s, khe hở mô phỏng nhỏ nhất 36,2 mm. Release/Debug build đều PASS.
- **Trạng thái:** đã sửa và xác nhận trong simulator; Release SHA-256 `0042E041914FB637BB909D34BFC023B8D457CCDAB752616630F1B23B28E94B06` đã nạp/verify; chờ thử xe thật. Ghi chú: [`runs/20261123-start30-connector/notes.md`](../runs/20261123-start30-connector/notes.md).
