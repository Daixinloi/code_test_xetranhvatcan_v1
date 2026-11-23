# Ngữ cảnh bàn giao cho phiên làm việc mới

Người dùng vừa tiếp quản dự án xe STM32 từ bạn. Đọc [bài toán và luật thi](01-bai-toan.md) để biết tiêu chí chấm điểm, [định hướng giải quyết](02-dinh-huong-giai-quyet.md) để biết thứ tự triển khai và [README](../README.md) để biết cây thư mục, toolchain và lệnh chạy nhanh.

Trước khi thu hoặc phân tích log, xử lý lỗi hay thay đổi bộ điều khiển, đọc [quy trình cải thiện](06-quy-trinh-cai-tien.md). Chi tiết định dạng log nằm trong [thiết kế ghi log](05-ghi-log-chay-thu.md).


### 2026-11-22 — Hạ nhẹ tuyến để né T11

- Người dùng gửi ảnh mô phỏng mới, báo tuyến đang va vào T11 và yêu cầu dịch xuống một chút. Không có log tọa độ/yaw; chưa xác nhận trên xe thật.
- Trong `code_test_xetranhvatcan/Inc/course_route.h`, đã khôi phục vùng X=700–790 mm về Y=870 mm và chỉ giữ đoạn Y=860 mm tại X=800–900 mm gần T11; giữ nguyên các điểm chuyển cong.
- `Tests/run_checks.ps1` đạt 9/9 nhóm. `sim_course.exe trace t11` dừng ở assertion khi còn tại tọa độ xuất phát `(420,-45) mm`, nên chưa xác nhận quỹ đạo T11; `cmake` không có trong PATH nên chưa build firmware. Ước tính theo map: tăng khoảng hở T11 10 mm và giảm khoảng hở T9 10 mm. Bước kế tiếp: xem trace mô phỏng hợp lệ cho cả T11/T9 rồi mới cân nhắc nạp; chưa xác nhận xe thật. Chi tiết ở [nhật ký lỗi](04-nhat-ky-loi.md).
### 2026-11-22 — Dịch phải đích và hiệu chỉnh hướng khi xe đang chạy

- Người dùng gửi ảnh mô phỏng, báo đoạn cuối chạm vạch đích; yêu cầu dịch chặng cuối sang phải khoảng 2 cm và không dừng để quay góc. Ảnh là bằng chứng mô phỏng, chưa có log hay xác nhận xe thật.
- Trong `code_test_xetranhvatcan/Inc/course_route.h`, dịch dần các điểm cuối từ X=1190 lên X=1210 mm và kéo đoạn xuống đích từ X=1300 lên X=1320 mm. Trong `Src/navigation.c`, nhánh fixed-line nay khởi động tiến theo yaw hiện tại, khi đang chạy retarget hướng tới waypoint với sai lệch giới hạn 12° mỗi lần cập nhật; bỏ quay tại chỗ cho tuyến này. `x`/các kiểm tra MPU, chuyển động và biên vẫn giữ.
- Release build thành công: Flash 65324/65536 byte (99.68%), RAM 17184/20480 byte. HEX SHA-256 `9383E787DDF70CA58B66FB86ED336B7DEAE62461337B3BC28633CBAA61D894CB` tại `code_test_xetranhvatcan/build/Release/code_test_xetranhvatcan.hex`. Chưa chạy test suite theo phạm vi phiên và chưa nạp.
- Tiếp theo: xem mô phỏng với tuyến cập nhật, sau đó thử có giám sát trên sân và xác nhận khoảng cách tới vạch. Firmware cong trước đó đã nạp nhưng phiên này chưa kiểm tra bản đang nằm trên xe.

### 2026-11-22 — Cua xuống sớm/đụng vùng cấm trong ảnh mô phỏng

- Người dùng gửi ảnh mô phỏng mới: vệt đỏ sau cua đi vào vùng phía trên vật cản giữa sân; đề nghị cua gắt hơn hoặc bẻ cua sớm để tránh va chạm. Chưa có tọa độ xe/log lượt này nên ảnh chỉ xác nhận hiện tượng trong mô phỏng.
- Sửa tiếp `code_test_xetranhvatcan/Inc/course_route.h`: giữ tuyến đi qua mép phải xa hơn trước khi hạ, chỉnh cụm điểm từ vùng `(1180,735)` đến `(1320,616)` mm để cua xuống gọn hơn. Trong `Src/navigation.c`, giới hạn heading chase khi đang chạy tăng 12°→14°, là ngưỡng `Motion_SteerForward` cho phép.
- Release build thành công: Flash 65316/65536 byte (99.66%), RAM 17184/20480 byte. HEX SHA-256 `756445BC45C25C96A997EAD7CD0960D2E055AD4325FBCBF2806DCBD741F635E4` ở `code_test_xetranhvatcan/build/Release/code_test_xetranhvatcan.hex`. Chưa chạy lại mô phỏng/test suite và chưa nạp bản này.
- Bước tiếp: kiểm tra ảnh mô phỏng sau khi dùng HEX mới; nếu vệt còn lệch, lấy log tọa độ/yaw quanh điểm cua. Bản trước SHA `9383E787…` đã được ghi nhận nạp ở lần bàn giao trước; cần xác minh lại firmware hiện trên xe trước khi chạy.

### 2026-11-22 — Lỗi chuyển động khi vào cua, giảm flash tuyến

- Người dùng báo xe thật dừng với lỗi chuyển động lúc bắt đầu rẽ sau khi nạp bản Release `756445BC…` (theo bàn giao ngay trước; cần xác minh hash trên xe nếu còn nghi ngờ). Bản 12° trước đó chạy được; chưa có telemetry hoặc mã `MotionStatus` chi tiết.
- Giả thuyết có căn cứ từ mã: lệnh chase 14° đúng biên `Motion_SteerForward` (từ chối nếu sai số lớn hơn 14°) có thể bị loại vì sai số float/offset. `FixedLineRetarget()` gom điều khiển vào một hàm và giới hạn ở 11.5° để có biên an toàn.
- `course_route.h` giờ lưu tọa độ cm dạng 8-bit trong flash, bung thành mm vào bảng RAM 432 byte ở `Navigation_Init()`. Sai khác tọa độ tối đa 5 mm. Release build: Flash 65076/65536 byte, RAM 17616/20480 byte; giảm flash 240 byte so với bản trước. HEX mới SHA-256 `78E144EB369AE83DA90F02BB5BAA3A2C786E77F668675F906C3CA5E1036507EB`.
- Chưa chạy test suite, mô phỏng hoặc xe thật với bản mới; chưa nạp. Cần thử lại ở đoạn vào cua và ghi `MotionStatus`, góc, E2 nếu lỗi lặp lại.

### 2026-11-22 — Vệt vẫn đâm T11/T12 trong mô phỏng

- Người dùng gửi ảnh mới: vệt vẫn vòng qua phía trên rồi cắt xuống khu T11/T12. Không có log tọa độ, ảnh xác nhận hiện tượng mô phỏng chứ chưa xác nhận số đo xe thật.
- Trong `Inc/course_route.h`, hạ hành lang ngang giữa các chướng ngại xuống khoảng Y=870–890 mm (87–89 cm) để cân khoảng hở dưới T11 và trên T9; giữ cung xuống cuối. Trong `Src/navigation.c`, hàm `FixedLineRetarget()` tăng giới hạn từ 11.5° lên 13°, vẫn thấp hơn ngưỡng từ chối 14°.
- Release build thành công: flash 65076/65536 byte, RAM 17616/20480 byte. HEX SHA-256 `67F15A77AF02908BA7B3077BAC2647FC70A2ED976EB451B41833D524ABBB8B56`. Chưa chạy mô phỏng/test suite và chưa nạp.
- Tiếp theo cần xem lại trace trên mô phỏng với build mới; nếu xe thật còn lỗi, ghi MotionStatus, yaw, E2 và tọa độ lúc bắt đầu cua.

### 2026-11-22 — Trở lại quay tại chỗ trước khi tiến

- Người dùng báo tuyến vẫn bị cắt vào T11/T12 và yêu cầu thử lại cách trước, trước khi thêm hiệu chỉnh hướng khi đang chạy.
- Trong `Src/navigation.c`, bỏ `FixedLineRetarget()`; tuyến cố định nay so hướng waypoint với yaw MPU, dùng `StartHeadingTurn()` khi lệch quá dung sai, chờ quay xong rồi mới tiến. Giữ nguyên nén tuyến và hành lang Y=87–89 cm.
- `Tests/run_checks.ps1`: cả 9 nhóm PASS. Release build thành công: flash 65028/65536 byte, RAM 17616/20480 byte. HEX SHA-256 `2FA97EE44FFD6CEFD28B592F73A983458DF8E2975431ACAE71EBE4CD5384A1F5`.
- Chưa nạp/kiểm chứng xe thật. Bước tiếp là thử mô phỏng tuyến; nếu ổn, nạp và ghi lượt xe mới. Nếu lỗi quay lại, ghi nhãn MotionStatus/yaw/E2 tại thời điểm lỗi.

### 2026-11-22 — Khôi phục dừng rồi quay ở waypoint

- Theo yêu cầu sau khi vệt còn chạm T11/T12, `FixedLineMove()` đã bỏ điều khiển heading liên tục và trở về hành vi trước: nếu bearing lệch dung sai thì dừng/chuyển sang quay MPU, đợi `MOTION_DONE`, sau đó tiến. Tuyến nén và điểm tuyến giữ nguyên.
- `Tests/run_checks.ps1`: 9 nhóm PASS. Release build: Flash 65028/65536 byte, RAM 17616/20480 byte; HEX SHA-256 `2FA97EE44FFD6CEFD28B592F73A983458DF8E2975431ACAE71EBE4CD5384A1F5`.
- Lượt `sim_course.exe` tổng hợp thất bại ở assertion thử `NAV_WAIT_CLEARANCE`; fixed-line mode hiện chủ động bỏ qua ToF, nên assertion của bài mô phỏng A* không đại diện cho tuyến cố định. Chưa có kết quả mô phỏng hình học cho bản mới. Chưa nạp/xe thật.
## Bàn giao phiên gần nhất

### 2026-11-23 — Bổ sung lách ngang ToF và nạp lại Release

- Giữ nguyên bảng waypoint. Trong `code_test_xetranhvatcan/Src/navigation.c`, tuyến cố định giảm tốc khi tia giữa dưới 80 mm; hai mẫu mới liên tiếp dưới 30 mm bắt đầu thử lách vuông góc 80 mm, ưu tiên phải. Sau khi quay lại hướng tuyến, chỉ tiếp tục nếu tia giữa và cả hai tia chéo đều mới và trên 50 mm. Nếu chưa đủ thoáng, thử phía còn lại; nếu map không có lối thì dừng `NAV_SEARCH_BLOCKED`.
- Giữ nhánh tia chéo xử lý tình huống sượt T9; offset đang là 40 mm. Không đổi route/waypoint.
- Mô phỏng PC: `nominal` finish 14,37 s (khe hở 40,3 mm); `t9` finish 16,05 s (40,1 mm); vật cản giữa nhỏ 40×40 mm ở `(990..1030, 840..880)` lách được và finish 17,68 s (khe hở tối thiểu 37,3 mm). Vật cản 100×100 mm tại `(990..1090, 800..900)` khiến hành lang bị chặn; xe dừng `NAV_SEARCH_BLOCKED` ở khoảng `(913,785) mm`, không va chạm trong mô phỏng. Đây chưa phải kiểm chứng xe thật.
- `Tests/run_checks.ps1`: 10 nhóm unit PASS và bốn trace nominal/T9/front_open/extra thoát mã 0. Release build: FLASH 64.260/65.536 B, RAM 17.648/20.480 B. HEX SHA-256 `B9F8E6C2712D730DFF5E2040C325C4BD16FB61B5ED3207F7B8CB98B7DBC8E373`.
- Đã nạp và CubeProgrammer báo `Download verified successfully`; trạng thái probe đọc được 3,26–3,27 V, STM32F103 64 KB, vector sau reset `0800D315`, TIM1 CCR1/CCR2 đều 0. Có một lần log nạp hiện mã USB thay cho serial; `STM32_Programmer_CLI -l` chỉ thấy một ST-LINK (SN `37FF71064E573436E9171343`). Chưa chạy xe thật.
- Bước tiếp theo: thử có giám sát gần T9 và một vật cản nhỏ; đặt xe trên mặt sân phẳng, sẵn sàng gửi `x`. Ghi lại tọa độ/yaw và ba tia ToF nếu dừng hoặc quệt. Ca khối chắn lớn trong hành lang còn chủ động dừng, không tự vòng qua.
- Chi tiết: [ghi chú lượt](../runs/20261123-tof-probe-avoid/notes.md), [nhật ký lỗi](04-nhat-ky-loi.md).

### 2026-11-23 — Sửa no-echo ToF bị báo lỗi trong tuyến cố định

- Người dùng báo xe báo lỗi map khi phía trước thoáng. Đã xác nhận cụ thể trong `main.c`: lần đọc ToF thành công trả `8190/8191` được gắn `no_echo=true`, `valid=false`; `FixedLineMove()` trước đó chỉ chấp nhận `valid`, nên gọi `NAV_TOF_FAULT`. Đây là lỗi phân loại mẫu cảm biến, không phải map chặn đường.
- `navigation.c`: tuyến fixed-line coi `8190/8191` là khoảng thoáng chỉ khi `no_echo=true` và mẫu còn mới. Timestamp cũ, I2C/đọc lỗi (no_echo=false), hay invalid không có sentinel vẫn dừng an toàn. Nhánh tia chéo cũng chấp nhận no-echo mới là thoáng.
- `run_checks.ps1`: 10 nhóm unit và 7 trace mô phỏng PASS; có ca `front_no_echo`, `t9_no_echo`, và `invalid_center` để xác nhận no-echo chạy được còn invalid thật vẫn fault. `t9_no_echo` về đích trong 16,05 s, khoảng hở mô phỏng 40,1 mm.
- Release: FLASH 64.308/65.536 B, RAM 17.648/20.480 B; HEX SHA-256 `095A24459DB6671BB11CF4255DBB7A357E7555BFC49547B94FAF0373212C102D`. Đã nạp, verify thành công bằng ST-LINK SN `37FF71064E573436E9171343`, điện áp 3,49 V; vector `0800D345`, TIM1 CCR1/CCR2 = 0. Chưa chạy xe thật sau nạp.
- Ghi chú: [lượt sửa no-echo](../runs/20261123-tof-noecho/notes.md). Nếu vẫn báo lỗi, cần đọc chính xác telemetry (`TOF` ba hướng và nhãn Navigation), vì các giá trị invalid ngoài sentinel vẫn được xử lý là lỗi cảm biến.

### 2026-11-23 — Bắt đầu né ở 2,5 cm, hạ tốc trước khi dò

- Theo yêu cầu sau khi xe đã về đích: giữ waypoint; đổi ngưỡng bắt đầu dò phía trước 30→25 mm. Dưới 80 mm, cruise giảm tuyến tính từ 210 mm/s xuống 90 mm/s tại 25 mm; sau hai mẫu ToF mới ở ngưỡng đó, xe dừng tiến và dùng dò vuông góc 80 mm đã có (phải trước), quay lại hướng tuyến, kiểm tra ba tia rồi mới tiếp tục.
- Lệnh tap xoay R/L trong `Teach` đổi từ 5° (`TAP_TURN_TENTHS=50`) lên 7° (`70`). Đây chỉ có trong firmware Teach; firmware Release chạy waypoint không dùng tap-turn này. Không nạp Teach để giữ điều hướng ToF tự động đang chạy.
- Giữ nguyên course route. `run_checks.ps1`: 10 nhóm unit + 7 trace đều PASS. Trace: nominal 14,37 s/40,3 mm; T9 16,05 s/40,1 mm; vật cản giữa 40×40 mm trong mô phỏng kích hoạt giảm tốc+dò và về đích 17,73 s/36,2 mm. Đây chưa chứng minh né thành công vật 100×100 mm trong mọi vị trí; ca vật chắn lớn tại đoạn hẹp vẫn dừng an toàn.
- Release build: FLASH 64.312/65.536 B, RAM 17.648/20.480 B; HEX SHA-256 `D60D399C824148ED89B88E5237E88428F64E76BE4199380355F59CE4712509E9`. Teach build riêng thành công (HEX SHA `6BAB5AA6163BAAB57BF75036C4BB4FE8FD3288A18397B6EA5EA98D0F4157467E`), chưa nạp.
- Đã nạp Release và verify thành công trên STM32F103 64 KB, ST-LINK `37FF71064E573436E9171343`, 3,27 V; vector `0800D349`; PWM CCR1/CCR2 = 0. Chưa chạy xe thật sau lần nạp.
- Ghi chú: [lượt chỉnh tốc/né](../runs/20261123-turn7-front-probe/notes.md). Cần thử có giám sát với vật cản thực, sẵn sàng dừng xe; nếu xe còn dừng, lấy ba giá trị ToF và nhãn Navigation.

### 2026-11-24 — Giải phóng 2.652 B flash, Release hết tràn (chưa nạp)

Yêu cầu: giữ nguyên thuật toán, chỉ bỏ phần không dùng đến mà tốn RAM/flash.

- **Đo trước khi sửa:** Release tràn **16 B** (FLASH 65.552/65.536, RAM
  17.640/20.480), Debug cũng từng vượt flash. Đã kiểm tra `--gc-sections` +
  LTO đã bỏ hàm không tham chiếu, nên phần dư nằm ở newlib stdio và ở ~92 lời
  gọi `writeReg()` lặp trong khởi tạo VL53L0X, không phải ở `navigation.c`.
  **Không** xoá `nav_trace`/`nav_diag`/`motion_diag`/`tof_diag`/`i2c_diag`/
  `mpu_diag` vì `Tools/map_monitor/stlink_feed.py` đọc các vùng này qua SWD;
  **không** xoá `grid_map.c`/đường né tham chiếu vì `Tests/test_navigation.c`
  và `sim_course` phụ thuộc.
- **Sửa 1 — `Src/vl53l0x.c`:** chuỗi nạp thanh ghi thành 3 bảng
  `static const Vl53l0xRegValue[]` + `WriteRegTable()`. Thứ tự đọc `0x91`
  (`stop_variable`) và mọi cặp (thanh ghi, giá trị) giữ nguyên;
  `runs/20261124-flash-trim/verify_init_table.py` xác nhận **92/92 MATCH**.
  Giảm 652 B flash.
- **Sửa 2 — bỏ `snprintf` khỏi firmware:** thêm `Inc/text_format.h`
  (`Text_String/Text_UInt/Text_Int/Text_HexByte`), dùng trong
  `Inc/bluetooth_status.h` và 2 chỗ ở `Src/main.c`. Khung telemetry + CRC +
  quy tắc bỏ khung khi thiếu chỗ giống hệt bản cũ. Giảm thêm 2.000 B flash,
  24 B RAM (newlib stdio không còn được link).
- **Sửa 3 — `Src/teach_drive.c`:** thêm `#include <string.h>` vì
  `bluetooth_status.h` không còn include stdio/string gián tiếp.
- **Kiểm thử:** thêm `Tests/test_text_format.c` (so khung telemetry với
  `snprintf` trên PC cho nhiều giá trị, gồm `E1/E2` cực trị, buffer quá nhỏ,
  đúng ngưỡng `length+5 == size`) và một ca trong `Tests/run_checks.ps1`.
  **10/10 nhóm PASS.**
- **Build:** Release **62.900/65.536 B (95,98 %, còn 2.636 B)**, RAM
  17.616/20.480 B; Debug **63.620/65.536 B (97,08 %)**; Teach 30 KB build OK.
  HEX Release 177.006 byte, SHA-256
  `455971CAA3AA4417828F2B7A72641870AE906105B0AA23D93AE5DF6F6468B158`.
- **Chưa nạp, chưa chạy xe thật.** Không đổi tuyến, PID, hệ số encoder hay
  tham số điều khiển nào. Chi tiết và log build ở
  [`runs/20261124-flash-trim/notes.md`](../runs/20261124-flash-trim/notes.md).
- **Lưu ý cho phiên sau:** firmware không còn newlib printf; nếu cần in số thực
  trong telemetry phải tự viết bộ chia hoặc nạp lại printf (sẽ tốn ~2 KB).


### 2026-11-23 — Đã né được vật cản cục bộ ở T9 trong simulator (chưa nạp)

Mục tiêu giữ nguyên: tuyến waypoint là đường mặc định, chỉ né khi ToF xác nhận
vật cản thật, né xong nhập lại tuyến.

- **Điều kiện kích hoạt né** (trong `FixedLineMove()`, `Src/navigation.c`): chỉ né
  khi tia giữa **thoáng** (`front[1].mm >= 300`) **và** một tia chéo **gần**
  (`front[0].mm <= 100` hoặc `front[2].mm <= 100`). Đo trên đường trống: tia giữa
  đọc 1500 mm trong khi tia phải đọc 92 mm tại cùng thời điểm, nên điều kiện "tia
  giữa dưới ngưỡng" như đề xuất trước **không thể** thấy vật cản này. Điều kiện
  đảo lại (giữa thoáng + chéo gần) vừa thấy được vật cản bên tuyến, vừa không
  kích hoạt nhầm khi tia chéo thấy tường lúc vào cua.
- **Cách né:** đẩy điểm ngắm sang bên kia vật cản `FIXED_LINE_OFFSET_MM = 35 mm`,
  giữ `FIXED_LINE_HOLD_MM = 400 mm` quãng chạy, rồi giảm dần trong
  `FIXED_LINE_RELEASE_MM = 120 mm` để nhập lại tuyến. Đo bắt waypoint tính theo
  điểm ngắm đã dịch nên chỉ số điểm vẫn tiến đều.
- **Quét tham số** (`sweep_offset_hold.txt`, `sweep_offset_band.txt`): hold 400 mm,
  offset 20–25 mm còn sượt vật cản; 30–40 mm về đích không chạm; 45–70 mm văng
  sang T7. Chọn 35 mm là giữa dải dùng được.
- **Kết quả simulator:** `trace nominal` **finish 14,37 s**, sai số ngang 1,9 mm,
  khe hở nhỏ nhất 40,3 mm — **không đổi** so với trước khi thêm né (0 dòng có
  offset khác 0 trong `trace_nominal.csv`). `trace t9` **finish 15,18 s**, sai số
  ngang 2,1 mm, khe hở 40,5 mm — hết chạm vật cản; offset hoạt động ở dòng
  80…163/304 rồi về 0,0 nên nhánh nhập lại tuyến có được chạy thử. Thêm 0,81 s ở
  `t9` là chi phí của lần né, đúng như dự kiến.
- **Kiểm thử:** `Tests/run_checks.ps1` 9/9 PASS. Release build FLASH **65 536 B /
  65 536 B (100,00 %, còn 0 B)**, RAM 17 640 B / 20 480 B (86,13 %); HEX SHA-256
  `8D3311E4D409B0B154DB77E7942E1A3EC198E147E01BC5EDEE07CAD1374648CE`.
- **Chưa nạp, chưa chạy xe thật.** HEX đang nằm trên xe vẫn là bản trước khi sửa
  bảng tuyến, nên chặng về đích của nó có thể sai.
- **Cần chú ý:** flash đã **đầy** (lớp né tốn khoảng 590 B; bản trước còn 592 B).
  Mọi thay đổi tiếp theo phải giải phóng flash trước, ví dụ bỏ đoạn giảm dần hoặc
  chuyển các hằng so sánh sang số nguyên. Bộ `sim_course.exe` đầy đủ vẫn dừng ở
  một `CHECK` cũ (`NAV_WAIT_CLEARANCE`) vì tuyến cố định bỏ qua nhánh A*; lỗi này
  có sẵn, không do lớp né. Chỉ hai ca `trace` là có nghĩa cho tuyến này.
- Chi tiết và bảng số đo ở [`runs/20261123-waypoint-avoid/notes.md`](../runs/20261123-waypoint-avoid/notes.md).

### 2026-11-23 — Sửa lỗi tràn bảng tuyến và khôi phục chạy liên tục (đường trống đã đạt)

Mục tiêu giữ nguyên: tuyến waypoint là đường mặc định, chỉ né khi ToF xác nhận
vật cản thật, né xong nhập lại tuyến. Phiên này sửa hai lỗi chặn tiến độ, chưa
tích hợp né.

- **Lỗi 1 — tràn số trong `Inc/course_route.h`.** Bảng nén khai báo
  `int8_t course_route_cm[][2]` nhưng X của tuyến lên tới 132 cm (128…132), vượt
  `int8_t` (max 127) nên các điểm cuối bị quấn thành số âm, tức chặng về đích
  trỏ ra ngoài sân. Đã đổi sang `struct { uint8_t x_cm; int8_t y_cm; }` để X
  không dấu. **Nếu HEX đang chạy trên xe được build từ bảng cũ, chặng cuối của
  nó sai** — cần nạp lại bản mới.
- **Lỗi 2 — `FixedLineMove()` phát lệnh chuyển động mới mỗi vòng.** Khi
  `state == NAV_MOVING` mã cũ rơi xuống cuối hàm và gọi
  `Motion_StartDistanceHeading()` lần nữa mỗi 10 ms, nên lệnh chạy bị khởi động
  lại liên tục và `distance_commands` tăng vô hạn. Đã khôi phục nhánh chạy liên
  tục: đang `NAV_MOVING` thì `Motion_RetargetForward()` giữ bánh đang lăn, chỉ
  quay tại chỗ khi lệch hướng quá dung sai.
- **`Tests/sim_course.c`**: bỏ `expect_continuous` cũ (nó cấm mọi lần dừng-quay,
  trong khi tuyến tay có góc vuông). Thay bằng chốt kết quả: phải `finish` và
  không chạm vật cản.
- **Mô phỏng:** `trace nominal` **đạt** — `finish`, 14,37 s, sai số ngang lớn
  nhất 1,9 mm, khoảng hở nhỏ nhất 40,3 mm. `trace t9` **chưa đạt** — vẫn chạm
  vật cản phụ ở (482, 785). Vết chạm là **sượt ngang**, không phải chắn trước
  mặt, nên chỉ nhìn tia giữa không phát hiện được.
- **Đã thử và đã bỏ:** một bản né dùng tia giữa để lệch aim rồi nhập lại tuyến.
  Đường trống bị chậm 14,37 → 19,4 s vì tia giữa thấy tường khi vào cua, tức
  đã làm đổi đường khi không có vật cản — vi phạm yêu cầu. Đã revert, giữ
  nguyên hành vi đường trống.
- `Tests/run_checks.ps1`: **9/9 PASS**. Release build: FLASH 64.944/65.536 B
  (99,10 %, còn 592 B), RAM 17.616/20.480; HEX SHA-256
  `906BF4C368AB62F02B6DE6BBD7FE564A478EBF46E4A069B9C4C56D33D5114905`.
  **Chưa nạp, chưa chạy xe thật.**
- **Việc còn dở:** né cục bộ + nhập lại tuyến. Nút thắt là chọn nguồn phát hiện:
  tia giữa không thấy vết sượt ngang, còn tia chéo thấy tường và làm xe lệch
  khỏi tuyến. Hướng tiếp theo: chỉ né khi tia giữa dưới ngưỡng **và** tia chéo
  xác nhận cùng phía, rồi kiểm lại `trace nominal` (phải giữ 14,37 s) cùng
  `trace t9`.

### 2026-11-23 — Ứng viên ToF tránh dừng giả ở T9 (chưa nạp)

- Người dùng báo xe dừng ngay trước khi qua T9. Không có telemetry hay số đo cảm
  biến của lượt; firmware được bàn giao gần nhất là `353DDF3D…`.
- Trong bản làm việc `runs/20261122-restore-session/firmware/`, tia chéo phải
  <40 mm nay chỉ lái lệch trái 2–8° nếu tia giữa >50 mm; tia giữa mới kích hoạt
  né dưới 30 mm. `ForwardRoom()` không còn lấy tia chéo làm giới hạn ở tuyến
  centered. Dò vuông góc thử bên phải trước 80 mm; chỉ thử trái nếu hình học map
  không đủ khoảng hở 80 mm.
- Release build đạt: 65.388/65.536 B flash, 17.184/20.480 B RAM; HEX SHA-256
  `73FE6488A84F5CFF68726C4055860DF8D8A272C897CA1DE245CA18CBFAD2ED25`.
  `Tests/run_checks.ps1` vẫn lỗi ca cũ `NAV_SEARCH_BLOCKED` tại `(511,997) mm`;
  Debug vượt flash 612 B. Theo yêu cầu, đã nạp bằng ST-LINK
  `37FF71064E573436E9171343`, SWD Under Reset 100 kHz, nguồn 3,27 V; CubeProgrammer
  xác minh toàn file rồi reset. Đọc lại vector `20005000 0800CE1D`, TIM1 CCR1/CCR2
  đều 0. Chưa chạy xe sau nạp, nên chưa xác nhận hành vi né trên sân.
- Tiếp theo: thử có giám sát ở T9, sẵn sàng gửi `x`; ghi ToF ba hướng, pose, yaw,
  trạng thái navigation và PWM nếu xe dừng. Ca `NAV_SEARCH_BLOCKED` trên PC vẫn
  cần phân tích. Chi tiết ở [nhật ký lỗi](04-nhat-ky-loi.md) và
  [ghi chú lượt](../runs/20261123-tof-stop-t9/notes.md).

### 2026-11-23 — Sửa quyền điều khiển né T9; bản cập nhật chờ kiểm tra nguồn

- Người dùng làm rõ waypoint cố định đã đi được, vấn đề nằm ở né T9; báo cảm giác
  xe xoay hướng thất thường. Trong bản làm việc đã bỏ lệch hướng 2–8° liên tục,
  giữ nguyên `course_route`, giữ `reference_active` khi né để có thể nhập lại tuyến,
  bật map phản xạ từ cả ba ToF, và để Navigation xử lý ToF khi tự động. Trước đó `Service_Front_Clearance()`
  bắt center <30 mm, lùi khẩn cấp, gọi `Navigation_InvalidatePose()` rồi chuyển
  quyền sang manual, nên nhánh né và nhập lại tuyến không thể chạy.
- Release build mới: flash 64.940/65.536 B, RAM 17.184/20.480 B, HEX SHA-256
  `30CBC1382B87A581BBB5B44F0F0CD504B068B1FCE69173500764ED0C5101C415`.
  Test PC vẫn dừng ở ca map hẹp cũ `(511,997) mm`; Debug vượt flash 220 B.
- Diagonal ToF giờ có thể tạo map hit sớm; giữ hit nếu đoạn hiện tại còn thông,
  để detour A* cục bộ né obstacle ngẫu nhiên và nhập lại waypoint. `course_route`
  không đổi. Biên đen hiện được bảo vệ bằng hình học map/odometry (turn padding
  74 mm); IR line chưa nối vào Navigation.
- Chưa nạp bản cập nhật này: trước khi ghi, ST-LINK `37FF71064E573436E9171343`
  báo 4,03 V, cao hơn giới hạn hoạt động 3,6 V của STM32F103. Vector trên chip
  vẫn là bản trước `20005000 0800CE1D`, hai PWM bằng 0. Cần đo trực tiếp chân
  3V3–GND trên bo; chỉ tiếp tục nạp khi xác nhận nguồn nằm trong dải an toàn.

### 2026-11-22 — Tối ưu flash: nén bảng tuyến sang cm (build xanh, 9/9 PASS)

Flash chỉ còn 60 B sau khi thêm lớp né mép trước, nên bảng tuyến tay được nén
từ 6 B/điểm (int16 mm × 3) xuống 2 B/điểm (uint8 cm x, int8 cm y) và bung ra
mm trong RAM lúc khởi tạo. Đây là thay đổi dung lượng, không đổi thuật toán.

- `Inc/course_route.h`: `course_route[71][3]` giờ là `static int16_t` (nằm ở
  RAM) và `CourseRoute_Init()` bung từ `course_route_cm[71]` — 142 B flash thay
  vì 426 B. Leg suy ra từ chỉ số điểm: 0..13 bắc, 14..68 đông, 69..70 nam.
  Nhánh `COURSE_ROUTE_USE_PLANNED` có `CourseRoute_Init()` rỗng để giữ nguyên
  chữ ký gọi.
- `Src/navigation.c`: gọi `CourseRoute_Init()` ở đầu `Navigation_Init()`; nới
  bán kính về đích của điểm cuối 8 → 12 mm, vì điểm bảng làm tròn 1280 mm nằm
  cách setpoint của lớp né (1275 mm) khoảng 5 mm.
- Sai số làm tròn: lớn nhất 5 mm mỗi trục trên 71 điểm; điểm kết thúc thành
  (1280, −70) thay vì (1275, −65) — vẫn còn 35 mm tới khối U và 75 mm tới T15.
- Build Release: FLASH 65288/65536 B (99,62 %, còn **248 B** so với 60 B
  trước), RAM 17616/20480 B (86,02 %); HEX `build/Release/…hex` 183.719 byte,
  SHA-256 `C96F0A770F499655402A908E79BBC97DEDF1DE8BADC1899922931D38CE9DF2AF`.
  `Tests/run_checks.ps1` 9/9 PASS; `validate_beam.py` vẫn hội tụ X=1274,9 với
  drift 0/88/120 mm. **Chưa nạp.**
- Công cụ: `runs/20261122-flash-optimize/apply_route_cm.py` (áp dụng, và chế độ
  `verify` đối chiếu bảng mm gốc), `course_route.h.mm.bak` (bản mm để lùi),
  `apply.txt` (kết quả). Nếu cần lùi: chép `course_route.h.mm.bak` đè lên
  `Inc/course_route.h` và bỏ dòng gọi `CourseRoute_Init()`.
- Ghi chú telemetry: trace **đã có** `front_mm[3]` và `stlink_feed.py` đã giải mã
  thành `tof_mm`, nên không cần thêm cột mới chỉ để đọc ToF; 248 B vừa giải
  phóng đủ chỗ nếu sau này muốn thêm `fixed_line_aim`/`fixed_line_edge`.

### 2026-11-22 — Thêm lớp né mép trước bằng ToF cho chặng xuống đích (đã build, chưa nạp)

Mục tiêu: xe không còn trôi vào T15/T16 ở chặng nam cuối. Bản tuyến đang chạy
hạ ở X=1275 nhưng lượt xe thật trước đó xuống ở X≈1390, nên lớp này giữ làn
bằng cảm biến thay vì tin encoder.

- `Src/navigation.c`, `FixedLineMove()`: khi `route_leg == U_SOUTH` và
  `y < FINISH_LANE_Y_MM` (620 mm), đọc `front[2]` (Truoc-Phai, −25°) và đặt
  `fixed_line_aim = (FINISH_BEAM_MM − front[2].mm) * FINISH_BEAM_SIN` với
  `FINISH_BEAM_MM` 118 mm (≈75 mm từ mép thân tới mép U X=1200) và
  `FINISH_BEAM_SIN` = sin 25° = 0,42262. Aim = `x + fixed_line_aim`; mất beam
  thì giữ aim cuối. Cửa sổ hợp lệ của beam là 40…480 mm.
- Bỏ qua chốt biên `U_RIGHT_LANE_MM + pad` khi `fixed_line_edge` bật, nếu không
  aim không thể nằm trên 1200 mm. Điều kiện về đích so với điểm bảng tuyến
  (`hypotf ≤ 8 mm`) chứ không so `remaining`, để offset giữ lại không chặn đích.
- `Inc/course_route.h` (và `Tools/plan_route.py`): bỏ hai điểm `{1275,616,3}`,
  `{1275,590,3}`; chặng cuối còn `{1275,643,3}` → `{1275,-65,3}`.
- Kiểm chứng trên PC: `runs/20261122-front-edge-avoid/validate_beam.py` (kết quả
  `validate_beam.txt`, UTF-16). Beam ở X=1275, Y=500 đọc 118,3 mm; vòng kín
  hội tụ X=1274,9 với drift 0/88/120 mm, còn 29,9 mm tới U và 80,1 mm tới T15;
  beam thấy khối U trong dải Y 153…659 nên dưới Y=153 là giữ aim cuối.
- Release build: FLASH 65476/65536 byte (99,91 %, còn 60 B), RAM 17.192/20.480;
  *(số liệu này đã bị thay thế bởi mục "Tối ưu flash" ở trên: 65288 B, RAM
  17616 B)*
  HEX `build/Release/code_test_xetranhvatcan.hex` 184.238 byte, SHA-256
  `C2B063DC2BF8139CC134CAB15D4EFD5D231ABA40B8EAACA9B8BF66CEB4BD8E26`
  (mtime 20:43:16 mới hơn `navigation.c` 20:40:49 và `course_route.h` 20:42:39,
  tức HEX khớp mã hiện tại). `Tests/run_checks.ps1` 9/9 nhóm PASS. **Chưa nạp.**
- Chưa kiểm chứng: lớp né có thật sự bật trên xe, và lượt xuống X≈1390 là do
  trôi hay do xe còn chạy HEX cũ. Telemetry/trace hiện **chưa có cột ToF**, nên
  chưa có dữ liệu cảm biến để huấn luyện/đối chiếu tuyến.
- **Cập nhật theo sơ đồ sân 2026-11-22** — Sơ đồ người dùng gửi khớp map mã
  (1800×1200 mm; U tới X=1200/Y=600; T15/T16 từ X=1400). Trong
  `FixedLineMove()`, giới hạn aim hiệu chỉnh bởi ToF vào X=1255…1345 mm, tức
  giữ biên dự phòng 10 mm ngoài nửa thân xe 45 mm ở hai phía hành lang. Mô hình
  `runs/20261122-front-edge-avoid/validate_beam.py` hội tụ X=1274.9 mm với drift
  0/88/120 mm và xác nhận mục tiêu hiệu chỉnh nằm trong hành lang ở các vị trí
  thử. `Tests/run_checks.ps1`: 9/9 nhóm PASS. Release build thành công, FLASH
  65,504/65,536 B (còn 32 B), RAM 17,192/20,480 B; HEX SHA-256
  `778040597E0D4589400CA0F3ED53C7981A14CADD240E3D94CD5606B97B311CBD`.
  Đây là kết quả build và mô hình PC; **chưa nạp, chưa chạy xe thật**. Bước kế:
  xác minh đúng HEX rồi nạp có giám sát và đo X/ToF ở chặng cuối; telemetry cần
  được đẩy ra UART/ST-LINK hoặc giải phóng flash trước khi thêm vào firmware. Lớp
  này chỉ hiệu chỉnh aim, không phải né tránh
  vật cản tổng quát.
- **Nạp HEX Release 2026-11-22** — Người dùng đo trực tiếp nguồn bo 3,3 V; ST-LINK
  ban đầu báo 4,07 V nhưng lần kết nối ghi thực tế báo 3,27 V. ST-LINK SN
  `37FF71064E573436E9171343`, STM32F103 medium-density/64 KB, SWD Under Reset
  100 kHz. HEX
  `build/Release/code_test_xetranhvatcan.hex`, 183.719 B, SHA-256
  `C96F0A770F499655402A908E79BBC97DEDF1DE8BADC1899922931D38CE9DF2AF`;
  CubeProgrammer báo `Download verified successfully`, rồi software reset.
  Đọc lại vector `0x08000000 = 0x20005000`; sau reset, kết nối Normal cho thấy
  TIM1 CCR1/CCR2 = 0. Chưa gửi lệnh động cơ hoặc chạy thử; bước kế là quan sát telemetry/trace chặng cuối khi
  người dùng sẵn sàng thử xe.
- Bước tiếp: thử xe có giám sát và xem trace chặng cuối X có quanh 1275; sau đó
  bổ sung `front[0..2]`, `fixed_line_aim`, `fixed_line_edge` vào telemetry qua
  UART/ST-LINK hoặc giảm footprint flash trước khi thêm.
- **Sự cố sau nạp 2026-11-22** — Người dùng báo app kết nối nhưng không có dữ
  liệu, xe không chạy. SWD Hot Plug, không gửi lệnh: `mpu_diag.stage=1`,
  `ack_mask=0`, `hal_error=0x20`, cả 5 ToF `state=5`; PB10/SCL thấp, PB11/SDA
  cao, I2C2 BUSY/BERR. Firmware khóa chuyển động khi MPU chưa sẵn sàng; đây là
  nguyên nhân đã xác nhận cho việc xe không chạy. Nguyên nhân gốc khiến I2C giữ
  bus chưa rõ. `auto_stream=true`, UART PA12/PA11 9600; bộ nhận chỉ ghi hai byte
  nhiễu `FC FF`, nên mất dữ liệu Bluetooth chưa xác định. Chi tiết và bước tiếp
  trong [nhật ký lỗi](04-nhat-ky-loi.md) và
  [notes lượt](../runs/20261122-postflash-no-output-no-motion/notes.md).
- **Khắc phục thử lỗi khởi tạo MPU** — Đã phát hiện startup không thử lại nếu
  MPU không ACK; chỉ lỗi đọc runtime mới kích hoạt I2C recovery. `Src/main.c`
  nay gọi recovery nếu init sau boot thất bại. PC: 9/9 nhóm PASS. Release
  FLASH 65.312/65.536 B, RAM 17.616/20.480 B; HEX SHA-256
  `131218ECF927D1ABB10F2564FF5EB55A135BBCA1E0DEA793B1FEE46205795746`.
  Đã nạp và verify Under Reset 100 kHz. Sau 5 giây Hot Plug: MPU READY
  (`stage=4`, `0x68`, WHO_AM_I `0x70`), 5/5 ToF hoạt động (146 mẫu mỗi mắt),
  PWM 0. Chưa gửi lệnh chạy. Khả năng tự phục hồi đã được chứng minh trong
  lượt này; nguyên nhân gốc SCL thấp chưa rõ. Bluetooth chưa được xác nhận:
  `auto_stream=true`, chưa có xác nhận điện thoại nhận dòng status.
- **Nạp lại cùng HEX theo yêu cầu** — Verify thành công, cùng ST-LINK SN,
  Under Reset 100 kHz, 3,27 V. Sau ~5 giây lần này MPU lại không ACK
  (`stage=1`, `ack_mask=0`), recovery báo `i2c_diag.state=2` sau 16 lần thử,
  năm ToF `state=5`, PWM=0. Cùng HEX có lần boot trước READY và lần boot này
  thất bại; chưa xác định nguyên nhân gốc I2C. Không gửi lệnh chạy. Serial vẫn
  chưa xác nhận. Chi tiết ở nhật ký và notes lượt.

### 2026-11-22 — Nạp bản chặng cuối trái 27 mm thành công

- HEX: `build/Release/code_test_xetranhvatcan.hex`
- SHA-256: `1B7E33A1E6F19E387322A3ABEFADAB6FD24FA2EB9EF805DD1849BF4611ABFFFC`
- SWD Under Reset 100 kHz: ghi và xác minh toàn file thành công, reset phần mềm.
- Nguồn đo được 3.27 V.
- Vector flash đọc lại `20005000 0800CD65`.
- Chưa chạy xe hoặc gửi lệnh sau lần nạp này.

### 2026-11-22 — Nạp bản chặng cuối trái 27 mm thất bại

- HEX mục tiêu: `build/Release/code_test_xetranhvatcan.hex`
- SHA-256: `1B7E33A1E6F19E387322A3ABEFADAB6FD24FA2EB9EF805DD1849BF4611ABFFFC`
- Đã nạp thử bằng SWD Under Reset 100 kHz, NORMAL 100 kHz và Under Reset 100 kHz lần cuối.
- Probe thấy nguồn 3.27 V nhưng STM32 không trả core ID (`Unable to get core ID`, `No STM32 target found`).
- Flash không được xác minh đã ghi; chưa chạy hoặc gửi lệnh tới xe.
- Tiếp theo: kiểm tra dây SWD/nRST, nối đất chung và nguồn 3.3 V ổn định, rồi nạp lại bằng `-c port=SWD mode=UR freq=100`.

### 2026-11-22 — chặng cuối trái 27 mm (X=1275), nới T11/T14, đã build chờ nạp

Người dùng chạy bản đã nạp (chặng cuối X=1302) và báo: T9, T11, T14 đã qua nhưng
còn sát, T15 và T16 vẫn đâm vào; yêu cầu giảm X chặng cuối khoảng 7 cm và hỏi
đang áp dụng thuật toán né vật cản nào.

**7 cm là không khả thi theo bảng vật cản.** Hành lang về đích bị kẹp giữa khối U
(mép phải X=1200) và T15/T16 (mép trái X=1400), rộng 200 mm. Thân xe 90×90 mm nên
tâm xe chỉ được nằm trong 1245…1355 mm. Dịch 70 mm về X=1232 đưa thân xe lấn 13 mm
vào khối U. Đã chọn X=1275: dịch trái 27 mm so với bản đang chạy, còn 30 mm tới U
và 80 mm tới T15/T16 tính từ mép thân xe.

Đã sửa `Inc/course_route.h` (73 điểm, cùng `Tools/plan_route.py` `CURRENT_ROUTE`):
hạ 4–5 mm các điểm 734–742 mm để nới T11, hạ 5–12 mm vùng T14 (1112–1190 mm) và
làm mềm khúc gối ở T14, chặng cuối dịch 1302 → 1275 mm với cua lượn một chiều
(không còn gù phải ở 1258 mm như bản cũ).

Khoảng hở thật tới vật cản (chưa nới biên): T11 83,5 → 87,1 mm; T14 97,1 → 108,2 mm;
T9 98,5 → 94,8 mm (đổi lấy T11 — hai khối nằm hai phía của khe 180 mm nên tổng
không đổi); U 103,2 → 75,0 mm; T15 154,0 mm tại góc, tức 80 mm tính từ mép thân xe
trong dải Y 400–500. `validate --inflate 45` và `--inflate 52`: 0 điểm và 0 đoạn cắt
vật cản. Ở `--inflate 64` còn 1 đoạn cắt góc nới T11 tại khe T9/T11 (khe này không
đủ chỗ cho thân 90 mm xoay mọi hướng).

Release build: FLASH 65.312/65.536 byte (còn 224 B), RAM 17.184/20.480 byte. HEX
SHA-256 `1B7E33A1E6F19E3872A3ABEFADAB6FD24FA2EB9EF805DD1849BF4611ABFFFC`,
kích thước 183.780 byte. `Tests/run_checks.ps1` 9/9 nhóm PASS. **Chưa nạp** — cần
người dùng chạy thử.

**Thuật toán né: vẫn chưa bật cho tuyến cố định.** `COURSE_FIXED_LINE_TEST 1` khiến
xe bám bảng điểm cố định và bỏ qua ToF, nên không có né phản ứng. Ba lựa chọn đã có
sẵn trong mã: (a) bật `COURSE_ROUTE_USE_PLANNED 1` để bay tuyến A* 94 điểm ngắn hơn
129 mm nhưng chỉ còn 1,4 mm hở; (b) bật `COURSE_FIXED_LINE_TEST 0` để dùng A* trên
MCU + `Grid_AddHit` tái hoạch định theo ToF; (c) viết né phản ứng mép trước bằng ba
cảm biến ToF. Cả ba đều cần ToF hoạt động ổn định.

### 2026-11-22 — hạ Y vùng T11/T14, chặng cuối trái 10 mm, đã nạp

Người dùng báo chạm nhẹ T15, T16 và phần dưới T11, T14; yêu cầu dịch chặng cuối
thêm 1 cm và hỏi thuật toán né đã áp dụng chưa.

Đo lại tuyến cũ bằng `Tools/plan_route.py validate --inflate 52`: khoảng hở nhỏ
nhất 2,8 mm, hai điểm sát nhất là `(734,893)` cạnh T11 (14,9 mm) và `(1146,720)`
cạnh T14 (16 mm) — khớp đúng chỗ chạm. Ở biên nới 64 mm (nửa đường chéo thân xe)
tuyến cũ có 3 điểm / 5 đoạn nằm trong vùng nới, tức thân xe chạm được.

Đã sửa `Inc/course_route.h`: hạ Y 3–15 mm ở vùng T11 (X 726–999), hạ Y 12–29 mm ở
vùng T14 (X 1132–1190), dịch chặng cuối trái 10 mm về X = 1302 mm
(`(1304,645) (1302,631) (1302,616) (1302,-65)`). `Tools/plan_route.py` cập nhật
theo để hai bảng khớp nhau.

Kết quả đo lại: 74 điểm, dài 2.551,0 mm, khoảng hở nhỏ nhất 18,0 mm ở biên nới
52 mm; ở biên nới 64 mm khoảng hở 6,0 mm với 0 điểm và 0 đoạn cắt vật cản — hết
hẳn 3 điểm / 5 đoạn cắt của bản cũ. Tuyến cố định nay an toàn ở mọi hướng thân xe.

Release build: FLASH 65.312/65.536 byte (còn 224 B), RAM 17.184/20.480 byte. HEX
SHA-256 `E40D2A3A7EF972F85495E4B934F90ECFE32D63283D43FD4A36B6D5F29CF35553`,
kích thước 183.780 byte. `Tests/run_checks.ps1` 9/9 nhóm PASS.

Khoảng hở thật tới từng vật cản (chưa nới biên): T11 83,4 mm, T14 86,6 mm,
T9 86,8 mm, T7 97,5 mm, khối U 103,2 mm, T15 151,9 mm. T15/T16 ở X = 1400–1500 mm
nên chặng xuống đích ở X = 1302 mm còn cách 98 mm tới mép trái T15.

Đã nạp qua SWD Under Reset 100 kHz lúc nguồn 3,27 V, download 100 %, verify thành
công, vector `0x08000000 = 20005000 0800CD65`. Ghi chú và log ở
`runs/20261122-avoid-t11-t14-final-10mm/`.

**Thuật toán né: chưa áp dụng cho tuyến cố định.** Xe hiện chạy bảng điểm cố
định, không phản ứng với vật cản. Trên MCU chỉ có `grid_static_obstacles[18]`
dùng cho thuật toán A* chạy trên PC (`Tools/plan_route.py`), không chạy trên xe.
Hai hướng đã có sẵn: (a) bật `COURSE_ROUTE_USE_PLANNED 1` để bay tuyến A* — ngắn
hơn 157 mm nhưng sát vật cản hơn, chưa chạy thử; (b) thêm né phản ứng thật sự
(lệch ngang khi ToF thấy vật cản rồi quay lại tuyến) — chưa có mã, cần ToF. Vì
hiện đang bỏ qua ToF nên chưa thể làm (b).

Việc tiếp theo: chạy xe thật với bản mới, quan sát T11, T14, T15, T16 và chặng
cuối; nếu còn chạm thì đo lại khoảng hở quanh điểm chạm rồi hạ Y tiếp.
- 2026-11-22 — Dịch chặng cuối sang phải 12 mm và thêm công cụ tìm đường ngắn nhất.
  - Đồng bộ `Inc/course_route.h` và `Src/navigation.c` của dự án chính về đúng bản đang chạy
    (tuyến cong 73 điểm + `FixedLineMove()` dùng `Motion_RetargetForward`). Trước đó dự án chính
    còn tuyến 72 điểm và `FixedLineMove()` dừng–quay cũ, không khớp bản đã nạp.
  - 5 điểm cuối dịch X 1300 → 1312 mm: `(1304,658) (1308,645) (1311,631) (1312,616) (1312,-65)`.
  - Thêm `Tools/plan_route.py` (A* + né vật cản, dùng `grid_static_obstacles[18]`) và sinh
    `Inc/course_route_planned.h` (94 điểm) sau công tắc `COURSE_ROUTE_USE_PLANNED` (mặc định 0).
  - Đo: tuyến cũ dài 2.526,8 mm (tính cả từ mũi xe), hở nhỏ nhất 5,7 mm ở inflate 50 mm;
    tuyến A* dài 2.388,8 mm (ngắn hơn 138 mm = 5,5 %), hở nhỏ nhất 0,5 mm. Ở inflate 64 mm
    tuyến cũ có 13 điểm/2 đoạn trong vùng nới → giải thích việc chạm nhẹ góc T11/T14.
  - Release build: Flash 65.312/65.536 byte (99,66 %, còn 224 byte), RAM 17.184/20.480 byte.
    HEX SHA-256 `3DB9DBED378B821D05CC65B2545B694060FDBB9C048EF7C1F6964E84311D239E`,
    183.780 byte. `Tests/run_checks.ps1`: 9/9 nhóm PASS. Chi tiết ở
    [runs/20261122-final-leg-shift-12mm/notes.md](../runs/20261122-final-leg-shift-12mm/notes.md).
  - Đã nạp bản mới: SWD Under Reset 100 kHz, nguồn 3,27 V, download 100 % sau 13,76 s,
    `Download verified successfully`, reset phần mềm. Vector sau nạp `20005000 0800CD65`.
    So với HEX cũ `38C477ED…` chỉ 4 dòng HEX khác nhau (đúng phần dữ liệu tuyến).
    Log ở `runs/20261122-final-leg-shift-12mm/flash.txt` và `readback.txt`.
  - Người dùng chạy thử bản trước đó và thấy ổn, chỉ còn chặng cuối chạm vạch — đúng với
    thay đổi 12 mm này. Chưa có log chạy cho bản 12 mm. Nếu vẫn chạm, tăng tiếp lên 15–20 mm.
  - Sửa lỗi công cụ `Tools/plan_route.py`: bảng `CURRENT_ROUTE` còn 5 điểm cuối cũ (X=1300)
    nên số "tuyến hiện tại" bị đo sai. Đã cập nhật theo bản đã nạp (X=1304…1312) và
    `FINISH_ENTRY`/`FINISH_NOSE` = 1312. Số đúng: tuyến tay 74 điểm (cả đoạn vào), dài
    2.535,4 mm, hở nhỏ nhất 2,8 mm ở inflate 52 mm, 0 va chạm.
  - Sinh lại `Inc/course_route_planned.h` ở inflate 52 mm (thay vì 50): 94 điểm, 2.393,4 mm,
    hở nhỏ nhất 1,4 mm, 0 va chạm → ngắn hơn 141,7 mm (5,6 %). Ở inflate 64 mm tuyến A* cần
    158 điểm (948 B bảng) nên không vừa flash Release còn 224 B; bảng 94 điểm chỉ chiếm 564 B
    khi bật công tắc và hiện không được biên dịch (công tắc = 0).
  - `Tests/run_checks.ps1`: 9/9 nhóm PASS, EXIT=0 sau khi sửa. Thuật toán né + đường ngắn nhất
    đã có sẵn trong firmware (`Src/grid_map.c`: `Grid_Begin`/`Grid_Advance` A*, `Grid_AddHit`
    tái hoạch định động); công cụ Python chỉ dùng cùng bảng vật cản để kiểm chứng và sinh tuyến.
- 2026-11-22 — Đã nạp xong bản cuối chat tuyến cong T9/T12 theo yêu cầu (lượt nạp sau cùng trong phiên).
  - Link chat chia sẻ `chatgpt.com/s/cx_6b02b1103a1c81918793f33249e58a36` trả “Shared chat not found”, nên dùng bản khôi phục từ chính chat đó ở `runs/20261122-restore-session/firmware/`. HEX Release SHA-256 `38C477EDC02F35B89C6B822356BEE126028F1A566B25A5EFD5DBECA422DBC928` (khớp hash bàn giao), 183.780 byte file HEX, Flash 65312/65536 byte.
  - Lần ghi đầu bị cắt ở khoảng 26% khi tiến trình CLI bị đóng cùng terminal; sau đó ST-LINK trả `DEV_CONNECT_ERR` rồi `DEV_USB_COMM_ERR` (4 lần thử lại). Người dùng rút/cắm lại USB ST-LINK và cắt–bật nguồn bo.
  - Nạp lại toàn bộ thành công: SWD Under Reset 100 kHz, nguồn 3,27 V, download 100% sau 13,8 s, `Download verified successfully`, reset phần mềm. Vector sau nạp `20005000 0800CD65`. Log `runs/20261122-restore-session/flash_final.txt` và `readback-vector8.txt`.
  - Tuyến trong bản này: qua T9 ở Y≈900 mm trên X≈542–677 mm, hạ dần về Y=875 mm ở X≈825–901 mm (trước T11), vòng qua vùng (1130,750) mm, bo về X=1300 mm rồi xuống đích Y=−65 mm. `FixedLineMove()` nối điểm bằng `Motion_RetargetForward` khi đang tiến, tầm nhìn 45 mm.
  - Chưa chạy xe thật sau lượt nạp này. Chi tiết sự cố ở [nhật ký lỗi](04-nhat-ky-loi.md).

- 2026-11-22 — Nạp lại theo yêu cầu lần thứ tư trong phiên. HEX Release `ADEBF3D2A72561EE0A552FF9A6A1055D61206AA6F51C351964EFBB7949D649FB` ghi và xác minh toàn file thành công qua SWD Under Reset 100 kHz, nguồn 3.27 V, reset phần mềm. Vector flash đọc lại `20005000 0800CD71`. Chưa gửi lệnh lái hoặc chạy xe thật sau lần nạp.

- 2026-11-22 — Theo yêu cầu nạp lại bản trước chỉnh tuyến, dùng HEX Release có sẵn và xác nhận SHA-256 `ADEBF3D2A72561EE0A552FF9A6A1055D61206AA6F51C351964EFBB7949D649FB`, khớp bản Release đã nạp trước đó. Trước nạp, ST-LINK nhận STM32F103, nguồn 3,27 V, PWM TIM1 CCR1/CCR2 đều 0. CubeProgrammer SWD Under Reset 100 kHz ghi và xác minh toàn file thành công, rồi reset. Sau nạp đọc vector `20005000 0800CD71`, nguồn 3,27 V, hai PWM vẫn 0. Chưa chạy xe. Source hiện vẫn giữ chỉnh sửa route T11 ở X=800–900 mm; HEX cũ không chứa thay đổi đó. Người dùng báo “không chạy được” nhưng chưa rõ là mô phỏng hay xe thật; cần xác định môi trường và lấy hiện tượng/log trước khi sửa tiếp.
- 2026-11-22 — Nạp lại thành công lần thứ ba trong phiên. ST-LINK Probe cuối cùng truy cập nguồn 3.26 V, SWD Under Reset 100 kHz, ghi và xác minh HEX Release `ADEBF3D2A72561EE0A552FF9A6A1055D61206AA6F51C351964EFBB7949D649FB` toàn file rồi reset phần mềm. Vector đọc lại `20005000` tại `0x08000000`; chưa đọc vector reset thứ hai hoặc gửi lệnh/chạy xe thật sau nạp.

- 2026-11-22 — Yêu cầu nạp lại lần thứ hai trong phiên. HEX Release mới nhất vẫn là `ADEBF3D2A72561EE0A552FF9A6A1055D61206AA6F51C351964EFBB7949D649FB`. Thử lại SWD Under Reset 100 kHz và NORMAL 100 kHz đều trả `DEV_CONNECT_ERR`, không ghi lại flash. Việc nạp không thành công và trạng thái chip không được xác minh thêm lần này.

- 2026-11-22 — Theo yêu cầu nạp, đã build Release bản dự án chính và nạp Release qua SWD Under Reset. `Tests/run_checks.ps1` đạt 9/9 nhóm. Flash 65028/65536 byte (99.22%), RAM 17616/20480 byte. HEX SHA-256 `ADEBF3D2A72561EE0A552FF9A6A1055D61206AA6F51C351964EFBB7949D649FB`, dung lượng 63.50 KB. CubeProgrammer ghi, xác minh toàn file và reset phần mềm thành công. Probe đo nguồn 4.07 V trong lúc nạp; chưa gửi lệnh hoặc chạy xe thật sau lần nạp này.

- 2026-11-22 — Theo ảnh mới báo tuyến đâm T11, ban đầu đã hạ Y=870→860 mm trên X=700–900 mm. Sau phản hồi người dùng rằng chỉ sửa T11, đã khôi phục X=700–790 mm về Y=870 mm; chỉ giữ X=800–900 mm ở Y=860 mm. Kiểm tra lại `Tests/run_checks.ps1` đạt 9/9 nhóm. Mô phỏng riêng `trace t11` từng dừng ở assertion ngay tại điểm xuất phát, nên chưa xác nhận quỹ đạo sau chỉnh; `cmake` không có trong PATH, chưa build Release. Chưa nạp/chạy xe thật. Cần xác nhận người dùng muốn nói mô phỏng không chạy hay xe thật không chạy, rồi lấy log/ảnh lỗi tương ứng.

- 2026-11-22 — Theo yêu cầu khôi phục bản cuối của chat tuyến cong T9/T12, đã dựng source riêng ở runs/20261122-restore-session/firmware/. HEX build lại khớp hoàn toàn SHA-256 38C477EDC02F35B89C6B822356BEE126028F1A566B25A5EFD5DBECA422DBC928 của bản gốc phiên này. Chín nhóm kiểm thử PC và trace tới đích qua. Đã nạp đúng HEX này, verify toàn file thành công rồi reset qua SWD Under Reset 100 kHz, nguồn 3,27 V. Source dự án chính hiện chứa các sửa từ phiên khác; khi nạp lại bản cong 38C477ED cần dùng thư mục khôi phục riêng. Chưa thử xe thật sau nạp.

- 2026-11-22 — Nạp lại bằng Release hiện có trong dự án chính theo yêu cầu. HEX SHA-256 `2FA97EE44FFD6CEFD28B592F73A983458DF8E2975431ACAE71EBE4CD5384A1F5`, dung lượng 63.50 KB. CubeProgrammer nạp và xác minh toàn file thành công qua SWD Under Reset 100 kHz, nguồn 3.27 V, reset phần mềm; vector đọc lại `20005000 0800CD71`. Chưa chạy xe thật sau lượt nạp.

- 2026-11-22 — Đã build lại source dự án chính và nạp Release theo yêu cầu. 9 nhóm kiểm thử PC đạt. Flash 65076/65536 byte (99.30%), RAM 17616/20480 byte. HEX SHA-256 `67F15A77AF02908BA7B3077BAC2647FC70A2ED976EB451B41833D524ABBB8B56`. CubeProgrammer nạp và xác minh toàn file thành công qua SWD Under Reset 100 kHz, nguồn 3.27 V, reset phần mềm; vector đọc lại `20005000 0800CDA1`. Chưa chạy xe thật sau lần nạp.

- 2026-11-22 — Nạp lại theo yêu cầu lần thứ hai trong phiên. HEX Release `78E144EB369AE83DA90F02BB5BAA3A2C786E77F668675F906C3CA5E1036507EB`, kích thước ELF text/data/bss `64832/244/17372` byte. CubeProgrammer nạp và xác minh toàn file thành công qua SWD Under Reset 100 kHz, nguồn 3.27 V, reset phần mềm; vector đọc lại `20005000 0800CDA1`. Chưa chạy xe thật sau lần nạp.

- 2026-11-22 — Đã nạp lại Release mới nhất theo yêu cầu. Build Release thành công: Flash 65316/65536 byte (99.66%), RAM 17184/20480 byte; HEX SHA-256 `756445BC45C25C96A997EAD7CD0960D2E055AD4325FBCBF2806DCBD741F635E4`. 9 nhóm kiểm thử PC đạt. CubeProgrammer nạp và xác minh toàn file thành công qua SWD Under Reset 100 kHz, nguồn 3.27 V, reset phần mềm; vector flash đọc lại `20005000 0800CD71`. Chưa chạy xe thật sau lần nạp.

- 2026-11-22 — Nạp lại bản Release mới nhất theo yêu cầu. Build Release thành công: Flash 65324/65536 byte (99.68%), RAM 17184/20480 byte; HEX SHA-256 `9383E787DDF70CA58B66FB86ED336B7DEAE62461337B3BC28633CBAA61D894CB`. STM32CubeProgrammer ghi và xác minh thành công bằng SWD Under Reset 100 kHz, probe đo nguồn 3.27 V, reset phần mềm; vector flash đọc lại `20005000 0800CD71`. Chín nhóm kiểm thử PC đạt. Chưa chạy xe thật hoặc gửi lệnh sau lượt nạp.

### 2026-11-22 — Tóm tắt ưu tiên cho chat mới

- Làm trong code_test_xetranhvatcan/. Mục tiêu hiện tại là tuyến cố định bỏ qua ToF, bám encoder E2 và MPU, đi chậm trên sân thật. Mốc tâm (420,-45) mm, hướng Bắc. Lệnh bản Release là v rồi j, x dừng.
- Người dùng báo T9 vừa qua và chạm nhẹ góc T12, yêu cầu tăng khoảng hở T9 20 mm và bo cong. Đã sửa Inc/course_route.h thành điểm trên cung Bezier, qua T9 Y khoảng 900 mm rồi hạ trước T11 về 875 mm; vòng qua vùng (1130,750) mm rồi bo về X=1300 mm xuống đích. Src/navigation.c FixedLineMove gọi Motion_RetargetForward để nối điểm khi chạy, tầm nhìn khoảng 45 mm.
- Bản cong T9/T12 đã được nạp (lượt nạp sau cùng 2026-11-22). Dùng HEX ở runs/20261122-restore-session/firmware/build/Release/code_test_xetranhvatcan.hex, SHA-256 38C477EDC02F35B89C6B822356BEE126028F1A566B25A5EFD5DBECA422DBC928, Flash 65312/65536 byte. HEX trong build/Release của dự án chính đã bị ghi đè bởi bản khác (ADEBF3D2…), nên nạp lại bản cong phải dùng thư mục khôi phục. Debug vượt flash khoảng 760 byte; giữ ELF cùng build để đọc RAM.
- 9 nhóm kiểm thử PC và trace MPU/encoder/tới đích qua. Hình học trace có thêm 17 mm mỗi phía trên map; xe thật chưa xác nhận. Ảnh và trace trong runs/20261122-t9-t12-curves/. Bước tiếp theo là kiểm tra nguồn/ST-LINK, nạp đúng Release mới khi được yêu cầu rồi ghi lượt thử qua T9/T12/T14.
- Hệ số encoder hiện tại do người dùng đổi là MOTION_FIELD_MM_PER_COUNT = MOTION_WHEEL_MM_PER_COUNT, khoảng 0,13872 mm/xung, bánh 34 mm và 770 xung/vòng. Các ghi chú 0,1265 bên dưới thuộc lịch sử.
- Phân biệt preset Teach cũ có F/B/R/L, Y reset, Z ghi, V lưu và U phát lại với Release hiện tại dùng v/j/x. Bản Release thường sử dụng tới vùng flash lưu tuyến Teach, cần sao lưu trước nếu muốn giữ tuyến khi đổi firmware.
- Nguồn từng dao động giữa khoảng 3,27 V và 4,1 V, MPU có lúc không ACK sau reset. Lần nạp thành công là khoảng 3,27 V. ST-LINK từng trả DEV_CONNECT_ERR/DEV_USB_COMM_ERR sau một lần ghi bị cắt giữa chừng; rút/cắm lại USB và cắt–bật nguồn bo thì nạp lại được ngay. Gặp lại lỗi này thì xử lý theo [nhật ký lỗi](04-nhat-ky-loi.md).
- Phiên tóm tắt này chỉ cập nhật tài liệu bàn giao, không nạp hay chạy xe.

- 2026-11-22 — Đã nạp bản tuyến cong T9/T12 mới nhất bằng SWD Under Reset 100 kHz lúc nguồn 3,27 V. SHA-256 HEX `38C477ED…`, Release 65312/65536 byte. CubeProgrammer ghi và xác minh toàn file thành công rồi reset; vector sau nạp `20005000 0800CD65`. Chưa kiểm chứng xe thật sau lượt nạp mới.

- 2026-11-22 — Người dùng báo chạm nhẹ T12, T9 vừa qua và yêu cầu nâng 20 mm cùng bo cong. Đã sửa course_route.h thành tuyến cong, qua T9 Y khoảng 900 mm, hạ trước T11 rồi vòng cao qua T12. FixedLineMove dùng Motion_RetargetForward bám điểm khi chạy với tầm nhìn khoảng 45 mm. Chín nhóm PC và trace MPU/encoder/tới đích qua, hình học trace thêm 17 mm mỗi phía trên map. Release 65312 byte build đạt, Debug vượt flash khoảng 760 byte. Chưa nạp và chưa chạy thật. Xem [ghi chú](../runs/20261122-t9-t12-curves/notes.md) cùng ảnh so tuyến.

- 2026-11-22 — Đã nạp Release 483C01A7 vào STM32 bằng SWD Under Reset 100 kHz lúc nguồn 3,27 V. STM32CubeProgrammer xác minh toàn file thành công, reset phần mềm; nguồn sau nạp đọc 3,26 V, vector flash `20005000 0800CCA9`. Chưa gửi lệnh lái hoặc chạy xe thật.

- 2026-11-22 — Đã build lại bản Release mới từ dự án chính và chạy 10 nhóm kiểm thử PC đạt. HEX Release `483C01A7…`, Flash 64732/65536 byte. Kết nối SWD 100 kHz đọc được; lần kiểm tra nạp cuối vẫn báo 4,08 V nên quy tắc an toàn đã tự dừng trước khi ghi flash. Chưa nạp firmware.

- 2026-11-22 — Theo yêu cầu nạp tuyến bù, đã xác minh HEX Release 483C01A7… và đọc STM32/PWM (hai bánh bằng 0). Probe báo 4,08 V, ADC VREFINT 1179/1188/1179 cho ước tính khoảng 4,14–4,17 V. Đã khôi phục ADC, chưa ghi flash. Cần nguồn bo về khoảng 3,3 V trước khi nạp.

- Đối chiếu cuối lượt bù vệt đỏ thấy ST-LINK xuất hiện lại, vector trên xe 20005000/0800CBE1, probe báo 4,08 V. Chưa nạp. HEX Release mới có SHA-256 bắt đầu 483C01A7.

- 2026-11-22 — Người dùng tự sửa navigation.c, course_route.h và hệ số encoder rồi gửi hình vệt đỏ, ToF đang bỏ qua trong nhánh fixed line. Đã sửa tuyến trong course_route.h, đi chéo quanh T9 và hạ xuống đích tại X = 1300 mm, Y cuối = -65 mm. Giữ hệ số hiện tại MOTION_FIELD_MM_PER_COUNT = MOTION_WHEEL_MM_PER_COUNT (khoảng 0,13872 mm/xung). Fixed line dùng macro rõ, kiểm tra lỗi MPU/chuyển động/biên và dung sai điểm 8 mm. Chín nhóm PC, test trace lỗi MPU/encoder và tới đích qua. Hình học trace có khoảng hở 17 mm mỗi phía trên map. Debug vừa 64 KB, Release 64732 byte, đã build. Chưa nạp vì không thấy ST-LINK. Chi tiết và trace ở [ghi chú](../runs/20261122-red-line/notes.md). Tuyến mới dùng bản thường Release và lệnh v rồi j, khác preset Teach.

- 2026-11-22 — Đã nạp bản Teach `022FA138…` có giữ `F`, bấm `R/L` khoảng 5° và giữ để xoay liên tục. Probe báo 3,27 V, SWD Under Reset 100 kHz xác minh toàn file thành công và reset. Vector đọc lại `20005000 08004EF5`, hai PWM bằng 0. MPU sau reset vẫn không ACK (`stage=1`, `ack_mask=0`, `hal_error=0x20`). Chưa thử xe/điện thoại với bản mới.

- 2026-11-22 — Người dùng báo `Z` rồi `U` không chạy lại. Đọc trực tiếp hai trang flash bản `E8AF0C87…` đều chưa có commit tuyến hợp lệ. Bản Teach mới `022FA138…` báo `DANG_GHI_GUI_V` hoặc `DANG_LUU` để làm rõ luồng `Z` → lái → `V` → `DA_LUU` → đặt lại xe → `U`. `F` chỉ chạy khi nhận lặp, `R/L` bấm một lần nhắm 5°, giữ thì xoay liên tục; app cần lặp khoảng 100 ms và ngừng lúc thả, mất lệnh 300 ms thì dừng. PWM cơ sở tiến/lùi 280/1000, xoay 230/1000. Build Teach (29652/63488 byte), Debug và 9 nhóm kiểm thử PC đạt. Lần kiểm tra cuối thấy probe báo 4,10 V; chưa nạp và chưa thử trên xe. Xem [nhật ký lỗi](04-nhat-ky-loi.md).

- 2026-11-22 — Đã nhắc cách lưu `V` → đợi `DA_LUU` và phát lại `U` sau khi đưa xe về đúng mốc và hướng lúc `Z`. Kiểm tra flash để xác nhận tuyến trên xe chưa thực hiện được vì không thấy ST-LINK. Không gửi lệnh chuyển động.

- 2026-11-22 — Người dùng báo động cơ không chạy. Đọc RAM xác nhận Bluetooth đã đưa `F/L/R` vào STM32 và app còn gửi `S`. MPU đang sẵn sàng, `pose_known=0`, PWM và số lần cấp động cơ đều 0. Thiếu xác nhận mốc bằng `Y/Z`; probe báo 4,10–4,11 V. Hướng dẫn thử `Y` → `RESET_PHIEN` → `F` → `x` sau khi nguồn về khoảng 3,3 V. Chưa đổi/nạp firmware hay chạy thử. Xem [nhật ký lỗi](04-nhat-ky-loi.md).

- 2026-11-22 — Theo yêu cầu đã nạp lại cùng bản Teach `E8AF0C87…` bằng SWD Under Reset 100 kHz. Probe báo 3,27 V trước, trong và sau nạp. CubeProgrammer xác minh toàn file và reset thành công ngay lần đầu. Đọc lại vector `20005000 08004CE1`, PWM hai bánh bằng 0, MPU lại không ACK (`stage=1`, `ack_mask=0`, `hal_error=0x20`). Chưa gửi lệnh lái. Cần kiểm tra nguồn MPU và so cách cấp nguồn giữa lần MPU phản hồi và lần không phản hồi.

- 2026-11-22 — Người dùng báo điện thoại đã kết nối được HC-06 với bản `E8AF0C87…`. SWD xác nhận MPU hiện sẵn sàng (`stage=4`, địa chỉ `0x68`, WHO_AM_I `0x70`, không lỗi I2C). Hai PWM bằng 0. Probe báo 4,10 V và ADC nội ước tính 4,18–4,20 V, đã khôi phục cấu hình ADC. Chưa chạy thử hoặc gửi lệnh chuyển động. Hướng dẫn dùng `Y` hoặc `Z` ở mốc, `F/B/R/L/x`, `V` lưu, `U` phát lại sau khi nguồn kiểm chứng khoảng 3,3 V.

- 2026-11-22 — Đã nạp bản đối chứng Bluetooth Teach `E8AF0C87…`. Trước nạp probe báo 3,27 V và ADC nội xác nhận khoảng 3,28 V. Lần ghi đầu sai dữ liệu tại `0x08006560`, lần ghi lại SWD Under Reset 100 kHz xác minh toàn file thành công rồi reset. Đọc lại vector `20005000 08004CE1`, byte tại `0x08006560` là `0x09`, hai PWM bằng 0. MPU vẫn không ACK (`stage=1`, `ack_mask=0`, `hal_error=0x20`). Chưa có COM và chưa đo kết nối điện thoại. Thử xe đứng yên 30 giây, đợi dòng `Goc:` và gửi `x` nếu kết nối giữ được.

- 2026-11-22 — Theo yêu cầu nạp bản đối chứng, đã xác minh HEX `E8AF0C87…`, đọc lại vector bản cũ và hai PWM bằng 0. Probe báo 4,11 V và ba mẫu ADC nội 1167/1167/1165 cho ước tính 4,21–4,22 V. Đã trả lại cấu hình ADC. Chưa ghi flash do nguồn đang ngoài dải. Cần sửa nguồn 3V3 trên bo trước khi nạp.

- 2026-11-22 — Chuẩn bị bản Teach đối chứng Bluetooth về định dạng `Goc | ToF | Lenh | E1/E2` có CRC-8 và chu kỳ 250 ms cũ. ToF in `---`, giữ `V/U/x`, ghi/phát và kiểm tra biên. Build Teach (HEX `E8AF0C87…`, Flash 29072/63488 byte), Debug (65520/65536 byte) và chín nhóm kiểm thử PC đạt. SWD ban đầu không lấy được core ID, sau đó đọc lại được vector bản đang nạp `20005000 08004B55` và PWM 0. Probe báo 4,11–4,12 V và phép đo ADC nội mới ước tính 4,16–4,21 V, khác lần trước khoảng 3,28 V. Chưa nạp bản đối chứng. Bản trên xe vẫn là `03B9B885…`. Cần xử lý nguồn rồi nạp đúng Teach mới và đo thời gian giữ kết nối khi đứng yên. Xem [nhật ký lỗi](04-nhat-ky-loi.md).

- 2026-11-22 — Người dùng báo đèn HC-06 chớp liên tục. Cần đối chiếu nhịp đèn ngay trong khoảng app báo `Connected` và kết quả thử khi tách TX/RX. SWD vẫn `DEV_CONNECT_ERR`. Chưa đổi firmware. Xem [nhật ký lỗi](04-nhat-ky-loi.md).

- 2026-11-22 — Ảnh app xác nhận HC-06 kết nối, nhận một phần `TEACH`, sau đó rớt ở mốc khoảng 5,43 giây trong hai lần. SWD vẫn `DEV_CONNECT_ERR`, không có COM. Đã ghi phép thử HC-06 cấp nguồn riêng khỏi TX/RX trong [nhật ký lỗi](04-nhat-ky-loi.md). Chưa sửa firmware và chưa xác định nguồn rớt.

- 2026-11-22 — Người dùng báo điện thoại hay rớt Bluetooth. Máy có 0 COM và SWD hiện lỗi `DEV_CONNECT_ERR`, chưa tái hiện. Đã ghi phép thử đứng yên bằng HC-06, đọc `TEACH` và gửi `x` để phân biệt kết nối radio, UART và sụt nguồn khi động cơ chạy trong [nhật ký lỗi](04-nhat-ky-loi.md). Firmware chưa đổi.

- 2026-11-22 — Đã hướng dẫn thử trực tiếp qua HC-06 theo thứ tự `Z` → lái ngắn → `V` → về mốc → `U`, kiểm tra `x` ở nơi trống. Không thấy ST-LINK lúc đối chiếu lại nên trạng thái MPU hiện tại chưa xác nhận. Lần đo gần nhất sau khi nạp vẫn báo lỗi I2C.

### 2026-11-22 — Đã nạp Teach với V lưu và U phát lại

- `V` thay lệnh lưu cũ. Khi tới đích, gửi `V` để dừng và lưu, đợi `DA_LUU`. Đặt lại xe đúng vị trí và hướng lúc gửi `Z`, rồi gửi `U` để xác nhận mốc và phát lại tuyến vừa lưu. `x` dừng. Hướng dẫn cập nhật trong [README](../README.md#lái-tay-ghi-đường-và-phát-lại-không-dùng-tof).
- Chín nhóm kiểm thử PC qua, gồm ghi/lưu bằng `V`, phát lại bằng `U` và dừng. Build Teach thành công (Flash 28620/63488 byte), Debug đã cập nhật. Nạp bản Teach SHA-256 `03B9B885…`, CubeProgrammer xác minh thành công rồi reset. Đọc vector `20005000 08004B55`, hai PWM bằng 0. Không gửi lệnh chuyển động.
- MPU tiếp tục không ACK ở 0x68/0x69 sau reset, xem [nhật ký lỗi](04-nhat-ky-loi.md). Chưa kiểm chứng Bluetooth, ghi hay phát lại trên xe thật.

### 2026-11-22 — Nạp và xác minh bản Teach ban đầu

- Đo nguồn analog qua ADC1 và VREFINT trực tiếp trên chip, ba mẫu 1499/1500/1496 cho ước tính khoảng 3,28 V. Probe vẫn báo 4,07–4,08 V. Đã khôi phục cấu hình ADC sau đo.
- Đã nạp HEX Teach (SHA-256 `8FACBA7E…`) ở SWD Under Reset 100 kHz, CubeProgrammer xác minh thành công rồi reset. Đọc vector sau nạp `20005000 08004B1D`, hai PWM đều 0. Chưa gửi lệnh chuyển động.
- Sau reset, MPU chưa phản hồi I2C ở 0x68/0x69. RAM cho thấy `motion_diag.state=1`, `mpu_diag.stage=1`, `ack_mask=0`, `hal_error=0x20`. Cần kiểm tra nguồn, SDA/SCL của MPU và khởi động lại, đợi MPU sẵn sàng trước khi thử. Chưa thu tuyến. Chi tiết trong [nhật ký lỗi](04-nhat-ky-loi.md).

### 2026-11-22 — Bản thử lái tay, ghi và phát lại không dùng ToF

- Người dùng chốt `F/B/R/L` cho tiến/lùi/xoay phải/xoay trái và `Y` reset phiên. `Z` bắt đầu ghi, `K` lưu, `x` dừng, `Y` rồi `P` phát lại từ đúng vạch. Bản thử riêng `Teach` tắt XSHUT ToF, dùng PWM cơ sở 300/1000 khi đi thẳng và 250/1000 khi xoay, chặn đoạn quá 20 giây, mất tiến độ, MPU lỗi và biên U ước tính. Hướng dẫn ở [README](../README.md#lái-tay-ghi-đường-và-phát-lại-không-dùng-tof).
- Firmware ghi E2 hoặc góc MPU tại điểm cắt PWM của từng đoạn. Tuyến tối đa 200 đoạn được lưu vào hai trang flash luân phiên từ `0x0800F800`, kèm mã lượt. Chỉ xác nhận sau tin `DA_LUU`. Các thông số giới hạn là lựa chọn cho phép thử sân cố định. Nhánh tránh vật ToF của bản thi vẫn ở preset Debug mặc định.
- Build Teach Debug thành công tại `build/Teach/code_test_xetranhvatcan.hex` và ELF tương ứng, Flash 28548/63488 byte, SHA-256 bắt đầu `8FACBA7E`. Build Debug thường thành công, Flash 65500/65536 byte. Chín nhóm kiểm thử PC qua, gồm luồng ghi/phát, lỗi ghi flash và giữ tuyến cũ, dừng `x`, MPU lỗi và biên sân. Kích thước và ký hiệu linker đã được đối chiếu trên ELF. Cơ khí, Bluetooth điện thoại, lưu flash thật và độ lặp lại trên sân còn chờ thử.
- Lỗi đứng ở T7 tiếp tục theo [nhật ký lỗi](04-nhat-ky-loi.md).

### 2026-11-22 — Tái hiện nhánh dừng và sửa bản ghi chẩn đoán

- Người dùng báo xe thật đi ổn tới vùng T7, bắt đầu cua được một đoạn rồi đứng yên. Đã ghi hiện tượng và phép đo cần lấy ở lần thử tiếp theo trong [nhật ký lỗi](04-nhat-ky-loi.md). COM16 và ST-LINK chưa kết nối lúc kiểm tra nên chưa có dữ liệu đúng thời điểm xe dừng.
- Đã thêm quy định ghi lỗi, cách sửa và kết quả kiểm chứng vào `AGENTS.md`. Phiên này cập nhật tài liệu và kiểm tra dữ liệu có sẵn.
- Quy trình so `j` trên sàn trống với `g` trên sân thật và các hướng xử lý theo nhãn dừng nằm trong nhật ký lỗi. COM16 vẫn chưa xuất hiện lúc kiểm tra lại.
- Tái hiện `first_bend_old` và `t9_short` kẹt ở `NAV_SEARCH_BLOCKED` từ mã nguồn hiện tại. Đã sửa firmware để `nav_trace` giữ trạng thái mới và số đo ToF ở đúng vòng chuyển trạng thái. Build `build/TurnTrace/`, 8 nhóm kiểm thử trên PC và bộ mô phỏng đều chạy qua. Bản chưa nạp vì chưa thấy ST-LINK. Chưa xác nhận hay sửa nguyên nhân khiến xe thật đứng ở cua T7.
- Đã đối chiếu cách nạp và thu log. Dùng HEX đúng bản qua ST-LINK để nạp, giữ ELF cùng build để đọc trace RAM. HC-06 gửi góc, ToF, nhãn và hai bộ đếm encoder qua COM. Máy chủ hỗ trợ ghi CSV và đọc trace RAM qua `/api/run` khi có ST-LINK. Hướng cải tiến là gửi thêm sự kiện dừng kèm lý do, tọa độ ước tính, góc đích và PWM. Phiên giải thích chưa triển khai telemetry mới.
- Kiểm tra lại việc thu log từng lượt. Máy vẫn báo không thấy COM16 và ST-LINK, các CSV hiện có là lượt trước. Cần xác nhận máy đang nhận bản tin mới và đã bật ghi CSV trước khi bắt đầu lượt cần chẩn đoán.
- Người dùng đề xuất mỗi lượt chạy lưu vào một thư mục và đưa mã lượt cho agent phân tích sau. Đã ghi [thiết kế](05-ghi-log-chay-thu.md) với nút chạy trên web, PC nhận HC-06 qua COM và thư mục `runs/<mã lượt>/`. Bước đầu tận dụng bản tin sẵn có, bước sau bổ sung sự kiện dừng từ firmware. Mới chốt thiết kế và đối chiếu mã hiện tại. Nút chạy và ghi riêng từng lượt chưa được triển khai.
- Đã thêm quy trình cải thiện riêng, gồm lưu log, khoanh vùng lỗi, sửa, chạy đối chứng và tiêu chí kiểm chứng. `AGENTS.md` và file ngữ cảnh dẫn agent đọc quy trình trước khi xử lý lỗi. Phiên này chỉ bổ sung tài liệu.
- Đã thêm [hướng dẫn ngắn gửi log cho AI](07-cach-gui-log-cho-ai.md). Trong lúc chờ web hỗ trợ tạo lượt tự động, người chạy xuất hoặc sao chép log Bluetooth từ điện thoại, lưu vào `runs/<mã lượt>/log.txt` và nhắn đường dẫn cùng hiện tượng trên sân. Chưa thử xuất log trên app của người dùng.
- Người dùng muốn bấm chạy từ điện thoại và để log tự lên máy tính. Thiết kế cập nhật để điện thoại mở web qua Wi-Fi, máy tính giữ Bluetooth và tự tạo thư mục cho lượt. Đã xác nhận web hiện chỉ nghe trên `127.0.0.1`. Chưa triển khai truy cập từ điện thoại hay nút chạy tự ghi.
- Đã tạo thư mục `runs/` rỗng ở gốc workspace, ngang hàng `docs/`. Trong lúc chờ chức năng tự ghi, dùng nó để lưu từng lượt thủ công. Log cũ vẫn ở `code_test_xetranhvatcan/build/`.

## Nơi làm việc và các phần chính

- Workspace chứa các bản lưu `baseline_v2_original/`, `calibrated_v2/` và `review_*/`. Thư mục dự án đang dùng là [`code_test_xetranhvatcan/`](../code_test_xetranhvatcan/), cũng là gốc CMake trong `.vscode/settings.json`. Mở nhầm `main.c` dưới một bản lưu sẽ dẫn tới sửa sai dự án. Kế hoạch gom bản lưu và log mới ở mức đề xuất.
- `Src/main.c` xử lý lệnh, I2C, vòng lặp và telemetry. `Src/motion.c` phụ trách bước tiến, quay và điều khiển PWM từ yaw/encoder. `Src/navigation.c` chạy tuyến chữ U, kiểm tra ToF, dò né và cập nhật vị trí. `Src/grid_map.c` chứa hình học sân và vật cản. `Inc/course_route.h` chứa các điểm bám tuyến. `Inc/navigation_config.h` và `Inc/motion.h` chứa bộ thông số thường cần đối chiếu.
- `Tools/map_monitor/` là web localhost. `server.py` phục vụ mô phỏng và dữ liệu thật từ COM hoặc ST-LINK, `teach.py` tích hợp bước lái tay thành tọa độ trên web, `route.py` tính tuyến tham chiếu. `Tests/run_checks.ps1` chạy kiểm thử C trên PC. `Tests/sim_course.c` mô phỏng tuyến và các ca sai lệch.

## Cấu hình cần nhớ

| Mục | Giá trị trong dự án chính |
| --- | --- |
| Đơn vị và trục | Firmware và dữ liệu tọa độ dùng mm. Trao đổi vị trí với người dùng thường dùng cm. Gốc ở góc dưới trái, +X về Đông, +Y về Bắc. |
| Mốc xuất phát mặc định | Tâm `(420, −45) mm`, hướng Bắc `90°` trong map. Y âm để mũi xe ở vạch và thân xe phía sau. Các lần thử cũ ở X = 20 cm là cấu hình trước đây. |
| Thân xe theo cấu hình | 90 × 90 mm. Biên bổ sung và các ngưỡng phanh trong code là tham số kỹ thuật. |
| ToF phía trước | `TL/G/TR`, góc tương đối `+25°/0°/−25°`. Độ lệch ngang `+25/0/−25 mm` đang là ước lượng. `HT/HP` là hai kênh sau. |
| Quy đổi từ kích thước bánh | Bánh đường kính 34 mm, cấu hình **770 xung/vòng**. Ưu tiên E2 vì E1 chưa được hiệu chuẩn. |
| Tỷ lệ tự lái mặc định | `MOTION_FIELD_MM_PER_COUNT = 0.1265f` cho cấu hình chữ U chuẩn. |
| Tỷ lệ thử trên web | `TEACH_MM_PER_COUNT = 0.0934` mm/xung cho đường ghi bằng Bluetooth. Ca mô phỏng cua đầu lỗi khi dùng mức này cho tự lái, nên mức thử chỉ có trong `teach.py`. |

`v` xác nhận mốc xuất phát và đổi yaw MPU sang hệ map. Yaw trên bản tin Bluetooth là góc tương đối theo mốc MPU. `v` không thể đặt lại vị trí xe ở giữa sân. Biết vị trí thật qua thước thì có thể dùng nó làm mốc cho bản ghi trên web, nhưng tọa độ đó chỉ thuộc phiên ghi trên web.

## Lệnh và quy trình quan sát

| Lệnh/công cụ | Dùng khi |
| --- | --- |
| `v` → `g` | Thử tự động trên sân thật. Đặt xe về mốc và đúng hướng trước khi gửi, giữ xe yên lúc MPU hiệu chuẩn. |
| `v` → `j` | Thử chuyển động trên sàn trống đủ diện tích. |
| `x` | Dừng hoặc hủy lượt chạy. |
| `w/s`, `a/d`, `u/p` | Bước tiến/lùi, xoay 90°, xoay nhỏ trái/phải. Mã nguồn hiện có xoay nhỏ 10°, sửa hướng MPU ở cuối bước nếu lệch quá 2° và bỏ tự lùi né trong chế độ lái tay. Xe chạy firmware cũ có thể vẫn xoay 45° và tự lùi. |
| `W/A/D` | Giữ phím tiến hoặc xoay trên firmware tương thích. Web chỉ bật `--hold-drive` và `--manual-trim` khi đã xác minh firmware trên xe hỗ trợ. |
| COM Bluetooth | Trước đây dùng `COM16`. Kiểm tra lại cổng, độ mới của `frame` trong `/api/status` và thiết bị đang giữ COM. Dùng `--no-com` để mở mô phỏng độc lập với Bluetooth. |
| ST-LINK | Dùng để nạp hoặc đọc trạng thái firmware thật. Khi đọc biến nội bộ cần ELF khớp với bản trên STM32. `nav_diag`, `motion_diag`, `tof_diag` và `control_diag` giúp xác định điểm dừng, cảm biến kích hoạt, góc, encoder và lệnh gần nhất. Đọc thử khi probe đang cắm và target có nguồn. |

Khi kiểm tra một lượt chạy, ghi rõ chế độ `g` hay `j`, bản firmware, vị trí đặt, thời điểm và nhãn lỗi. Đối chiếu thêm góc, E2, ToF đầu, PWM và vị trí tại lúc dừng nếu có. Tránh suy ra lỗi chỉ từ một nhãn. ToF trả mã ngoài tầm khi giao dịch hợp lệ là trường hợp khác với mẫu cũ hoặc lỗi I2C.

## Trạng thái bàn giao và chỗ dễ nhầm

- Người dùng có nền tảng IT và mới làm quen IoT. Khi giải thích, bắt đầu từ dữ liệu đầu vào, trạng thái xe và điều kiện chuyển trạng thái.
- Web `http://127.0.0.1:8765` có 19 ca mô phỏng C, Python phục vụ kết quả và JavaScript phát lại. Ca T1–T17 và ca trượt một lần 5 mm về đích. Ca thêm vật cản ở chặng Đông bị kẹt. Vị trí thử này chưa được xác nhận là một trong bốn vị trí bốc thăm chính thức. Chưa có đủ lượt chạy thật cùng firmware để đánh giá độ lặp lại.
- Đã từng có ca trên xe thật cua sớm hoặc muộn, dừng sau khi xoay và quệt hoặc kẹt quanh **T9, T11, T13, T14**. Mô phỏng có thể qua được dù xe thật vẫn lỗi. Cần ghi dữ liệu tại điểm lệch trước khi đổi đồng thời tuyến, PID và hệ số xung.
- Một lần web chỉ `(150,77; 119,06) cm` trong khi người dùng đo `(100; 90) cm`. Log cũ cho thấy có bước tự lùi bị ghi như tiến. Đã sửa bộ ghi để bỏ bước có `LUI-KHAN` và bước tiến trả xung E2 theo chiều lùi, rồi đặt lại mốc phiên ghi. Sai số hai trục khác nhau, vì vậy hệ số `0.0934` chỉ là giá trị thử đã dùng trên web.
- Trong đợt tạo README, đã build Debug, chạy đủ 8 nhóm kiểm thử trên PC và mở web offline thành công. Firmware không được nạp trong đợt đó. Khi làm việc với xe thật, xác minh lại phiên bản đang chạy và kết nối hiện tại. Các file trong `build/` chứa nhiều bản thử, dùng sai HEX hoặc ELF sẽ gây nhầm khi nạp hoặc đọc trạng thái.
- `MOTION_SETUP.md` và `NAVIGATION_SETUP.md` chứa nhiều ghi chú theo từng phiên bản. Khi thông số xung, vị trí đặt xe, lệnh quay hoặc hành vi tự lùi khác nhau, kiểm tra mã nguồn dự án chính và firmware trên xe. Quy trình thi và tính điểm lấy từ bản luật trong [mô tả bài toán](01-bai-toan.md).
- Việc cần làm trước khi thi gồm xác nhận nút vật lý khởi động và chạy tiếp, đo bốn dấu đặt vật ngẫu nhiên trên sân, hiệu chuẩn E2 trên đoạn thẳng và xác nhận xe về đích ổn định trên sân thật.






- 2026-11-22 — Sau phản hồi xe chạm T11 và dừng trước đích, tuyến trong `runs/20261122-restore-session/firmware/Inc/course_route.h` được làm phẳng qua dưới T11 (hành lang Y=850–879 mm, X=542–999) và điểm cuối kéo từ Y=-65 xuống Y=-120 mm. Release build 65204 B flash/17184 B RAM, 9 nhóm PC PASS. Chưa nạp/chạy xe thật. Không tăng PWM; ưu tiên xác nhận về đích sạch rồi mới tối ưu tốc độ. Chi tiết ở mục lỗi mới nhất trong `docs/04-nhat-ky-loi.md`.
- 2026-11-22 — Đã nạp HEX Release tuyến làm phẳng qua T11 và điểm cuối Y=-120 mm. SHA-256 `42CECC84E4C4E53E5096DA65A95A97D694342216B26B695456F97738BE20A3B7`. ST-LINK Under Reset 100 kHz, nguồn 3,23 V; CubeProgrammer xác minh toàn file và reset thành công. Đọc lại vector `20005000 0800CD65`, PWM 0. Chưa chạy xe thật với tuyến này.
- 2026-11-22 — Bật pipeline ToF cho tuyến tham chiếu bằng `COURSE_FIXED_LINE_TEST 0` trong bản phục hồi. Nhánh này giảm tốc theo ToF trước, dùng ba cảm biến trước/logic bước né cục bộ có kiểm tra map. `Tests/run_checks.ps1` 9/9 PASS; Release 65180 B flash, RAM 17184 B. Debug link lỗi tràn flash 432 B. Chưa nạp/chưa chạy xe. Chưa triển khai đúng thủ tục xoay vuông góc, kiểm tra min 80 mm rồi dịch ngang 80 mm; cần coi là tích hợp ban đầu, không khẳng định đủ yêu cầu khoảng hở. Xem mục mới nhất `docs/04-nhat-ky-loi.md`.
- 2026-11-22 — Theo số đo người dùng xác nhận có thể xoay ở 30 mm, center ToF detect 50→30 mm; diagonal thresholds giữ 50 mm. Giảm tốc tuyến tính từ 80 mm tới 30 mm, cruise floor 90 mm/s; motion controller đã ramp setpoint. Giảm turn-hit margin 26/28→10/12 mm. Release build PASS (65176 B flash/17184 B RAM), nhưng `Tests/run_checks.ps1` FAIL ở `test_navigation.c:602`, `NAV_SEARCH_BLOCKED` tại x=511,y=997. Chưa nạp/chưa chạy xe. Cần sửa ca định tuyến kẹt trước nạp; dò 90°/dịch ngang 80 mm vẫn chưa được triển khai chính xác.
- 2026-11-22 — Theo yêu cầu ToF: center né<30 mm/release70; hai diagonal<20 mm/release40. Giảm tốc tuyến tính từ80 mm tới30 mm, 90→250 mm/s; tốc độ thẳng thoáng 250 mm/s, ramp gia tốc do motion controller. Release build65196 B flash/17184 B RAM. Tests chưa PASS: case obstacle ở x≈511,y≈997 dừng `NAV_SEARCH_BLOCKED`, vì cả hướng dò ngang va footprint bản đồ. Chưa nạp; cần sửa route/test hình học an toàn. Debug trước đó tràn452 B.
- 2026-11-22 — Người dùng báo lượt xe vừa thử không né/giảm tốc. Nguyên nhân cấu hình: firmware thực nạp gần nhất SHA `42CECC84…` vẫn `COURSE_FIXED_LINE_TEST=1`, bỏ qua ToF. Ứng viên ToF hiện chưa nạp. Ứng viên: mid 30 mm, diagonal20 mm, decel from80 mm, straight target250 mm/s; Release build65196 B/17184 B. Test suite còn fail tại route case `NAV_SEARCH_BLOCKED` x≈511,y≈997; chưa nạp. Bước kế: giải quyết/định nghĩa an toàn vị trí bị kẹt, rồi mới nạp.
- 2026-11-22 — Theo yêu cầu, đã nạp ứng viên ToF SHA `353DDF3DAA6BCCFB4B013F40B720B7DAAEF1772E7B3478600DF10AFF92F423A8` qua SWD Under Reset 100 kHz. Probe báo 4,09 V; download và verify toàn file thành công, reset. Chưa chạy thử thật; ứng viên vẫn có ca mô phỏng NAV_SEARCH_BLOCKED ở đoạn hẹp.

### 2026-11-23 — Tái hiện lỗi simulator qua T9, chưa có bản sửa đạt

- Đọc lại quy trình cải thiện và chạy đúng ca `trace t9` trong simulator của firmware phục hồi.
- Ca va vào vật cản cục bộ giả lập `(530..570, 815..855) mm` tại tâm xe xấp xỉ `(477,786) mm`, yaw `79.2°`. Trace ở `runs/20261123-sim-t9/trace.csv`; chi tiết trong [nhật ký lỗi](04-nhat-ky-loi.md).
- Các thử nghiệm ép heading đi thẳng lên tránh được vật cản nhưng làm kẹt/không nhập lại waypoint nên đã loại bỏ; không có thay đổi firmware mới được giữ từ lượt này. Tọa độ `course_route` không đổi. Ca simulator vẫn FAIL; không build/nạp bản mới.
- Tiếp theo: instrument ca mô phỏng quanh thời điểm ToF hit để ghi `observed_hits`, `Grid_Cell(x,y)`, bit static/dynamic của ô bắt đầu, `BeginReferenceDetour` result và planner state; sau đó sửa điều kiện detour để lập đường trước khi ô xe bị inflation chặn, rồi kiểm tra cả đi vòng và rejoin. Firmware Release gần nhất ghi trong bàn giao trước vẫn chưa nạp; cần xác nhận điện áp bo trước khi cân nhắc nạp.

### 2026-11-23 — `trace t9` đã qua vật cản và về đích trong simulator

- Trong bản phục hồi, dời waypoint qua hành lang T9 vào giữa khe T7/vật cản giả lập; đi lên Y=920 mm, vượt sang X=620 mm rồi mới hạ về Y=900 mm để tránh chạm mép vật cản khi thân xe quay xuống. `U_STRAIGHT_SIDE_MM` 20→10 mm; turn pad cục bộ T9=64 mm; bỏ cộng 5 mm footprint ở đoạn này. Giữ các đường bảo vệ map/biên khác.
- Thêm điều kiện regression `trace t9` phải `finish`. Đã build simulator GCC và chạy ca này: PASS, không va chạm, đích `(1305,-55) mm`, khoảng hở giữa nhỏ nhất41,3 mm, 48,39s mô phỏng. Trace ở `runs/20261123-sim-t9/trace_after.csv`.
- Test map kỳ vọng inflation 55 mm. `Tests/run_checks.ps1` vẫn FAIL tại assertion navigation cũ `NAV_SEARCH_BLOCKED` `(511,997) mm`; trace `nominal`/`t11` có assertion cũ đòi không được quay tại chỗ và hiện không phù hợp với đoạn khe hẹp. Chưa build firmware do thiếu ARM GCC/CMake/Ninja trên PATH, chưa nạp/chưa xác nhận trên xe thật. Thay đổi này chỉ được xác nhận bằng simulator.

### 2026-11-23 — Đã build Release mới, nạp đang chờ nguồn bo an toàn

- Theo yêu cầu người dùng, build Release từ firmware phục hồi sau chỉnh tuyến T9: flash 65.136/65.536 B, RAM17.184/20.480 B. HEX SHA-256 `3E42F156DF2D972086928B7E023F00A71BA12ADC655B5A3E87268768D03DB91B`.
- Đọc ST-LINK bằng STM32CubeProgrammer 2.23.0, SWD Under Reset 100 kHz; báo STM32F103 và **4,03 V**. Không nạp vì vượt trần 3,6 V; HEX mới chưa được ghi lên chip. Cần đo trực tiếp 3V3–GND, hạ về khoảng3,3 V, rồi đọc lại probe trước khi tiếp tục. Xem [nhật ký lỗi](04-nhat-ky-loi.md).

### 2026-11-23 — Đã nạp Release tuyến qua T9

- Người dùng xác nhận điện áp đo trực tiếp là 3,3 V. Đã nạp HEX Release SHA-256 `3E42F156DF2D972086928B7E023F00A71BA12ADC655B5A3E87268768D03DB91B` bằng ST-LINK `37FF71064E573436E9171343`, SWD 100 kHz, Under Reset. CubeProgrammer verify toàn file thành công và reset.
- Đọc lại vector `20005000 0800CD1D`; TIM1 CCR1/CCR2 bằng0. Không chạy xe sau nạp. Simulator T9 PASS; test navigation tổng quát còn fail ca cũ `(511,997) mm`. CubeProgrammer tiếp tục báo4,03 V dù người dùng đo3,3 V; đã ghi nhận sai khác trong nhật ký lỗi. Cần chạy thử có giám sát, sẵn sàng gửi `x`.

### 2026-11-23 — Đã thay bản vòng lớn bằng tuyến mặc định + né ToF giữa

- Người dùng làm rõ tuyến mặc định đã chạy được; chỉ vật cản mới thực sự ở trước xe mới được làm lệch tuyến. Đã khôi phục waypoint mặc định quanh T9/T11 và hoàn các lề hình học bị giảm riêng cho bản vòng lớn.
- Centered route chỉ dùng ToF giữa để xác nhận obstacle và tạo detour; tia chéo không vẽ hit vào map nên cạnh T7/T9 không làm đổi hướng. Hit giữa bị chặn được đưa vào `BeginReferenceDetour()` trước nhánh cung; sau né giữ waypoint để nhập lại tuyến.
- `trace reference_detour` và `heading_recovery` PASS. Bộ test tổng quát vẫn còn ca cũ `NAV_SEARCH_BLOCKED` tại `(511,997) mm`. Release: flash 64.948/65.536 B, RAM 17.184/20.480 B, SHA `5AB6E90E2A4FE700052BB3DC77339EBD54FDB3AF5966FE81035C7E49FBB2B2CA`.
- Đã nạp/verify toàn file qua SWD Under Reset 100 kHz. Đọc lại vector `20005000 0800CC65`, PWM CCR1/CCR2=0. Chưa chạy xe thật sau nạp. CubeProgrammer báo 4,06 V; người dùng đã đo trực tiếp 3,3 V. Bước tiếp theo: chạy tuyến không có vật cản trước để xác nhận giữ nguyên đường, sau đó đặt một vật cản ở chính giữa phía trước tại đoạn thoáng và sẵn sàng gửi `x`.

### 2026-11-23 — Phiên tiếp theo: giữ waypoint, chưa hoàn tất tích hợp ToF

- Người dùng nhấn mạnh tuyến waypoint đã chạy đến đích; yêu cầu bám tuyến đó,
  không để thuật toán tự đổi đường khi không có vật cản.
- Mã dự án chính từng bật `COURSE_FIXED_LINE_TEST`, nhánh này thoát sớm trước
  reactive ToF. Guard lùi chung trong `main.c` cũng có thể tranh quyền với
  Navigation; đã thêm guard để không hủy pose khi Navigation đang chạy.
- `Tests/run_checks.ps1` hiện PASS 9/9 nhóm. Simulator `trace nominal` và `trace
  t9` không đạt với nguồn/harness hiện tại; do đó chưa có bằng chứng tuyến/né
  mới đúng. Không build firmware hoặc nạp phiên này.
- **Việc còn dở:** tích hợp phát hiện ToF, detour và nhập lại waypoint trong cùng
  nhánh bám tuyến; chỉ build/nạp sau khi mô phỏng đạt đường trống lẫn có vật cản.
  Mã chi tiết ở mục mới nhất trong [nhật ký lỗi](04-nhat-ky-loi.md).

### 2026-11-23 — Tiếp tục từ Cline: né waypoint qua T9, đang giải phóng flash

- Đọc bản dán Cline: nó đã chọn trigger **ToF giữa thông thoáng ≥300 mm và một
  tia chéo ≤100 mm**, vì ca T9 sượt ngang trong khi tia giữa vẫn đọc 1500 mm.
  Offset né 35 mm, giữ 400 mm rồi ramp về 0 trong 120 mm; bảng waypoint giữ làm
  đường mặc định. Sweep ghi 30–40 mm là dải qua, 20–25 sượt, ≥45 chạm T7.
- Mã hiện tại ở `code_test_xetranhvatcan/Src/navigation.c` và bảng đã sửa tràn X
  ở `Inc/course_route.h`. Simulator: đường trống finish 14,37 s, hở tối thiểu
  40,3 mm; ca T9 finish 15,18 s, hở 40,5 mm. `Tests/run_checks.ps1` 9/9 PASS.
- Đã bổ sung kiểm tra 3 range hợp lệ; giữa mới ≤200 ms; mỗi tia chéo không cũ
  hơn mẫu giữa quá 100 ms. Sau guard, test và hai trace vẫn đạt; trace chưa mô
  phỏng riêng thời điểm ToF invalid/stale.
- Đã nén ramp về số nguyên mm (làm tròn tối đa 1 mm), giữ trace và giảm 12 B.
  Release hiện vẫn vượt flash **24 B** (65.560 B trên 65.536 B); HEX cũ 100% từ
  trước thay đổi freshness không khớp source. Chưa có firmware buildable mới,
  chưa nạp, chưa chạy xe.
- **Bước kế tiếp:** tối ưu/reclaim ít nhất 36 B, build Release mới và kiểm tra
  lại hash; chỉ sau đó mới cân nhắc nạp. Chi tiết ở
  [`runs/20261123-waypoint-avoid/notes.md`](../runs/20261123-waypoint-avoid/notes.md).

### 2026-11-23 — Rà soát tối ưu không đổi thuật toán

- Đã rà map Release và cách link: đang bật `-ffunction-sections`,
  `-fdata-sections`, `--gc-sections`; hàm không được tham chiếu đã bị loại.
  Các module firmware còn trong map được tham chiếu từ đường chạy, telemetry,
  điều khiển, ToF hoặc kiểm lỗi; không xóa tùy tiện.
- Thử `-Oz` riêng cho main, grid map, motion, VL53L0X và MPU; đều không giảm
  flash (một số tăng 40–808 B). `-fshort-enums` không đổi dung lượng. Đã hoàn tác
  các thử nghiệm compiler; không đổi thuật toán hay logic.
- Test PC 9/9 PASS; trace nominal 14,37 s/hở 40,3 mm và T9 15,18 s/hở 40,4 mm.
  Release vẫn vượt 24 B (65.560/65.536 B). HEX cũ SHA `8D3311E4…` không khớp
  source hiện tại. Chưa có firmware buildable mới, chưa nạp.
- Cần tối ưu có chủ đích một nhánh/chuỗi telemetry hoặc reclaim mã đủ 24 B rồi
  chạy lại test, hai trace và Release. Không bỏ kiểm tra ToF stale hoặc đổi
  tuyến để lấy dung lượng.

### 2026-11-23 — Tối ưu offset né, vẫn chưa đủ chỗ để nạp

- Đổi `fixed_line_offset` từ `float` sang `uint8_t`. Các giá trị được gán đều là
  số nguyên mm (0–35), nên giữ nguyên phép tính offset; trace không đổi. Tiết kiệm
  8 B flash; lần Release cuối còn vượt **16 B** (65.552/65.536 B), vì vậy không
  tạo được HEX mới.
- Thử bỏ unwind tables và linker `--relax`: không giảm dung lượng. Thử
  `-fno-math-errno`: tăng lên 44 B vượt; `-fno-ipa-cp`: tăng lên 804 B vượt.
  Đã hoàn tác các cờ thử; CMake trở về `navigation.c -Oz` và Release `-Os -g0`.
- `Tests/run_checks.ps1`: tất cả 9 nhóm PASS. Trace mới biên dịch từ source:
  nominal finish 14,37 s, hở 40,3 mm; T9 finish 15,18 s, hở 40,4 mm.
- Không nạp: HEX còn trên đĩa là artifact cũ, không khớp source. Cần giảm ít
  nhất 16 B bằng thay đổi tương đương, build Release thành công, rồi mới kiểm tra
  ST-LINK/nguồn và nạp. Xe thật chưa được kiểm tra ở phiên này.

### 2026-11-23 — Release fit và đã nạp lại theo yêu cầu

- Đổi so sánh `fixed_line_offset == 0.0f` thành `== 0u` sau khi trường đã là
  `uint8_t`. Quan trọng hơn, cấu hình Release được chạy lại sau khi hoàn tác
  thử nghiệm `-fno-ipa-cp`; build từ cờ chuẩn `-Os -g0` thành công: flash
  62.900/65.536 B, RAM 17.616/20.480 B. HEX SHA-256
  `18A3401F0BFE16EF6DDA28250B3DA86136A6503A5E56D0096B31A9D5C111F587`.
- `Tests/run_checks.ps1`: 9/9 nhóm PASS. Simulator mới biên dịch:
  nominal finish 14,37 s/hở 40,3 mm; T9 finish 15,18 s/hở 40,4 mm.
- Đã nạp và verify toàn file bằng CubeProgrammer 2.23.0, ST-LINK
  `37FF71064E573436E9171343`, SWD Under Reset 100 kHz. Điện áp probe lúc kết nối
  3,27 V. Đọc lại vector `20005000 0800CDC5`; CCR1/CCR2 đều 0. Không gửi lệnh
  chuyển động; chưa xác nhận hành vi xe chạy thật sau lần nạp.
- Log: `runs/reflash_release_build.log`, `runs/reflash_checks.log`,
  `runs/reflash_nominal.log`, `runs/reflash_t9.log`, `runs/reflash_program.log`,
  `runs/reflash_readback_vector.log`, `runs/reflash_readback_pwm.log`.
### 2026-11-23 — Đổi điểm xuất phát X=30 cm và nối chéo về waypoint cũ

- Mặc định `navigation_config.start_x_mm=300`, giữ `start_y_mm=-45`; `COURSE_ROUTE_START_X_MM` đồng bộ X=300 để tiếp tục fixed-line mode.
- Thêm điểm nối chéo sớm `(420,100) mm` để sang làn waypoint cũ X=420 mm trước khi lên ngang T3; sau đó giữ tuyến cũ. Điểm cuối kéo từ Y=-70 xuống -100 mm theo báo cáo xe hụt đích khoảng 30 mm.
- Khi ToF giữa có mẫu mới đầu tiên dưới 25 mm, xe dừng lệnh tiến và chờ mẫu mới thứ hai trước khi xoay dò; giữ ngưỡng 25 mm cùng logic waypoint. Sửa này tránh góc thân quét vào vật cản trong mô phỏng.
- Mô phỏng `run_checks.ps1`: 10 nhóm unit + 7 trace PASS. Điểm nối `(420,100) mm`: nominal/T9 từ X=300 mm về đích; ca `front_open` cũng về đích, hở nhỏ nhất 36,5 mm. Invalid center vẫn fault và obstacle lớn vẫn dừng an toàn.
- Release/Debug đều build được; Release HEX SHA-256 `0042E041914FB637BB909D34BFC023B8D457CCDAB752616630F1B23B28E94B06` **đã nạp và verify** bằng CubeProgrammer 2.23.0, ST-LINK `37FF71064E573436E9171343`, 3,22 V. Đọc vector `20005000 0800D361`; TIM1 CCR1/CCR2 = 0. Chưa chạy xe thật; bước tiếp là thử có giám sát đoạn chéo và qua đích.
- Ghi chú lượt: [`runs/20261123-start30-connector/notes.md`](../runs/20261123-start30-connector/notes.md).

### Cập nhật 2026-11-23 — Dời điểm rẽ chéo lên Y=20 cm

- Người dùng báo điểm nối Y=10 cm rẽ chéo quá sớm; đổi connector từ `(420,100)` sang `(420,200) mm`. Xuất phát vẫn `(300,-45) mm`; phần waypoint cũ và logic dừng chờ hai mẫu ToF được giữ nguyên.
- `Tests/run_checks.ps1`: 10 nhóm unit + 7 trace PASS. Nominal/T9/front_open về đích sau 15,56/17,23/18,95 s; hở mô phỏng nhỏ nhất 40,3/40,1/36,2 mm. Release/Debug build PASS.
- Release hiện tại SHA-256 `F63A7E6DE3ED8BBB95786F807E7EBE44EE35DDE19A3CBD41047ACA91605182EC` đã nạp/verify. CubeProgrammer 2.23.0, ST-LINK `37FF71064E573436E9171343`, 3,26 V; vector `20005000 0800D361`, TIM1 CCR1/CCR2 = 0. Chưa chạy xe thật.
- Chi tiết: [`runs/20261123-start30-connector/notes.md`](../runs/20261123-start30-connector/notes.md).

### Cập nhật 2026-11-23 — Tăng tốc khi thoáng, hạ tốc gần vật cản

- Fixed-line cruise trên đoạn thoáng tăng 210→300 mm/s. ToF giữa bắt đầu giảm tốc từ 160 mm xuống 90 mm/s tại 25 mm; nếu ToF chéo mới báo vật cản trong 180 mm thì giới hạn 200 mm/s; pha dò/né giới hạn 140 mm/s.
- `Tests/run_checks.ps1`: 10 nhóm unit + 7 trace PASS. Simulator nominal 14,77 s / 40,4 mm; T9 16,44 s / 40,4 mm; vật cản 40×40 mm phía trước 18,25 s / 37,0 mm. So với trước cùng tuyến: 15,56 / 17,23 / 18,95 s. Chưa xác nhận tốc độ và độ trượt trên xe thật.
- Release/Debug build PASS. Release SHA-256 `CBB76A40D99A2F08A97F33D2147DF064E532218FA267CF5376B07FBB9070351A` đã nạp/verify bằng CubeProgrammer 2.23.0, ST-LINK `37FF71064E573436E9171343`, 3,27 V. Vector sau reset `20005000 0800D3A5`; TIM1 CCR1/CCR2 = 0. Chưa chạy xe thật sau khi nạp.
- Ghi chú: [`runs/20261123-speed-profile/notes.md`](../runs/20261123-speed-profile/notes.md).

### 2026-11-23 — Giảm giật hướng khi bám waypoint

- Người dùng báo xe vẫn đi zic-zac và hỏi có thể chỉnh lái/bẻ góc trong lúc chạy không.
- Tuyến waypoint và luật ToF được giữ nguyên. Trong `code_test_xetranhvatcan/Src/navigation.c`, khi xe chạy theo tuyến bình thường, điểm ngắm chuyển từ waypoint hiện tại sang waypoint kế tiếp đã lấy mẫu; nhờ vậy hướng điều khiển thay đổi sớm và đều hơn trên các đoạn cong. Khi có lệch tránh, xe vẫn ngắm waypoint đã dịch như trước. Hướng dùng để bắt đầu dò vật cản giữ theo waypoint gốc để không đổi hành vi né.
- Thử nghiệm lookahead 45 mm và giới hạn heading ±12° làm một số trace T9/đầu tuyến thất bại nên đã bỏ. Bản cuối dùng một waypoint lấy mẫu phía trước. `Tests/run_checks.ps1`: 10 nhóm unit và 7 trace PASS. Simulator: nominal về đích 16,00 s (khe hở 34,8 mm), T9 16,90 s (35,2 mm), vật cản trước 19,76 s (38,2 mm). So với speed-profile trước đó 14,77/16,44/18,25 s, bản này chậm hơn; tốc độ đặt cruise vẫn 300 mm/s. Chưa chạy xe thật.
- Release build PASS: FLASH 64.644/65.536 B, RAM 17.656/20.480 B; Debug PASS: FLASH 65.396/65.536 B, RAM 17.672/20.480 B. HEX Release SHA-256 `9056360DCFABACEE69039BEBDE127322EBBE8C6301E69502223FF055895DCA42`.
- Theo yêu cầu người dùng, đã nạp HEX này bằng CubeProgrammer 2.23.0 qua ST-LINK `37FF71064E573436E9171343`, SWD 100 kHz/Under Reset. Probe báo 3,23 V; `Download verified successfully`. Đọc lại vector `20005000 0800D495`, TIM1 CCR1/CCR2 đều 0. Chưa chạy xe sau nạp.
- Bước tiếp: chạy thử có giám sát để xác nhận độ mượt và độ trượt thực tế; nếu xe lệch, thu telemetry vị trí/yaw ở đoạn cua.
- Chi tiết: [`runs/20261123-waypoint-smoothing/notes.md`](../runs/20261123-waypoint-smoothing/notes.md).

### 2026-11-23 — Khôi phục cách bám waypoint trước bản làm mượt

- Người dùng báo bản làm mượt waypoint bị lỗi và yêu cầu quay lại bản trước. Trong `Src/navigation.c`, đã bỏ mục tiêu waypoint kế tiếp và đưa hướng dò ToF về hướng ngắm cũ; hành vi bám route trở lại ngắm waypoint hiện tại. Không đổi route, ToF hay profile tốc độ.
- `Tests/run_checks.ps1`: 10 nhóm unit + 7 trace PASS. Release PASS: FLASH 64.532/65.536 B, RAM 17.656/20.480 B; Debug PASS: FLASH 65.276/65.536 B, RAM 17.672/20.480 B.
- Bản Release phục hồi đã nạp/verify bằng CubeProgrammer. SHA của HEX tái build là `2ED13149325596D6D45004D3B196FC5D58957F82A7416C18AEBE9ECC5E1F2128`; khác SHA artifact cũ `CBB76A40…` đã ghi ở bàn giao trước, dù logic nguồn trước bản làm mượt đã được khôi phục. Chưa xác định lý do sai khác byte (artifact cũ không còn trong workspace). Readback vector `20005000 0800D425`, CCR1/CCR2=0; probe 3,27 V. Chưa chạy xe sau nạp.
- Bước tiếp: thử xe có giám sát; nếu bản phục hồi vẫn lỗi, ghi đúng vị trí và telemetry để khoanh vùng.
