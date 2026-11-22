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






