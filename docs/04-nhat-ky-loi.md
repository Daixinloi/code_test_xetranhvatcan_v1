# Nhật ký lỗi và cách sửa

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





