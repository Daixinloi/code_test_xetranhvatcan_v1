# Thay điểm xuất phát sang x=30 cm và nối lại tuyến

Ngày: 2026-11-23

## Cập nhật sau lượt nạp — dời điểm nối chéo lên Y=200 mm

- Người dùng thấy lần rẽ chéo ở Y=100 mm quá sớm, yêu cầu tăng Y lên 200 mm.
- `course_route.h`: điểm nối đổi `(420,100)` → `(420,200) mm`; các điểm còn lại giữ nguyên.
- `Tests/run_checks.ps1`: 10 nhóm unit + 7 trace PASS. Nominal/T9/front_open lần lượt về đích trong 15.56/17.23/18.95 s; khoảng hở mô phỏng 40.3/40.1/36.2 mm.
- Release và Debug build PASS. Release SHA-256 `F63A7E6DE3ED8BBB95786F807E7EBE44EE35DDE19A3CBD41047ACA91605182EC`.
- Đã nạp bản chỉnh Y=200 bằng STM32CubeProgrammer 2.23.0; ST-LINK `37FF71064E573436E9171343`, SWD Under Reset 100 kHz, điện áp 3,26 V. CubeProgrammer báo `Download verified successfully`, sau đó reset. Đọc lại vector `20005000 0800D361`; TIM1 CCR1/CCR2 = 0. Chưa gửi lệnh chuyển động/chưa chạy xe thật.

## Yêu cầu

- Điểm đặt mới: tâm xe `x=300 mm`, giữ `y=-45 mm` và hướng bắc như cấu hình.
- Từ điểm đặt mới, đi chéo nhập lại làn waypoint cũ tại `x=420 mm`, sau đó chạy tuyến cũ.
- Người dùng báo lượt trước còn thiếu khoảng 30 mm mới qua đích; kéo điểm cuối thêm 30 mm.

## Thay đổi

- `Inc/navigation_config.h`: mặc định start X từ 420 lên 300 mm.
- Trạng thái hiện tại `Inc/course_route.h`: start X của fixed-line là 300 mm; waypoint nối chéo `(420,200) mm`, rồi giữ nguyên các waypoint cũ từ `(420,580) mm` trở đi; điểm cuối `(1320,-100) mm`.
- Khi ToF giữa có mẫu mới đầu tiên dưới 25 mm, `Src/navigation.c` hủy lệnh tiến và giữ xe đứng yên chờ mẫu mới thứ hai xác nhận; sau đó mới xoay dò. Giữ nguyên ngưỡng 25 mm, điều kiện hai mẫu, khoảng dò và thuật toán waypoint. Thay đổi này tránh xe lăn thêm một nhịp rồi quét góc thân vào vật cản trong lúc quay.

## Kiểm chứng

- `Tests/run_checks.ps1`: 10 nhóm unit và 7 trace PASS. Với điểm nối `(420,200) mm`, nominal/T9 từ `(300,-45) mm` về đích không va chạm trong 15.56/17.23 s, khe hở mô phỏng 40.3/40.1 mm. `front_open` 40×40 mm tại `(990..1030,840..880) mm` về đích trong 18.95 s, khe hở nhỏ nhất 36.2 mm. `front_no_echo` và `t9_no_echo` PASS; invalid center vẫn fault; obstacle lớn vẫn dừng an toàn.
- Release build PASS: FLASH 64,336/65,536 B, RAM 17,656/20,480 B; HEX SHA-256 `F63A7E6DE3ED8BBB95786F807E7EBE44EE35DDE19A3CBD41047ACA91605182EC` (bản Y=200 hiện tại, chưa nạp).
- Debug build PASS: FLASH 65,104/65,536 B, RAM 17,672/20,480 B.
- Nạp Release bằng STM32CubeProgrammer 2.23.0, ST-LINK `37FF71064E573436E9171343`, SWD Under Reset 100 kHz; probe báo 3,22 V. CubeProgrammer báo `Download verified successfully`, sau đó reset. Đọc lại vector `20005000 0800D361`; TIM1 CCR1/CCR2 ở `0x40012C34/38` đều 0. Chưa gửi lệnh chuyển động và chưa chạy xe thật.
- Mô phỏng xác nhận route và ca vật cản đã nêu; không thay thế lượt thử có giám sát trên sân.

## Bước tiếp theo

Tiếp theo thử có giám sát trên sân với start X=30 cm, xác nhận đoạn chéo nhập làn và qua đích; ghi log ToF/yaw nếu hành vi khác mô phỏng. Chưa nạp.
