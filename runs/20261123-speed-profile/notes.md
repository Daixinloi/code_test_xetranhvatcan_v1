# Tăng tốc đoạn thoáng, hạ tốc gần vật cản

Ngày: 2026-11-23

## Thay đổi

- Trong `code_test_xetranhvatcan/Src/navigation.c`, nâng tốc độ mục tiêu tuyến fixed-line ở đoạn thoáng từ 210 lên 300 mm/s.
- Bắt đầu giảm tốc theo ToF giữa từ 160 mm; giảm tuyến tính về 90 mm/s tại ngưỡng dừng 25 mm.
- Nếu một tia chéo mới cho thấy vật cản trong 180 mm, giới hạn tốc độ 200 mm/s.
- Trong pha quay/dò né, giới hạn 140 mm/s để xe xử lý chậm hơn. Không đổi waypoint, ngưỡng bắt đầu né 25 mm hay điều kiện hai mẫu xác nhận.

## Kiểm chứng PC

- `Tests/run_checks.ps1`: 10 nhóm unit PASS; 7 trace PASS. Bổ sung assertion xác nhận có đoạn clear đạt 300 mm/s, tia chéo gần làm giảm tốc và ToF giữa làm giảm tốc trước khi probe.
- Trace nominal: 14.77 s, khe hở mô phỏng 40.4 mm; trước chỉnh 15.56 s, 40.3 mm.
- Trace T9: 16.44 s, khe hở 40.4 mm; trước chỉnh 17.23 s, 40.1 mm.
- Trace `front_open` (khối 40×40 mm): 18.25 s, khe hở 37.0 mm; trước chỉnh 18.95 s, 36.2 mm.
- Các thời gian/khe hở là simulator, chưa phải số đo xe thật.

## Build và trạng thái mạch

- Release PASS: FLASH 64,404/65,536 B, RAM 17,656/20,480 B; SHA-256 `CBB76A40D99A2F08A97F33D2147DF064E532218FA267CF5376B07FBB9070351A`.
- Debug PASS: FLASH 65,172/65,536 B, RAM 17,672/20,480 B.
- Đã nạp bản Release SHA `CBB76A40D99A2F08A97F33D2147DF064E532218FA267CF5376B07FBB9070351A` bằng STM32CubeProgrammer 2.23.0, ST-LINK `37FF71064E573436E9171343`, SWD Under Reset 100 kHz; probe 3,27 V. CubeProgrammer báo `Download verified successfully`, sau đó reset. Đọc vector `20005000 0800D3A5`; TIM1 CCR1/CCR2 = 0. Chưa chạy xe thật sau khi nạp.

## Bước tiếp theo

Nạp sau khi người dùng yêu cầu; thử có giám sát trên mặt sân, sẵn sàng dừng bằng `x`. So thời gian thật, độ trượt và log encoder/yaw/ToF với lượt trước.
