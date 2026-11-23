# Mượt hướng khi bám waypoint

Ngày: 2026-11-23

## Mục tiêu và giới hạn

Người dùng báo xe zic-zac và muốn xe chỉnh hướng trong lúc tiến. Giữ nguyên route, tốc độ đặt và xử lý ToF; không nạp firmware trong lượt này.

## Thay đổi

- `code_test_xetranhvatcan/Src/navigation.c`: khi bám tuyến bình thường và đang điều khiển tiến, điểm ngắm là waypoint đã lấy mẫu kế tiếp thay vì điểm hiện tại. Cách này cho bộ điều khiển thấy sớm hướng cong trên dãy waypoint dày.
- Khi đang lệch tránh, giữ nguyên điểm ngắm waypoint có offset. Khi tia giữa kích hoạt dò, hướng dò lấy từ waypoint gốc để giữ hình học né đã kiểm chứng.
- Không đổi route trong `Inc/course_route.h`, ngưỡng ToF hay `FIXED_LINE_CRUISE_MM_S`.

## Thử nghiệm

- Thử lookahead dài 45 mm: trace T9 thất bại. Giới hạn heading ±12°: trace nominal lệch khỏi connector. Lookahead 20 mm: trace `front_open` thất bại khi detour. Đã loại bỏ các biến thể thất bại; chỉ ghi để lưu kinh nghiệm.
- Bản cuối: `Tests/run_checks.ps1` PASS cả 10 nhóm unit và 7 trace.
- Simulator cuối:
  - nominal: finish, 16,00 s, khoảng hở 34,8 mm.
  - T9: finish, 16,90 s, khoảng hở 35,2 mm.
  - vật cản giữa `front_open`: finish, 19,76 s, khoảng hở 38,2 mm.
- So speed-profile trước: 14,77/16,44/18,25 s. Bản mới chậm hơn trong simulator; cần cân nhắc sau khi xác nhận đường chạy mượt trên xe. Mô phỏng chưa xác nhận độ trượt hoặc độ mượt thực tế.
- Release build PASS: FLASH 64.644/65.536 B, RAM 17.656/20.480 B. Debug PASS: FLASH 65.396/65.536 B, RAM 17.672/20.480 B.
- Release HEX SHA-256: `9056360DCFABACEE69039BEBDE127322EBBE8C6301E69502223FF055895DCA42`.

## Nạp firmware

Theo yêu cầu người dùng, đã nạp file trên bằng STM32CubeProgrammer 2.23.0, ST-LINK SN `37FF71064E573436E9171343`, SWD 100 kHz, Under Reset. Probe báo 3,23 V; CubeProgrammer báo `Download verified successfully` và reset MCU. Readback vector: `20005000 0800D495`; TIM1 CCR1/CCR2 đều `00000000`. Không gửi lệnh chuyển động; chưa thử xe thật sau nạp.

## Trạng thái / tiếp theo

Đã sửa và nạp, chờ thử xe thật. Khi thử, đặt xe ở mặt sân phẳng và giám sát gần T9/đoạn cua; nếu lệch, lưu telemetry x/y/yaw trước khi tăng thêm lookahead hoặc tốc độ.

## Cập nhật sau phản hồi lỗi và rollback

Người dùng báo firmware smoothing bị lỗi. Đã hoàn tác hai thay đổi trong `Src/navigation.c` để trở lại cách ngắm waypoint hiện tại và hướng dò ToF trước bản smoothing. Test đạt 10 nhóm + 7 trace; Release/Debug build PASS. Release tái build SHA `2ED13149325596D6D45004D3B196FC5D58957F82A7416C18AEBE9ECC5E1F2128` đã nạp và verify, ST-LINK `37FF71064E573436E9171343`, probe 3,27 V. Readback vector `20005000 0800D425`, CCR1/CCR2=0. SHA này khác artifact trước smoothing `CBB76A40…`; artifact cũ không còn trong workspace nên không khẳng định byte-identical. Chưa chạy xe thật sau rollback; chưa rõ triệu chứng cụ thể ngoài “bị lỗi”.
