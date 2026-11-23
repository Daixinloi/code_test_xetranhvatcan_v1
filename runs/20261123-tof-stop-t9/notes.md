# Lượt 2026-11-23 — xe dừng gần T9

- Môi trường: xe thật, người dùng điều khiển/tự chạy trên sân; vị trí báo miệng là trước khi qua T9.
- Trước khi nạp, flash có vector `20005000 0800CD3D`, khớp ứng viên trước SHA `353DDF3D…`.
- Quan sát: xe đứng yên lúc chuẩn bị qua T9. Không có timestamp, lệnh, log ToF, tọa độ, yaw hoặc trạng thái điều hướng.
- Mong đợi: tia chéo phải gần làm xe chỉnh lái nhẹ sang trái khi tia giữa còn thoáng; chỉ tia giữa gần mới bắt đầu dò né.
- Giả thuyết mã: tia chéo phải đang hạ `ForwardRoom` và có ngưỡng cận đặc biệt 50 mm ở T9, làm dừng/né dù tia giữa thông.
- Ứng viên source/build: `../20261122-restore-session/firmware/`; Release SHA `73FE6488A84F5CFF68726C4055860DF8D8A272C897CA1DE245CA18CBFAD2ED25`.
- Nạp theo yêu cầu: ST-LINK `37FF71064E573436E9171343`, STM32F103 medium-density 64 KB, nguồn 3.27 V, SWD Under Reset 100 kHz. CubeProgrammer ghi và xác minh toàn file thành công, rồi reset. Đọc lại vector `20005000 0800CE1D`; TIM1 CCR1/CCR2 đều 0. Log ở `flash.log.out` và `readback.log`.
- Kiểm tra PC: Release build thành công; test navigation dừng ca `NAV_SEARCH_BLOCKED` tại `(511,997) mm`; Debug vượt flash 612 B. Xe chưa chạy thử sau lần nạp.
- Cần thu lần sau: firmware hash/vector, `TL/G/TR`, `nav_diag.state`, pose/yaw/PWM ở 1 giây trước và sau khi xe dừng.

## Hiệu chỉnh sau phản hồi “xoay hướng linh tinh” — 2026-11-23

- Người dùng xác nhận tuyến waypoint cố định trước đó đã đi qua được; lỗi là dừng/né ở T9. Bỏ heading bias liên tục 2–8° theo tia chéo phải; giữ nguyên `course_route`.
- Đã tìm thấy cản trở chính trong mã: `Service_Front_Clearance()` can thiệp cả khi `Navigation_IsActive()`, gọi `Navigation_InvalidatePose()` và lùi khẩn cấp lúc center <30 mm, nên né cục bộ không thể chạy/rejoin. Bản ứng viên mới bỏ guard chung trong lúc Navigation đang sở hữu lượt, giữ `reference_active` để nhập lại tuyến.
- Per Hybrid Waypoint + Local Reactive Avoidance: giữ nguyên waypoint; ba ToF đều có thể lập hit động, guard lùi chung không giành quyền khi Navigation active, và hit được giữ để tính detour/rejoin. Bỏ heading bias liên tục. Vạch đen được xử lý bằng footprint/map; IR line chưa tích hợp vào Navigation.
- Release SHA `30CBC1382B87A581BBB5B44F0F0CD504B068B1FCE69173500764ED0C5101C415`, flash 64,940 B, RAM 17,184 B. Test vẫn fail ca map hẹp `(511,997) mm`; Debug vượt 220 B. Chưa có mô phỏng centered riêng về né T9/Ô 4–5.

## 2026-11-23 — Chốt lại tuyến mặc định và quyền kích hoạt né

- Theo phản hồi người dùng, bỏ bản vòng lớn qua T9 và khôi phục waypoint mặc định đã chạy được. Hai ToF chéo không tạo map hit trong centered route; chỉ hai mẫu liên tiếp từ ToF giữa xác nhận vật cản phía trước mới tạo detour cục bộ.
- Ca PC `trace reference_detour` PASS và nhập lại waypoint; `heading_recovery` PASS. Release SHA `5AB6E90E2A4FE700052BB3DC77339EBD54FDB3AF5966FE81035C7E49FBB2B2CA`, flash 64.948 B, RAM 17.184 B.
- Đã nạp và verify toàn file; vector `20005000 0800CC65`, PWM=0. Chưa chạy xe thật sau nạp.
- Chưa nạp: ST-LINK báo 4.03 V trước khi ghi (STM32F103 max 3.6 V). Đo trực tiếp 3V3–GND trước khi tiếp tục. Chip vẫn có bản trước SHA `73FE6488…`, vector `20005000 0800CE1D`; PWM CCR1/CCR2 bằng 0.
