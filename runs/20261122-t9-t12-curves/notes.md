# Tuyến cong tăng khoảng hở T9 và T12

- Ngày 2026-11-22, người dùng báo T9 vừa đủ qua, xe chạm nhẹ góc T12 trên ảnh mới, và yêu cầu nâng đoạn T9 20 mm cùng đường cong. Bản nạp gần nhất theo ngữ cảnh là Release 483C01A7. Ảnh là quan sát người chạy, chưa có telemetry cùng lượt.
- course_route.h tạo các điểm trên các cung Bezier bậc hai. Điểm khống chế qua T9 nâng từ (503,880) lên (503,900) mm, chặng ngang giữ Y = 900 mm tới X khoảng 710 mm rồi hạ nhẹ về Y = 875 mm trước T11. Đoạn sau đi qua vùng (1130,750) mm rồi bo về X = 1300 mm để xuống đích. Hệ số encoder giữ như bản trước.
- Nhánh FixedLineMove bám mục tiêu cách trước tối thiểu khoảng 45 mm, chuyển mục tiêu khi đang chạy và gọi Motion_RetargetForward hiện có. MPU, encoder, giới hạn biên và đường dừng x vẫn hoạt động. Khi góc quá lớn hoặc tiến độ kết thúc, xe có thể dừng để chỉnh hướng. ToF vẫn được bỏ qua trong nhánh fixed line hiện tại.
- Mô phỏng test_navigation --trace tới (1300,-57) mm, hướng Nam. Nội suy trace từng 1 mm và 0,1° xác nhận thân 90x90 mm cộng 17 mm mỗi phía không chạm 17 khối, vùng giữa hoặc biên sân trên map cấu hình. Không suy ra khoảng hở thực tế từ ảnh.
- Chín nhóm kiểm thử PC qua. Test trace vẫn kiểm tra lỗi MPU, lỗi encoder và tới đích. Đã tạo simulation.json và route-comparison.png.
- Release build thành công, Flash 65312/65536 byte. Debug build được thử nhưng vượt dung lượng flash khoảng 760 byte. HEX để thử nằm trong build/Release.
- 2026-11-22 đã nạp bản này qua SWD Under Reset 100 kHz lúc nguồn 3,27 V. SHA-256 `38C477EDC02F35B89C6B822356BEE126028F1A566B25A5EFD5DBECA422DBC928`, CubeProgrammer xác minh toàn file thành công và vector sau nạp là `20005000 0800CD65`. Chưa kiểm chứng xe thật sau lượt nạp.
