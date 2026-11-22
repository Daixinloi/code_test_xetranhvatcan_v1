# Định hướng giải quyết

Mục tiêu kỹ thuật trước mắt là chạy trọn sân thật và về đích sạch ở tốc độ ổn định. Sau khi lặp lại được kết quả, rút ngắn cung cua và tăng tốc đoạn thẳng. Thứ tự này phù hợp với [cách tính điểm của đề](01-bai-toan.md).

## Kiến trúc điều khiển

1. **Tuyến tham chiếu.** Dùng map 17 khối tĩnh và tuyến Bắc → Đông → Nam đã có. Sinh đường ngắn có thể chạy được, bo các góc dựa trên bán kính quay và kiểm tra cả diện tích thân xe khi tiến hay đổi hướng. Kiểm tra riêng lối vào và lối qua đích. Chỉ kết thúc khi cả xe đã qua vạch và còn nằm trong bề rộng cửa đích.
2. **Bám hướng và vận tốc.** Dùng encoder E2 đo quãng đường, yaw MPU để cập nhật hướng và điều khiển chênh lệch PWM hai bánh trong lúc chạy. Tăng tốc từ từ khi vào đoạn thẳng và giảm tốc từ trước khi vào cua. Khi còn đường an toàn, giữ PWM hai bánh trên ngưỡng thắng ma sát và ưu tiên cua bằng chênh lệch tốc độ.
3. **Né sớm theo ToF.** Dùng mắt giữa để phát hiện vật trên hướng tiến, hai mắt chéo quan sát khoảng hở từng bên. Kiểm tra cung né theo phần thân xe quét qua và cả hai loại biên. Nếu hai phía đều có cung an toàn, ưu tiên phải theo hướng xe. Nếu một phía thoáng hơn rõ rệt, ưu tiên phía đó. Khi hướng hành trình đã thông và thân xe đủ chỗ, nối lại tuyến ngay. Ưu tiên tiến theo cung. Khi phía trước bị chắn, chỉ đổi hướng nếu còn không gian quét an toàn. Giữ phanh khi chưa thể xác nhận đường an toàn hoặc dữ liệu cảm biến đã cũ.
4. **Tách lớp điều hướng và bảo vệ.** Tận dụng tuyến chữ U và các nhánh dò đã có. Rà các trường hợp lặp quét, rẽ lại dù đã thông và dừng vì mục tiêu quá sát biên. Nếu cần đơn giản hóa, ưu tiên nhánh bám tuyến và né cục bộ mặc định. Giữ A* ở các cấu hình đang dùng nó cho tới khi có kiểm thử thay thế tương đương. Vòng điều khiển cần tiếp tục đọc ToF, MPU và lệnh dừng trong mọi trạng thái dò đường.

## Khép khoảng cách giữa mô phỏng và xe thật

- **Hiệu chuẩn quãng đường và góc riêng.** Đánh dấu quãng đường thẳng đã đo, ghi chênh lệch E2 có dấu và yaw đầu/cuối. Chạy vài lượt cùng điều kiện, lấy `mm/xung = quãng đường thực (mm) / |ΔE2|` và so độ lặp lại. Hiệu chuẩn tiếp góc quay và khoảng quét thân xe. Cần phân biệt quãng đường tiến với số xung phát sinh khi xoay, tự lùi và trượt bánh.
- **Đối chiếu các mốc có ích.** Ghi vị trí thật và vị trí firmware tại cua đầu, vùng T9/T11, góc phải T13/T14 và lúc qua đích. Dùng log ToF, yaw, E2 và trạng thái điều hướng để xác định vì sao xe quyết định cua hoặc dừng. Cấu hình ToF và map hiện là số đo ban đầu hoặc ước lượng theo ảnh, nên kiểm lại bằng thước trên sân thật.
- **Thử tăng dần độ khó.** Dùng `v` → `j` trên sàn trống đủ rộng để kiểm tra quãng đường và bám cua. Dùng `v` → `g` khi thử trên sân có vật cản. Kiểm thử mô phỏng với tuyến danh định, sai số xung, lệch yaw, vật cản bổ sung, cảm biến cũ và hai biên. Mô phỏng giúp phát hiện hồi quy. Xe thật là nơi xác nhận quãng đường, độ trượt và khả năng né.

## Chuẩn bị chế độ thi

- **Lượt 1** dùng lái tay để kiểm tra vị trí vật cản, lối hẹp và sai số trên mặt sân. Ghi lại map hoặc số đo hữu ích trong giới hạn 3 phút. Đối chiếu bốn dấu đặt vật ngẫu nhiên với sân chính thức trước khi bổ sung bốn ca mô phỏng và thử thật.
- **Lượt 2** cần luồng chờ khóa xe và khởi động bằng **nút trên xe** theo hiệu lệnh. Kiểm tra hoặc triển khai nút và cơ chế tiếp tục sau khi đặt lại đúng vạch chia phần. Lệnh Bluetooth `v` → `g` hiện phục vụ thử nghiệm. Cách định vị lại ở vạch vừa qua là hạng mục riêng, vì `v` luôn đặt lại mốc xuất phát.
- **Tiêu chí kiểm chứng.** Xe xuất phát bằng nút, chạy tự động trên sân cố định và từng vị trí vật ngẫu nhiên, vượt các phần hợp lệ, giữ bánh trong phạm vi luật và đưa cả thân qua đích trong thời gian cho phép. Thử thêm tình huống chỉnh xe rồi bấm chạy tiếp, cũng như tình huống cần dừng khi cảm biến mất mẫu. Ghi kết quả từng ca cùng loại firmware đã nạp trước khi tối ưu thời gian.

Mã và lệnh dùng để bắt đầu nằm trong [ngữ cảnh bàn giao](03-ngu-canh.md) và [README](../README.md).
