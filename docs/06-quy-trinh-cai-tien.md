# Quy trình cải thiện bộ điều khiển xe STM32

Dùng quy trình này khi phân tích một lượt chạy, xử lý lỗi hoặc thay đổi firmware. Mục tiêu là về đích ổn định theo [luật thi](01-bai-toan.md), trước hết trên sân cố định rồi với từng vị trí bốc thăm. Chỉ tối ưu tốc độ sau khi xe đã chạy sạch và lặp lại được kết quả.

## Quy tắc làm việc

- **Mỗi lượt có một mã và một thư mục riêng.** Ghi điều kiện thử, bản firmware thật sự chạy và kết quả. Giữ nguyên log gốc. Ghi phân tích vào `notes.md` và liên kết hai mã lượt khi so sánh trước và sau. Cấu trúc dữ liệu và cách thu nằm trong [thiết kế ghi log](05-ghi-log-chay-thu.md).
- **Phân biệt quan sát, giả thuyết và nguyên nhân đã xác nhận.** Mô tả điều người thử thấy và điều telemetry cho biết. Ghi rõ đoạn mất log, phiên bản chưa xác nhận hoặc điểm chưa tái hiện được. Kết quả mô phỏng chỉ xác nhận hành vi trong mô phỏng.
- **Thay đổi đúng phần nghi ngờ.** Ưu tiên một thay đổi có mục đích rõ trong mỗi vòng thử. Ghi lý do và dấu hiệu dự kiến sẽ cải thiện. Nếu cần đổi nhiều phần cùng lúc, ghi lý do và cách xác định phần nào tạo ra khác biệt.
- **Giữ các đường bảo vệ khi chỉnh.** Kiểm tra lệnh dừng `x`, mẫu ToF/MPU cũ, vùng cấm và biên sân. Chỉ sửa quy tắc phanh hoặc né khi có phép thử làm rõ cả lúc được đi tiếp và lúc phải dừng.

## Một vòng cải tiến

| Bước | Việc cần làm |
| --- | --- |
| 1. Chọn ca và ghi mốc | Chọn hiện tượng cần tái hiện, bản firmware, chế độ `g` hay `j`, vị trí xuất phát và vị trí vật cản. Ghi trước dấu hiệu thành công, chẳng hạn xe qua hết cua sau T7 và tiếp tục chặng Đông. |
| 2. Chạy và thu log | Bật ghi trước lệnh chạy. Đánh dấu lúc quan sát thấy sự cố và lúc gửi `x` riêng. Sau lượt, người thử đưa mã lượt cho agent, kèm vị trí thực tế nếu biết. |
| 3. Phân tích | Đọc `run.json`, `events.jsonl`, `telemetry.csv` và `notes.md` trong đúng thư mục. Xem sự kiện đầu tiên làm xe lệch kế hoạch, rồi đối chiếu với ToF, góc, encoder và PWM nếu có. Nếu thiếu bằng chứng, ghi phép đo cần thêm hoặc tái hiện bằng mô phỏng. |
| 4. Sửa và kiểm tra trên PC | Ghi file, tham số hoặc nhánh được sửa và lý do. Chạy ca hồi quy liên quan, kiểm tra các ca đang đạt và build firmware. Ghi kết quả, kể cả phương án đã thử nhưng thất bại. |
| 5. Chạy đối chứng trên xe | Xác minh bản đã nạp rồi tạo lượt mới với điều kiện tương đương. So kết quả và dữ liệu với lượt trước. Ghi riêng kết quả xe báo và kết quả người thử thấy trên sân. |
| 6. Chốt và bàn giao | Cập nhật `notes.md` và [nhật ký lỗi](04-nhat-ky-loi.md) với mã lượt, nguyên nhân, cách sửa và mức kiểm chứng. Cập nhật [ngữ cảnh phiên](03-ngu-canh.md) với việc còn mở và phép thử tiếp theo. |

## Khi nào coi là đạt

- **Một lỗi đã kiểm chứng** khi chạy lại ở môi trường từng gặp lỗi, có log đủ để đối chiếu và đạt tiêu chí đã ghi trước. Ví dụ, bản sửa ca T7 cần qua được cua sau T7 trên sân thật. Một lượt mô phỏng qua cua có thể dùng làm kết quả kiểm tra trước khi nạp.
- **Bộ điều khiển sẵn sàng để tối ưu thời gian** khi cùng một bản firmware về đích sạch qua nhiều lượt trên sân có 17 vật cản tĩnh và đã kiểm tra riêng cả bốn vị trí bốc thăm chính thức. Ghi hạng mục về đích, thời gian, va chạm, lỗi vạch và số lần chỉnh xe cho mỗi kịch bản.
- Nếu một ca chưa lặp lại được kết quả, giữ nó trong danh sách thử. Lượt sau cần nêu rõ đang kiểm tra điều gì và lấy thêm dữ liệu nào.
