# Cách gửi log một lượt chạy cho AI

**Phương án tự động đang được thiết kế.** Điện thoại mở web máy tính qua Wi-Fi và bấm **Bắt đầu lượt thử**. Máy tính nối Bluetooth với xe và tự lưu log vào `runs/`. Trong lúc chờ triển khai, dùng cách dưới đây.

## Thu log trên điện thoại

1. Kết nối điện thoại với **HC-06** bằng app Bluetooth Serial. Bật ghi log trước khi chạy, hoặc xóa riêng màn hình terminal để dễ sao chép đúng lượt. Chờ thấy xe gửi các dòng có `Goc`, `ToF`, `Lenh` và `E2`.
2. Đặt xe ở vạch xuất phát. Gửi `v` rồi `g` để thử sân thật, hoặc `v` rồi `j` để thử sàn trống. Nếu xe đứng giữa đường, ghi lại nó đang ở đâu và giữ các dòng log ngay trước và sau lúc đứng. Gửi `x` khi cần dừng.
3. Xuất log từ app. Nếu app không có nút xuất, sao chép toàn bộ nội dung lượt đó thành `log.txt`. Chép file vào một thư mục riêng trong workspace, ví dụ `runs/t7-luot-01/log.txt`. Mỗi lượt dùng một thư mục mới.

## Đưa log cho AI

Mở Codex tại gốc workspace và gửi đường dẫn kèm điều bạn thấy trên sân. Nếu AI đang ở trên máy khác, đính kèm file log vào chat.

> Đọc `runs/t7-luot-01/log.txt`. Mình chạy `v` rồi `g`. Xe đi ổn tới T7, cua được một đoạn thì đứng, khoảng giây thứ 12. Bản firmware chưa xác nhận. Hãy chỉ ra dấu hiệu xuất hiện đầu tiên và ghi kết quả vào nhật ký lỗi.

Ghi tên firmware thật sự đã nạp nếu biết. Nếu chưa biết, ghi **chưa xác nhận**. Khi thử lại sau khi sửa, gửi cả đường dẫn lượt trước để AI so sánh.

Web hiện có thể ghi CSV khi **máy tính** giữ kết nối COM. Nút chạy tự động và tự tạo thư mục theo lượt còn ở giai đoạn [thiết kế](05-ghi-log-chay-thu.md). Khi chạy bằng điện thoại, gửi file xuất từ app để AI có dữ liệu của lượt đó.
