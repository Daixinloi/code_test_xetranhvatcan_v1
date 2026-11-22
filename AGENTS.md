# Hướng dẫn cho agent làm việc với dự án xe STM32

## Duy trì ngữ cảnh qua các phiên

1. **Bắt đầu mỗi phiên**, đọc hết [`docs/03-ngu-canh.md`](docs/03-ngu-canh.md) trước khi phân tích, sửa file, chạy thử hoặc đưa ra quyết định. Khi phiên mới chỉ nối tiếp một việc đang dở, vẫn đọc lại file này để lấy trạng thái đã bàn giao. Kiểm tra lại những dữ kiện dễ thay đổi như firmware đã nạp, COM và ST-LINK khi chúng liên quan đến việc đang làm.
2. **Trong phiên**, cập nhật file ngữ cảnh ngay khi có quyết định hoặc thay đổi quan trọng. Ghi quyết định và lý do ngắn gọn, file hoặc thông số liên quan, bằng chứng kiểm chứng và việc còn chưa rõ. Áp dụng cho thay đổi tuyến, PID, hệ số encoder, cách thử, bản firmware và kết quả chạy xe ảnh hưởng đến phiên sau.
3. **Trước khi kết thúc**, cập nhật mục **Bàn giao phiên gần nhất** trong cùng file trước khi gửi câu trả lời cuối. Ghi ngày, việc đã làm, điều đã kiểm chứng, việc còn dở và bước tiếp theo nếu có. Nếu không phát sinh thay đổi quan trọng, ghi một dòng tóm tắt phiên. Nếu chế độ phiên hoặc quyền ghi không cho phép cập nhật file, nêu phần cần bàn giao trong câu trả lời cuối.

Giữ ngữ cảnh đủ ngắn để phiên sau đọc một lượt. Cập nhật phần mô tả trạng thái hiện tại khi thông tin thay đổi. Giữ quyết định cũ có giá trị giải thích và gắn mốc hoặc nhãn lịch sử. Chỉ ghi dữ kiện hữu ích, vị trí log và kết luận có căn cứ. Giữ thông tin bí mật và log thô ở ngoài file ngữ cảnh.

## Ghi lỗi và cách sửa

- Trước khi phân tích log, xử lý lỗi xe hoặc thay đổi bộ điều khiển, **đọc và làm theo [quy trình cải thiện](docs/06-quy-trinh-cai-tien.md)**. Quy trình quy định cách thu từng lượt, so trước/sau, kiểm chứng trên xe thật và bàn giao kết quả.
- Khi phát hiện lỗi hoặc người dùng báo lỗi, thêm hoặc cập nhật mục tương ứng trong [nhật ký lỗi](docs/04-nhat-ky-loi.md) ngay cả khi chưa tái hiện được. Ghi ngày, môi trường, bước tái hiện, hành vi mong đợi và hành vi thực tế. Với xe thật, ghi bản firmware, lệnh, vị trí và dẫn đến log khi có.
- Ghi rõ nguyên nhân đã xác nhận hoặc giả thuyết đang kiểm tra. Khi thử cách sửa, cập nhật cùng mục với file hoặc tham số đã đổi, lý do, phép thử và kết quả. Giữ những thử nghiệm thất bại có ích cho việc khoanh vùng.
- Ghi trạng thái của lỗi, ví dụ đang chẩn đoán, đã sửa chờ thử hoặc đã kiểm chứng. Chỉ đánh dấu đã kiểm chứng khi có kết quả ở môi trường mà lỗi xuất hiện. Dẫn mục lỗi quan trọng vào phần bàn giao ngữ cảnh để phiên sau tìm được.

## Làm việc trong workspace

- Firmware đang phát triển nằm trong [`code_test_xetranhvatcan/`](code_test_xetranhvatcan/). Đối chiếu cây thư mục trong [README](README.md) trước khi sửa. Chỉ sửa các bản lưu `baseline_v2_original/`, `calibrated_v2/` và `review_*/` khi nhiệm vụ yêu cầu trực tiếp.
- Khi đổi hành vi liên quan đến cuộc thi, đọc [mô tả bài toán](docs/01-bai-toan.md) và [định hướng giải quyết](docs/02-dinh-huong-giai-quyet.md). Ghi rõ đâu là yêu cầu của đề, đâu là lựa chọn kỹ thuật hoặc quy trình thử. Đối chiếu tham số và hành vi với mã nguồn hiện tại hoặc firmware thật đang chạy.
- Tách kết quả theo môi trường kiểm chứng trong ghi chú kỹ thuật. Phân biệt kiểm thử trên PC, mô phỏng web, telemetry thật, build firmware và xác nhận đã nạp đúng bản. Khi ghi tọa độ hoặc khoảng cách, chỉ rõ đơn vị mm hay cm và nguồn đo.
- Với thay đổi firmware, chạy kiểm thử liên quan trong `Tests/` và build Debug từ thư mục dự án chính khi có toolchain. Với thay đổi web hoặc bộ ghi đường, kiểm tra chế độ offline và phần ghi/đọc dữ liệu liên quan. Báo chính xác bước đã chạy và kết quả trong phần bàn giao.
- Trước khi nạp hoặc cho xe chạy thật, xác nhận thao tác thuộc phạm vi yêu cầu, thiết bị và file firmware đúng, xe đã ở trạng thái phù hợp. Ưu tiên đọc trạng thái trước khi gửi lệnh chuyển động. Khi thay đổi thuật toán, kiểm tra đường dừng `x`, xử lý mẫu ToF/MPU cũ và ràng buộc biên sân.
