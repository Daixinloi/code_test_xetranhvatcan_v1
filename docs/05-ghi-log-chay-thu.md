# Thiết kế ghi log từng lượt chạy thật

Mỗi lần bấm chạy trên web tạo một thư mục mới. Máy tính nhận dữ liệu từ xe, ghi vào thư mục đó và hiển thị mã lượt chạy. Sau khi xe dừng, người dùng đưa mã cho agent để phân tích. Lượt sau ghi vào thư mục khác để so được kết quả trước và sau khi sửa.

## 1. Dữ liệu đi từ xe đến agent thế nào

```text
STM32 → HC-06 → cổng COM của máy tính → server.py → runs/<mã lượt>/
                                         └────→ web hiển thị trạng thái
```

Máy tính kết nối Bluetooth với HC-06. Người dùng có thể bấm **Bắt đầu lượt thử** trên trình duyệt điện thoại hoặc máy tính. Khi dùng điện thoại, điện thoại mở địa chỉ IP của máy tính qua cùng Wi-Fi. Máy tính gửi lệnh qua Bluetooth và ghi trực tiếp vào `runs/`. Agent đọc file trong workspace theo mã lượt sau khi người dùng yêu cầu. Nếu dùng app Bluetooth Serial kết nối thẳng từ điện thoại, dùng [cách xuất file thủ công](07-cach-gui-log-cho-ai.md).

1. Người dùng chọn **Sân thật** (`g`) hoặc **Sàn trống** (`j`) và kịch bản thử. Đặt xe vào mốc xuất phát trước khi bấm **Bắt đầu lượt thử**.
2. Máy chủ xác nhận xe đang gửi bản tin hợp lệ, còn mới và đang ở trạng thái cho phép bắt đầu. Máy chủ tạo thư mục, mở file ghi và lưu cấu hình lượt thử trước khi gửi `v`. Chờ xe báo sẵn sàng rồi gửi `g` hoặc `j`. Ghi riêng thời điểm gửi lệnh và thời điểm trạng thái của xe xác nhận lệnh đã có hiệu lực. Nếu không xác nhận được, báo lỗi và lưu phần đã ghi.
3. Trong lúc chạy, web hiện mã lượt, trạng thái xe, số bản tin đã thu và thời gian từ bản tin cuối. Khi mất kết nối, lưu đoạn đã nhận và đánh dấu bản ghi bị gián đoạn. Nếu nối lại trong lượt, ghi rõ khoảng trống. Chỉ gửi lệnh chạy lần nữa khi người dùng bắt đầu lượt mới.
4. Sau khi xe báo đích, báo lỗi kết thúc hoặc nhận lệnh `x`, ghi thêm trạng thái cuối rồi chốt lượt. Nếu xe đứng ở trạng thái chờ hoặc dò lối, giữ ghi cho đến khi người dùng dừng hoặc kết thúc lượt. Tách thời điểm thấy xe đứng khỏi thời điểm bấm `x` để giữ được nguyên nhân đứng ban đầu.

Hiện firmware đã hiểu `v`, `g`, `j`, `x` qua Bluetooth. Web hiện hỗ trợ ghi CSV cho cả thời gian máy chủ chạy bằng `--record` và chỉ nghe trên `127.0.0.1`. Để bấm từ điện thoại, cần bổ sung điều phối lượt, chuyển tiếp các lệnh tự lái qua COM và tùy chọn mở web trong mạng nội bộ. Chỉ cho phép thiết bị đã ghép với web gửi lệnh chạy hoặc dừng, đồng thời giữ chế độ localhost làm mặc định. Có thể làm bước này với định dạng firmware đang có.

## 2. Log cần chứa gì

| Dữ liệu | Dùng để trả lời câu hỏi nào | Nguồn |
| --- | --- | --- |
| Giờ máy tính nhận bản tin, trạng thái, góc MPU, ToF `TL/G/TR/HT/HP`, encoder `E1/E2` | Xe báo gì trước, trong và sau khi đổi hướng hoặc đứng? | Bản tin Bluetooth hiện tại |
| Lệnh đã gửi, trạng thái xác nhận, mất kết nối, số dòng bị loại | Xe đã bắt đầu chạy chưa? Khoảng mất dữ liệu dài bao lâu? | Máy chủ thu log |
| Bản firmware được dùng, chế độ `g`/`j`, mốc đặt xe và vị trí khối bổ sung | Hai lượt có thực sự cùng điều kiện để so không? | Thông tin lượt thử. Ghi rõ phiên bản nếu đã xác minh, để trống nếu chưa biết |
| Tick trên xe, số thứ tự bản tin, trạng thái trước/sau và lý do dừng | Vòng điều khiển nào ra quyết định dừng? Có thiếu bản tin không? | Phần bổ sung cho firmware |
| Tọa độ `(x,y)` xe ước tính, góc mục tiêu, PWM hai bánh và kết quả chuyển động | Xe phanh vì ToF, còn cấp PWM nhưng bánh không tiến, hay cung kế tiếp bị từ chối? | Phần bổ sung cho firmware hoặc trace ST-LINK khi có |
| Mốc thực tế, vị trí dừng quan sát được, va chạm, xe qua hết vạch đích hay chưa | Dữ liệu xe tự báo khớp sân thật đến đâu? | Người thử hoặc video |

Trên màn hình chỉ cần hiện kết nối, thời gian nhận dòng mới nhất, trạng thái, góc, ba ToF đầu và lý do dừng nếu có. Giữ dữ liệu chi tiết trong thư mục để phân tích sau. Ưu tiên gửi bản tin chi tiết **khi chuyển trạng thái**, do đường Bluetooth đang dùng 9600 baud và bản tin định kỳ chỉ được lên lịch mỗi 250 ms. Giữ trace RAM làm nguồn bổ sung khi Bluetooth hụt đúng thời điểm. Chỉ dùng phần trace từ firmware và ELF cùng bản, và đọc trước khi reset hoặc bắt đầu lượt mới.

## 3. Lưu một thư mục cho mỗi lượt

```text
runs/
└── 2026-11-22_093015-001_san-that/
    ├── run.json         Chế độ, kịch bản, firmware, thời gian và kết quả lượt
    ├── telemetry.csv    Các bản tin hợp lệ nhận được từ xe
    ├── events.jsonl     Lệnh, xác nhận, thay đổi trạng thái và gián đoạn kết nối
    └── notes.md         Quan sát trên sân, phân tích và thay đổi cho lượt kế tiếp
```

Tên thư mục dùng giờ địa phương và mã phân biệt để các lượt không ghi đè nhau. `run.json` lưu trạng thái thu log và kết quả chạy riêng. Ví dụ, xe có thể về đích nhưng log bị mất giữa đường. Ghi riêng kết quả xe báo và kết quả người thử xác nhận trên sân. Nếu có trace ST-LINK hoặc video, dẫn đường dẫn trong `notes.md`. Giữ nguyên dữ liệu đo cũ sau khi chốt lượt và bổ sung phân tích vào ghi chú.

## 4. Kiểm tra bộ thu log

Hai lần bấm chạy tạo hai thư mục khác nhau. Mỗi lượt có log từ trước lệnh chạy tới lúc dừng, kèm trạng thái thu dữ liệu. Thử thêm mất COM, xe không xác nhận lệnh và người dùng bấm dừng. Sau đó mới bổ sung và kiểm tra bản tin chi tiết từ STM32.

Cách dùng dữ liệu để khoanh vùng lỗi, sửa và chạy đối chứng nằm trong [quy trình cải thiện bộ điều khiển](06-quy-trinh-cai-tien.md).
