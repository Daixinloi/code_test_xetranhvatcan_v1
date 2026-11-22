# Bài toán xe dò mê cung

Nguồn yêu cầu là **Luật thi xe dò mê cung**, môn Nhập môn Hệ thống nhúng, bản cập nhật **12/10/2026**, trong file `Luat thi Nhung.pdf` được cung cấp khi bàn giao. Các số hiệu điều bên dưới theo tài liệu đó.

## Mục tiêu và sân thi

Xe phải tự chạy hết đường chữ U từ vạch xuất phát đến đích, tránh vật cản và đi trong phần sân hợp lệ. Chỉ về đích khi **cả xe** đã qua vạch đích. Ưu tiên của đề là về đích và vượt vật cản, sau đó giảm lỗi, rồi mới tối ưu thời gian.

- Sân rộng **180 × 120 cm**, dùng gạch 60 × 60 cm. Năm ô thi theo thứ tự là ô 1 ở nhánh vào, ô 2 góc trái, ô 3 đoạn ngang, ô 4 góc phải và ô 5 ở nhánh về đích. Vùng giữa chữ U là vùng cấm. Băng dính đen dán sát ngoài mép làn ở biên ngoài và mép vùng cấm. Đặt cả xe sau vạch xuất phát.
- Có **17 vật cản tĩnh** bằng xốp, kích thước **10 × 10 × 10 cm**, giữ đúng vị trí trên sơ đồ chính thức trong cả hai lượt. Phần mềm dùng tên T1–T17 để đối chiếu với map.
- Lượt tự động có thêm **một vật cản ngẫu nhiên**. Đại diện nhóm bốc một trong bốn vị trí đã đánh dấu ở ô góc phải và ô đích, sau khi xe đã được khóa. Thầy đặt khối vào vị trí vừa bốc và trả thăm vào hộp. Luật xác nhận cả bốn vị trí đều còn lối đi. Bản PDF xác định khu vực và số vị trí, chưa ghi tọa độ của bốn dấu.

## Tổ chức hai lượt — Điều 1, 2 và 8

| Lượt | Thể thức |
| --- | --- |
| 1 — Quét map | Tối đa 3 phút cho mỗi nhóm, chỉ có vật tĩnh. Được lái tay, chạy nhiều lần và va chạm không bị phạt. Lượt này không tính điểm. Thời gian dư được cộng vào thời gian tự chạy của lượt 2. |
| 2 — Thi | Khung chuẩn 5 phút gồm 30 giây đặt và khóa xe, bốc thăm, 4 phút chạy tự động và 30 giây chốt điểm, dọn sân. Thời gian tự chạy được cộng phần dư của lượt 1. Chỉ tính điểm lượt này. |

- Có 17 nhóm. Thứ tự được bốc trước buổi thi và dùng cho cả hai lượt. Lịch chuẩn tối đa 2 giờ 16 phút. Có thể sửa xe và code trước khi đến lượt.
- Trong lượt 2, khóa xe sau khi đặt sau vạch xuất phát. Từ lúc khóa xe, cấm nạp code. Khi có hiệu lệnh, nhóm **bấm nút trên xe** để bắt đầu. Việc chạm xe ngoài lần bấm khởi động được tính theo Điều 5. Đồng hồ chạy đến khi cả xe về đích, hết giờ hoặc nhóm xin dừng.
- Xe tối đa **15 × 10 cm**, được đo bằng khung ở lượt 1. Mục điều khiển không dây của luật yêu cầu sử dụng **SPI, I2C hoặc UART**. Lượt 2 được quay video để xem lại khi có tranh chấp và thầy quyết định cuối cùng.

## Vượt vật cản, phạm lỗi và chỉnh xe — Điều 3 đến 5

- **Vượt theo vạch chia phần.** Ô thẳng 1, 3, 5 chia 6 phần bằng vạch vuông góc với chiều chạy. Ô góc 2, 4 chia 3 phần bằng vạch chéo từ góc trong chữ U đến vạch biên. Vạch chia phần được kẻ mờ hoặc dùng băng màu sáng. Mỗi khối nằm gọn trong một phần. Cả xe qua vạch cuối của phần trong đường hợp lệ thì được tính các khối thuộc phần đó, gồm khối ngẫu nhiên nếu có.
- **Điểm của từng khối.** Một khối chỉ được cộng điểm một lần và chỉ chịu phạt xô dịch một lần. Chạm nhẹ mà khối vẫn đúng dấu thì vẫn có thể được tính. Xô lệch hoặc làm đổ khối bị trừ điểm và mất điểm của chính khối đó. Đi tắt qua giữa chữ U hoặc vòng hẳn ngoài map để vượt khối làm mất điểm vượt tương ứng.
- **Lỗi vạch đen tính theo bánh.** Chạm, đè vạch hoặc lấn một phần bánh qua vạch được phép. Mỗi lần có cả một bánh nằm hẳn ngoài vạch, không còn phần nào chạm vạch, bị trừ 5 điểm. Phần thân xe nhô ra không bị tính lỗi này. Bánh tự lựa và bi đỡ cũng được tính là bánh.
- **Chỉnh xe khi kẹt, lạc hoặc ra map.** Ngoài lần bấm khởi động, mỗi lần chạm tay vào xe tính là một lần chỉnh. Khi cả xe ra khỏi map, kể cả đi tắt qua vùng cấm, phải chỉnh xe. Đặt lại ngay sau vạch chia phần cuối cùng mà cả xe đã qua, đuôi sát vạch, đầu theo chiều đua. Nhóm chọn vị trí ngang trong làn miễn là xe không chạm khối. Nếu chưa qua vạch chia phần nào, đặt sau vạch xuất phát.
- **Tiếp tục sau khi chỉnh.** Bấm nút trên xe để chạy tiếp. Được reset, cấm nạp code. Đồng hồ vẫn chạy, điểm đã có giữ nguyên và không giới hạn số lần chỉnh. Các phần phía trước chỉ có điểm khi cả xe qua vạch cuối của phần tương ứng.

## Điểm và xếp hạng — Điều 3, 6 và 7

| Hạng mục lượt 2 | Điểm |
| --- | --- |
| Vượt một khối tĩnh | +10 |
| Vượt khối ngẫu nhiên | +15 |
| Cả xe qua vạch đích | +40 |
| Tốc độ, chỉ khi về đích | `max(0, (240 − t) / 6)` với `t` tính bằng giây |
| Chạy sạch, về đích với 0 lỗi và 0 lần chỉnh | +10 |
| Một bánh ra hẳn ngoài vạch đen | −5 mỗi lần |
| Xô lệch hoặc làm đổ vật cản | −5 và mất điểm vượt khối đó |
| Chỉnh xe | −10 mỗi lần |

Điểm vượt vật tối đa là `17 × 10 + 15 = 185`. Tổng điểm thấp nhất là 0. Xe về đích sạch trong 78 giây với đủ 18 khối được `185 + 40 + 27 + 10 = 262`. Xếp theo tổng điểm lượt 2. Nếu bằng điểm, ưu tiên ít lỗi hơn, sau đó về đích nhanh hơn. Nếu vẫn bằng thì đồng hạng. Hạng 1–3 được A+, hạng 4–6 được A, hạng 7–12 được B+ và hạng 13–17 được B.
