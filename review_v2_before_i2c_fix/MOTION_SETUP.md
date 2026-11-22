# Robot: bản (2) tích hợp encoder và MPU

Mã trong thư mục đang mở ở IDE và thư mục `(2)` được đồng bộ từ cùng một bản:
phần motor/ToF của bản `(2)`, bổ sung lại điều khiển quãng đường và gyro.

## Lệnh chạy tay

- `w` / `W` / `F`: tiến 50 mm rồi dừng cả hai bánh.
- `s` / `B`: lùi 50 mm rồi dừng cả hai bánh.
- `a` / `A` / `L`: quay trái 90 độ theo MPU.
- `d` / `D` / `R`: quay phải 90 độ theo MPU.
- `q` / `e`: cua trái/phải 45 độ theo MPU.
- `u` / `p`: xoay tại chỗ trái/phải 45 độ theo MPU.
- `x` / `X` / dấu cách / `S`: hủy thao tác, dừng xe.
- `+` / `-`: tăng/giảm tốc độ cơ bản, không reset bước đang chạy.
- `c`: dừng xe, dò và hiệu chuẩn lại MPU; giữ xe đứng yên khoảng 3 giây.
- `m`: bật/tắt chế độ tự động bám line và tránh vật cản của bản `(2)`.
- `k`: đảo mức nhận line đen của IR như bản `(2)`.
- `t`: bật/tắt dòng trạng thái; `h`: dừng xe và in menu.
- `i` / `o`: dừng rồi đảo quy ước chiều motor A/B.

Lệnh giống nhau nhận lặp trong lúc bước đang chạy không khởi động lại bước đó.
Lệnh khác thay thế thao tác hiện tại. `S` khi app nhả nút sẽ dừng sớm.

## Quãng đường 5 cm

Trong `Inc/motion.h`: đường kính bánh 34 mm, encoder 2 là 818 số đếm/vòng
(đã tính quadrature ×4 và hộp số). Hai vòng đo ổn định là 822 và 814 số đếm;
vòng đầu 1393 bị loại vì lệch nhiều. 50 mm tương ứng khoảng 383 số đếm.

Chỉ encoder 2 đã được đo. Điều khiển mới cấp PWM cùng chiều cho cả hai motor,
không dừng riêng từng bánh theo encoder 1 chưa xác minh. Đến 383 số đếm ở
encoder 2 thì dừng đồng thời cả hai. Nếu encoder 2 không tiến thêm ít nhất 2
số đếm trong 400 ms, dừng cả hai và báo `LOI_ENCODER2`; giới hạn cả bước 5 giây.
PWM cơ bản 420 giống bản `(2)`, chỉ giảm nhẹ gần đích (mốc bù 360 trong công thức).
Khoảng cách thực tế vẫn cần đối chiếu bằng thước do trượt bánh và quãng trôi.

## MPU và góc quay

Driver nhận MPU6050 (ID 0x68) và MPU6500 (ID 0x70), tự dò địa chỉ 0x68/0x69.
Bo đã đọc trước đây là MPU6500 ở 0x68. Cấu hình gyro ±1000 độ/giây, hệ số
32.8 LSB/(độ/giây); không dùng nhầm hệ số 131 của ±250 độ/giây.

Giữ MPU nằm ngang, +Z hướng lên, xe đứng yên trong lúc khởi tạo/hiệu chuẩn.
`Goc` là yaw tương đối từ lần hiệu chuẩn, cập nhật cả khi xe đứng yên.
`a`/`d` đặt đích lệch ±90 độ so với hướng hiện tại, không reset yaw mỗi lần.
Tốc độ góc được tăng/giảm liên tục bằng phản hồi gyro, tối đa 220 độ/giây;
PWM đổi có giới hạn để giảm giật. Phanh một lần gần đích, không đảo chiều kéo
qua lại và không chạy từng nhịp. Dung sai hoàn tất ±2 độ; ngoài dung sai báo
`GOC_LECH`, quá 8 giây báo `QUA_THOI_GIAN`. Mất MPU thì khóa quay, gửi `c` để thử lại.
Các pha xoay 90 độ sau lùi khẩn cấp trong chế độ tự động cũng dùng gyro.

## ToF và đặt điểm 0

Thứ tự dòng ToF: hông trái, chéo trái, giữa, chéo phải, hông phải.

| Vị trí | XSHUT | I2C 7-bit trong bản (2) | Tổng offset |
|---|---|---|---|
| Hông trái | PB0 | 0x30 | 0 mm |
| Chéo trái | PB14 | 0x32 | 55 mm |
| Giữa | PB13 | 0x34 | 30 mm |
| Chéo phải | PB12 | 0x36 | 85 mm |
| Hông phải | PB1 | 0x38 | 0 mm |

Ba offset phía đầu gồm offset cũ 25/0/25 cộng các mốc còn dư trên màn hình
30/30/60. Công thức là max(0, raw - offset), không ép sàn 15 mm ở ba ToF đầu.
Số đo lỗi từ 8000 trở lên được giữ nguyên và in `---`; không biến lỗi thành 0.
Đây là bù theo các mốc người dùng cung cấp, chưa xác minh độ chính xác toàn miền.

Chọn `1`/`2`/`3` tương ứng trái/giữa/phải rồi gửi `z` để dừng xe và đặt mẫu
thô hiện tại của riêng cảm biến đó thành điểm 0. Cần mẫu hợp lệ dưới 400 mm,
không quá 500 ms. Offset chỉnh bằng phím chỉ lưu trong RAM, reset sẽ dùng lại
55/30/85. `[`/`]` tăng/giảm offset phải 5 mm; `{`/`}` tăng/giảm offset trái 5 mm.

Bản tích hợp này giữ thuật toán bám line/tránh vật cản của bản `(2)` trong
chế độ `m`: giữa dưới 50 mm, chéo dưới 30 mm thì lùi; hai ToF hông chỉ dùng
kiểm tra đã qua vật cản. Lái tay `w`/`s` là bước 5 cm; tự động bám line vẫn
chạy liên tục như bản `(2)`. Không ghép lại bộ khóa lùi chẩn đoán từng gây
chặn lệnh thử motor. Phần điều khiển và khởi tạo motor giữ đúng bản `(2)` chạy tốt.

## Dòng trạng thái và kiểm tra

```text
Goc:12.3 do | ToF:HT=100,TL=120,G=200,TR=150,HP=90 mm | Lenh:RE-T90
```

Mỗi lượt chỉ gửi một ký tự UART và đọc lần lượt một ToF, tránh chặn lấy mẫu gyro.
IR vẫn được đọc để bám line nhưng không thêm trường vào dòng trạng thái.

`build.bat` chỉ build; `build_and_flash.bat` build rồi nạp; file nằm trong `build/Debug`.

```powershell
gcc -std=c11 -Wall -Wextra -Werror -ITests/fakes -IInc Tests/test_motion.c Src/motion.c -lm -o build/test_motion.exe
.\build\test_motion.exe
gcc -std=c11 -Wall -Wextra -Werror -IInc Tests/test_tof_offset.c -o build/test_tof_offset.exe
.\build\test_tof_offset.exe
```

Kiểm tra mô phỏng xác nhận quay trái/phải có quán tính, không đảo chiều,
điều khiển PWM mượt, tiến/lùi dừng đồng thời theo encoder 2, lỗi encoder/MPU,
và các mốc ToF về 0 mà vẫn giữ số đo lỗi. Cần thử lại góc/quãng đường thực tế.