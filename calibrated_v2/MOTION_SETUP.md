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

Các lệnh tay tiến/lùi liên tiếp giữ chung yaw của bước đầu làm hướng chuẩn;
không nhận góc lệch cuối mỗi bước thành hướng mới. `x`/dừng chủ động, lệnh xoay,
đổi chế độ hoặc hiệu chuẩn lại MPU sẽ tạo hướng chuẩn mới. Bộ PID dùng góc
lệch (14 PWM/độ) và tốc độ gyro (1,5 PWM/(độ/giây)) để bù hai bánh. Bù tối đa
150 PWM hoặc 35% tốc độ đặt, thay đổi tối đa 900 PWM/giây; không đảo chiều bánh.
Tích phân có giới hạn 60 PWM và chống tích lũy lúc bão hòa; tốc độ gyro được lọc rung.
Lệnh tay `w/s` cấp ngay PWM đặt (mặc định 420, có giới hạn theo bước chạy) để
vượt lực cản khởi động; vẫn giữ hướng MPU và dừng ở 5 cm. Bước tự động A* dùng
ramp từ 300, tăng/giảm tối đa 700 PWM/giây. Khi mất mẫu gyro trên 100 ms hoặc bão
hòa, khóa yaw đến khi hiệu chuẩn lại; không dùng tiếp góc đã thiếu một đoạn quay.
MPU lỗi thì dừng bước chạy. Khi IR không yêu cầu đánh lái, tự động cũng giữ hướng;
sau mỗi lần IR đánh lái thì chốt lại hướng mới.

Chạy map giữ hướng chuẩn cố định qua các bước, thay vì nhận góc lệch cuối bước
làm hướng mới. Xem `NAVIGATION_SETUP.md` và `Inc/navigation_config.h`; A* đã tích
hợp và đã bật chạy thử: xe 9×9 cm, xuất phát (20;−4,5) cm hướng Bắc,
ToF chéo ±25°. Gửi `v` xác nhận điểm đầu rồi `g` chạy map; xem hướng dẫn để
kiểm tra khoảng lệch ngang ToF ước lượng và sai số vị trí trước khi chạy sân.

Giữ khoảng cách hoạt động cả khi lái tay: một trong ba ToF đầu dưới 50 mm sẽ hủy
thao tác đang chạy, phanh 40 ms, rồi lùi có giữ hướng MPU. Các mắt đã kích hoạt
đều đạt 70 mm thì dừng và chờ ổn định 120 ms, tránh giật quanh mốc 50 mm.
Mẫu dùng phải mới trong 200 ms; mất mẫu của mắt đã kích hoạt thì dừng, chỉ lùi
tiếp sau khi có mẫu hợp lệ ổn định. Lùi tối đa tổng cộng 2 giây hoặc khoảng
15 cm theo encoder 2; đạt giới hạn báo `GIOI_HAN_LUI`, chuyển về chế độ tay,
giữ dừng và khóa tự thử lùi lại. Lệnh chuyển động mới sẽ bật lại bộ giữ khoảng
cách và được xử lý bình thường; giới hạn lùi không giữ khóa giao diện lệnh.
Trạng thái là `LUI-KHAN:GIU-5CM`. Không tiếp tục bước 5 cm đã bị hủy; chế độ tay
chờ lệnh mới, chế độ tự động có thể tiếp tục tránh vật cản khi đã đủ khoảng cách.
Phím `x`/`S`/dấu cách khóa tự lùi đến lệnh chuyển động tiếp theo. Khởi động ban
đầu có bật giữ khoảng cách; hiệu chuẩn/đảo motor cũng khóa tự lùi đến lệnh mới.
Ngưỡng dựa trên số đo đã trừ offset; cần đối chiếu khoảng cách thật bằng thước.

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
chế độ `m`: cả ba ToF đầu dưới 50 mm thì lùi; hai ToF hông chỉ dùng
kiểm tra đã qua vật cản. Lái tay `w`/`s` là bước 5 cm; tự động bám line vẫn
chạy liên tục như bản `(2)`. Không ghép lại bộ khóa lùi chẩn đoán từng gây
chặn lệnh thử motor. Phần điều khiển và khởi tạo motor giữ đúng bản `(2)` chạy tốt.

## Dòng trạng thái và kiểm tra

```text
Goc:12.3 do | ToF:HT=100,TL=120,G=200,TR=150,HP=90 mm | Lenh:RE-T90
```

MPU được lấy mẫu mỗi 10 ms (100 Hz). ToF được hỏi mỗi 10 ms theo lịch
trái-giữa-phải-hông trái-trái-giữa-phải-hông phải, ưu tiên ba mắt phía đầu.
Thời gian chờ mỗi giao dịch ToF là 20 ms; mức 5 ms trước đây quá sát khi
software UART dùng ngắt. Mỗi lượt chỉ gửi một ký tự UART để tránh chặn gyro.
IR vẫn được đọc để bám line nhưng không thêm trường vào dòng trạng thái.

Khi ToF hoặc MPU báo lỗi giao dịch I2C, xe dừng và hiển thị `I2C_PHUC_HOI`.
Chương trình đưa SCL/SDA về GPIO open-drain, phát xung giải phóng bus và STOP,
khởi tạo lại I2C, cấp lại địa chỉ cho 5 ToF rồi hiệu chuẩn gyro khi đứng yên.
Offset ToF trong RAM được giữ. Không tự tiếp tục thao tác đang chạy trước lỗi.
Nếu đường SDA/SCL còn bị giữ thấp, báo `I2C_DAY_BI_GIU`, giữ dừng và thử lại
theo nhịp tối thiểu 2 giây; không tiếp tục hỏi dồn vào bus đang lỗi.
Phím `c` cũng yêu cầu phục hồi cả bus và hiệu chuẩn, thay vì chỉ thử lại MPU.

Khởi tạo ToF kiểm tra kết quả reference calibration và bật continuous; không
báo OK khi các bước này thất bại. Chờ reference tối đa 200 ms, thử lại tối đa
hai lần khi khởi động, trong lúc motor dừng và trước khi hiệu chuẩn MPU.
`control_diag` trong debugger lưu ký tự lệnh nhận được, số lệnh chấp nhận,
số lần bị I2C/giữ khoảng cách chặn và kết quả bước chạy gần nhất. Không thêm
trường vào dòng trạng thái trên app.

Nếu gyro biến động lúc khởi động, không khóa vĩnh viễn ở
`NOT_STILL_OR_BIAS`. Chương trình chuyển sang `MPU_GIU_YEN_2S`, thu lại 200
mẫu mỗi 10 ms khi toàn bộ đầu ra motor đã dừng. Có rung/biến động thì bắt đầu
lại cửa sổ; không nới ngưỡng phương sai. UART/ToF vẫn chạy trong lúc chờ.
Sau hiệu chuẩn thành công, đặt lại góc 0, hủy tham chiếu map cũ và giữ dừng;
phải gửi lệnh mới, không tự tiếp tục lệnh bị lỗi. Bench/auto đang cấp motor
không được coi là đứng yên dù Motion không có bước đang chạy.
`motion_diag.calibration_samples/calibration_restarts` cho biết tiến độ.

Lỗi mất mẫu trong lúc tự lùi chỉ chờ tối đa 1 giây rồi giữ dừng và giải phóng
quyền điều khiển, không khóa lệnh vĩnh viễn. `v` luôn hủy trạng thái tự lùi trước
khi xác nhận lại điểm đầu. Trạng thái map ghi rõ mắt lỗi, ví dụ `MAP_CHO_TOF_TL_TR`.
`g` chạy sân với điều kiện đủ ba số đo đầu. Sau `v`, phím `j` dành riêng cho
thử A* trên sàn trống đã kiểm tra: chấp nhận mẫu 8190/8191 còn mới, không giả
thành số đo xa; vẫn dừng nếu mất ACK/I2C/mẫu cũ hoặc có số đo gần dưới 5 cm.
Map gộp ô thẳng hàng tối đa 15 cm (`g`), tối đa 10 cm và tốc độ đặt 400 (`j`)
để giảm số lần dừng-khởi động. Lệnh tay `w/s` vẫn 5 cm. MPU sửa góc, không
phát hiện được mất quãng đường khi bánh quay trơn hoặc trượt ngang.

Kiểm tra riêng hai kênh motor: **nhấc cả hai bánh khỏi sàn**, gửi `b` thường.
Xe cấp PWM 500 bằng nhau cho A/B trong 700 ms rồi tự dừng, không bù MPU/ToF,
không tự lặp. `x` hoặc một lệnh chuyển động mới hủy phép thử ngay. Encoder
được xóa đầu phép thử để đọc số xung sau đó. Chỉ phép thử này bỏ qua phản hồi;
lái tay và A* vẫn giữ MPU/ToF. Driver `Src/motor.c` và `Inc/motor.h` đã khôi
phục nguyên byte từ ZIP `(2)` đã thử cả hai bánh. Mọi nhánh chạy dùng chung
`board_drive.c`: giữ trạng thái PB8/PB9 đi kèm lệnh motor như bản gốc; nhánh
đóng vòng trước đây trả về sớm nên từng bỏ qua hai chân này. Với board thực,
khôi phục các chân này đã được thử và cả hai bánh quay ngay. Chưa suy ra
vai trò điện vật lý của hai chân chỉ từ sơ đồ.
`board_drive_diag` giữ cặp lệnh và PB8/PB9 không bằng 0 gần nhất; không khẳng
định điện áp thật tại motor. `tof_diag[5]` phân biệt số đo hợp lệ, số đo không
hợp lệ, mẫu cũ, lỗi I2C và lỗi khởi tạo; số đo 8190/8191 không đổi thành 0.

Driver ToF lưu `dev[i].last_error` ngay tại giao dịch hỏng. Nếu đọc khoảng
cách thất bại, không xóa ngắt mẫu và không đưa dữ liệu dở dang vào điều khiển.
Trường hợp chỉ một cảm biến không khởi tạo được, các cảm biến còn đáp ứng vẫn
được phép stream. Biến `i2c_diag` lưu số lần thử/phục hồi, lỗi gần nhất và mask
ToF sẵn sàng; mask 0x1F sau phục hồi nghĩa là đủ 5 cảm biến. Có thể yêu cầu
phục hồi qua debugger bằng `i2c_diag.request=1`; xe dừng trước khi xử lý.

`build.bat` chỉ build; `build_and_flash.bat` build rồi nạp; file nằm trong `build/Debug`.

```powershell
gcc -std=c11 -Wall -Wextra -Werror -ITests/fakes -IInc Tests/test_motion.c Src/motion.c Src/board_drive.c Tests/fakes/board_gpio.c -lm -o build/test_motion.exe
.\build\test_motion.exe
gcc -std=c11 -Wall -Wextra -Werror -IInc Tests/test_tof_offset.c -o build/test_tof_offset.exe
.\build\test_tof_offset.exe
gcc -std=c11 -Wall -Wextra -Werror -ITests/fakes -IInc Tests/test_vl53l0x.c Src/vl53l0x.c -o build/test_vl53l0x.exe
.\build\test_vl53l0x.exe
```

Kiểm tra mô phỏng xác nhận quay trái/phải có quán tính, không đảo chiều,
điều khiển PWM mượt, tiến/lùi dừng đồng thời theo encoder 2, lỗi encoder/MPU,
và các mốc ToF về 0 mà vẫn giữ số đo lỗi. Kiểm tra driver còn xác nhận không
xóa ngắt khi đọc lỗi, giữ mã lỗi I2C để phục hồi và đọc lại được sau lỗi.
Cần thử lại góc/quãng đường thực tế.
