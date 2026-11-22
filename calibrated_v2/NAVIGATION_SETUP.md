# Chạy thử map A* với xe 9 × 9 cm

Đã bật cấu hình chạy thử có giám sát (`navigation_config.enabled=true`). Không
còn khóa `MAP_CAN_CAU_HINH` do thiếu kích thước/góc ToF. Chỉ chạy khi người dùng
đặt xe đúng vị trí rồi gửi `v`, `g`; bật nguồn không tự chạy A*.

Driver motor giữ nguyên `motor.c/h` từ ZIP `(2)` đã được thử cả hai bánh.
`board_drive.c` giữ PB8/PB9 đi kèm mọi lệnh tiến/lùi/xoay/A*/lùi giữ khoảng
cách, để tránh nhánh điều khiển mới bỏ sót trạng thái chân như trước. Việc
khôi phục các chân này đã được kiểm tra trên xe bằng lệnh `b`, hai bánh quay
ngay. Lệnh tay vẫn là bước 5 cm và xoay theo MPU, không trở thành chạy liên tục.
`tof_diag[5]` phân biệt lỗi khởi tạo/I2C/mẫu cũ với kết quả không hợp lệ;
8190/8191 không được dùng làm khoảng trống hoặc cự ly 0.

## Cấu hình hiện tại

| Thông số | Giá trị |
|---|---|
| Sân | 180 × 120 cm, ô 5 × 5 cm, 36 × 24 ô |
| T1–T17 | Mỗi vật cản 10 × 10 cm |
| Xe | Dài 9 cm, rộng 9 cm, tính cả bánh theo số người dùng cung cấp |
| Tâm xuất phát | x=20 cm, y=−4,5 cm; mũi xe chạm vạch y=0 |
| Hướng đầu xe | Bắc (+Y), 90° trong hệ tọa độ map |
| Mặt ba ToF đầu | Cách tâm 45 mm về trước |
| Góc trái/giữa/phải | +25° / 0° / −25°, theo người dùng xác nhận |
| Lệch ngang trái/giữa/phải | +25 / 0 / −25 mm, ước lượng theo ảnh; cần đo để hiệu chỉnh |
| Điểm đích mẫu | Tâm ô (157,5; 7,5) cm, nằm trong vùng Ô5 |

Gốc map ở góc dưới trái, +X sang phải, +Y lên trên. Các thông số nằm trong
`Inc/navigation_config.h`, đơn vị **mm**. `Goc` trên app là yaw tương đối lúc
hiệu chuẩn; khi xe hướng Bắc sau khởi động, `Goc` có thể gần 0°. `v` tạo phép
đổi từ yaw này sang hệ map Bắc=90°, không ép số đo trên app thành 90°.

Các góc và kích thước do người dùng cung cấp; khoảng lệch ngang ±25 mm là
ước lượng, không phải số đã đo bằng thước. Nếu tâm mặt đo thực tế lệch khác,
sửa `sensor_left_mm`. Nếu ToF lùi vào sau mép mũi, sửa `sensor_forward_mm`.
Không dùng góc ±45° của phiên bản cũ.

## Cách test

### Sàn trống và sân thật là hai chế độ khác nhau

- **`v` rồi `g`**: chạy sân thật, giữ yêu cầu ba ToF đầu có số đo hợp lệ.
- **`v` rồi `j`**: thử đường A* trên sàn trống đã kiểm tra đủ diện tích
  180 × 120 cm và lối vào, không dùng chế độ này để thi tránh vật cản.
  Lệnh đang chạy có nhãn `TEST-*`, kết thúc là `TEST-DEN-DICH`.

`j` khởi tạo lại bản đồ thử từ 17 vật cản tĩnh và khối giữa, bỏ các ô
đã ghi từ lần chạy trước. Trong chế độ này, cả số đo hợp lệ của đồ vật
trong phòng cũng không được thêm vào bản đồ ảo; chúng vẫn được dùng để
dừng/bảo vệ khi gần dưới 5 cm. `g` vẫn ghi vật cản mới và lập lại đường.

Trên sàn trống, ToF có thể trả 8190/8191 vì không có mặt phản xạ phù hợp.
Chỉ `j` cho phép tiếp tục với các mã này khi thiết bị đã khởi tạo, không
báo lỗi giao dịch và mẫu vẫn mới trong 200 ms. Không đổi chúng thành cự ly
xa giả, không dùng chúng để vẽ map và không khẳng định đã đo được đường
trống. Mất kết nối/mẫu cũ vẫn dừng; mẫu đo hợp lệ dưới 5 cm vẫn kích hoạt
bảo vệ. Tốc độ đặt test tối đa 400, đoạn thẳng tối đa 10 cm. `x` dừng.
Đổi giữa `g` và `j` khi đang chạy sẽ dừng và yêu cầu đặt/xác nhận lại điểm
đầu; không tự đổi chế độ giữa đường.

`v` hủy tự lùi đang chờ mẫu trước khi xác nhận điểm đầu. Nếu tự lùi mất
mẫu quá 1 giây thì giữ dừng, báo `TOF_DAU_MAT_MAU` và không giữ khóa giao
diện mãi. Đây là sửa lỗi trước đây khiến `v/g` bị bỏ qua im lặng.
Khi thiếu ToF, nhãn chỉ rõ kênh, ví dụ `MAP_CHO_TOF_TL_G_TR` hoặc
`MAP_LOI_TOF_TL_TR`; không ép xe chạy khi chế độ sân thật thiếu dữ liệu.

### Thứ tự thử

1. Đặt xe trên đúng sân, bật pin và để yên khoảng 5 giây để hiệu chuẩn MPU.
2. Để thoáng phía sau, kiểm tra riêng `w`, `s` mỗi lệnh khoảng 5 cm; `a`, `d`
   mỗi lệnh khoảng 90°. Dùng thước và đường kẻ kiểm tra sai số thật.
3. Lần lượt đưa vật trước TL, G, TR để kiểm tra đúng kênh ToF. Dưới 50 mm xe
   lùi giữ khoảng cách và dừng khi các mắt đã kích hoạt đạt khoảng 70 mm.
4. Sau test tay, **đặt xe lại đúng vị trí ban đầu**, tâm (20; −4,5) cm, hướng
   Bắc. Đừng chỉ đặt tâm ở y=0: vị trí đó đã tiến vào sân thêm 4,5 cm.
5. Gửi `v`: phải thấy `MAP_SAN_SANG`. Gửi `g`: bắt đầu A*. Đầu tiên xe tính
   đường, tiến vào sân theo hướng Bắc, căn tới tâm ô đầu rồi đi theo đường.
6. Test 17 vật cản tĩnh trước. Khi đã ổn định mới thêm vật cản thứ 18, đặt từ
   trước lúc chạy và để đủ lối cho thân xe. Khi nhìn thấy ô chiếm chỗ mới, xe
   dừng bước hiện tại, cập nhật map rồi tìm lại đường từ vị trí ước lượng đó.
7. `x` / `S` / dấu cách dừng và hủy chạy map. Muốn test lại, đặt xe về đúng điểm
   đầu rồi gửi `v`, `g`. `m` là thuật toán bám line cũ, không phải A*.

Không gửi `v` ở giữa sân để tiếp tục: lệnh này xác nhận xe ở điểm đầu, không
phải tự định vị. Giữ nguồn nếu muốn giữ các ô vật cản mới đã quan sát; reset
sẽ nạp lại bản đồ tĩnh, xóa các ô mới trong RAM.

Nếu khởi động lúc xe bị rung, trạng thái `MPU_GIU_YEN_2S` có nghĩa chip đã
được nhận nhưng đang đợi đủ mẫu gyro ổn định. Đặt xe yên để tự hiệu chuẩn
lại; quá trình này không chặn UART/ToF. Sau khi thành công xe vẫn dừng, hủy
tham chiếu góc/map cũ và chờ lệnh mới. Muốn chạy map phải đặt lại đúng vị trí
đầu và xác nhận `v`, không dùng lại tọa độ đã xác nhận trước lần đặt lại góc.

## Vì sao lối vào đã được mở

Bản trước cộng 5 cm quanh **toàn thân** xe, cộng thêm biên vị trí 2,5 cm,
cho bán kính cấm 13,9 cm và làm khóa lối vào T1. Yêu cầu người dùng là giữ
5 cm tại **ba ToF đầu**, nên hiện tách hai điều kiện:

- Thân xe khi xoay: ceil(sqrt(90²+90²)/2)=64 mm, cộng biên hình học 10 mm
  thành **74 mm**. Vật cản được mở rộng theo hình chữ nhật để chứa vòng xoay.
- Ba tia ToF đầu: dự báo khoảng 50 mm ở đúng góc +25/0/−25°. A* bỏ những
  cạnh mà thân đi lọt nhưng một tia đo sẽ báo quá gần vật cản đã biết.

Biên 10 mm là cấu hình chạy thử, **không phải khẳng định sai số vị trí dưới
10 mm**. Đo thực tế trên đúng sàn để điều chỉnh. Nếu xe trượt/encoder sai,
biên này có thể không đủ. Không tự giảm thêm biên khi báo hết đường.

Bộ giữ khoảng cách thực tế vẫn dùng ngưỡng **50 mm cho cả ba mắt đầu**, nhả
ở 70 mm. Hai ToF hông dùng xác nhận đã qua vật cản trong chế độ cũ, không
kích hoạt giữ khoảng cách đầu và không thay thế cảm biến phía sau.

## Thuật toán và các sửa lỗi

A* đi bốn hướng, heuristic Manhattan, tối ưu số bước ô trên đồ thị đã khóa
vật cản/thân xe/tia đầu. Vì điều kiện đầu xe phụ thuộc hướng di chuyển, một
cạnh có thể hợp lệ theo một chiều nhưng không hợp lệ theo chiều ngược lại.
Chưa tối ưu tổng thời gian xoay. Tìm đường dự kiến khoảng **60 ô**, chưa tính
đoạn vào sân và căn tâm; số ô thay đổi khi cập nhật vật cản.

Có cache kiểm tra tia theo bốn hướng. Mỗi lượt chỉ mở rộng tối đa một nút
khi bật mô hình ToF, để nhường thời gian lấy mẫu MPU. Khi thêm ô mới, cache
bị xóa và đường được tìm lại. Mẫu ToF phải mới trong 200 ms; `---` không
được coi là đường trống.

Trên STM32 8 MHz, khối tìm đường dùng `-O2` kể cả preset Debug. Việc kiểm
tra vật cản mới duyệt danh sách các ô thực sự đã quan sát, không quét đủ
864 ô cho từng tia. Như vậy không bỏ thêm dữ liệu và không đổi kết quả A*,
nhưng giảm khoảng chặn lấy mẫu MPU. `nav_diag.planner_last_ms/planner_max_ms`
ghi thời gian mỗi lượt tính; `motion_diag.last_fault/last_fault_tick` và
`max_sample_gap_ms` giữ nguyên bằng chứng lỗi dù MPU tự hiệu chuẩn lại.
Không nới ngưỡng mất mẫu gyro 100 ms để che việc tính đường quá chậm.

Vị trí vật cản từ mẫu đo được tính từ **tâm mặt ToF**, không từ tâm xe:

`điểm chạm = tâm xe + vị trí lắp đã xoay theo yaw + tia đo ở góc yaw±25°`

Chỉ dùng các điểm trong 300 mm để bổ sung map. Điểm nằm trong hoặc sát 10 mm
với vật cản tĩnh không vẽ thêm ô mới, tránh lỗi làm tròn tại mặt vật cản gây
nở map giả. Các ô quan sát mới được giữ lại; không xóa chỉ vì lần sau không
thấy. Chưa biết toàn bộ kích thước vật cản mới từ một tia; bổ sung dần các
mặt khi quan sát được. Không ép quay lại đường cũ: tìm A* mới tới đích.

Tâm xuất phát âm được lưu bằng kiểu có dấu. Trước khi chạy, kiểm tra cả hành
lang vào và đường tới đích. Đoạn vào tiến tối đa 50 mm/lần, giữ hướng Bắc,
không xoay ngoài sân. Từ y=−45 mm tới hàng y=75 mm dài khoảng 120 mm; sau đó
căn x=200 tới x=225 mm một lần để đúng tâm ô đầu. Các đoạn chuyển động thực
đều được kiểm tra va chạm liên tục, không chỉ kiểm tra hai tâm ô.

Sau khi dừng **giữa ô** để lập lại đường, bỏ bước trở về tâm ô cũ, đi thẳng
tới ô kế trong đường mới. Việc trở về tâm cũ từng làm xe quay lại quá gần
vật cản; có kiểm tra hồi quy bằng mô phỏng 17+1 vật cản và tia 25°.

Góc tiến/lùi theo MPU có lọc rung, tích phân giới hạn và chống bão hòa; PWM
được tăng dần để giảm trượt. Các bước A* giữ hệ góc cố định của map, không
chốt lại góc lệch sau mỗi ô. Quãng đường dùng encoder 2 đã đo 818 xung/vòng,
bánh 34 mm. Không dừng riêng bánh theo encoder 1 chưa hiệu chỉnh.

Lệnh tay `w/s` liên tiếp giữ cùng góc chuẩn ban đầu qua các bước, kể cả đổi
tiến sang lùi; không chốt lại góc lệch sau mỗi ô. Gửi `x`/lệnh xoay/đổi chế
độ/hiệu chuẩn lại sẽ bắt đầu một hướng chuẩn mới. Khi thử đường thẳng 50 cm,
gửi từng lệnh `w` sau khi bước trước đã dừng, 10 lần; không gửi `x` giữa các
bước nếu muốn giữ cùng hướng. Quãng đường thực vẫn cần đo bằng thước.

A* vẫn dùng ô 5 cm nhưng các ô thẳng hàng được gộp thành một lượt di chuyển
tối đa 15 cm ở `g`, 10 cm ở `j`, để giảm số lần dừng-khởi động gây trượt.
Không gộp qua góc cua; MPU, ToF và kiểm tra vật cản vẫn chạy liên tục trong
đoạn gộp, có vật cản mới thì dừng và lập lại đường từ vị trí ước lượng hiện
tại. `nav_diag.path_index` là ô đích của đoạn đang chạy, không phải số ô đã
đi qua; tọa độ ước lượng cập nhật trong lúc chạy.

Lệnh tay `w/s` khôi phục lực khởi động tức thời ở PWM đặt (mặc định 420), để
nhấn nút ngắn vẫn khởi động được hai bánh. Ramp từ 300 chỉ áp dụng cho bước
tự động. Giữ hướng MPU, encoder 5 cm và ngưỡng ToF 5 cm vẫn hoạt động khi lái tay.

## Khi xe dừng

- `MAP_CHO_TOF`: thiếu mẫu hợp lệ của một trong ba mắt đầu; sau 2 giây không
  hồi phục báo `MAP_LOI_TOF`. Kiểm tra dòng TL/G/TR trước khi gửi chạy lại.
- `MAP_KHONG_CO_DUONG`: ô hiện tại/đích bị khóa, hết đường, hoặc đoạn từ vị
  trí thực tới ô kế không đủ khoảng hở. Dừng, không tự lùi mù.
- `MAP_LOI_VAO_BI_CHAN`: hành lang tiến qua vạch có vật cản/không đủ chỗ.
- `MAP_LOI_MPU`, `MAP_LOI_CHUYEN_DONG`: dừng vì mất hướng/encoder/timeout.
- Bộ lùi giữ 5 cm can thiệp sẽ hủy xác nhận vị trí map, không tự tiếp tục từ
  tọa độ trước khi lùi. Cần đặt lại điểm đầu hoặc thêm cơ chế định vị thực.
- `DA_DEN_DICH`: đến ô đích mẫu trong Ô5; chưa có pha chạy hết thân xe ra
  ngoài vạch đích. Nếu luật yêu cầu vượt hẳn vạch, cần cấu hình pha kết thúc.

MPU chỉ sửa hướng; không biết cả hai bánh quay trơn mà xe không tịnh tiến
đủ, hoặc xe trượt ngang. Mã tích phân encoder/yaw để **ước lượng** vị trí,
không ép vị trí về tâm ô để che sai số. Để chạy thi đáng tin cậy cần đo sai
số tích lũy và thêm đối chiếu tường/mốc bằng ToF hoặc camera. Xe chưa có
ToF sau, không thể khẳng định vùng lùi luôn trống.

## Bản đồ và kiểm tra

`build/map_review.svg` vẽ hình chữ nhật từ đúng `Src/grid_map.c`, đường từ
host A*, xe tại vị trí xuất phát và ba tia 5 cm. Tọa độ các vật cản được đọc
từ ảnh sân; đối chiếu lại bằng thước trước khi chạy thật.

Để tạo lại:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -IInc Tools/check_navigation_config.c Src/grid_map.c -lm -o build/check_navigation_config.exe
.\build\check_navigation_config.exe
python Tools/render_map.py
powershell -NoProfile -ExecutionPolicy Bypass -File Tests/run_checks.ps1
```

Các kiểm tra gồm A* so với BFS, va chạm đoạn thực/căn tâm ô, cache tia mới,
góc ±25° và vị trí mặt đo, đường 17 vật cản với tia mô phỏng, thêm một vật
cản 10×10 cm chưa có trong map vẫn lập lại đường và tới đích, lỗi ToF/MPU/
encoder, mất mẫu góc, dừng, PWM giữ hướng và driver ToF. Mô phỏng dùng mô hình
động cơ/cảm biến lý tưởng hóa, không thay thế chạy trên xe thật.

`nav_diag` trong debugger cung cấp tọa độ ước lượng, góc map, số lần lập lại
đường, số ô mới và tiến độ. App vẫn chỉ in góc, 5 ToF và lệnh đang thực hiện.
