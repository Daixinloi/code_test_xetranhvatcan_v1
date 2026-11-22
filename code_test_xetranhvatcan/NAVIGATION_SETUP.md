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
| Mốc chuẩn bị qua đích | Tâm ô (157,5; 12,5) cm trong Ô5, có thể đổi làn nếu bị chắn |
| Làn kết thúc theo tuyến tham chiếu | X≈130 cm, Y≈−5,5 cm; qua vạch theo hướng Nam với biên 1 cm |

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

- **`v` rồi `g`** chạy sân thật theo hướng từng chặng và dò bằng ToF.
- **`v` rồi `j`** thử đường chữ U trên sàn trống đã kiểm tra đủ diện tích
  180 × 120 cm và lối vào, không dùng chế độ này để thi tránh vật cản.
  Với điểm xuất phát chuẩn (42; −4,5) cm, `j` bám tuyến tham chiếu trên map;
  các cấu hình xuất phát khác vẫn dùng A*. Lệnh đang chạy có nhãn `TEST-*`,
  kết thúc là `TEST-DEN-DICH`.

Với `g`, vòng điều khiển tiến Bắc lên khoảng Y=73 cm, rẽ Đông tới
X≈132,5 cm rồi đi Nam về đích. Xe dùng hình học bản đồ để kiểm tra thân,
vùng cấm và giữ tâm cách viền ngoài 10 cm trong lúc dò. Lối vào và lối
qua vạch đích được kiểm tra riêng.
Xe dùng ToF để phát hiện vật cản mới, kể cả khi vị trí chưa có trên bản
đồ. Nếu một tia đo xuyên qua vị trí dự kiến của một vật cản di động
ở hai mẫu cách nhau ít nhất 30 ms, xe có thể dùng lối vừa xác nhận thông.
Khi T9 vẫn chắn lối thấp, xe chừa chỗ xoay trước góc T9. Sau T14 xe có
thể tiến thêm tới X=138 cm để lấy chỗ quay Nam.

Với điểm xuất phát chuẩn, hai mẫu ToF giữa xác nhận vật cản trước khi
xe thử né. Xe ưu tiên phía có khoảng đo rộng hơn. Khi cả hai mắt chéo
đều báo từ 15 cm, xe thử phía phải trước. Xe kiểm tra thân xe, viền sân
và vùng cấm dọc đường né trước khi đi vào cung. PID tiếp tục điều khiển
hai bánh trong cung và xe nối lại tuyến khi đã chỉnh về hướng chặng.
PWM tối thiểu khi tiến theo map là 300 trên thang 0–1000. Ở đoạn đầu
và vùng tiếp cận T9, xe kiểm tra cung tiến để đổi hướng khi còn khoảng
hở và giữ phanh nếu không có cung an toàn.

Nếu cung bị chặn, xe thử phía còn lại hoặc dò theo khoảng hở có thể đi
được. Ở điểm xuất phát chuẩn, `g` và `j` chỉ tiến khi né. Khi cả hai
phía đều thiếu khoảng hở, xe giữ phanh và tiếp tục kiểm tra ToF. Gửi
`x` để dừng lượt chạy.

Với điểm xuất phát chuẩn, tuyến `j` đi Đông trong dải Y≈68–88 cm, kết thúc
ở làn X≈130 cm rồi qua vạch tới Y≈−5,5 cm. Các cấu hình khác vẫn dùng A*
với mốc chặng Đông ưu tiên (137,5; 77,5) cm.
Nếu không đủ chỗ xoay tại mốc chặng, xe không tiếp tục tiến thẳng ra biên;
các bước dò dự phòng ở cấu hình chuẩn giới hạn đích bước ở Y≈101 cm. Khi
né vật cản chặng cuối, làn X có thể đổi nếu làn X≈130 cm bị chắn.

`j` xuất phát với bản đồ thử gồm 17 vật cản tĩnh và khối giữa.
Khi chạy theo tuyến tham chiếu, xe quan sát tới 29 cm ở đoạn đầu và
24,5 cm ở các đoạn sau. Một điểm được ghi khi hai mẫu riêng biệt của cùng một cảm biến
chiếu tới hai vị trí cách nhau tối đa 2,5 cm. Mỗi điểm ghi nhận trong ô con 2,5 cm và được
nới theo biên an toàn của thân xe. Hai hay nhiều điểm trong cùng ô lưới
5 cm được gộp thành vùng bao phủ các ô con đã ghi.

Mỗi lần xác nhận lại điểm xuất phát bằng `v`, xe bỏ các ô vật cản đo được
từ lượt trước và quét mới. Các ô cố định của sân vẫn được nạp từ cấu hình.

Trong cửa xuất phát, `g` chỉ thêm số đo trong 18 cm và nằm trong hành
lang ngang theo bề rộng thân xe cùng biên định vị. Các mắt chéo vẫn
được kiểm tra qua ngưỡng phanh gần ở cả hai bên.

Khi xe vào sân theo hướng Bắc, số đo của ToF trái chiếu lệch tối đa 2 cm
ra khỏi cạnh phải của T1 được ghép với vật T1 đã có trên bản đồ. Số đo đó
vẫn được kiểm tra qua ngưỡng dừng gần.

Trong sân, mắt giữa kích hoạt phanh khi số đo dưới 5 cm. Sau khi phanh,
xe chỉ đổi hướng nếu ToF và khoảng quét thân xe cho phép. Xe kiểm tra
mẫu ToF mới trước khi tiến tiếp.
Nếu vật cản mới nằm ngoài phần quét của đoạn vào, xe đi hết đoạn đó
rồi lập đường tiếp theo từ vị trí bên trong sân.

Trên sàn trống, ToF có thể trả 8190/8191 vì không có mặt phản xạ phù hợp.
Cả hai chế độ chữ U chấp nhận các mã này nếu cảm biến đã khởi tạo, giao
dịch thành công và mẫu còn mới trong 200 ms. Chỉ khoảng cách hợp lệ được
dùng để thêm vật cản. Mất kết nối hoặc mẫu cũ làm xe dừng. Dưới 5 cm, xe
tạm phanh và chờ mẫu mới. Khi đường thông, `g` tiếp tục sau khoảng 20 ms
ổn định, còn `j` dùng 150 ms. Xe tiếp tục tính đường hoặc
kiểm tra lại đoạn kế tiếp từ vị trí hiện tại. Xe lập lại đường nếu bản đồ
có vật cản mới. Phím `x` hủy lượt chạy ngay. Debugger lưu mắt ToF và khoảng
cách vừa kích hoạt bảo vệ trong `nav_diag.near_front_mask` và `near_front_mm`.
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
5. Gửi `v` để xác nhận điểm đầu, rồi gửi `g` để xe tiến vào sân theo
   hướng Bắc và bắt đầu tuyến chữ U.
6. Test 17 vật cản tĩnh trước. Khi đã ổn định mới thêm vật cản thứ 18, đặt từ
   trước lúc chạy và để đủ lối cho thân xe. Xe dùng ToF để dò phía có thể
   lách qua và kiểm tra hình học ở từng bước.
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
  thành **74 mm**. Khi chạy sân thật `g`, các đoạn thẳng gần trục sân
  dùng **65 mm** từ tâm xe, tương ứng khoảng **20 mm** từ mép thân xe khi
  xe thẳng hướng. Đoạn chéo, chỗ xoay, cửa xuất phát và đích dùng biên
  **74 mm**. Vùng cấm giữa chữ U cộng thêm **50 mm**.
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
Đường tính theo từng chặng. Số ô thay đổi khi cập nhật vật cản hoặc đổi mốc.

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
lang vào và đường tới đích. Đoạn vào tiến tối đa 50 mm/lần. Nếu hành lang
thẳng bị chắn trong chế độ `g`, xe thử hướng chéo qua cửa xuất phát trước.
Nếu cần, xe dịch ngang trong cửa rồi tiến vào. Chỉ bắt đầu hướng đã kiểm
tra cả đường quét của thân xe và đường nối vào chặng Bắc.
Lượt đi thẳng từ y=−45 mm tới hàng y=75 mm dài khoảng 120 mm; sau đó
căn x=200 tới x=225 mm một lần để đúng tâm ô đầu. Các đoạn chuyển động thực
đều được kiểm tra va chạm liên tục, không chỉ kiểm tra hai tâm ô.

Sau khi dừng **giữa ô** để lập lại đường, bỏ bước trở về tâm ô cũ, đi thẳng
tới ô kế trong đường mới. Việc trở về tâm cũ từng làm xe quay lại quá gần
vật cản; có kiểm tra hồi quy bằng mô phỏng 17+1 vật cản và tia 25°.

Góc tiến/lùi theo MPU có lọc rung, tích phân giới hạn và chống bão hòa; PWM
được tăng dần để giảm trượt. Các bước A* giữ hệ góc cố định của map, không
chốt lại góc lệch sau mỗi ô. Quãng đường dùng hệ số chạy thử 780 xung/vòng
trên Encoder 2, bánh 34 mm. Hai bánh dừng cùng lúc theo Encoder 2.

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
- `U_LOI_LOI_RA`: hành lang đi thẳng qua vạch đích bị chặn.
- `DA_DEN_DICH`: xe đã đi qua vạch theo quãng đường ước lượng từ encoder.

MPU chỉ sửa hướng; không biết cả hai bánh quay trơn mà xe không tịnh tiến
đủ, hoặc xe trượt ngang. Mã tích phân encoder/yaw để **ước lượng** vị trí,
không ép vị trí về tâm ô để che sai số. Để chạy thi đáng tin cậy cần đo sai
số tích lũy và thêm đối chiếu tường/mốc bằng ToF hoặc camera. Xe chưa có
ToF sau, không thể khẳng định vùng lùi luôn trống.

## Bản đồ và kiểm tra

Trang localhost phát lại lộ trình danh định từ bộ điều hướng firmware và
hiển thị dữ liệu góc, ToF, trạng thái từ HC‑06. Vị trí xe trên sân là mô phỏng.
Trang chỉ đọc COM, các nút điều khiển mô phỏng hoạt động trong trình duyệt.
Từ thư mục workspace, chạy:

```powershell
python code_test_xetranhvatcan/Tools/map_monitor/server.py --com COM16
```

Mở [localhost:8765](http://localhost:8765). Dùng `--no-com` để chạy mô phỏng
khi cần dành cổng Bluetooth cho ứng dụng khác. Nhấn Ctrl+C trong terminal
đã mở máy chủ để giải phóng cổng COM.

Khi ghi demo đẩy tay, thêm `--record code_test_xetranhvatcan/build/demo_encoder.csv`
với tên tệp mới. CSV ghi góc, ToF và hai bộ đếm encoder từ các khung có CRC
hợp lệ. Đặt xe ở mốc xuất phát và cho bánh lăn trong suốt lượt đo. Bộ đếm có
thể vòng qua giới hạn 16 bit và firmware đặt lại bộ đếm khi bắt đầu lệnh chạy
motor, nên lượt đo đẩy tay cần thực hiện khi xe đứng ở chế độ chờ.

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
