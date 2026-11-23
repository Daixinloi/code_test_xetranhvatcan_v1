# 2026-11-22 — Nạp xong nhưng không thấy UART và xe không chạy

## Điều kiện và quan sát
- Người dùng báo sau khi nạp HEX Release `C96F0A770F499655402A908E79BBC97DEDF1DE8BADC1899922931D38CE9DF2AF`: kết nối không in dữ liệu và xe không chạy.
- ST-LINK SN `37FF71064E573436E9171343`, STM32F103 medium-density, 3.27 V theo probe, Hot Plug 100 kHz. Không gửi lệnh chuyển động.
- Đọc RAM qua ELF cùng build: `mpu_diag.stage=1`, `ack_mask=0`, `hal_error=0x20` (timeout), `motion_diag.state=1` (MPU chưa khởi tạo). Năm `tof_diag` ở state 5 / không có mẫu hợp lệ. `i2c_diag` chưa ghi lỗi/recovery.
- I2C2: PB10/SCL thấp, PB11/SDA cao; SR2 BUSY=1, SR1 có cờ STOPF/BERR (`0x110`). Bằng chứng phù hợp với bus I2C bị giữ lỗi; chưa xác định linh kiện/dây nào kéo SCL.
- TIM1 CCR1/CCR2 đọc 0 sau reset.
- UART diagnostics: `auto_stream=true`; `control_diag.rx_count=2`, hai byte gần nhất `FC FF`, `movement_accepted=0`. GPIOA config PA12 output (TX), PA11 input (RX); IDR PA12/PA11 đang HIGH khi lấy mẫu. Chưa thu được waveform hay log Bluetooth. Firmware phát status 9600 baud qua PA12, nhận lệnh ở PA11.

## Kết luận hiện tại
- Nguyên nhân xe không chạy đã khoanh vùng: MPU không ACK do I2C chưa hoạt động; lệnh điều khiển bị khóa. SCL thấp/BUSY cho thấy lỗi bus hoặc phần cứng, chưa xác nhận chính xác nguyên nhân gốc.
- Mất dòng serial là vấn đề riêng/chưa xác nhận. Stream bật trong RAM nhưng hai byte nhận được là nhiễu; cần kiểm tra dây TX/RX, GND chung, baud 9600 và đo tín hiệu PA12 trước khi sửa firmware.

## Bước tiếp
1. Giữ xe đứng yên, tắt nguồn trước khi rút/cắm cảm biến. Kiểm tra PB10=SCL, PB11=SDA, GND chung và xem SCL có được nhả cao khi tháo từng nhánh cảm biến I2C.
2. Sau khi SCL/SDA đều idle-high, bật lại và đọc `mpu_diag`/`tof_diag` qua Hot Plug; không thử chạy nếu MPU chưa READY.
3. Kiểm tra HC-06: STM32 PA12(TX) → RX module, PA11(RX) ← TX module, baud 9600, GND chung; đối chiếu bằng logic analyzer/USB-UART nếu vẫn im lặng.
4. Giữ nguyên HEX hiện tại cho đến khi tách được vấn đề bus I2C và đường Bluetooth; chưa có firmware sửa trong lượt này.

## Sửa firmware và kiểm chứng sau nạp
- Phân tích mã xác nhận startup chỉ gọi `Motion_Init()` một lần. Khi MPU không ACK (stage 1), `mpu_ok=false`; phục hồi chỉ được yêu cầu nếu lỗi đọc runtime chuyển `mpu_diag.stage` sang 5. Trường hợp boot không ACK bị khóa đến lần khởi động lại.
- Sửa `Src/main.c`: sau `Run_Full_Initialization()`, nếu `mpu_ok` sai thì gọi `Request_I2C_Recovery(HAL_I2C_GetError(&hi2c2))`. Cơ chế cũ thử phục hồi I2C định kỳ và vẫn giữ động cơ dừng nếu MPU chưa READY.
- `Tests/run_checks.ps1`: 9/9 nhóm PASS. Release build: FLASH 65.312/65.536 B, RAM 17.616/20.480 B. HEX SHA-256 `131218ECF927D1ABB10F2564FF5EB55A135BBCA1E0DEA793B1FEE46205795746`, 183.793 B.
- Theo quyền nạp trước đó, đã nạp qua SWD Under Reset 100 kHz; 3,27 V theo probe. `Download verified successfully`, software reset. Không gửi lệnh chuyển động.
- Sau 5 giây, SWD Hot Plug cho thấy MPU READY: `stage=4`, address `0x68`, WHO_AM_I `0x70`, `ack_mask=1`, `hal_error=0`; cả 5 ToF `state=1`, mỗi cảm biến 146 mẫu hợp lệ. PB10/PB11 trở lại mức cao; TIM1 CCR1/CCR2 = 0. Bằng chứng cho thấy nhánh phục hồi trong firmware đã đưa cảm biến lên READY ở lượt này; chưa tách được liệu transient do startup hay do bus.
- `auto_stream=true`; sau reset `control_diag.rx_count=0`. Không có bằng chứng từ app/waveform rằng điện thoại nhận được status; UART là PA12 TX / PA11 RX, 9600 baud. Việc mất chữ chưa được xác nhận đã sửa.
- Chưa chạy xe và chưa gửi lệnh. Tiếp theo: xem app có nhận status sau reset; nếu không, đo PA12/HC-06 RX và kiểm tra UART 9600/GND. Chỉ thử chuyển động khi người dùng đặt xe trên vùng trống và yêu cầu.

## Nạp lại theo yêu cầu người dùng
- Nạp lại đúng HEX SHA-256 `131218ECF927D1ABB10F2564FF5EB55A135BBCA1E0DEA793B1FEE46205795746` qua ST-LINK `37FF71064E573436E9171343`, Under Reset 100 kHz; nguồn probe 3,27 V; `Download verified successfully`, software reset.
- Sau khoảng 5 giây, probe đúng SN đọc `mpu_diag.stage=1`, `ack_mask=0`, `hal_error=0x20`; `i2c_diag.state=2`, `last_error=0x20`, `attempts=16`; cả 5 ToF `state=5`; TIM1 CCR1/CCR2 0. Không có lệnh chuyển động được gửi.
- Khác lần nạp trước cùng HEX, khi sau 5 giây MPU stage=4 và 5 ToF hoạt động. Khởi động không ổn định; code retry đã gọi nhưng lần này bus vẫn không phục hồi. Chưa xác định nguyên nhân gốc (I2C line/device/power sequence). Hash HEX xác minh vẫn đúng.
