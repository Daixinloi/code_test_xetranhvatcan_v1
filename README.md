# Xe tránh vật cản STM32

Firmware cho xe dùng **STM32F103C8T6**, cảm biến khoảng cách VL53L0X, MPU, encoder và Bluetooth HC-06. Xe hỗ trợ lái tay và chạy tự động trên sân chữ U 180 × 120 cm.

Thư mục đang phát triển là [`code_test_xetranhvatcan/`](code_test_xetranhvatcan/). VS Code dùng thư mục đó làm gốc CMake. Các thư mục bên cạnh là bản lưu để đối chiếu.

## Cây thư mục

```text
code_test_xetranhvatcan_v1/
├── README.md
├── .vscode/                          Cấu hình VS Code của workspace
├── docs/                             Luật thi, định hướng và ngữ cảnh bàn giao
├── code_test_xetranhvatcan/          Dự án chính
│   ├── Src/                          Mã firmware STM32
│   ├── Inc/                          Header và cấu hình firmware
│   ├── Drivers/                      STM32 HAL và CMSIS
│   ├── Tests/                        Kiểm thử trên PC và mô phỏng sân
│   ├── Tools/                        Công cụ kiểm tra cấu hình và vẽ bản đồ
│   │   └── map_monitor/              Web mô phỏng, theo dõi và ghi đường xe
│   ├── cmake/                        Toolchain ARM và cấu hình CubeMX cho CMake
│   ├── build/                        Đầu ra biên dịch, log và dữ liệu thử xe
│   ├── CMakeLists.txt, CMakePresets.json
│   ├── *.ioc, startup_*.s, *.ld      Cấu hình CubeMX, khởi động và bộ nhớ STM32
│   ├── *.bat                         Script build và nạp qua ST-LINK
│   └── *_SETUP.md                    Hướng dẫn điều khiển và chạy sân
├── baseline_v2_original/            Bản gốc để đối chiếu
├── calibrated_v2/                   Bản hiệu chỉnh trước đây
└── review_*/                        Các bản chụp ở những lần sửa trước
```

## Tìm code ở đâu

| Muốn xem | Bắt đầu từ |
| --- | --- |
| Lệnh Bluetooth, vòng lặp chính và đọc cảm biến | [`Src/main.c`](code_test_xetranhvatcan/Src/main.c) |
| Tiến, lùi, xoay và giữ hướng bằng MPU và encoder | [`Src/motion.c`](code_test_xetranhvatcan/Src/motion.c), [`Inc/motion.h`](code_test_xetranhvatcan/Inc/motion.h) |
| Điều khiển bánh xe, PWM và encoder | [`Src/board_drive.c`](code_test_xetranhvatcan/Src/board_drive.c), [`Src/motor.c`](code_test_xetranhvatcan/Src/motor.c) |
| Điều hướng, tránh vật cản và bản đồ | [`Src/navigation.c`](code_test_xetranhvatcan/Src/navigation.c), [`Src/grid_map.c`](code_test_xetranhvatcan/Src/grid_map.c) |
| Cấu hình sân và tuyến chữ U | [`Inc/navigation_config.h`](code_test_xetranhvatcan/Inc/navigation_config.h), [`Inc/course_route.h`](code_test_xetranhvatcan/Inc/course_route.h) |
| Máy chủ web, dữ liệu COM và ghi đường lái tay | [`Tools/map_monitor/`](code_test_xetranhvatcan/Tools/map_monitor/) |

## Tài liệu bàn giao

- [Bài toán và luật thi](docs/01-bai-toan.md)
- [Định hướng giải quyết và kiểm chứng](docs/02-dinh-huong-giai-quyet.md)
- [Ngữ cảnh cho phiên làm việc mới](docs/03-ngu-canh.md)
- [Nhật ký lỗi và cách sửa](docs/04-nhat-ky-loi.md)
- [Thiết kế ghi log từng lượt chạy](docs/05-ghi-log-chay-thu.md)
- [Quy trình cải thiện bộ điều khiển](docs/06-quy-trinh-cai-tien.md)
- [Cách gửi log một lượt chạy cho AI](docs/07-cach-gui-log-cho-ai.md)

## Chạy nhanh trên Windows

Mở PowerShell tại gốc workspace rồi vào dự án chính.

```powershell
cd code_test_xetranhvatcan
```

### Build và nạp firmware

Cần CMake, Ninja và ARM GCC (`arm-none-eabi-gcc`) trên `PATH`. Nếu đã cài STM32CubeIDE đúng đường dẫn máy phát triển, `build.bat` thiết lập `PATH` và chạy build Debug. Đường dẫn công cụ được cấu hình trong `build.bat` và `.vscode/settings.json`.

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Kết quả là `build/Debug/code_test_xetranhvatcan.hex`, cùng các bản `.bin` và `.elf`. Để nạp bản vừa build, kết nối ST-LINK và chạy `.\flash.bat`. Script dùng file `.hex` sẵn có, vì vậy hãy build lại sau khi sửa nguồn rồi mới nạp. Script nạp gọi STM32CubeProgrammer CLI hoặc ST-LINK Utility.

### Kiểm thử và mở web

Cần `gcc` dành cho PC trên `PATH`. Web còn cần Python 3 vì máy chủ biên dịch mô phỏng C khi khởi động.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File Tests/run_checks.ps1
python Tools/map_monitor/server.py --no-com
```

Mở [localhost:8765](http://localhost:8765) để xem mô phỏng. Khi cần xem dữ liệu xe qua HC-06, chạy `python Tools/map_monitor/server.py --com COM16` và thay `COM16` bằng cổng của máy bạn. `Ctrl+C` trong terminal dừng web và giải phóng cổng COM.

- [Hướng dẫn điều khiển tay, encoder, MPU và ToF](code_test_xetranhvatcan/MOTION_SETUP.md)
- [Hướng dẫn cấu hình sân, chạy tự động và kiểm tra map](code_test_xetranhvatcan/NAVIGATION_SETUP.md)

### Lái tay, ghi đường và phát lại không dùng ToF

Bản thử riêng dùng `cmake --preset Teach` và `cmake --build --preset Teach`. Nạp đúng file `build/Teach/code_test_xetranhvatcan.hex` qua ST-LINK. Kết nối điện thoại trực tiếp với HC-06 và gửi ký tự chữ hoa theo bảng. Có thể dùng `x` hoặc `X` để dừng ngay.

| Lệnh | Tác dụng trong bản Teach |
| --- | --- |
| `F` | Giữ để tiến thẳng, PWM cơ sở 280/1000. |
| `B` | Lùi với PWM cơ sở 280/1000, bấm `x` để dừng. |
| `R` / `L` | Bấm một lần xoay khoảng 5° sang phải / trái. Giữ để xoay liên tục, PWM 230/1000. |
| `x` | Dừng lái tay, giữ phiên ghi đang mở. Nếu đang phát lại thì hủy lượt. |
| `Y` | Xóa bản ghi đang mở và đặt lại mốc khi xe đã dừng ở vạch xuất phát. |
| `Z` | Đặt mốc và bắt đầu ghi từ vạch xuất phát. |
| `V` | Dừng xe, kết thúc phiên ghi và lưu tuyến vào flash trên STM32. |
| `U` | Xác nhận xe đã về đúng mốc xuất phát và phát lại tuyến vừa lưu. |
| `J` | Chạy tuyến thử dựng từ bản đồ cố định, qua dưới T14 và xuống đích. Đặt xe ở mốc xuất phát như khi gửi `Z`. |
| `P` | Phát lại tuyến đã lưu sau khi đặt xe về đúng mốc và gửi `Y`. |

1. Đặt tâm xe ở `(420, −45) mm`, đầu hướng Bắc, rồi giữ xe yên đến khi điện thoại nhận `MPU_OK_DAT_XE_GUI_Y_HOAC_Z`. Gửi `Z` rồi dùng `F/B/R/L`. Để giữ `F/R/L`, app phải gửi lặp đúng ký tự khi giữ, ví dụ mỗi 100 ms, và ngừng gửi lúc thả. Xe dừng khi thiếu lệnh lặp trong 300 ms. Khi app có lệnh lúc thả, dùng `x`. Giới hạn mỗi đoạn là 20 giây.
2. Tới đích, gửi `V` và đợi `DA_LUU` ở trường `Lenh`, kèm mã `ID` và số đoạn `SEG`. Có thể tắt nguồn sau xác nhận này. `Y` giữ tuyến đã lưu. Bản thử lưu tối đa 200 đoạn của tuyến mới nhất. Ghi lại mã `ID` và xuất log Bluetooth của điện thoại vào thư mục lượt trong `runs/` nếu cần đối chiếu.
3. Lượt sau, đặt xe về đúng vị trí và hướng như lúc gửi `Z`. Khi xe đứng yên và MPU sẵn sàng, gửi `U`. Xe dùng E2 để phát lại đoạn thẳng và MPU để phát lại đoạn xoay. Kết thúc thành công báo `PHAT_LAI_XONG`.

Xe bỏ qua ToF trong cả lúc ghi lẫn phát lại. Giữ nguyên bố trí vật cản khi thử lại và quan sát xe để gửi `x` nếu cần. Giới hạn biên dựa vào vị trí ước tính từ encoder và MPU. Khi điện thoại mất kết nối, lệnh lùi hoặc phát lại đã bắt đầu có thể tiếp tục chạy. Lần thử này kiểm tra độ lặp lại trên sân cố định.

Tuyến `J` đi ngang dưới T14 ở khoảng Y = 700 mm, quay xuống tại X = 1300 mm rồi đi thẳng qua vạch đích. Nút `U` vẫn dành cho tuyến đã lưu bằng `V`.

## Đề xuất dọn thư mục

1. Kiểm kê các bản sao rồi gom `baseline_v2_original/`, `calibrated_v2/` và `review_*/` vào `archive/`. Ghi ngắn mục đích và mốc thay đổi của từng bản để dễ đối chiếu.
2. Dành `build/` cho đầu ra CMake. Gom log Bluetooth, dữ liệu ST-LINK và đường chạy cần lưu vào `runs/` sau khi kiểm tra file nào đang được web sử dụng.
3. Giữ bố cục `Src/`, `Inc/`, `Drivers/` và các file CubeMX. Nếu gom script hoặc tài liệu sau này, kiểm tra lại đường dẫn của CMake, VS Code và script nạp. Chạy lại build Debug, kiểm thử trên PC và web offline sau mỗi nhóm thay đổi.
