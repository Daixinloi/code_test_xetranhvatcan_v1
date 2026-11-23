# 2026-11-22 — Hạ Y vùng T11/T14, dịch chặng cuối trái 10 mm

## Yêu cầu (từ người dùng, kèm ảnh map + vệt đỏ)
- Chặng cuối dịch thêm 1 cm (lần trước đã dịch phải 12 mm).
- Xe chạm nhẹ T15, T16 và chạm nhẹ phần dưới T11, T14 → hạ Y ở hai vùng đó.
- Hỏi thuật toán né đã áp dụng vào xe chưa.

## Đo lại tuyến cũ (trước khi sửa)
Dùng `Tools/plan_route.py validate --inflate 52`, tuyến 74 điểm (cả điểm xuất phát):
- Dài 2.535,4 mm, khoảng hở nhỏ nhất **2,8 mm** trên biên nới 52 mm.
- Ở biên nới 64 mm (nửa đường chéo thân xe): 3 điểm / 5 đoạn nằm trong vùng nới.
- Hai điểm sát nhất: `(734,893)` cạnh T11 = 14,9 mm và `(1146,720)` cạnh T14 = 16 mm
  (đo trên biên nới 52 mm). Khớp đúng chỗ người dùng báo chạm.

## Thay đổi trong `Inc/course_route.h`
1. Vùng T11 (X 726–999) hạ Y 3–15 mm: `(726,892) (734,880) (742,876) (768,872)
   (776,870) (784,868) (792,867) (800,866) (808,866) (816,866) (825,866) (833,866)
   (901,866) (914,866) (927,864) (940,861) (952,857) (965,852) (976,846) (988,840)
   (999,833)`.
2. Vùng T14 (X 1132–1190) hạ Y 12–29 mm: `(1132,724) (1146,720) (1160,718) (1175,716)
   (1190,714)`.
3. Chặng cuối dịch trái 10 mm: `(1304,645) (1302,631) (1302,616) (1302,-65)`.
   Lý do: xe chạm T15/T16 ở X≈1400, tức đang lệch phải; kéo chặng cuối về 1302 mm
   tăng khoảng hở phía T15/T16.

## Kết quả đo lại sau khi sửa
- Biên nới 52 mm: 74 điểm, dài 2.551,0 mm, khoảng hở nhỏ nhất **18,0 mm**, 0 va chạm.
- Biên nới 64 mm (nửa đường chéo thân xe): khoảng hở nhỏ nhất **6,0 mm**, 0 điểm và
  0 đoạn cắt vật cản → thân xe không thể chạm vật cản ở bất kỳ hướng nào.
- Trước khi sửa ở 64 mm là 3 điểm / 5 đoạn cắt → nay hết.

## Build
- Release: FLASH 65.312 / 65.536 byte (99,66 %, còn 224 B), RAM 17.184 / 20.480 byte.
- HEX `build/Release/code_test_xetranhvatcan.hex`, SHA-256
  `E40D2A3A7EF972F85495E4B934F90ECFE32D63283D43FD4A36B6D5F29CF35553`, 183.780 byte.
- `Tests/run_checks.ps1`: 9/9 nhóm PASS, EXIT=0.

## Nạp
- SWD Under Reset 100 kHz, nguồn 3,27 V, STM32F103 medium-density 64 KB.
- Download 100 % sau 13,916 s, `Download verified successfully`, reset phần mềm.
- Vector sau nạp `0x08000000 = 20005000 0800CD65`.
- Log: `flash.txt`, `readback.txt`.

## Chưa làm
- Chưa chạy xe thật với bản này.
- Chưa áp dụng thuật toán né phản ứng (reactive avoidance) cho tuyến cố định; xem
  mục "Thuật toán né" trong `docs/03-ngu-canh.md`.
