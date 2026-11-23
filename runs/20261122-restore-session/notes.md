# Khôi phục firmware cuối chat tuyến cong T9/T12

- HEX Release hiện có ở dự án chính đã bị ghi đè bằng bản 2FA97EE4, dừng rồi quay waypoint.
- Dựng bản sao source vào firmware/, khôi phục bảng tuyến mm và FixedLineMove nối điểm bằng Motion_RetargetForward của cuối chat này.
- Build lại cho HEX khớp hoàn toàn SHA-256 38C477EDC02F35B89C6B822356BEE126028F1A566B25A5EFD5DBECA422DBC928. Flash 65312 byte. Đây là cùng nội dung HEX gốc đã bàn giao trong chat.
- Chín nhóm Tests/run_checks.ps1 và test_navigation --trace đều đạt. Trace ở simulation.json.
- Đã nạp firmware/build/Release/code_test_xetranhvatcan.hex qua SWD Under Reset 100 kHz lúc nguồn 3,27 V. CubeProgrammer xác minh toàn file thành công và reset phần mềm. Chưa chạy xe thật.
- 2026-11-22 nạp lại theo yêu cầu từ link chat chia sẻ (link trả “Shared chat not found”, nên dùng bản khôi phục này). Lần ghi đầu bị cắt ở khoảng 26% khi tiến trình CLI bị đóng cùng terminal; sau đó ST-LINK trả DEV_CONNECT_ERR/DEV_USB_COMM_ERR. Rút/cắm lại USB ST-LINK và cắt–bật nguồn bo thì nạp lại toàn bộ thành công: download 100% sau 13,8 s, `Download verified successfully`, reset phần mềm, nguồn 3,27 V, vector sau nạp `20005000 0800CD65`. Log `flash_final.txt`, `readback-vector8.txt`. Chưa chạy xe thật.


## 2026-11-22 — Làm phẳng đoạn T11 và kéo điểm cuối qua vạch

Người dùng báo tuyến hạ rồi bẻ chếch lên đâm T11 và dừng trước đích (ảnh, chưa có tọa độ/telemetry). Thay cụm waypoint gãy quanh T11 bằng hành lang Y=850–879 mm từ X=542 đến X=999 mm để tránh đoạn bật ngược lên; đưa đích từ `(1300,-65)` xuống `(1300,-120)` mm để thêm dự phòng qua vạch. Release build đạt (65204 B flash, 17184 B RAM), 9/9 nhóm PC PASS. Chưa nạp/chưa kiểm chứng xe thật. Không tăng PWM vì chưa có thời gian chạy ổn định hoặc log.

Đã nạp bản HEX 42CECC84... thành công, verify/reset, nguồn 3.23 V; vector khớp, PWM=0. Chưa chạy thật.

2026-11-22: Enabled existing ToF reference/local-detour pipeline by setting COURSE_FIXED_LINE_TEST=0, since fixed-line branch bypassed all ToF processing. Release build and 9/9 PC groups pass. Debug link overflows flash by 432 B. This does not implement the requested perpendicular 90-degree scan/80-mm lateral probe exactly; map clearance not certified to 80-mm edge distance. Not flashed or field-tested.

## 2026-11-23 — Candidate for stop near T9 (not flashed)

User reported a stop shortly before passing T9; no run log or sensor values were available. In this source copy, centered-route `ForwardRoom()` now uses only the centre beam, the right diagonal below 40 mm steers left by 2–8 degrees when the centre is above 50 mm, and only the centre beam starts avoidance. Centered lateral probes now require a full 80 mm, try right first, and try the opposite side if map clearance rejects that probe. Release built at 65,388/65,536 B flash and 17,184/20,480 B RAM; HEX SHA-256 `73FE6488A84F5CFF68726C4055860DF8D8A272C897CA1DE245CA18CBFAD2ED25`. PC tests still stop at the existing blocked-probe case `(511,997) mm`; Debug overflows by 612 B. Per user's request, this Release HEX was flashed and fully verified through ST-LINK `37FF71064E573436E9171343` at 3.27 V, SWD Under Reset 100 kHz. Post-reset vector `20005000 0800CE1D`; TIM1 CCR1/CCR2 are 0. Not yet tested driving on the vehicle. See `runs/20261123-tof-stop-t9/notes.md` and `docs/04-nhat-ky-loi.md`.
