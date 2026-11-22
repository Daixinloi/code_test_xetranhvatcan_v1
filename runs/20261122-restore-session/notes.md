# Khôi phục firmware cuối chat tuyến cong T9/T12

- HEX Release hiện có ở dự án chính đã bị ghi đè bằng bản 2FA97EE4, dừng rồi quay waypoint.
- Dựng bản sao source vào firmware/, khôi phục bảng tuyến mm và FixedLineMove nối điểm bằng Motion_RetargetForward của cuối chat này.
- Build lại cho HEX khớp hoàn toàn SHA-256 38C477EDC02F35B89C6B822356BEE126028F1A566B25A5EFD5DBECA422DBC928. Flash 65312 byte. Đây là cùng nội dung HEX gốc đã bàn giao trong chat.
- Chín nhóm Tests/run_checks.ps1 và test_navigation --trace đều đạt. Trace ở simulation.json.
- Đã nạp firmware/build/Release/code_test_xetranhvatcan.hex qua SWD Under Reset 100 kHz lúc nguồn 3,27 V. CubeProgrammer xác minh toàn file thành công và reset phần mềm. Chưa chạy xe thật.
- 2026-11-22 nạp lại theo yêu cầu từ link chat chia sẻ (link trả “Shared chat not found”, nên dùng bản khôi phục này). Lần ghi đầu bị cắt ở khoảng 26% khi tiến trình CLI bị đóng cùng terminal; sau đó ST-LINK trả DEV_CONNECT_ERR/DEV_USB_COMM_ERR. Rút/cắm lại USB ST-LINK và cắt–bật nguồn bo thì nạp lại toàn bộ thành công: download 100% sau 13,8 s, `Download verified successfully`, reset phần mềm, nguồn 3,27 V, vector sau nạp `20005000 0800CD65`. Log `flash_final.txt`, `readback-vector8.txt`. Chưa chạy xe thật.

