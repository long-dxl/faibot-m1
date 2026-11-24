# Nguồn thông số servo

## STS3215
- Feetech, trang sản phẩm STS3215 bản 7,4 V: mô-men chặn 19,5 kg·cm @ 7,4 V, kích thước 45,2 × 24,7 × 35 mm, phản hồi vị trí/tốc độ/tải/điện áp/dòng/nhiệt. https://www.feetechrc.com/en/2020-05-13_56655.html (tra 22/11/2026).
- Bản 12 V (C018) dùng trong M1: 30 kg·cm theo tên thương mại phổ biến "STS3215 30KG" [chưa xác minh bằng datasheet hãng]. Mô-men định mức: chưa có số hãng, để trống.
- Khối lượng 55 g dùng trong sim: [chưa xác minh], chỉ ảnh hưởng nhỏ tới mô-men tĩnh.

## STS3095
- STS3095-C002 12 V: mô-men chặn 95 kg·cm, định mức 22,5 kg·cm, 31 vòng/phút không tải, dòng chặn 9,8 A. Nguồn: trang bán https://openelab.io/products/feetech-st3095-c002-serial-servo (tra 22/11/2026, theo báo cáo phần cứng F016, `archive/v0.4/docs/F016-bao-cao.md`; chưa có datasheet chính thức của Feetech).

## Giao thức bus
- Cấu trúc gói, checksum, bảng địa chỉ STS (vị trí đích 42, vị trí hiện tại 56, tốc độ đích 46, chế độ 33, khoá EEPROM 55) theo tài liệu giao thức Feetech SCS/STS. Test vector `FF FF 01 04 02 38 02 BE` (đọc vị trí ID 1) và `FF FF 01 09 03 2A 00 08 00 00 E8 03 D5` (ghi vị trí 2048, tốc độ 1000) là ví dụ trong tài liệu đó, đã kiểm lại checksum bằng tay.
