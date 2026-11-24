# Năm số chính, khối Phần cứng mở

Dữ liệu máy đọc: `so-chinh.json`. Nhãn [tính] lấy từ CAD hoặc SDK; [ước] có giả định nêu kèm.

| Số | Giá trị | Nhãn | Nguồn |
|---|---|---|---|
| Chiều cao tối đa (trụ nâng lên hết) | 1406 mm | [tính] | `lop.py` (hộp bao GLB ở lift 0,45 m) + GLB e81a130 + `sdk/faibotic/spec.py:98` (LIFT_RANGE) |
| Khối lượng robot (cận trên theo CAD) | 21,2 kg | [tính] | `docs/vi/specifications.md:203`, `product/00-tong-quan.md:30` |
| Khớp có động cơ | 16 | [tính] | `docs/vi/specifications.md:102`, `sdk/faibotic/spec.py:427` |
| Tải đỉnh ở tầm 0,25 m | 2,1 kg | [tính] | `docs/vi/specifications.md:155`, `product/00-tong-quan.md:22` |
| Dung lượng pin | 172,8 Wh | [tính] | `docs/vi/specifications.md:208`, `product/00-tong-quan.md:29` |

Dòng phụ (không phải chiều cao robot): độ cao tâm kẹp lớn nhất so với sàn, lift 0,45 m, 1528 mm [tính], `docs/vi/specifications.md:88`.

Lưu ý khi dùng trên web:
- Chiều cao 1406 mm là hộp bao GLB theo trục z, tay buông xuống, đỉnh là cột camera.
- Khối lượng là cận trên (chi tiết in tính khối đặc); cân thật chờ EVT.
- Tải đỉnh chưa đo trên robot thật.
- Pin giả định cell 4 Ah; thời gian chạy chờ EVT. Chưa có pin trong mô hình 3D.
