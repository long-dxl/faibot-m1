# MuJoCo Menagerie: robotstudio_so101 (mesh servo STS3215)

Nguồn: https://github.com/google-deepmind/mujoco_menagerie, thư mục `robotstudio_so101/`, commit `3abca2e302625cec9429c49806121ea2855cef81` (28/09/2026), tải 23/11/2026.
Menagerie ghi gói này dẫn xuất từ MJCF công khai của The Robot Studio SO-ARM100 (https://github.com/TheRobotStudio/SO-ARM100, `Simulation/SO101`).
Giấy phép: Apache License 2.0 (tệp `LICENSE` bên cạnh, chép nguyên văn từ `robotstudio_so101/LICENSE`).

| Tệp | Nguồn | SHA-256 | Ghi chú |
|---|---|---|---|
| `sts3215_03a_v1.stl` | `robotstudio_so101/assets/sts3215_03a_v1.stl` | a37c871fb502483ab96c256baf457d36f2e97afc9205313d9c5ab275ef941cd0 | nguyên văn, 19.080 tam giác, đơn vị m |
| `LICENSE` | `robotstudio_so101/LICENSE` | c71d239df91726fc519c6eb72d318ec65820627232b2f796219e87dcf35d0ab4 | nguyên văn |
| `derived/sts3215_03a_v1_sim.stl` | dẫn xuất từ `sts3215_03a_v1.stl` | xem `cad/CHANGES.md` v0.4.6 | **ĐÃ SỬA** (Apache-2.0 điều 4b): rút gọn còn 1.500 tam giác bằng Blender 4.0.2 Decimate (collapse), `derived/decimate_blender.py`; vẫn kín, hộp bao lệch bản gốc <= 0,2 mm |

Dùng trong Faibot M1 (`cad/faibotic/servos.py`): thân servo STS3215 (M1-SV-001) ở pan, gập cổ tay, cuộn cổ tay, kẹp và 3 bánh. Mesh được đặt (quay, tịnh tiến; tay phải đổi dấu một trục để giữ phép quay đúng), gộp theo khớp vào `sim/assets/*__servo_body.stl`, rút gọn tiếp trong `web-assets/m1.glb`; bản đầy đủ dùng cho ảnh render. Màu đen theo vật liệu `sts3215_03a_v1_material` của Menagerie (rgba 0,1 0,1 0,1).

Không dùng tên "SO-101", "SO-ARM100", "The Robot Studio", "MuJoCo Menagerie" làm nhãn hiệu của sản phẩm Faibotic (Apache-2.0 điều 6).
