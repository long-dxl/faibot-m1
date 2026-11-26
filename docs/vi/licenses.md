# License

Mã trong repo này (sim/, sdk/, demos/, tests/, examples/) do Faibotic tự viết, Copyright (c) 2026 Leo Dao (Faibotic), theo Apache License 2.0. Giấy phép từng thư mục, từng tệp: [NOTICE](../../NOTICE). Không copy URDF/MJCF/mesh của SO-101, XLeRobot, Menlo, trừ một ngoại lệ ghi ở mục CAD: mesh thân servo STS3215 của MuJoCo Menagerie `robotstudio_so101` (Apache-2.0, từ 23/11/2026); không chép mã LeRobot (giao thức Feetech viết lại từ tài liệu giao thức của hãng). Không chứa dòng nào từ BOM Menlo Asimov.

## CAD (cad/)

Kế thừa một phần CAD AlohaMini2 (https://github.com/liyiteng/AlohaMini, commit 59f024b, 20/11/2026) và AM-ARM200 (https://github.com/liyiteng/AM-ARM, commit 5b03af1, 18/06/2026), cả hai theo Apache License 2.0, tải 22/11/2026. Dùng thương mại được, kèm điều kiện điều 4: bản license ở `cad/upstream/LICENSE`, thông báo bản quyền và ghi công giữ ở `cad/NOTICE` và README của upstream, `cad/CHANGES.md` liệt kê tệp đã sửa hoặc thay (STEP/STL không có header). Không dùng tên AlohaMini, AM-ARM làm nhãn hiệu (điều 6). Hai kho chỉ có LICENSE gốc, không có header license riêng ở tệp CAD; phụ lục LICENSE để trống tên chủ bản quyền.

Mesh servo STS3215 (từ 23/11/2026): `cad/upstream/menagerie-so101/sts3215_03a_v1.stl` chép nguyên văn từ MuJoCo Menagerie (https://github.com/google-deepmind/mujoco_menagerie, `robotstudio_so101/assets/`, commit 3abca2e, 28/09/2026; Menagerie ghi gói này dẫn xuất từ The Robot Studio SO-ARM100), Apache License 2.0, bản license ở `cad/upstream/menagerie-so101/LICENSE`. Tệp đã sửa: `derived/sts3215_03a_v1_sim.stl` (rút gọn 1.500 tam giác), và các tệp chứa mesh này sau khi đặt, gộp, rút gọn: `sim/assets/*__servo_body.stl`, `web-assets/m1.glb`, ảnh `media/renders-web/`. Ghi công ở `cad/NOTICE`, `sim/assets/NOTICE`, `web-assets/NOTICE`, `cad/upstream/menagerie-so101/README.md`. Linh kiện servo vẫn là hàng thương mại Feetech; chỉ hình học hiển thị là mã mở. Vị trí servo cổ tay và kẹp đọc từ mesh URDF AM-ARM200 (cùng commit 5b03af1, chỉ lấy số đo, không chép tệp mesh vào repo). Mô hình STS3095 vẽ sạch bằng build123d (`cad/faibotic/servos.py`), Copyright (c) 2026 Leo Dao (Faibotic), CC BY-NC-SA 4.0.

Cấu hình P-L (v0.5, từ 23/11/2026): thêm 11 STEP mobile_base2 và URDF `alohamini2.urdf` của AlohaMini2 (cùng commit 59f024b, Apache-2.0, nguyên văn, không sửa; danh sách và SHA-256 ở `cad/CHANGES.md`). Tay AM-ARM200 follower dùng nguyên STEP/URDF đã có. Mesh URDF và STL bố trí của AlohaMini2 chỉ tải về máy dựng để đo vị trí (đăng ký ICP), không chép vào repo. Đế "kiểu LeKiwi" chỉ theo cách bố trí (servo dẫn thẳng bánh omni 4 in); không chép tệp LeKiwi.

Kẹp (24/11/2026): kẹp mặc định của M1 là kẹp gốc của AM-ARM200 follower, gồm `OB_F_J7_Gripper` (thân kẹp, M1-AR-007), `OB_F_J7_Jaw` (hàm động, M1-AR-008) và `OB_F_Camera` (giá camera cổ tay, M1-AR-027), nguyên STEP trong `cad/upstream/AM-ARM/am-arm200/stp/follower/`, Apache License 2.0, cùng điều kiện như tay AM-ARM200 ở trên. Kẹp song song của một hãng khác đã loại khỏi bản public vì giấy phép upstream chưa ổn định; hồ sơ và tệp cũ nằm trong lưu trữ nội bộ.

Đế kiểu LeKiwi (v0.5): chỉ theo cách bố trí đế và linh kiện kit LeKiwi 12V; không chép tệp nào của LeKiwi.

Phần Faibotic tự vẽ (`cad/faibotic/*.py`, `cad/export/`, `cad/renders/`, mesh chứa chi tiết Faibotic vẽ trong `sim/assets/`, `web-assets/m1.glb`): Copyright (c) 2026 Leo Dao (Faibotic), CC BY-NC-SA 4.0 (`LICENSE-hardware`); dùng thương mại: liên hệ founder@faibotic.com. Công cụ: build123d (Apache-2.0), trimesh (MIT), matplotlib (PSF/BSD), numpy, scipy chỉ để dựng và render, không phân phối kèm.

Phụ thuộc Python (pip freeze trên hai máy dựng, 22/11/2026):

| Gói | Phiên bản | License | Vai trò |
|---|---|---|---|
| mujoco, mujoco-core, mujoco-studio, mujoco-studio-wasm | 4.0.1 | Apache-2.0 | mô phỏng, render |
| numpy | 2.5.5 | BSD-3-Clause (kèm 0BSD, MIT, Zlib, CC0) | tính toán |
| glfw (pyGLFW) | 2.11.0 | MIT | cửa sổ teleop (kéo theo mujoco) |
| PyOpenGL | 3.1.10 | BSD | kéo theo mujoco |
| websockets | 17.4 | BSD-3-Clause | kéo theo mujoco |
| pyserial | 3.5 | BSD-3-Clause | cổng nối tiếp cho backend feetech |
| gpiod | 2.6.0 | LGPL-2.1-or-later (PyPI license_expression, tra 22/11/2026: https://pypi.org/project/gpiod/) | GPIO Pi 5 cho kịch bản EVT (`product/evt/`); chỉ cài trên Pi, không có trong requirements, không phân phối kèm; dùng qua import, không sửa mã |
| smbus2 | 0.6.1 | MIT (PyPI classifier, tra 22/11/2026: https://pypi.org/project/smbus2/) | I2C đọc INA226 cho kịch bản EVT; chỉ cài trên Pi, không có trong requirements |
| pytest | 9.2.0 | MIT | test |
| pluggy 1.7.0, iniconfig 2.3.1 | | MIT | kéo theo pytest |
| packaging 26.3 | | Apache-2.0 OR BSD-2-Clause | kéo theo pytest |
| Pygments 2.21.0 | | BSD-2-Clause | kéo theo pytest |

Công cụ ngoài, không phân phối kèm: ffmpeg (LGPL/GPL tuỳ bản build, dùng libx264) để ghi mp4; tuỳ chọn opencv-python (Apache-2.0) cho camera thật. Phần mã không nhúng mã bên thứ ba; NOTICE chỉ áp cho `cad/` (xem trên).
