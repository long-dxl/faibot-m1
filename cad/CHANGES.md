# Danh sách thay đổi (Apache License 2.0, điều 4(b))

Tệp này liệt kê các tệp kế thừa mà Faibot M1 v0.5 đã sửa hoặc dẫn xuất, và sửa gì. Nguồn và commit gốc của từng phần: xem `NOTICE` cùng thư mục.

## Tệp kế thừa giữ nguyên

Mọi tệp trong `upstream/` được chép nguyên văn từ commit nêu trong `NOTICE`, không sửa trực tiếp tệp nào:

- AM-ARM (Apache-2.0): STEP tay follower `am-arm200/stp/follower/*`, URDF `am-arm200/urdf/urdf/am-arm200.urdf`, mesh URDF `am-arm200/urdf/am-arm200-meshes/*.STL`.
- AlohaMini (Apache-2.0): STEP đế và trụ nâng `AlohaMini2/hardware/mobile_base2/stp/*`, URDF `AlohaMini2/urdf/alohamini2/urdf/alohamini2.urdf`, bánh răng trục nâng `OB_Z_Axis_Servo_Gear.stp`.
- MuJoCo Menagerie (Apache-2.0): mesh servo STS3215 trong `upstream/menagerie-so101/`.

Trong lắp ráp, các chi tiết này chỉ được đặt vị trí (phép quay, tịnh tiến); hình học không đổi.

## Tệp dẫn xuất đã sửa

- `upstream/AM-ARM/am-arm200/urdf/derived/*.stl` (dẫn xuất từ mesh URDF AM-ARM `left_Base`, `left_Rotation_Pitch`, `left_Upper_Arm`, `left_Lower_Arm`, `left_Wrist_Pitch_Roll`, `left_wrist_yaw`, `left_Fixed_Jaw`, `left_Moving_Jaw`, `left_camera`): ĐÃ SỬA. Mỗi mesh được tách thành phần thân in, phần kim loại và từng thân servo; bỏ chi tiết nhỏ dưới 6 mm; rút gọn số tam giác. Công cụ: `faibotic/extract_urdf_servos.py`. Danh sách và mã băm: `derived.json`.
- Mesh mô phỏng và mô hình 3D trên web (`sim/assets/*.stl`, `web-assets/m1.glb`) dựng từ các STEP và mesh kế thừa ở trên: ĐÃ SỬA. Rút gọn tam giác, làm mượt pháp tuyến, gộp theo cụm, đặt vào khung toạ độ của robot; tay phải là ảnh gương của tay trái.
- Mesh servo STS3215 từ MuJoCo Menagerie: ĐÃ SỬA (rút gọn, đặt lại gốc toạ độ) khi dùng cho servo bánh và servo trục nâng trong mô phỏng.

## Phần do Faibotic làm mới

Đế mở rộng và đối trọng, trạm hai tay khung chữ U và đế ngàm, gá servo trục nâng, bánh răng trục nâng 20 răng, khay, hộp điện: vẽ mới bằng mã CAD tham số trong `faibotic/`, không chép hình học kế thừa. Giấy phép của phần này: xem `LICENSE-hardware` ở thư mục gốc.
