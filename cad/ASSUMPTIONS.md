# Giả định CAD v0.5 P-L (23/11/2026)

Mọi số dưới đây có cờ. Đo hàng thật khi về rồi sửa nguồn (`faibotic/params.py`, `faibotic/assembly_pl.py`) và chạy lại `assembly_pl.py --masses`, `pl_sim_dims.py`. Giả định của bản cũ (vai ghép đôi, trục nâng cũ, vỏ pin cũ): `archive/v0.4/cad/ASSUMPTIONS.md`.

## Servo

| Hạng mục | Giá trị dùng | Nguồn / cờ |
|---|---|---|
| STS3095 C002 thân | 65 x 30 x 48 mm, 187,5 g (một nguồn ghi 194,5 g) | trang thông số của hãng, tra 22/11/2026 [số hãng tự báo, chưa có bản vẽ hãng] |
| STS3215 C018 | 70,5 g; thân theo mesh MuJoCo Menagerie SO-101 (chỉ hình hiển thị) | [số hãng tự báo] |
| Trục ra STS3095 | song song cạnh 30 mm, cách đầu thân 10 mm, giữa chiều cao 48 mm | [chưa xác minh], đo servo thật |
| Còi STS3095 | Ø25, dày 3,5; 4 x M3 trên đường tròn Ø20 | [chưa xác minh], đo còi thật |
| Servo trục nâng trong hốc khung T | khối bao, chưa lọt hốc | chưa chốt (DK-N41) |

## Vật liệu và khối lượng (`export/M1-PL-masses.json`)

| Hạng mục | Giá trị dùng | Nguồn / cờ |
|---|---|---|
| PETG | 1,27 g/cm³ | [điển hình, chưa kiểm nguồn] |
| Trụ, khung T, bánh răng AlohaMini2 in PETG | infill hiệu dụng 0,40 | [giả định], chờ cân bản in |
| Sàn, giá servo bánh, khay, cột khay, nêm camera in PETG | infill hiệu dụng 0,5 | [giả định] |
| Tay AM-ARM200 | khối đặc 1,0 g/cm³ theo URDF upstream (cận trên); bản in 4 thành 15% nhẹ hơn (`faibotic/print_mass_pl.py`) | [URDF upstream], [tự tính] |
| Tấm thép đối trọng M1-BS-032 + M1-BS-039 | thép 7,85 g/cm³, 8,5 + 6 mm, tổng 8,23 kg | [tự tính]; mức ~8 kg theo DK-N40, Q8 mặc định (a) |
| Nan nhôm 20x40 M1-BS-031, thanh ngang 2040 | 0,6 kg/m | [giả định] |
| Bánh omni 4 in M1-BS-035 | 0,2 kg/bánh | [giả định], chưa có số bên bán |
| Khoang điện tử M1-BS-038 (pin, Pi, E-stop) | khối giữ chỗ 1,1 kg | [giả định], chờ bố trí điện chuỗi L |
| Solenoid chốt M1-LF-050, công tắc M1-LF-061 | 60 g, 5 g, khối bao | [giả định] |
| Camera H65V1 | 30 g | [số hãng tự báo] |

## Hình học

| Hạng mục | Giá trị dùng | Nguồn / cờ |
|---|---|---|
| Đặt trụ, khung T, tay | theo khớp URDF AlohaMini2 và AM-ARM200 (`export/AlohaMini2_placement.json`) | [URDF upstream]; chi tiết upstream chỉ đặt vị trí, không sửa |
| Bánh răng trục nâng | STEP 26 răng, STL 28 răng (upstream không thống nhất), mô-đun 1,5 | chưa chốt (DK-N41), đếm răng bản in |
| Hành trình nâng | lift CAD -160..+290 mm (= SDK 0..0,45 m); dải răng -165,4..394,1, chốt 320,3 | `export/M1-PL-lift-scan.json` [tự tính] |
| Giá J5 lồng khuỷu J4 (upstream) | 186–338 mm³ ở tư thế down/ready | chưa chốt (DK-N38) |
| Khe yêu cầu | ≥ 0,3 mm; giao cắt ngoài cặp chủ ý 0 mm³ | SPEC; `assembly_pl.py --check` |
