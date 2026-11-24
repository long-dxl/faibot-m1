# Thông số kỹ thuật Faibot M1

Faibot M1 v0.5 là robot hai tay trên đế omni có trục nâng. Mọi số dưới đây đến từ mô hình CAD và SDK, **chưa đo trên robot thật**.

Cách đọc nhãn:

- **[tính]**: lấy từ CAD hoặc SDK (`sdk/faibotic/spec.py`, `web-assets/joints.json`, `cad/export/`).
- **[ước]**: ước lượng, có giả định nêu kèm.
- **[đặt]**: giá trị đặt trong SDK, không phải kết quả tính.
- **[đo]**: đo trên robot thật. Chưa có robot, nên chưa có số nào mang nhãn này.
- **chờ EVT**: chưa biết, sẽ đo ở lô thử đầu tiên.

Các bảng số được sinh bằng `python3 scripts/sinh_specifications.py --ghi docs/vi/specifications.md`, không gõ tay. Sửa số ở `spec.py` hoặc `joints.json` rồi chạy lại.

## Hình học

Tay trái và tay phải đối xứng gương, cùng giới hạn. Góc 0 = tay duỗi thẳng ngang ra trước.

<!-- sinh:khop -->
| Khớp mỗi tay | Từ (độ) | Đến (độ) | Hành trình (độ) | Ghi chú |
|---|---|---|---|---|
| Xoay đế tay (pan) | -70,0 | 70,0 | 140,0 | [tính] giới hạn mềm SDK; ±70° vì tay chạm tay đỡ bên kia và cột khi gập |
| Vai (shoulder) | -80,0 | 48,5 | 128,5 | [tính] 0° = tay duỗi ngang ra trước, dương = gập xuống; đệm chặn chạm ở 50° |
| Khuỷu (elbow) | -10,0 | 127,0 | 137,0 | [tính] giới hạn trên giảm theo vai, xem bảng ghép bên dưới |
| Gập cổ tay (wrist flex) | -77,5 | 87,5 | 165,0 | [tính] giới hạn trên giảm theo vai và khuỷu |
| Yaw cổ tay (wrist yaw) | -81,0 | 80,0 | 161,0 | [tính] quét va chạm CAD, biên 3°; hẹp lại theo gập cổ tay (bảng ghép yaw–gập) |
| Cuộn cổ tay (wrist roll) | -160,0 | 160,0 | 320,0 | [tính] có vùng cấm −147…−87°, xem ghi chú |
| Kẹp (gripper) | 0 | 100 % | 64° góc hàm | [tính] 100 % = mở mềm 64°, đầu hai ngón cách 97,25 mm |

| Trục | Từ (mm) | Đến (mm) | Hành trình (mm) | Ghi chú |
|---|---|---|---|---|
| Trục nâng (lift) | 0 | 450 | 450 | [tính] hành trình SDK 0,00…0,45 m, tương ứng -160…290 mm trong hệ CAD |
| Đế di động (x, y) | không giới hạn |  |  | Tịnh tiến theo hai trục, đế 3 bánh omni |
| Đế di động (xoay) | không giới hạn |  |  | Xoay tại chỗ quanh trục đứng |

Giới hạn ghép khuỷu theo vai (nội suy tuyến tính):

| Vai (độ) | -80,0 | -32,0 | -16,0 | 0,0 | 8,0 | 16,0 | 24,0 | 32,0 | 36,0 | 40,0 | 44,0 | 48,5 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Khuỷu lớn nhất (độ) | 127,0 | 127,0 | 119,0 | 111,0 | 104,6 | 98,2 | 91,8 | 85,4 | 82,2 | 79,0 | 74,0 | 62,0 |

Giới hạn ghép gập cổ tay theo vai (hàng) và khuỷu (cột), độ:

| Vai \ Khuỷu (độ) | -10,0 | 70,0 | 100,0 | 110,0 | 120,0 | 127,0 |
|---|---|---|---|---|---|---|
| -80,0 | 87,0 | 87,0 | 87,0 | 87,0 | 87,0 | 74,5 |
| 0,0 | 87,0 | 87,0 | 87,0 | 82,0 | 62,0 | 47,0 |
| 30,0 | 87,0 | 87,0 | 87,0 | 59,5 | 54,5 | 49,5 |
| 40,0 | 87,0 | 87,0 | 82,0 | 59,5 | 57,0 | 39,5 |
| 48,5 | 87,0 | 87,0 | 67,0 | 62,0 | 57,0 | 22,0 |
<!-- /sinh:khop -->

**Cuộn cổ tay có vùng cấm −147…−87°.** Ở vùng này giá camera cổ tay chạm vỏ cổ tay [tính, quét CAD]. Hướng xử lý (cắt giá hoặc cấm vùng trong SDK) chờ EVT; trước đó đừng ra lệnh cuộn vào vùng này.

**Yaw cổ tay.** Mỗi tay có 6 khớp + kẹp (6+1), đúng như tay AM-ARM200 gốc: gập, yaw, cuộn cổ tay mỗi khớp một servo STS3215. Giới hạn yaw −81…+80° theo quét va chạm CAD: vỏ cổ tay chạm giá cổ tay ở khoảng ±84° [tính, biên 3°]. Khi gập cổ tay xuống, phía yaw dương hẹp lại vì đáy vỏ cổ tay chạm cẳng tay; SDK tự kẹp theo bảng ghép yaw–gập [tính].

Khoảng cách trục tới trục của từng khâu (tay trái, tư thế 0; tay phải lấy gương theo y):

<!-- sinh:khau -->
| Khâu (trục tới trục) | Khoảng cách (mm) | Theo x (mm) | Theo y (mm) | Theo z (mm) |
|---|---:|---:|---:|---:|
| Xoay đế tay (pan) → vai | 159,2 | 58,0 | 0,0 | 148,2 |
| Vai → khuỷu | 134,9 | 134,9 | 0,0 | 0,0 |
| Khuỷu → gập cổ tay | 198,4 | 198,4 | 0,0 | 3,2 |
| Gập cổ tay → yaw cổ tay | 28,4 | 28,1 | 3,8 | 1,3 |
| Yaw cổ tay → cuộn cổ tay | 64,8 | 61,0 | 0,0 | 22,0 |
| Cuộn cổ tay → tâm kẹp | 94,4 | 93,8 | 10,1 | 0,2 |
| Tâm kẹp → đầu ngón | 10,0 | 10,0 | 0,0 | 0,0 |

| Kích thước trạm và đế | mm |
|---|---:|
| Khoảng cách hai trục pan trái, phải | 475 |
| Trục pan trước tâm đế | 59,9 |
| Đáy đế tay so với sàn, lift = 0 | 413,7 |
| Trục vai so với sàn, lift = 0 | 561,9 |
| Trục vai so với sàn, lift = 0,45 m | 1011,9 |
| Bán kính đặt bánh omni quanh tâm đế | 250 |
<!-- /sinh:khau -->

Tầm với của tâm kẹp, tính trong giới hạn khớp và giới hạn ghép:

<!-- sinh:tamvoi -->
| Tầm với, tâm kẹp, pan 0, cuộn 0, tay trái | mm | Nhãn |
|---|---:|---|
| Bán kính ngang lớn nhất, tâm kẹp, từ trục pan | 576 | [tính] |
| Tầm lùi ra sau trục pan lớn nhất, tâm kẹp | 89 | [tính] |
| Tâm kẹp cao nhất so với trục vai | 516 | [tính] |
| Tâm kẹp thấp nhất so với trục vai | -479 | [tính] |
| Độ cao tâm kẹp lớn nhất so với sàn (lift 0,45 m) | 1528 | [tính] |
| Độ cao tâm kẹp nhỏ nhất so với sàn (lift 0) | 83 | [tính] |
| Độ cao tâm kẹp nhỏ nhất so với sàn (lift 0,45 m) | 533 | [tính] |

| Dưới mặt bàn (robot đứng cạnh bàn, tay chĩa xuống) | mm | Nhãn |
|---|---:|---|
| Sâu nhất dưới mặt bàn 850 mm, lift 0,45 m | 317 | [ước] mặt bàn là giả định |
| Tầm ngang của điểm sâu nhất | 173 | [tính] |

Quét lưới 96043 tư thế (bước 3° ở vai, khuỷu, gập cổ tay) trong giới hạn khớp và giới hạn ghép, bằng `faibotic.kinematics.arm_fk`, rồi bỏ các tư thế vi phạm vùng cấm của SDK (`limits.keepout_violations`, cấu hình di động, lift 0,45 m; riêng dòng lift 0 áp vùng cấm ở lift 0): 14 % tư thế bị loại. Vùng cấm gồm đầu camera, cột camera, mặt bệ, mặt khay. Chưa tính va chạm với bàn lab và thân robot ngoài các vùng đó. Số 576 mm là cực đại của quét (gập cổ tay −7,5°); bảng tải ở mục Tải dùng tay duỗi thẳng (0,57 m). Cả hai cùng tính từ trục pan, không phải từ trục vai (khoảng 516 mm).
<!-- /sinh:tamvoi -->

## Servo

Mỗi khớp một servo, 18 servo (7 STS3095 và 11 STS3215), tất cả chạy 12 V. Yaw cổ tay là servo ID 21 (tay trái) và 22 (tay phải). ID gán theo bảng này khi lắp, xem thêm `docs/vi/assembly-guide.md`.

<!-- sinh:servo -->
| Khớp | Servo | ID tay trái | ID tay phải | Điện áp | Mô-men chặn (kg·cm) |
|---|---|---:|---:|---:|---:|
| Xoay đế tay (pan) | STS3095-C002 | 1 | 7 | 12 V | 95 |
| Vai | STS3095-C002 | 2 | 8 | 12 V | 95 |
| Khuỷu | STS3095-C002 | 3 | 9 | 12 V | 95 |
| Gập cổ tay | STS3215-C018 | 4 | 10 | 12 V | 30 |
| Yaw cổ tay | STS3215-C018 | 21 | 22 | 12 V | 30 |
| Cuộn cổ tay | STS3215-C018 | 5 | 11 | 12 V | 30 |
| Kẹp | STS3215-C018 | 6 | 12 | 12 V | 30 |
| Bánh omni 0 | STS3215-C018 | 13 |  | 12 V | 30 |
| Bánh omni 1 | STS3215-C018 | 14 |  | 12 V | 30 |
| Bánh omni 2 | STS3215-C018 | 15 |  | 12 V | 30 |
| Trục nâng | STS3095-C002 | 16 |  | 12 V | 95 |

| Loại | Số servo | Chặn (kg·cm) | Định mức (kg·cm) | Dòng chặn ở 12 V (A) | Khối lượng (g) |
|---|---:|---:|---:|---:|---:|
| STS3095-C002 | 7 | 95 | 22,5 | 9,8 | 187,5 |
| STS3215-C018 | 11 | 30 | chờ EVT (hãng chưa công bố) | 2,7 | 70,5 |

| Bus | ID | Vị trí |
|---|---|---|
| BUS_B | 13, 14, 15, 16 | đế: 3 bánh và trục nâng |
| BUS_L | 1, 2, 3, 4, 5, 6, 21 | trạm, tay trái |
| BUS_R | 7, 8, 9, 10, 11, 12, 22 | trạm, tay phải |
<!-- /sinh:servo -->

## Kẹp

<!-- sinh:kep -->
|  |  |
|---|---|
| Loại | Kẹp một hàm động, một ngón cố định (kẹp gốc của tay AM-ARM200) |
| Độ mở lớn nhất giữa hai đầu ngón | 97,25 mm [tính] |
| Khe khi khép | 7,7 mm giữa tâm hai đầu ngón khi chạm [tính] |
| Góc mở hàm | 64° (mở mềm) [tính] |
| Lực đầu ngón khi servo chặn | 35 N [tính]; lực thực chờ EVT |
| Khối lượng cả kẹp, có servo, chưa camera | 173 g [tính, khối đặc] |
| Camera | Camera USB 720p trên giá cổ tay, nhìn xuống phía ngón |
| Hộp, bề rộng cạnh kẹp, khuyến nghị tối đa | khoảng 78 mm [ước, 80 % độ mở] |
| Trụ, đường kính, khuyến nghị tối đa | khoảng 73 mm [ước, 75 % độ mở] |
| Cầu, đường kính, khuyến nghị tối đa | khoảng 63 mm [ước, 65 % độ mở] |
<!-- /sinh:kep -->

Vật khuyến nghị lấy theo độ mở ở trên, chừa biên để vật không trượt khỏi hai ngón. Kích thước vật tròn nhỏ hơn hộp vì hai ngón phẳng chỉ chạm vật tròn ở một điểm mỗi bên. Hệ số 80 %, 75 %, 65 % là [ước], chờ EVT để đối chiếu với vật thật.

## Tải

**Chưa đo.** Số dưới đây là mô-men tĩnh tính trên mô hình (trọng lực cả chuỗi tay cộng tải ở tâm kẹp, không tính gia tốc, ma sát, kéo cáp), so với ngưỡng 60 % mô-men chặn của servo. Cách tính đầy đủ nằm ở `docs/M1-LITE/tai-cong-bo-PL.md`.

Câu công bố (sinh bằng `scripts/tinh_tai.py` từ cùng mô hình, chưa đo trên robot thật):

> Tải đỉnh 2,0 kg ở tầm 0,25 m (gần thân, kẹp chúi xuống) [tính]; tải liên tục 0 kg ở tầm với tối đa 0,57 m [tính]; vật khuyến nghị ≤ 0,05 kg ở tầm 0,45 m (kẹp ngang) [tính]. Chưa đo trên robot thật.

Đỉnh theo tầm với, mỗi tay:

<!-- sinh:tai -->
| Tư thế | Tầm tâm kẹp từ trục pan (m) | Tâm kẹp so với trục vai (m) | Đỉnh, khối đặc (kg) [tính] | Đỉnh, ước tính in (kg) [ước] | Khớp giới hạn |
|---|---:|---:|---:|---:|---|
| duỗi hết | 0,57 | 0,00 | 0,46 | 0,52 | vai |
| kẹp ngang | 0,45 | 0,00 | 0,78 | 0,78 | vai/gập |
| kẹp ngang | 0,50 | 0,00 | 0,72 | 0,77 | vai |
| kẹp ngang | 0,55 | 0,00 | 0,54 | 0,60 | vai |
| kẹp ngang | 0,40 | -0,15 | 0,78 | 0,78 | vai/gập |
| kẹp ngang | 0,45 | -0,15 | 0,78 | 0,78 | vai/gập |
| kẹp ngang | 0,50 | -0,15 | 0,66 | 0,72 | vai |
| kẹp chúi xuống | 0,25 | -0,20 | 2,05 | 2,14 | vai |
| kẹp chúi xuống | 0,30 | -0,20 | 1,42 | 1,51 | vai |
| kẹp chúi xuống | 0,35 | -0,20 | 1,00 | 1,10 | vai |
| kẹp chúi xuống | 0,25 | -0,30 | 1,90 | 2,02 | vai |
| kẹp chúi xuống | 0,30 | -0,30 | 1,34 | 1,45 | vai |
<!-- /sinh:tai -->

- **Tải liên tục 0 kg ở tầm với tối đa 0,57 m [tính]; đo nhiệt và dòng ở EVT.** Riêng tay duỗi hết không tải đã cần 31,0 kg·cm ở vai, so với định mức 22,5 kg·cm của servo vai [tính]. Chỉ khi kẹp chúi xuống sát thân (0,33 kg ở tầm 0,25 m, 0,07 kg ở tầm 0,30 m, tâm kẹp thấp hơn trục vai 0,20 m) hoặc kẹp ngang ở tầm 0,40–0,45 m (0,10–0,11 kg) mới có tải liên tục lớn hơn 0 trên mô hình [tính].
- Không ghi con số tải nào không kèm tầm với.
- Vật khuyến nghị lấy nhỏ hơn của tải đỉnh và tải liên tục ở tầm làm việc điển hình (kẹp ngang, tâm kẹp ngang trục vai, tầm 0,45 m), làm tròn xuống 0,05 kg [tính].
- Định mức liên tục của servo cổ tay, cuộn, kẹp: chờ EVT (hãng chưa công bố).

## Đế và trục nâng

<!-- sinh:denang -->
| Hạng mục | Giá trị | Nhãn và nguồn |
|---|---|---|
| Tốc độ đế lớn nhất (tịnh tiến) | 0,40 m/s | giới hạn đặt trong SDK [đặt]; chờ EVT |
| Tốc độ xoay đế lớn nhất | 0,25 rad/s (14 °/s) | giới hạn đặt trong SDK để giữ chống lật [đặt]; chờ EVT |
| Dốc tăng và giảm tốc tịnh tiến | 0,05 m/s² | giới hạn đặt trong SDK [đặt]; chờ EVT |
| Bánh omni | 101,6 mm đường kính, 3 bánh, đặt cách tâm 250 mm | [tính] |
| Hành trình trục nâng | 450 mm | [tính] |
| Tốc độ trục nâng | 14,5 mm/s | [ước] giữ số cũ tới khi đo; thanh răng cho phép nhanh hơn |
| Thời gian đi hết hành trình | 31 s | [ước] suy từ hai số trên |
| Bước răng thanh răng (khoảng nấc chốt) | 4,71 mm | [ước] mô-đun răng giả định 1,5 |
| Tải thiết kế treo trên bánh răng trục nâng | 8,28 kg | [ước] xe nâng + trạm + hai tay + tải 0,5 kg mỗi tay; chờ cân |
| Khối lượng đối trọng thép | 8,23 kg | [tính] |
| Khối lượng trụ nâng | 2,23 kg | [ước] PETG, infill giả định |
| Khối lượng xe nâng | 0,915 kg | [tính] |
| Khối lượng trạm, chưa tay | 2,094 kg | [ước] CAD và giả định |
| Khối lượng phần quay theo pan, mỗi tay | 1,69 kg | [tính] cận trên, khối đặc |
| Khối lượng mỗi tay theo CAD, đủ 7 servo | 2,13 kg | [tính] chi tiết in khối đặc, servo theo datasheet |
| Khối lượng cả robot theo CAD | 21,2 kg | [tính] cận trên; cân thật chờ EVT |
<!-- /sinh:denang -->

- **Chống lật:** hệ số 1,638 tính tĩnh [tính]; 1,503 khi tính động với dốc gia tốc của đế (tốc độ ≤ 0,40 m/s, quay ≤ 0,25 rad/s, dốc 0,05 m/s²) [tính]. Mục tiêu ≥ 1,5. Đối trọng thép là phần giữ hệ số này, nên không tháo.
- Trục nâng có chốt hãm cơ khí: mất điện thì xe nâng tụt tối đa một nấc răng rồi tựa chốt [ước]. Thử rơi: chờ EVT.
- Khối lượng cả robot ≈ 21,2 kg theo CAD (thân 16,9 kg, mỗi tay 2,13 kg gồm 7 servo), cận trên vì chi tiết in tính khối đặc [tính]. Cân thật: chờ EVT.
- Kích thước ngoài toàn bộ: chờ EVT.

## Khay đế

Khay trên đế, trước trụ nâng, là sàn chở máy tính: một laptop (≤ 2,5 kg, tới 16 inch) **hoặc** một Jetson (Orin Nano/NX, AGX Orin) kèm bộ DC-DC, cộng phụ kiện khi di chuyển (E-stop rời, sạc, cáp). Tay robot không đặt vật lên khay. Không ngồi, không đứng lên khay.

| Hạng mục | Giá trị | Nhãn |
|---|---|---|
| Tải định mức | 3 kg phân bố; thử 4,5 kg, võng ≤ 2 mm | [đặt]; thử thật chờ EVT |
| Võng tính ở 4,5 kg | 0,45 mm ở góc trước | [tính] dầm công-xôn, PETG in E 1,5 GPa |
| Mặt khay | x 64–320 mm trước tâm đế, rộng 380 mm, cao 210 mm | [tính] |
| Giữ đồ | gờ 4 cạnh cao 10 mm, 2 dây đai qua khe sẵn, 4 dải cao su chống trượt | [đặt] |
| Vùng tâm vật cho phép | x 80–200 mm, lệch bên ≤ 40 mm (dồn giữa, sát gờ sau) | [tính] theo chống lật |
| Thông gió | khe hở 32 % dưới vùng laptop; ô quạt 80 mm cho Jetson | [tính] |
| Gá Jetson | lỗ VESA 75 và 100 (heat-set M4), lỗ carrier Orin Nano (heat-set M3) | [đặt]; lỗ carrier chờ đối chiếu bản vẽ hãng |
| Gá lên đế | trượt lên thanh nhôm 2040, vặn 2 núm M5, không dụng cụ | [đặt] |
| Khối lượng khay | ~0,45 kg, in PETG thành 4 mảnh | [ước] |

**Chống lật khi có vật trên khay** (tay theo CAD, hai tay duỗi ngang mang 1 kg mỗi tay, lift ở đỉnh, vật ở góc xấu nhất của vùng cho phép) [tính]:

| Vật trên khay | Tĩnh | Động (dốc đế mặc định) |
|---|---:|---:|
| Khay trống | 1,681 | 1,595 |
| Laptop 2,5 kg | 1,657 | 1,568 |
| Jetson AGX Orin + DC-DC 1,25 kg | 1,669 | 1,581 |
| 3 kg (định mức) | 1,652 | 1,563 |
| Laptop 2,5 kg lệch trước và lệch bên (x 270, y 80 mm): **ngoài vùng** | 1,497 | 1,424 |

- Vật nặng đặt giữa và lùi sau thì giúp chống lật; đẩy ra trước hoặc sang bên thì hại. Vì vậy SDK chặn khai báo ngoài vùng.
- **Khai báo trong SDK:** `Robot("feetech", khay=KhayTai("laptop", kg=2.2, x=0.19))` hoặc `robot.set_khay_tai(...)`; `None` khi khay trống. Khai báo ngoài vùng hoặc quá 3 kg báo lỗi `CFG-02`. Khi có vật, lift không xuống dưới 40 mm (laptop mở nắp: khoảng 226 mm, để màn hình không chạm xe nâng) và tâm kẹp không xuống dưới đỉnh vật + 3 cm.
- **Laptop mở nắp:** nắp mở không quá ~95°, vì chân trụ nằm ngay sau mép khay.

## Điện

- **Pin:** 12 V danh định (3S, đầy 12,6 V), 3S4P, 12 cell 21700, 172,8 Wh [tính, giả định cell 4 Ah], có BMS 3S 100 A.
- **Thời gian chạy pin: chờ EVT.** Với mục tiêu chạy 2 giờ, dòng trung bình phải ≤ 6,62 A [ước, hệ số dùng được 0,85]; chưa có số đo teleop.
- **Dòng chặn:** xem bảng dưới. Cầu chì mỗi tay chọn theo dòng chặn của tay đó.
- **Ngân sách dòng:** firmware giới hạn tổng dòng 56,1 A; mạch có 12 cầu chì 3 A cho các nhánh servo và phụ tải, 204 sợi dây trong bảng đi dây.

<!-- sinh:dien -->
| Hạng mục | Giá trị | Nhãn và nguồn |
|---|---|---|
| Dòng chặn tổng 18 servo ở 12 V | 98,3 A | [tính] cộng dòng chặn từng servo; xảy ra khi mọi servo kẹt cùng lúc |
| Dòng chặn mỗi tay (7 servo) | 40,2 A | [tính] qua một cầu chì và một cặp tiếp điểm giắc trạm |
| Dòng chặn nhánh đế (3 bánh + trục nâng) | 17,9 A | [tính] |
<!-- /sinh:dien -->

- **Dừng khẩn (E-stop):** nút 2 tiếp điểm NC nối tiếp trong mạch cuộn contactor 100 A. Nhấn nút thì contactor nhả và servo mất nguồn. Nhả nút **không** tự chạy lại: phải nhấn Reset.
- **Vòng E-stop qua trạm:** trạm hai tay có E-stop riêng ở tai trái. Vòng dừng đi qua giắc nối trạm và hai công tắc khoá. Mở một khoá, tháo trạm hoặc nhấn E-stop đều làm contactor nhả.
- **Khoá trạm (DOCK):** hai khoá kéo giữ trạm với xe nâng; mỗi khoá có công tắc báo đóng. Thiếu một khoá thì servo không có điện và SDK từ chối `enable()`.
- **Camera và hub USB** lấy nguồn 12V_AUX riêng, không qua contactor, nên vẫn hoạt động khi E-stop.
- **Nắp giắc:** nửa giắc trên xe luôn có điện ở hai chân khi tháo trạm. Dùng **nắp che thường ngày** (không cầu nối) khi cất xe hoặc đổi trạm. **Nắp bảo trì** (có cầu nối bỏ qua vòng trạm) chỉ kỹ thuật viên dùng, không để trên xe.
- **Chưa cấp pin thật trước khi qua kiểm tra chất lượng:** mọi lần cấp điện trước đó dùng nguồn bàn 12 V có giới hạn dòng. Xem `product/06-lap-rap.md` mục "Cấp điện lần đầu".

## Phần mềm

- **SDK Python `faibotic`:** điều khiển 18 servo qua 3 bus (BUS_L, BUS_R trên trạm; BUS_B trên đế), đọc cùng một bộ thông số với mô phỏng.
- **Hai cấu hình:** `tram_ban` (trạm đặt trên bàn, không đế, không trục nâng) và `di_dong` (trạm trên xe). SDK tự nhận theo servo thấy được trên bus.
- **Mô phỏng:** MuJoCo, cùng khối lượng và giới hạn khớp với SDK; cảnh web dùng `web-assets/joints.json`.
- **Ba camera:** 1 camera đầu trên cột trạm, chúi mặc định 50° (chỉnh 20–70°) [đặt]; 2 camera cổ tay.
- **Vùng cấm quanh camera đầu:** khuỷu, cổ tay và tâm kẹp phải cách trục chúi camera đầu từ 0,14 m, và cách trục cột từ 0,13 m [tính, quét mesh]. Đã định nghĩa trong SDK, chưa nối vào lệnh điều khiển.
- **Tự hành, mặc định tính toán từ xa:** Raspberry Pi 5 trên robot lo servo, camera, E-stop và watchdog 200 ms; policy chạy trên laptop hoặc máy có GPU qua Wi-Fi/LAN (cầu policy-bridge, WSS). Mất nhịp quá 200 ms thì đế dừng. Laptop có thể đặt trên khay đế để teleop hoặc chạy policy tại chỗ.
- **Jetson trên robot (tuỳ chọn, lộ trình):** đặt trên khay đế, cấp nguồn qua nhánh DC-DC có cầu chì riêng, không qua contactor servo. Không nằm trong bộ chuẩn.
- **Thu dữ liệu:** plugin LeRobot, ghi LeRobotDataset v3, chạy cùng một kịch bản trên mô phỏng và robot thật.
- SDK từ chối lệnh trục nâng khi nguồn chưa đủ, trừ khi bật cờ thử có người giám sát (`allow_lift_underpowered`). Chạy đế không có trạm cần cờ `allow_no_dock=True`.

## Nguồn số liệu và việc còn mở

Số trong tài liệu này đến từ `sdk/faibotic/spec.py`, `web-assets/joints.json`, `cad/export/AM_gripper_PL.json`, `docs/M1-LITE/tai-cong-bo-PL.md`, `product/00-tong-quan.md`, `product/01-SoR.md`; `scripts/kiem_cheo.py` đối chiếu các nguồn này với nhau; các số gõ tay ngoài khối sinh (tải liên tục, pin, thời gian lắp) chưa có phép kiểm tự động.

Chờ EVT: tải liên tục; thời gian chạy pin; kích thước ngoài; khối lượng thật; tốc độ đế và trục nâng đo được; độ rơ và độ lặp lại; dòng và nhiệt độ servo khi giữ tải; thời gian cắt của E-stop; độ cứng của tay in.
