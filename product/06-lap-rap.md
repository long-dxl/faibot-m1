# Lắp ráp Faibot M1 v0.5, cấu hình P-L

Thứ tự lắp một robot P-L, từ đế tới lần cấp điện đầu. Bản hiện tại dựng từ CAD nháp và bảng dây, **chưa lắp thử bộ #0**. Mọi mô-men siết đều chưa có nguồn trong repo, nên mỗi chỗ ghi "đo và ghi": dùng cờ lê lực, siết tăng dần, ghi số vào phiếu IPQC (trụ nâng, đối trọng thép 8 kg và khoá DOCK là chỗ không được siết theo cảm giác). Số chốt sau bộ #0.

Từ S2 hai tay không còn bắt lên khung T hai bên. Chúng nằm trên một **trạm** tháo lắp được, gắn lên xe nâng bằng hai chốt côn và hai khoá kéo (mục 4, 4A, 9A). Bộ đế bàn là phần bán riêng (mục 9B).

Không đấu pin thật trước khi qua cổng G-EVT-điện (`docs/M1-LITE/R2-plan.md` mục 7.6). Mọi lần cấp điện trước đó dùng nguồn bàn có giới hạn dòng.

## Nguồn và quy ước

Ảnh từng bước (`media/lap-rap/buoc-NN.png`, tạo bằng `cad/faibotic/render_buoc_lap.py` từ `assembly_pl.py`): phần xám là đã lắp, phần xanh là phần thêm ở bước đó, cùng một góc nhìn. Ảnh đánh số theo chuỗi lắp cả robot (khay ở cuối), không theo số mục. Chuỗi đầy đủ: đế (01), bánh (02), trụ (03), xe nâng (04), trạm (05), tay trái (06), tay phải (07), kẹp (08), camera (09), khay (10). Ảnh là CAD, chưa phải ảnh bộ #0.

| Cần gì | Ở đâu |
|---|---|
| Cấu hình | `docs/M1-LITE/R2-plan.md` mục 2.8 |
| Mã chi tiết | `product/parts-register.csv` |
| Chi tiết in | `product/danh-sach-in.md`, `product/danh-sach-in.csv` |
| Từng sợi dây (mã, AWG, chiều dài cắt, đầu cos, cầu chì, vòng dịch vụ) | `electrical/bang-day.csv`. Tài liệu này chỉ nêu mã dây, không chép lại bảng |
| Sơ đồ chuỗi L, bảng trạng thái E-stop | `electrical/bang-di-day-evt.md` mục L.1–L.4 |
| Vật tư dây, đầu cos | `electrical/vat-tu-day.csv` |
| Thân, đế, trụ (CAD) | `cad/faibotic/assembly_pl.py` |
| Trạm hai tay, đế ngàm, đế bàn (CAD) | `cad/faibotic/tram_hai_tay.py`, `cad/faibotic/tram_de.py`, `docs/M1-TRAM/R3b-thiet-ke-tram.md`, `docs/M1-TRAM/S2-build-tram.md` |
| Giắc 21W4, vòng E-stop trạm | `electrical/bang-day.csv` (E01–E11, GD1–GD4, AX01–AX06), `electrical/bang-day-de-ban.csv`, DK-N55, DK-N56 |
| Kẹp | `cad/upstream/AM-ARM/am-arm200/stp/follower/` (OB_F_J7_Gripper, OB_F_J7_Jaw, OB_F_Camera); hướng dẫn gốc bước 5 và 6 |
| Hướng dẫn gốc | `cad/upstream/AM-ARM/am-arm200/hardware_assembly.md`, `cad/upstream/AlohaMini/AlohaMini2/docs/assembly_guide.md` |
| Phiếu kiểm | `product/phieu-kiem/IQC.md`, `IPQC.md`, `FQC.md` |

Mã IPQC ở cuối mỗi bước là dòng cần điền trong `product/phieu-kiem/IPQC.md`. Hình: chỉ trỏ tới ảnh P-L đã có trong repo. Bước chưa có ảnh ghi "chưa có hình", chụp ở bộ #0.

Chưa chốt, ảnh hưởng tới lắp:

| Mục | Ảnh hưởng | Điều kiện mở |
|---|---|---|
| Đối trọng thép ~8 kg (Q8 đã chốt) cho hệ số lật ≥ 1,5 (khay 1 kg ở góc xa); 5 kg chỉ cho 1,15 | Tấm thép M1-BS-032 | DK-N40 (kiểm CAD + thử nghiêng EVT) |
| Giới hạn khớp P-L chưa quét va chạm riêng; giá J5 chạm khuỷu J4 khi gập cổ tay từ khoảng 30° trong CAD | Chưa nạp giới hạn mềm vào EEPROM | DK-N38 |
| Mối Post3–Post4 của trụ chưa kiểm được trong CAD | Kiểm khe bằng mắt khi dán keo trụ | DK-N39 |
| Bề rộng trạm 0,614 m (CAD, đế J1 thật 138,6 mm) so với khổ 0,60 m | Khổ qua cửa lab; có thể phải đổi ray | DK-N52 (chốt) |
| Giắc 21W4: định mức 40 A, điện trở tiếp điểm, bố trí hàng chân, thứ tự tiếp xúc khi rút chưa có datasheet; USB tốc độ cao qua chân tín hiệu chưa thử | Chưa hàn dây vào giắc thật khi chưa đối chiếu bản vẽ hãng; thử USB ở EVT | DK-N55, DK-N45 |
| Khoá kéo 100 kg: tải thật chỉ có trang bên bán | Hệ số an toàn khoá ~9 chỉ đúng nếu khoá chịu đủ | IQC Q-IQC-44 |
| Servo trục nâng chưa lọt hốc khung T giữa trong CAD; chốt, công tắc chỉ là khối bao; chưa có chặn cứng hai đầu; bánh răng 20 răng chưa thử ăn khớp | Lắp thử, đo, ghi | DK-N41, DK-N18 |
| Gá servo trục nâng chạm trụ trong CAD giữ chỗ | Chặn in thử xe nâng | DK-N51 |
| Tải công bố | Không thử tải trên mức đã công bố | Q7 |
| Bánh omni: CAD dùng 4 in (101,6 mm), BOM A05 ghi 127 mm | Giá servo bánh và cao khay đổi theo bánh | chưa chốt |
| Vỏ pin 12 cell | Chưa có vỏ đúng pack 3S4P | DK-N7 |

## Dụng cụ chung

- Lục giác và tua vít cho ốc M2, M3, M4, M5, M6; cờ lê hoặc tuýp M8 cho cọc thanh cái.
- Cờ lê lực hoặc tua vít lực (để đo và ghi mô-men siết).
- Mỏ hàn có đầu ép hạt chèn nhiệt M3.
- Keo epoxy AB (BOM N12).
- Kìm tuốt dây, kìm bấm cos cho đầu SC10-6 (8 AWG), SC25/SC35 (dây 2 và 4 AWG), kìm bấm đầu 5264, đầu pin E7508, đầu Dupont; súng khò ống co nhiệt.
- Đồng hồ vạn năng, kẹp dòng DC, nguồn bàn 12 V có giới hạn dòng.
- Thước kẹp, thước đo góc, cân treo (cho FQC).
- Laptop có bo bus servo để gán ID (hoặc Pi 5 của robot).

## 0. Chuẩn bị

1. Hàng về: làm IQC (`product/phieu-kiem/IQC.md`).
2. In chi tiết theo `product/danh-sach-in.md`.
3. Dán nhãn từng servo theo vị trí (bước 1 của hướng dẫn follower AM-ARM200).
4. Gán ID từng servo, mỗi lần một servo trên bus, baud 1.000.000 (như hướng dẫn upstream). ID theo `sdk/faibotic/spec.py` `SERVO_IDS`:

| Vị trí | Loại | ID |
|---|---|---:|
| left_pan, left_shoulder, left_elbow | STS3095 C002 12 V | 1, 2, 3 |
| left_wrist_flex, left_wrist_yaw, left_wrist_roll, left_gripper | STS3215 C018 12 V | 4, 21, 5, 6 |
| right_pan, right_shoulder, right_elbow | STS3095 C002 12 V | 7, 8, 9 |
| right_wrist_flex, right_wrist_yaw, right_wrist_roll, right_gripper | STS3215 C018 12 V | 10, 22, 11, 12 |
| wheel_0, wheel_1, wheel_2 | STS3215 C018 12 V | 13, 14, 15 |
| lift | STS3095 C002 12 V | 16 |

Hướng dẫn AlohaMini2 dùng ID khác (bánh 8, 9, 10; trục nâng 11). Robot này theo bảng trên. Tay P-L là 6+1 như AM-ARM200 gốc: servo yaw cổ tay ID 21 (trái) và 22 (phải), 18 servo tổng cộng.

Kiểm: IPQC-01.

## 1. Đế

Hình: `media/qc/g2a/34_lift0_ready.webp`, `media/qc/g2a/side_lift0_ready.webp`.

![Bước 01: đế, nan, sàn, đối trọng](../media/lap-rap/buoc-01.png)

![Bước 02: bánh omni, servo bánh, giá servo](../media/lap-rap/buoc-02.png)

![Bước 10: khay và cột đỡ khay (bước 6 của mục này, ảnh cuối chuỗi)](../media/lap-rap/buoc-10.png)

| | |
|---|---|
| Chi tiết | Tấm thép đối trọng M1-BS-032 (đồng thời là moay-ơ và đế trụ); 3 nan nhôm 20 x 40 M1-BS-031 (1 x 128 mm, 2 x 93 mm); 3 giá servo bánh M1-BS-036 (vách + nắp); 3 servo STS3215 M1-SV-001 (ID 13–15); 3 bánh omni M1-BS-035; sàn M1-BS-037; khay đế M1-ST-012 (4 mảnh in, thay M1-ST-010 theo ), cột đỡ M1-ST-011 cắt ngắn, 2040 190 mm (phần dư T05a), bộ K01 (dây đai, cao su chống trượt), K02 (2 núm M5, 2 đai ốc T, heat-set M3 ×4, M4 ×8) |
| Ốc | M1-BS-033 (M3/M4/M5/M6, đai ốc trượt rãnh 20). Cỡ ốc từng mối chưa chốt: CAD chưa vẽ lỗ bắt |
| Dụng cụ | Lục giác, cờ lê lực |

1. Đặt tấm thép nằm phẳng, lưỡi trước hướng về phía trước robot.
2. Bắt 3 nan lên tấm thép theo góc 0°, 120°, 240°. Nan dài 128 mm nằm ở 0° (ra trước, trên lưỡi tấm).
3. Lắp servo bánh vào giá in ở đầu mỗi nan: vách sau bắt vào đầu nan, nắp ôm thân servo. Servo 13 ở nan 0° (trước), 14 ở 120°, 15 ở 240° (theo thứ tự bánh trong CAD; đối chiếu chiều quay với SDK ở bước 10).
4. Lắp bánh omni thẳng lên còi servo, kiểu LeKiwi (không có gối đỡ ngoài). Vít bắt bánh vào còi: chưa có hướng dẫn trong repo, theo lỗ của bánh mua về.
5. Lắp sàn in lên trên nan, chừa lỗ quanh đế trụ. Sàn in nhiều mảnh; cách nối mảnh chưa vẽ.
6. Lắp cột đỡ khay M1-ST-011 (cắt ngắn, đỉnh 187,6 mm) trên nan trước; bắt thanh 2040 190 mm nằm ngang lên đỉnh cột bằng 2 ốc M5.
6a. Ghép khay M1-ST-012: 4 mảnh nối mộng đuôi én + vít M3 qua gân; ép heat-set (M4 lỗ VESA, M3 lỗ carrier) nếu khách dùng Jetson; dán 4 dải cao su; luồn 2 dây đai qua khe.
6b. Đặt khay lên 2040 (4 vấu ôm hai mặt bên thanh), thả 2 đai ốc T vào rãnh trên, vặn 2 núm M5. Mặt khay cao 0,21 m so sàn nhà (CAD). Tháo/lắp không dụng cụ, mục tiêu ≤ 30 s.
   **Cảnh báo chống lật:** khay chỉ chở một laptop ≤ 2,5 kg HOẶC Jetson + DC-DC, cộng phụ kiện, tổng ≤ 3 kg; tay không đặt vật lên khay. Vật nặng đặt giữa, tựa gờ sau (tâm x 0,08–0,20 m, |y| ≤ 0,04 m), laptop nằm ngang, luôn cài 2 dây đai. Khai báo trong SDK (`KhayTai`); vật lệch trước + bên (x 0,27, y 0,08) cho lật động 1,42 < 1,5. Laptop mở nắp: nắp ≤ ~95° (chân trụ sau khay), khai báo `cao=0.26`, lift ≥ ~0,226 m.
7. Đi trước 3 dây nguồn servo bánh (P.W1+/−, P.W2+/−, P.W3+/−) và chuỗi DATA (D.WHEEL_0, D.WHEEL_1, D.WHEEL_2) dọc nan về tâm đế. Chưa cắm.

Kiểm (IPQC-02):
- Mô-men siết ốc nan, giá servo, bánh: đo và ghi.
- Bánh quay tay trơn, con lăn không kẹt.
- Ba bánh chạm sàn cùng lúc trên mặt phẳng (đo khe hở dưới bánh nếu có).
- Cân đế: đo và ghi (theo thép ~8 kg).
- Khay đế: tháo/lắp không dụng cụ ≤ 30 s; ấn tay góc trước khay không rơ, không kêu.

## 2. Mô-đun nguồn và E-stop

Hình: `electrical/so-do-khoi.png`, `electrical/di-day.png`.

Mạch theo chuỗi L (`electrical/bang-di-day-evt.md` mục L.1). Bước này đi dây khi hệ thống chưa nối pin: giắc J0 để rời. Bước 1 và 3 làm việc với pack thật: cell chưa sạc đầy, cất nơi an toàn; bọc cách điện đầu cực và từng đầu dây cân bằng ngay khi lắp; không nối pack vào J0 hay tải nào cho tới khi qua kiểm nguội và FQC.

| | |
|---|---|
| Chi tiết | Pack 3S4P (M1-PW-001, 12 cell), BMS Daly 3S 100 A (M1-PW-002), F1 MEGA 80 A (M1-PW-003), giắc J0 Anderson SB120 (BOM N21), contactor K1 100 A cuộn 12 V (M1-PW-004), F3/F4 MEGA 40 A + đế (BOM N18, hạ từ 60 A), 3 cọc M8 BAT_F, SERVO+, GND (COC-M8), 4 aptomat DC FPI C6, FCTRL C4, FCHG C6, FAUX C2 (12V_AUX) trên ray DIN (BOM N23), buck TPS54560 5 A (BOM V04), rơ-le tự giữ KH JQ1P (BOM N04), module rơ-le K5, module opto 4 kênh, 2 domino TB1/TB2, diode 1N5408 và zener 1N5351B ở cuộn K1, 5 đế cầu chì inline FHAC (FW1–3 ATO 3 A, FLIFT 7,5 A, FLAT 2 A), cổng sạc XT60E-F, nút E-stop LA38P-11ZS 2NC (BOM V05), nút Reset LA38-11BN + khối NO thêm (BOM N03) |
| Dây | Trục chính BW00–BW06, BW04R; nhánh tay BW07A, BW08A (đầu phía SERVO+); Q.PI.i, PW01–PW04; Q.CTRL.i, C01–C20; Q.CHG.i, CH01, CH02; Q.AUX.i (12V_AUX, đi tiếp ở mục 4A); BL0–BL3; FH.W1.i/o–FH.W3.i/o; FH.LIFT.i, FH.LAT.i; P.W1–P.W3; G01–G08 |
| Đầu cos | SC35-8 (2 AWG), SC25-8 và SC25-6 (4 AWG), RV5.5-8, PTV1.25-18, E7508, RV1.25-4 theo cột `dau_tu`, `dau_den` của `bang-day.csv` |
| Dụng cụ | Kìm bấm cos, súng khò, cờ lê M8, đồng hồ vạn năng |

1. Vỏ pin: đặt pack, BMS, F1 và J0 trong vỏ. Vỏ đúng cho 12 cell **chưa chốt (DK-N7)**; bộ #0 dùng vỏ tạm, ghi lại vỏ đã dùng.
2. F1 sát cực + của pack, đoạn BW01 ≤ 10 cm, có boot cách điện (đoạn duy nhất không có cầu chì phía trước).
3. Hàn dây cân bằng BL0–BL3 từ tap pack vào BMS.
4. Tấm điện trên đế: bắt 3 cọc M8, K1, F3/F4, ray DIN với 3 aptomat, buck, K5, opto, KH, TB1/TB2.
5. Đi trục chính theo thứ tự mã: BW03 (J0 → BAT_F), BW04R (J0 → GND), BW05 (BAT_F → K1), BW06 (K1 → SERVO+), BW07A và BW08A (SERVO+ → F3, F4).
6. Nhánh BAT_F: Q.PI.i → FPI → PW01/PW02 → buck → PW03/PW04 (cáp USB-C cắt một đầu, chỉnh buck về 5 V trước khi nối Pi); Q.CTRL.i → FCTRL → C01; Q.CHG.i → FCHG → CH01/CH02 → cổng sạc.
7. Mạch dừng C02–C20: E-stop và Reset nằm trong hộp nút ở đỉnh cột (C02 đi từ tấm điện lên đỉnh cột). Hai tiếp điểm NC của E-stop nối tiếp (C03). Diode và zener song song cuộn K1.
8. Bánh: FH.W1.i–FH.W3.i từ cọc SERVO+ qua đế cầu chì inline, rồi P.W1–P.W3 tới servo bánh. Chưa cắm servo.
9. Dán nhãn mã dây ở hai đầu mỗi sợi.

Kiểm (IPQC-03):
- Mô-men siết cọc M8 và cos: đo và ghi.
- Lực kéo thử mỗi đầu cos bấm: kéo tay, không tuột; ghi.
- Thông mạch từng sợi theo `bang-day.csv`; không chập BAT_F, SERVO+ với GND.
- E-stop chưa nhấn: hai NC thông; nhấn: hở.
- J0 rời: BAT_F không có điện.

## 3. Trụ nâng, thanh răng, chốt hãm, công tắc

Hình: `media/qc/g2a/front_lift0_ready.webp`, `media/qc/g2a/front_lift290_straight.webp`.

![Bước 03: trụ nâng](../media/lap-rap/buoc-03.png)

![Bước 04: xe nâng, bánh răng, chốt hãm, công tắc](../media/lap-rap/buoc-04.png)

| | |
|---|---|
| Chi tiết | Đế nối trụ M1-LF-055; trụ M1-LF-054, 053, 052, 051 (ba đoạn trên có thanh răng); khung T giữa M1-LF-057 với 8 vòng bi 624ZZ (BOM V09); bánh răng 20 răng M1-LF-002 (thay bản cũ); chốt Φ12 x 25 M1-LF-004 (BOM N10); servo STS3095 trục nâng (ID 16, bus BUS_B); solenoid chốt M1-LF-050 (EML-0530B, BOM N09) trên giá in M1-LF-060; đầu chốt và 2 công tắc hành trình M1-LF-041; công tắc báo chốt rút M1-LF-061 (KW12-N, BOM N08); rơ-le K10 |
| Ốc | 4 x M3 x 10 bắt bánh răng vào đĩa ra servo (hướng dẫn AlohaMini2 bước 4); ốc bắt đế nối trụ xuống tấm thép: cỡ chưa chốt |
| Dây | FH.LIFT.i, P.LIFT+, P.LIFT- (nguồn servo, 16 AWG); D.LIFT (DATA, lên BUS_B); L01, L03, L04 (cuộn chốt); G09–G12 (công tắc dưới, trên); G13, G13V, G13G (K10); G15, G16 (cảm biến chốt) |
| Dụng cụ | Keo epoxy, lục giác, cờ lê lực |

1. Bắt đế nối trụ xuống mặt trên tấm thép.
2. Dựng trụ theo thứ tự upstream: đế nối → Post4 → Post3 → Post2 → Post1. Bôi epoxy mỗi mặt tiếp xúc, lau keo thừa trước khi khô. Hướng thanh răng theo CAD (`cad/export/AlohaMini2_placement.json`), phía chốt hãm. Mối Post3–Post4 trong CAD chưa kiểm được (DK-N39): kiểm khe bằng mắt khi dán.
3. Ép 8 vòng bi 624ZZ vào khung T giữa. Vòng bi phải nằm phẳng mặt, quay trơn.
4. Bắt sẵn 4 vít M3 x 10 vào bánh răng, luồn chốt 12 x 25 từ bên phải, xoay bánh răng tới khi đầu vít lọt lỗ đĩa ra servo, siết qua cửa sổ bên. Servo trục nâng chưa lọt hốc trong CAD (DK-N41): thử lắp, ghi khe hoặc chỗ vướng.
5. Lắp thử khung T giữa. Đây là xe nâng, và là nơi bắt đế ngàm của trạm (mục 4A); không còn khung T trái, phải, xà ngang. Khi khớp mới dán epoxy. Để keo khô hẳn trước khi gắn trạm (upstream).
6. Luồn khung T giữa vào ray của trụ. Xe nâng phải trượt êm, không kẹt.
7. Lắp giá chốt, solenoid, đầu chốt, công tắc báo chốt rút trên mặt khung T giữa. Giá in chưa có lỗ bắt (DK-N41): bộ #0 khoan theo mẫu thật, ghi vị trí.
8. Lắp 2 công tắc hành trình dưới và trên khung T giữa, và đệm PU chặn hai đầu. Chặn cứng hai đầu chưa vẽ (DK-N41): bộ #0 tự làm chặn, ghi kích thước.
9. Đi dây FH.LIFT.i, P.LIFT±, D.LIFT, L01, L03, L04, G09–G16 theo bảng dây. P.LIFT±, D.LIFT, L03, L04, G15, G16 đi qua vòng dịch vụ trục nâng (mục 9). G13, G13V, G13G ngắn, đi trên tấm điện.

Kiểm (IPQC-04):
- Mô-men siết vít bánh răng: đo và ghi.
- Đẩy xe nâng bằng tay (servo chưa cấp điện) hết hành trình: êm, không kẹt; ghi hành trình đo. CAD: -160 … +290 mm so với điểm 0 của URDF.
- Bánh răng ăn khớp suốt thanh răng; ghi số răng bánh in được (thiết kế 20 răng).
- Cuộn chốt không điện: chốt bung, cài vào răng; xe nâng tựa chốt không trượt.
- Công tắc hành trình và công tắc chốt: đồng hồ đo đổi trạng thái khi nhấn tay.

## 4. Trạm hai tay: khung, gá vai, cột camera, hộp điện

Mục đích: dựng trạm, một khối tháo được mang hai tay, camera đầu, E-stop và hộp điện. Dựng và kiểm trên bàn, ngoài robot; xong mới gắn lên xe nâng (mục 4A). Trạm chưa có tay nặng khoảng 2,1 kg; có hai tay khoảng 6–6,4 kg [ước, CAD S2 và R3b]. Cân thật và ghi ở FQC-11.

Hình: ảnh CAD bên dưới; ảnh thật chụp ở bộ #0. CAD: `cad/faibotic/tram_hai_tay.py`.

![Bước 05: trạm hai tay](../media/lap-rap/buoc-05.png)

| | |
|---|---|
| Chi tiết khung | Ray trước nhôm 2040 dài 0,60 m (M1-TR-001); 2 tai 2040 dài 0,20 m (M1-TR-002); ke góc và con trượt T; 2 tay cầm in (M1-TR-014) ở đuôi tai |
| Gá vai | 2 gá đáy tay in (M1-TR-003), bắt lên rãnh T của ray |
| Bạc chốt, khoá | 2 khối bạc in (M1-TR-004) dưới hai tai, mỗi khối một bạc thép Ø12/Ø8 x 20 (M1-TR-004b): bên trái lỗ tròn H7, bên phải lỗ dẹt dọc trục y; 2 khoá kéo 100 kg có chốt an toàn (M1-TR-008); 2 công tắc KW12 báo khoá đóng (M1-TR-009, BOM N08) |
| Giắc, E-stop | Nửa đực giắc 21W4 (M1-TR-010) trên giá in dưới tai trái (M1-TR-011); hộp E-stop (M1-TR-012) ở mặt ngoài tai trái, nút 2NC (M1-TR-012b) |
| Camera đầu | Cầu in bắc qua cụm chốt hãm (M1-TR-005); cột 2020 dài khoảng 0,46 m (M1-TR-006); gá pan + tilt (M1-TR-007) với 2 núm khoá M5 và đệm ma sát (M1-TR-007b); camera USB (M1-CM-001, BOM A06) |
| Hộp điện trạm (M1-TR-013, ngoài tai phải) | 2 hộp cầu chì lưỡi FB-L, FB-R (BOM V07); 2 module ADuM3160 (BOM N22); 2 bo bus BUS_L, BUS_R (BOM A07, M1-EL-003); hub USB 2.0 7 cổng có nguồn 5 V; buck 12 V → 5 V |
| Dây | BW07S, BW07SN, BW08S, BW08SN (8 AWG, giắc 21W4 → hộp cầu chì); AX03, AX04, AX05, AX06 (12V_AUX → buck → hub); E02–E08 (vòng E-stop, công tắc khoá, DOCK); U01 (camera đầu → hub), U03B, U04, U04B, U05, U05B; GB.L, GB.R; dây nguồn và DATA hai tay (mục 5) |
| Dụng cụ | Lục giác M3 và M5, kìm bấm cos SC10-6 (8 AWG) và SC25-6, thước kẹp, thước đo góc |

1. Khung chữ U. Bắt hai tai vào mặt sau ray, ở y ±0,11 m so với giữa ray (tim cách nhau 0,22 m, ôm cột nâng với khe 0,05 m mỗi bên), đi về phía sau. Dùng ke góc và con trượt T. Kiểm vuông bằng ke.
2. Gá vai. Bắt hai gá đáy tay lên ray bằng con trượt T. Tâm hai gá cách nhau **0,475 m**, mỗi bên 0,2375 m so với giữa ray. Rãnh T cho dời ±25 mm nếu thử thật cần. Siết sơ, đo khoảng vai (IPQC-05A), rồi siết hẳn.
3. Bạc chốt. Ép bạc thép vào khối bạc in, trái lỗ tròn, phải lỗ dẹt. Bắt khối bạc dưới tai. Không dùng lỗ in làm bạc: lỗ in sai 0,2–0,3 mm và mòn. Hai chốt khác loại nên không lắp ngược trạm được.
4. Khoá kéo và công tắc. Bắt mỗi khoá trên một tai. Công tắc KW12 đặt cạnh khoá, lẫy đè vào công tắc khi khoá đóng hẳn. Lẫy vừa nhấc là công tắc mở vòng E-stop, trước khi giắc tách.
5. Giắc. Bắt nửa đực 21W4 vào giá in dưới tai trái, đầu chân quay xuống.
6. E-stop trạm. Bắt hộp ở mặt ngoài tai trái, nút quay ra ngoài, với tay tới được từ phía trước trạm. Nối tiếp NC1 và NC2 (E05).
7. Cột camera. Bắc cầu in qua cụm chốt hãm, dựng cột 2020 ở giữa ray, giữa hai tay. Lắp gá pan + tilt lên đỉnh cột. Gá chỉnh tay: pan ±30°, tilt 20–70° (vạch mỗi 5°), mỗi bậc khoá bằng một núm M5 ép đệm ma sát; chốt định vị ở 50° và 35°. **Đặt chúi 50°**. Camera bắt 2 vít M2 x 12 [giả định; lỗ bắt trên gá chưa vẽ]. Khe nhỏ nhất giữa thân cột và tay: 17 mm ở khoảng vai 0,475 m [CAD, R3b].
8. Hộp điện. Bắt hộp M1-TR-013 ngoài tai phải. Trong hộp: FB-L, FB-R, 2 ADuM3160, 2 bo bus, hub, buck.
   - Nguồn tay: BW07S, BW07SN, BW08S, BW08SN từ tiếp điểm A1–A4 của giắc vào hai hộp cầu chì.
   - 12V_AUX: AX03, AX04 (chân 9 và 16 của giắc) nuôi buck; buck nuôi hub (AX05, AX06).
   - USB: hub nối lên giắc bằng U03B (chân 10, 13–15; cặp D+/D− xoắn tới sát chân). Hub → ADuM3160 → bo bus trái (U04, U04B) và phải (U05, U05B). Camera đầu vào hub (U01).
   - GND bus nối GND servo của hộp cùng phía (GB.L, GB.R), không nối GND Pi.
9. Vòng E-stop trên trạm: E02 (chân 1 → khoá trái), E03 (khoá trái → khoá phải), E04 (khoá phải → E-stop), E05 (NC1 → NC2), E06 (E-stop → chân 3). Nhánh đọc khoá: E07 (khoá trái → chân 5, DOCK_L), E08 (khoá phải → chân 7, DOCK_R). Chân 2, 4, 6, 8 phía trạm để trống; cầu chắn nằm ở nửa cái trên xe (mục 4A).
10. Cỡ cọc vào và kiểu ngõ ra của hộp V07 chưa chốt (DK-N34): bấm cos theo hộp thật.

Kiểm (IPQC-05, IPQC-05A):
- Mô-men siết cọc vào hộp cầu chì và cọc 21W4: đo và ghi.
- Đúng cỡ cầu chì từng mạch theo `bang-day.csv`.
- Khoảng vai 0,475 m: đo và ghi.
- Công tắc KW12 đổi trạng thái khi khoá đóng, mở.
- Vòng E-stop trên trạm thông khi hai khoá đóng, hở khi mở một khoá hoặc nhấn E-stop.
- Điện trở cách ly GND servo ↔ GND Pi > 1 MΩ (đo sau bước 8, DK-N36).

## 4A. Đế ngàm trên xe nâng và dây vòng E-stop

Mục đích: gắn phần cố định của giao diện lắp nhanh lên xe nâng, và đi dây vòng E-stop sao cho thiếu trạm, hở khoá hoặc nhấn E-stop thì K1 nhả.

| | |
|---|---|
| Chi tiết | Đế ngàm chữ U in (M1-TR-020) bắt lên khung T giữa (M1-LF-057), mặt trên có 3 đệm tựa thép (đầu vít M6 mài phẳng, chỉnh cao được); 2 chốt côn thép Ø8 (M1-TR-021): một vào bạc tròn, một vào bạc dẹt; 2 móc giữ cho khoá kéo (M1-TR-022); nửa cái giắc 21W4 (M1-TR-023) gắn nổi ±0,8 mm bằng vít vai qua lỗ rộng và vòng cao su; khối cọc M6 4 cực |
| Dây | BW07, BW07N, BW08, BW08N (4 AWG, F3/F4 → khối cọc); BW07T, BW07TN, BW08T, BW08TN (8 AWG, khối cọc → A1–A4); E01, E09, E10, E11, AX01, AX02, GD1–GD4, U03 |
| Dụng cụ | Lục giác M3, M5, M6; kìm bấm SC10-6; mỏ hàn (cầu chắn); đồng hồ vạn năng |

1. Bắt đế ngàm lên khung T giữa. Chỉnh ba đệm tựa cao bằng nhau (đo ở IPQC-05A).
2. Bắt hai chốt côn và hai móc giữ vào con trượt T thép, không vặn vít vào nhựa. Chốt hướng lên ở y ±0,11 m; móc ở phía sau, cho khoá kéo.
3. Lắp nửa cái giắc 21W4 gắn nổi: vít vai qua lỗ rộng, vòng cao su hai bên. Giắc phải xê dịch được khoảng ±0,8 mm bằng tay.
4. Bắt khối cọc M6. Từ F3/F4, BW07 và BW08 (4 AWG) vào khối cọc, về GND bằng BW07N và BW08N. Từ khối cọc tới tiếp điểm nguồn A1–A4: đuôi 8 AWG ngắn BW07T, BW07TN, BW08T, BW08TN (cos SC10-6). Giắc chỉ nhận dây tới 8 AWG.
5. **Cầu chắn GND.** Hàn cầu giữa cốc hàn các chân 2, 4, 6, 8 và 16 (GD1 chân 2–4, GD2 chân 4–6, GD3 chân 6–8, GD4 chân 8–16). Các chân này không mang dòng. Chúng nằm giữa các chân của vòng E-stop để một dây mòn vỏ hay hai chân chạm nhau không làm vòng bỏ qua khoá hoặc E-stop.
6. Đi dây vòng và dây phụ vào nửa cái:
   - chân 1: E01 (+12V_CTRL vào vòng); chân 3: E09 (vòng về);
   - chân 5: E10 (DOCK_L); chân 7: E11 (DOCK_R);
   - chân 9: AX01 (12V_AUX); chân 16: AX02 (GND_AUX);
   - chân 10, 13–15: U03 (USB). Chân 11, 12, 17 để trống.

   Bố trí hàng chân (trên 1–9, dưới 10–17, so le) là giả định tới khi có bản vẽ hãng (DK-N55). Đối chiếu giắc thật ở IQC trước khi hàn.
7. **Ống bọc riêng.** E01, E09, E10, E11 và AX01: mỗi sợi một ống bọc, không chung vỏ với sợi khác, đi qua vòng dịch vụ trục nâng (mục 9). Dán nhãn mã dây hai đầu.
8. Hai nắp cho nửa cái trên xe. Khi không có trạm, nửa cái vẫn có điện ở chân 1 (+12V_CTRL) và chân 9 (12V_AUX): không chạm, không để vật dẫn rơi vào.
   - **Nắp che thường ngày** (không cầu nối): che kín A1–A4 và chân 1, 9. Dùng khi cất xe hoặc đổi trạm (mục 9A, bước 5). Vòng E-stop vẫn hở nên K1 không đóng được.
   - **Nắp bảo trì** (có cầu nối chân 1–3, bỏ qua vòng trạm): màu khác nắp thường, nhãn "BẢO TRÌ – BỎ QUA VÒNG TRẠM", kỹ thuật viên giữ, không để trên xe. Chỉ dùng khi chạy đế không có trạm. SDK chỉ cho chạy với cờ `allow_no_dock=True` (R3b 5.7). E-stop đỉnh cột vẫn cắt K1.
   - Hai nắp in (đề xuất số 024 nắp che, 025 nắp bảo trì trong dãy mã TR của trạm; chưa có trong sổ mã), xem `danh-sach-in.md`.

Kiểm (IPQC-05A): đồng hồ, chưa cấp điện.
- Chân 2, 4, 6, 8 thông với chân 16; không thông với chân 1, 3, 5, 7, 9.
- Nắp che thường ngày: chân 1 không thông chân 3. Nắp bảo trì: chân 1 thông chân 3.
- Ống bọc riêng đủ cho 5 sợi.
- Mô-men siết khối cọc, ốc đế ngàm: đo và ghi.

## 5. Hai tay AM-ARM200, 6+1

Hình: ảnh CAD bên dưới (ảnh thật của tay đã lắp chụp ở bộ #0; ảnh upstream nằm ở repo AM-ARM, thư mục `media/assembly/follower/`, không chép vào repo này).

![Bước 06: tay trái](../media/lap-rap/buoc-06.png)

![Bước 07: tay phải](../media/lap-rap/buoc-07.png)

| | |
|---|---|
| Chi tiết (mỗi tay) | Đế tay J1 M1-SH-005; khung xoay pan J2 M1-SH-019; vai J3 M1-SH-006; khuỷu J4 M1-AR-001; giá J5 M1-AR-002; vỏ J5/J6 M1-AR-003; đáy M1-AR-004; nắp M1-AR-005; chốt J5 M1-AR-011 (hoặc M1-AR-006 không ổ bi); 3 STS3095 (pan, vai, khuỷu); 3 STS3215 (gập, yaw, cuộn); vòng bi 6701ZZ (BOM V10) |
| Ốc | M3 x 10 (M1-AR-013, upstream ghi 74 ốc/tay); hạt chèn nhiệt M3 (M1-AR-012, BOM X01; số hạt mỗi tay follower chưa đếm, tạm 4 [giả định], xem ghi chú dưới bảng) |
| Dây (tay trái; tay phải thay LEFT bằng RIGHT) | P.LEFT_PAN±, P.LEFT_SHOULDER±, P.LEFT_ELBOW± (16 AWG tới mối nối, rồi đuôi .A± và .B± 22 AWG vào cả hai cổng STS3095); P.LEFT_WRIST_FLEX±, P.LEFT_WRIST_YAW±, P.LEFT_WRIST_ROLL± (22 AWG vào cổng A); DATA D.LEFT_PAN … D.LEFT_WRIST_FLEX, D.LEFT_WRIST_YAW, D.LEFT_WRIST_ROLL |
| Dụng cụ | Lục giác M3, mỏ hàn chèn nhiệt, cờ lê lực |

Theo thứ tự của hướng dẫn follower AM-ARM200. Bắt mọi ốc lỏng trước, căn xong mới siết (upstream).

1. Đế và vai: lắp servo pan vào đế J1, luồn cáp qua lỗ. Lắp servo vai, link vai, servo phía đối diện, vỏ cánh tay trên (upstream bước 2).
2. Khuỷu: lắp vòng bi và ống đệm, chốt và vòng bi (hoặc bản không ổ bi), servo khuỷu, vỏ khuỷu (upstream bước 3).
3. Ép hạt chèn nhiệt vào J56. Giữ hạt thẳng, phẳng mặt (upstream bước 4).
4. Cổ tay: luồn cáp servo cổ tay trước khi đóng (upstream bước 5). Lắp servo yaw STS3215 (ID 21 trái, 22 phải) vào hốc servo yaw trong vỏ J56 như upstream, không lắp khoá yaw. Servo yaw đi cáp DATA và nguồn nối tiếp sau servo gập cổ tay, trước servo cuộn. Cụm kẹp gốc J7 lắp ở mục 6 và giá camera OB_F_Camera ở mục 7.
5. Bỏ bước lắp bo điều khiển vào đế tay của upstream: bo bus đặt trong hộp điện trạm (mục 4). Tấm gá M1-AR-010 chưa chốt có dùng.
6. Đi dây nguồn và DATA theo bảng dây. Cáp 3 chân giữa hai servo chỉ giữ lõi DATA; cắt V+ và GND ở giữa, mỗi nửa nối vào cặp nguồn của servo đầu đó. Chừa vòng dịch vụ qua mỗi khớp theo cột `vong_dich_vu_mm` (bước 9).
7. Bắt hai tay lên hai gá đáy tay của trạm (mục 4, bước 2) bằng vít M3 dài và đai ốc M3. Đế J1 bắt lên gá. Làm trên trạm đã tháo khỏi xe, đặt trên bàn.

Hạt chèn nhiệt follower [giả định]: 4 hạt mỗi tay (8 cho hai tay) lấy từ cột Qty = 4 của BOM AM-ARM200. Cột này khó là số hạt: dòng leader cùng BOM ghi Qty = 4 nhưng mô tả "total 36 pcs". Mỗi tay vì thế có thể cần nhiều hơn 4; chuẩn bị dư và đếm lỗ hạt trên bản in J56 và đế J1 trước khi chốt.

Kiểm (IPQC-06):
- Mô-men siết ốc vào hạt chèn: đo và ghi; không ốc nào xoay trơn trong hạt. Đếm số hạt thực dùng của một tay và ghi lại (xem ghi chú hạt chèn ở mục này).
- Quay tay từng khớp (servo chưa cấp điện): không kẹt, không kéo căng dây. Ghi góc gập cổ tay bắt đầu chạm khuỷu (CAD báo từ khoảng 30°, DK-N38).
- Yaw cổ tay (ID 21, 22) quay trơn, không kẹt, không kéo căng dây; cầu chì 3 A ×11 đã cắm đúng chỗ (4 mỗi hộp FB-L, FB-R cho 4 STS3215 của tay, 3 trên nhánh bánh).

## 6. Kẹp gốc AM-ARM200

Kẹp là phần gốc của AM-ARM200 follower (Apache-2.0): thân kẹp J7 (M1-AR-007), hàm động J7 (M1-AR-008), servo kẹp STS3215 (ID 6, 12). Giá camera cổ tay ở mục 7 (M1-AR-027). Hình P-L: chưa có hình, chụp ở bộ #0 (hình upstream: `cad/upstream/AM-ARM/am-arm200/media/assembly/follower/`).

![Bước 08: kẹp gốc AM-ARM200 (thân J7, hàm, servo kẹp)](../media/lap-rap/buoc-08.png)

| | |
|---|---|
| Chi tiết (mỗi kẹp) | Thân kẹp OB_F_J7_Gripper (M1-AR-007); hàm OB_F_J7_Jaw (M1-AR-008); servo kẹp STS3215 (ID 6, 12) |
| Ốc | M3 x 10 lục giác theo hướng dẫn gốc bước 5 (BOM V08, 74 ốc mỗi tay gồm kẹp) |
| Dây | P.LEFT_GRIPPER±, D.LEFT_GRIPPER (tay phải tương tự) |

1. Lắp servo kẹp và hàm động vào thân kẹp theo bước 5 của hướng dẫn follower AM-ARM200 (gripper linkage).
2. Thân kẹp quay theo khớp cuộn cổ tay; hàm động quay quanh trục servo kẹp (vuông góc trục cuộn), khép vào ngón cố định. Mở hết 64° thì đầu hai ngón cách 97 mm; lực đầu ngón 35 N khi servo chặn; kẹp gồm servo 173 g (khối đặc theo URDF, bản in nhẹ hơn).
3. Tuỳ chọn: miếng đệm ma sát ở ngón kẹp (hướng dẫn gốc bước 7, cao su hoặc silicone).

Kiểm (IPQC-07):
- Khe ngón mở hết và khép: đo và ghi, so với 97,25 mm (CAD) giữa hai đầu ngón khi mở hết, hai ngón chạm khi khép. Dung sai: chờ EVT.
- Cuộn cổ tay ±160° bằng tay: kẹp không chạm cổ tay, cẳng tay, trừ vùng -147..-87° nơi giá camera chạm đáy J56 (DK-N54, chưa chốt cách xử lý).

## 7. Camera

![Bước 09: camera (giá cổ tay OB_F_Camera, camera cổ tay, camera đầu)](../media/lap-rap/buoc-09.png)

| | |
|---|---|
| Chi tiết | 3 camera USB 720p (M1-CM-001, BOM A06, loại có cáp ≥ 1,5 m): 1 camera đầu trên cột trạm (mục 4, bước 7) và 2 camera cổ tay; giá camera cổ tay OB_F_Camera (M1-AR-027, ×2 tay) |
| Ốc | 2 x M2 x 12 Phillips mỗi camera (BOM N17; cách bắt nắp lưng của hướng dẫn AlohaMini2 bước 5; giá cổ tay theo hướng dẫn follower bước 6) |
| Dây | U01 (camera đầu → hub trên trạm), U06, U07 (camera cổ tay → hub) |

1. Camera đầu: bắt lên gá pan + tilt ở đỉnh cột trạm (mục 4, bước 7), chúi 50°, chỉnh được về 35°. Không còn camera ngực và nêm.
2. Camera cổ tay: bắt giá OB_F_Camera ở đầu cổ tay bằng 2 vít M2 x 12 (hướng dẫn follower bước 6), đặt bo camera 36 x 36 vào tấm giá. Giá nằm trên thân kẹp, camera cao hơn trục cuộn ~70 mm, chúc ~24° xuống phía ngón (theo khung camera URDF upstream, vị trí đề xuất); bộ #0 ghi cách bắt.
3. Đi cáp U06, U07 dọc tay, chừa vòng ở khớp cuộn (300 mm theo bảng dây). U01 nằm trọn trên trạm, không qua vòng trục nâng.

Kiểm (IPQC-08): ảnh ba camera rõ, không đảo chiều; cáp không căng khi tay và trục nâng chạy hết hành trình bằng tay.

## 8. Pi 5, bo bus đế, GPIO

| | |
|---|---|
| Chi tiết | Raspberry Pi 5 2 GB (M1-CP-001), tản nhiệt (M1-CP-002), thẻ microSD 32 GB (M1-CP-003); bo bus đế BUS_B (BOM A07; từ S2 mang cả servo trục nâng ở cổng S2) |
| Dây | PW03, PW04 (5 V từ buck vào USB-C); U02 (Pi → BUS_B), U03 (Pi → giắc 21W4 → hub trạm, qua vòng trục nâng); G01–G13, G13V, G13G, G15, G16 (opto, K5, công tắc, K10, cảm biến chốt); dây DOCK_L, DOCK_R và opto thứ hai (G17–G19 và kế tiếp theo bảng dây); D.WHEEL_0–2, D.LIFT |

1. Bắt Pi trên tấm điện. Cắm PW03/PW04 **chỉ sau khi** đã đo buck ra 5 V trên nguồn bàn (bước 10).
2. BUS_B cắm thẳng Pi qua U02, **không qua cách ly** (DK-N36): dây GND 22 AWG về GNDS phải chắc.
3. Đi G01–G13, G13V, G13G, G15, G16 theo chân Pi trong bảng dây (ESTOP_OK BCM 17, ARM_SENSE BCM 27, RESET_SENSE BCM 23, K5_EN BCM 22, LIFT_BOT BCM 5, LIFT_TOP BCM 6, LIFT_LATCH_EN BCM 24, LIFT_LATCH_RETRACTED BCM 25). DOCK_L BCM 16, DOCK_R BCM 26, kéo lên, tích cực mức thấp [đề xuất]; đứt dây đọc là khoá mở.

Kiểm (IPQC-09): thông mạch G01–G13, G13V, G13G, G15, G16 đúng chân; điện trở GND servo ↔ GND Pi > 1 MΩ, kể cả qua vỏ servo, khung nhôm, camera cổ tay (DK-N36).

## 9. Đi dây và vòng dịch vụ

Mục đích: để dây không căng ở cuối hành trình trục nâng và ở các khớp, và để dây vòng E-stop đi riêng.

| Vòng | Dây | Dài vòng (bảng dây) |
|---|---|---:|
| Trục nâng: chữ U neo giữa cột | BW07, BW07N, BW08, BW08N, P.LIFT+, P.LIFT-, D.LIFT, L03, L04, G15, G16, AX01, AX02, E01, E09, E10, E11, U03 | 520 mm |
| Khớp vai | P.*_SHOULDER±, D.*_SHOULDER | 60 mm |
| Khớp khuỷu | P.*_ELBOW± (110 mm), D.*_ELBOW (50 mm) | |
| Gập cổ tay | P.*_WRIST_FLEX± (160 mm), D.*_WRIST_FLEX (50 mm) | |
| Cuộn cổ tay ±160° | P.*_WRIST_ROLL± (200 mm), P.*_GRIPPER± (300 mm), D.*_WRIST_ROLL (40 mm), D.*_GRIPPER (100 mm), U06, U07 (300 mm) | |

- Vòng trục nâng tính cho hành trình 0,40 m, còn CAD có 450 mm: chưa chốt (DK-N35). Bộ #0: chạy hết hành trình bằng tay, kiểm vòng không căng.
- Bán kính uốn dây 4 AWG silicon 60 mm [giả định]; mỏi uốn thử chu kỳ ở EVT (DK-N35).
- Bọc ống bảo vệ cho bó dây trên cột (như AlohaMini2 bước 6).
- Dây vòng E-stop (E01, E09, E10, E11) và AX01 đi **trong ống bọc riêng, từng sợi một**, qua vòng này. Không buộc chung vỏ với dây nguồn, dây USB hay dây khác (mục 4A bước 7).
- Dây giữa xe nâng và trạm đều đi qua vòng chữ U. Trạm tháo ra không kéo theo cáp: hai nửa giắc 21W4 gắn cứng. Kẹp giảm lực cho bó dây ≤ 50 mm trước nửa cái giắc.
- Phía trạm: dây nguồn, USB và DATA hai tay đi trong khung và hộp điện trạm, không ra ngoài trạm.

Kiểm (IPQC-10): mọi đầu dây có nhãn mã; không dây nào căng ở cuối hành trình trục nâng, gập và cuộn cổ tay; ống bọc riêng đủ cho E01, E09, E10, E11, AX01.

## 9A. Đổi trạm (gắn và tháo)

Mục đích: đổi trạm hai tay (hoặc gắn trạm lên đế bàn) bằng tay, không dụng cụ. Mục tiêu ≤ 2 phút; ước khoảng 45–60 s mỗi chiều [ước, cộng 4 bước; đo ở EVT]. Trạm có hai tay nặng khoảng 6 kg [ước]: nhấc bằng hai tay cầm, giữ thẳng đứng.

Trước khi làm: không ai trong tầm tay robot; hai tay robot ở tư thế nghỉ trong lòng trạm.

**Tháo trạm**

1. Nút "Tháo trạm" trong Studio **[chưa có trong Studio, DK-N59]**. Làm tay thay thế:
   - đưa hai tay về tư thế nghỉ trong lòng trạm;
   - hạ trục nâng bằng SDK về độ cao chốt cài được. Độ cao đổi trạm chưa có số (DK-N59): ghi số ở EVT;
   - cài chốt hãm và **nhìn xác nhận chốt đã cài** (cảm biến chốt báo "cài"). Cho phép hạ thêm một bước răng cho chốt vào hẳn rồi mới tắt mô-men (DK-N21);
   - tắt mô-men, hạ K5;
   - nhấn E-stop đỉnh cột (bước 2).
   Trục nâng phải nằm trên chốt trước bước 2: sau bước 2 servo trục nâng mất điện, xe nâng chỉ còn chốt giữ.
2. Nhấn E-stop (nút đỏ trên trạm, ở tai trái, hoặc nút ở đỉnh cột). K1 nhả, servo mất điện. Camera đầu, hub và bo bus vẫn còn điện (12V_AUX không qua K1): đó là thiết kế, Studio vẫn thấy camera.
3. Mở hai khoá kéo. Khi lẫy vừa nhấc, công tắc khoá mở vòng E-stop. Mở cả hai.
4. Nhấc trạm thẳng đứng bằng hai tay cầm. Đặt xuống chỗ chắc: đế bàn, hoặc bàn có lót, chân đệm xuống dưới.
5. **Che nửa cái giắc trên xe** bằng nắp che thường ngày: không cầu nối, che kín A1–A4 và chân 1, 9. Chân 1 và chân 9 của nửa cái luôn có điện khi tháo trạm; không chạm. Không dùng nắp bảo trì cho việc này.

Không rút trạm khi đang chạy hoặc khi K1 còn đóng. Giắc không cắt dòng tải.

**Gắn trạm**

1. Kiểm: E-stop đang nhấn, trục nâng ở độ cao đổi trạm và chốt cài, nắp che giắc đã gỡ, hai khoá kéo đang mở.
2. Nhấc trạm thẳng đứng, đưa lên phía trên đế ngàm. Hạ xuống: chốt côn vào bạc trước (dẫn 8 mm), giắc tự dẫn nhờ vỏ nổi, rồi trạm tựa lên ba đệm. Hai chốt khác loại và vỏ giắc chữ D chỉ vào một chiều, nên không lắp ngược được.
3. Đóng hai khoá kéo. Khoá ép trạm xuống đệm và đẩy giắc vào hết. Cài chốt an toàn của khoá. Hai công tắc khoá đóng.
4. Nhả E-stop. Bật lại K5 (Pi hạ K5 ở lúc tháo; bật lại bằng SDK, nút "Gắn trạm" trong Studio chưa có, DK-N59). Rồi nhấn Reset (đỉnh cột). K1 đóng; Studio báo đủ hai khoá.
5. Thiếu một khoá thì robot không cấp điện servo và SDK từ chối `enable()`: nhìn Studio để biết khoá nào.

Làm xong, ghi thời gian vào FQC-11.

## 9B. Bộ đế bàn tuỳ chọn (bán riêng)

Mục đích: đặt trạm hai tay lên bàn lab, chạy bằng nguồn riêng và laptop, không cần xe nâng. **Không thuộc robot chuẩn**: bộ này bán riêng và không cộng vào vật tư robot chuẩn. Chưa lắp thử; chưa có gói bán chính thức. **Không dùng bộ đế bàn trước khi đóng DK-N57**: rơ-le KB đóng vào khoảng 12 mF có thể dính tiếp điểm, và đế bàn không có ARM_SENSE để phát hiện.

| | |
|---|---|
| Chi tiết | Khung đế nhôm 2040 0,60 x 0,28 m (M1-TB-002); 2 tai kẹp chữ F in (M1-TB-003) và 2 kẹp F; đế ngàm (M1-TB-010), 2 chốt côn Ø8 (M1-TB-011), 2 móc giữ (M1-TB-012), nửa cái giắc 21W4 thứ ba (M1-TB-013) cùng kiểu với bộ ngàm trên xe; hộp rơ-le (M1-TB-004): rơ-le 12 V 40 A (KB), rơ-le tự giữ JQ1P (KHB), nút Reset, 2 cầu chì lá 15 A mỗi tay, cầu chì vòng 1 A, cầu chì 12V_AUX 2 A, diode 1N5408 ở cuộn KB, cổng USB ra laptop; 4 chân cao su (M1-TB-005); adapter 12 V 15 A; 2 tụ 4.700 µF 25 V |
| Dây | `electrical/bang-day-de-ban.csv` (BGD1–BGD4 cầu chắn GND như trên xe, B06–B09 nguồn hai tay, B13–B14 vòng, B24–B25 12V_AUX, B26 USB) |

1. Kẹp đế vào mép bàn bằng hai kẹp chữ F **trước khi** gắn trạm. Đứng tự do thì trạm lật về phía trước (hệ số 0,54, tay duỗi ngang, tải 0,5 kg mỗi tay). Lực kẹp cần khoảng 54 N mỗi kẹp ở tải 1 kg mỗi tay, hệ số động 2 [tự tính, chưa đo].
2. Gắn trạm theo mục 9A; đế bàn dùng đúng bộ ngàm như xe.
3. Nối adapter vào hộp rơ-le bằng dây 2 lõi bắt cọc; không dùng jack tròn. Adapter phải có bảo vệ quá dòng ≤ 16,4 A (DK-N57). Cầu chì mỗi tay trong hộp 15 A theo bảng dây đế bàn (sổ mã còn ghi 20 A).
4. Nối USB ra laptop. Hub và camera đi chân 12V_AUX, không qua rơ-le: laptop vẫn thấy camera khi nhấn E-stop.
5. E-stop trạm nối tiếp cuộn rơ-le tự giữ: nhả E-stop xong phải nhấn Reset mới chạy lại.
6. Ngân sách dòng servo đề xuất 11,4 A (cộng 12V_AUX 0,6 A là 12,0 A, 80% adapter) [đề xuất, DK-N57].

Kiểm: IPQC-05A (cầu chắn của đế bàn), FQC-11.

## 10. Cấp điện lần đầu

Làm đúng thứ tự. Bước nào không đạt thì dừng, ghi phiếu, không sang bước sau.

1. **Kiểm nguội.** J0 rời, servo chưa cắm. Đo thông mạch theo bảng dây; đo không chập BAT_F, SERVO+, +12V_CTRL với GND. Đo thông mạch vòng E-stop qua trạm và cầu chắn chân giắc (IPQC-05A).
2. **Mạch dừng trên nguồn bàn.** Nguồn bàn 12 V giới hạn 3 A cấp vào BAT_F (chưa có Pi, chưa cắm servo). Đi từng dòng bảng trạng thái L.3 (`bang-di-day-evt.md`): Reset → K1 đóng; E-stop → K1 nhả; nhả E-stop → không tự đóng lại; Reset kẹt; rút một dây E-stop. Với trạm đã gắn, thử thêm phép thử vòng trạm (mở từng khoá, nhấn E-stop trạm, DK-N56) ở FQC-11. `product/evt/estop_jig.py --che-do jig` cho mạch bộ #0 cũ, chưa viết lại cho chuỗi L (DK-N32): ghi tay vào phiếu FQC.
3. **Buck và Pi trên nguồn bàn.** Đo buck ra 5 V không tải, rồi cắm Pi. Đo 5,1 V tại cổng USB-C Pi khi tải 3 A, đặt `PSU_MAX_CURRENT` (DK-N31).
4. **Servo trên nguồn bàn, từng nhóm.** Cắm bánh, rồi tay trái, tay phải, trục nâng. Đặt giới hạn dòng nguồn bàn, ghi số. Quét bus: đủ ID 1–16, đúng model, ghi firmware. `product/evt/quet_bus.py` đang mặc định số servo bản cũ: chạy `--che-do rieng` từng ID hoặc đối chiếu tay với bảng ở bước 0. Chưa ghi giới hạn mềm vào EEPROM khi DK-N38 còn mở.
5. **Trục nâng và chốt.** Kiểm công tắc bằng `product/evt/cong_tac_hanh_trinh.py`. SDK từ chối lệnh lift trừ khi `allow_lift_underpowered=True` (thử có người giám sát). Thử rút và cài chốt theo L.2 bước 4.
6. **Đấu pin.** Chỉ khi cổng G-EVT-điện đạt: đủ 6 mục của R2 mục 7.6 có bằng chứng đo (cổng gom DK-N1, N2, N3, N7, N17, N19, 09, 10, 11, 22, F7, S1…S5) và kỹ sư điện đã ký bảng đi dây. Các điều kiện chuỗi L cũng ghi "đấu pin thật": DK-N22 (J0 thay công tắc cách ly cũ), DK-N28, DK-N29 (cầu chì lưỡi chỉ đạt với cell 40T), DK-N30 (xung đóng K1), DK-N36. Trình tự theo L.2: cắm J0 → Pi khởi động, K1 hở, chốt cài → Pi bật K5 → nhấn Reset → K1 đóng → SDK chỉ cho chạy sau khi nhả Reset và ARM_SENSE giữ cao. Không rút J0 khi đang có tải.

Kiểm: IPQC-11 (bước 1–5), FQC (bước 6 trở đi, gồm FQC-11 cho trạm).

## 11. Sau khi lắp

Làm FQC theo `product/phieu-kiem/FQC.md`. Mục "chưa kiểm" tính là không đạt khi ký phiếu.
