# Hướng dẫn lắp Faibot M1

Faibot M1 v0.5 gồm bốn cụm: **Đế di động**, **Điện**, **Trục nâng**, **Trạm hai tay**. Lắp từng cụm một, kiểm xong cụm này mới sang cụm sau.

Bản này dựng từ CAD và bảng dây, **chưa lắp thử trên robot thật**. Mọi mô-men siết đều chưa có số: dùng cờ lê lực, siết tăng dần, ghi số vào phiếu kiểm (trụ nâng, đối trọng thép 8 kg và khoá DOCK không siết theo cảm giác). Số chốt sau lần lắp thử đầu tiên (chờ EVT). Hình bước hiện là ảnh render từ CAD.

> **Cảnh báo điện.** Không đấu pin thật trước khi qua kiểm tra chất lượng cuối (FQC). Mọi lần cấp điện trước đó dùng nguồn bàn 12 V có giới hạn dòng.

Trước khi bắt đầu:

- In đủ chi tiết theo `product/danh-sach-in.md` và `product/danh-sach-in.csv`; nhận hàng và kiểm tra đầu vào (IQC) theo `product/phieu-kiem/IQC.md`.
- Đọc trước hai bước rồi mới siết.
- Chi tiết kế thừa từ tay AM-ARM200 và trụ AlohaMini2: đọc thêm hướng dẫn gốc ở `cad/upstream/` (giấy phép và ghi công ở `cad/NOTICE`).

## Dụng cụ

Danh sách đầy đủ, kèm cỡ, loại và bước dùng: [docs/vi/tools.md](tools.md).

- Lục giác và tua vít cho ốc M2, M3, M4, M5, M6; cờ lê hoặc tuýp M8 cho cọc thanh cái.
- Cờ lê lực hoặc tua vít lực.
- Mỏ hàn có đầu ép hạt chèn nhiệt M3.
- Keo epoxy hai thành phần.
- Kìm tuốt dây, kìm bấm cos (8 AWG, 4 AWG, 2 AWG), kìm bấm đầu cắm nhỏ, súng khò ống co nhiệt.
- Đồng hồ vạn năng, kẹp dòng DC, nguồn bàn 12 V có giới hạn dòng.
- Thước kẹp, thước đo góc, cân treo (cho FQC).
- Máy tính có bo bus servo để gán ID.

## Thứ tự làm việc

1. **In.** In chi tiết theo `product/danh-sach-in.csv` (vật liệu, số thành, infill ghi ở đó). Kiểm cong vênh, nứt trước khi đưa vào lắp.
2. **Nhận hàng và IQC.** Kiểm từng lô theo `product/phieu-kiem/IQC.md`; lô chưa ký thì không đưa vào robot.
3. **Cụm phụ.** Làm riêng từng cụm theo phần bên dưới: đế, điện, trục nâng, trạm hai tay (dựng và kiểm trên bàn, ngoài robot). Mỗi cụm có phiếu kiểm trong lúc lắp (IPQC) ở `product/phieu-kiem/IPQC.md`.
4. **Lắp tổng.** Gắn trạm lên xe nâng, đi dây vòng dịch vụ, cấp điện lần đầu trên nguồn bàn, từng nhóm servo.
5. **FQC.** Kiểm cuối theo `product/phieu-kiem/FQC.md`. Mục "chưa kiểm" tính là không đạt khi ký phiếu.

## Cách đọc hình

- **Xám** là phần đã lắp ở các bước trước. **Xanh** là phần thêm vào ở bước đó. Mọi hình cùng một góc nhìn.
- Hình `buoc-NN` là hình cộng dồn theo chuỗi lắp cả robot: đế (01), bánh (02), trụ (03), xe nâng (04), trạm (05), tay trái (06), tay phải (07), kẹp (08), camera (09), khay (10). Số hình không trùng số bước trong bảng của từng cụm.
- Hình `cum-*` tô xanh cả cụm trên robot đủ.
- Mã chi tiết (`M1-xx-nnn`) và ID servo giống nhau ở mọi tài liệu. Tra mã ở `product/parts-register.csv`.
- Cột **Ốc vít** lấy từ `media/hardware/bang-oc.csv`. Ô ghi "chờ EVT" là chưa chốt cỡ, vì CAD chưa vẽ lỗ bắt.
- Hình riêng cho từng bước nhỏ chưa có: các lưới bên dưới dùng hình cộng dồn tạm. Ô "chưa có hình" sẽ được thay khi có render từng bước. Phụ đề trên `buoc-05.png` bị cột camera đè một đoạn ("giắc, hộp"); đọc tên bước ở bảng, hình chỉ dùng để xem vị trí.

## Trình tự lắp

1. Gán ID cho mọi servo, mỗi lần một servo trên bus, rồi dán nhãn vị trí.
2. Cụm **Đế di động**: tấm thép đối trọng, nan, giá servo bánh, bánh omni, sàn, khay.
3. Cụm **Điện**: vỏ pin, tấm điện, trục chính, mạch dừng, dây bánh (không có pin; giắc J0 để rời).
4. Cụm **Trục nâng**: đế nối trụ, trụ, xe nâng, chốt hãm, công tắc, rồi đế ngàm của trạm.
5. Cụm **Trạm hai tay**: dựng khung, cột camera, hộp điện, hai tay, kẹp, camera. Làm trên bàn.
6. Gắn trạm lên xe nâng, đi dây vòng dịch vụ.
7. Cấp điện lần đầu trên nguồn bàn theo `product/06-lap-rap.md` mục "Cấp điện lần đầu".
8. FQC.

## Servo ID

Gán ID trước khi servo vào robot: mỗi lần chỉ một servo trên bus, baud 1.000.000. Số lấy từ `sdk/faibotic/spec.py` (`SERVO_IDS`).

| ID | Vị trí | Servo | Điện áp | Bus |
|:-:|---|---|:-:|---|
| 1 | Tay trái, xoay đế tay (pan) | STS3095 | 12 V | BUS_L |
| 2 | Tay trái, vai | STS3095 | 12 V | BUS_L |
| 3 | Tay trái, khuỷu | STS3095 | 12 V | BUS_L |
| 4 | Tay trái, gập cổ tay | STS3215 | 12 V | BUS_L |
| 5 | Tay trái, cuộn cổ tay | STS3215 | 12 V | BUS_L |
| 6 | Tay trái, kẹp | STS3215 | 12 V | BUS_L |
| 7 | Tay phải, xoay đế tay (pan) | STS3095 | 12 V | BUS_R |
| 8 | Tay phải, vai | STS3095 | 12 V | BUS_R |
| 9 | Tay phải, khuỷu | STS3095 | 12 V | BUS_R |
| 10 | Tay phải, gập cổ tay | STS3215 | 12 V | BUS_R |
| 11 | Tay phải, cuộn cổ tay | STS3215 | 12 V | BUS_R |
| 12 | Tay phải, kẹp | STS3215 | 12 V | BUS_R |
| 13 | Bánh omni 0 (trước) | STS3215 | 12 V | BUS_B |
| 14 | Bánh omni 1 | STS3215 | 12 V | BUS_B |
| 15 | Bánh omni 2 | STS3215 | 12 V | BUS_B |
| 16 | Trục nâng | STS3095 | 12 V | BUS_B |
| 21 | Tay trái, yaw cổ tay | STS3215 | 12 V | BUS_L |
| 22 | Tay phải, yaw cổ tay | STS3215 | 12 V | BUS_R |

Tay là 6+1 như AM-ARM200 gốc, có servo yaw cổ tay (ID 21 và 22, 18 servo tổng cộng). Hướng dẫn gốc của trụ AlohaMini2 dùng ID khác cho bánh và trục nâng; robot này theo bảng trên.

---

## Cụm 1. Đế di động

Đế 3 bánh omni 12 V, bán kính đặt bánh 0,25 m, đối trọng thép. Nguồn chi tiết: `product/06-lap-rap.md` mục 1. Phiếu kiểm: IPQC-02.

Dụng cụ: lục giác, cờ lê lực. Tấm thép đối trọng nặng, nhấc bằng hai tay.

| Bước | Linh kiện (mã chi tiết) | Ốc vít |
|:-:|---|---|
| 1 | Đặt tấm thép đối trọng nằm phẳng, lưỡi trước hướng ra trước: M1-BS-032 (kèm tấm phụ M1-BS-039) | Bu-lông M6 giữa hai tấm: cỡ chờ EVT |
| 2 | Bắt 3 nan nhôm 20×40 theo góc 0°, 120°, 240°: M1-BS-031 (1 × 128 mm ở 0°, 2 × 93 mm) | Ốc M3/M4/M5/M6 và đai ốc trượt rãnh 20 (bộ M1-BS-033): cỡ từng mối chờ EVT |
| 3 | Lắp servo bánh vào giá in ở đầu nan: giá M1-BS-036 (vách + nắp, ×3), servo M1-SV-001 (ID 13, 14, 15) | M3×10 (bộ ốc chung) |
| 4 | Lắp bánh omni thẳng lên còi servo: M1-BS-035 (×3) | Vít bắt bánh vào còi theo lỗ bánh mua về: chờ EVT |
| 5 | Lắp sàn in lên nan, chừa lỗ quanh đế trụ: M1-BS-037 | M3 theo bộ M1-BS-033: chờ EVT |
| 6 | Lắp cột đỡ khay trên nan trước (M1-ST-011, bản cắt ngắn, đỉnh 187,6 mm), bắt thanh nhôm 2040 dài 190 mm nằm ngang lên đỉnh cột | 2 ốc M5 qua 2040 vào cột |
| 7 | Ghép khay M1-ST-012: 4 mảnh in nối mộng đuôi én, vít M3 qua gân; ép heat-set M4 (VESA) và M3 (carrier Jetson) nếu dùng Jetson; dán 4 dải cao su chống trượt; luồn 2 dây đai qua khe | Vít M3, heat-set M3 ×4, M4 ×8 (bộ K02); dây đai và cao su (bộ K01) |
| 8 | Đặt khay lên thanh 2040 sao cho 4 vấu ôm hai mặt bên thanh, thả 2 đai ốc T vào rãnh trên, vặn 2 núm M5. Tháo: vặn ngược 2 núm, nhấc khay | 2 núm M5 + 2 đai ốc T (bộ K02), không cần dụng cụ |
| 9 | Đi sẵn dây nguồn và DATA servo bánh dọc nan về tâm đế, chưa cắm | Không có |

**Cảnh báo khay đế (chống lật):**

- Khay chỉ chở **một** laptop (≤ 2,5 kg) **hoặc** một Jetson kèm bộ DC-DC, cộng phụ kiện; tổng ≤ 3 kg. Tay robot không đặt vật lên khay. Không ngồi, không đứng lên khay.
- Đặt vật nặng **giữa khay, tựa sát gờ sau**: tâm vật cách tâm đế 80–200 mm về phía trước, lệch bên ≤ 40 mm. Laptop đặt nằm ngang, cạnh dài theo chiều rộng robot. Vật lệch ra trước hoặc sang bên làm hệ số chống lật tụt dưới 1,5.
- Luôn cài 2 dây đai trước khi đế chạy.
- Khai báo vật trong SDK: `Robot(..., khay=KhayTai("laptop", kg=..., x=..., y=...))` hoặc `robot.set_khay_tai(...)`. SDK từ chối vị trí ngoài vùng và giữ lift ≥ 40 mm.
- Laptop mở nắp khi chạy: nắp mở không quá ~95° (chân trụ ngay sau khay) và khai báo `cao=0.26`; lift khi đó giữ ≥ khoảng 226 mm để màn hình không chạm xe nâng.
- Jetson đặt trên vùng gá giữa khay (lỗ VESA), có ô quạt hút từ dưới; không che kín.

Hình 10 bên dưới còn là khay cũ; hình khay mới sẽ cập nhật.

Hình (cộng dồn):

| | | |
|:-:|:-:|:-:|
| ![Đế, nan, sàn, đối trọng](../../media/lap-rap/buoc-01.png)<br>Hình 01: đế và đối trọng | ![Bánh, servo bánh, giá servo](../../media/lap-rap/buoc-02.png)<br>Hình 02: bánh, servo bánh, giá | ![Khay và cột đỡ khay](../../media/lap-rap/buoc-10.png)<br>Hình 10: khay và cột đỡ |
| ![Cụm đế trên robot đủ](../../media/hardware/hl/cum-de.png)<br>Cụm đế trên robot đủ | chưa có hình bước 3 (servo bánh) | chưa có hình bước 5 (sàn) |

Kiểm: bánh quay tay trơn, con lăn không kẹt; ba bánh chạm sàn cùng lúc; đo và ghi mô-men siết ốc nan, giá servo, bánh; cân đế.

---

## Cụm 2. Điện

Chú giải: BAT_F là nhánh nguồn pin sau cầu chì tổng; J0 là giắc cách ly bảo trì; K1 là contactor chính; 12V_AUX là nguồn phụ 12 V cho vòng dừng và công tắc khoá trạm; DOCK là cơ cấu khoá trạm vào xe.

Chuỗi nguồn: pack 3S4P → BMS → cầu chì tổng → giắc cách ly bảo trì J0 → contactor K1 → các nhánh. Nguồn chi tiết: `product/06-lap-rap.md` mục 2, 8, 9, 10; từng sợi dây: `electrical/bang-day.csv`; sơ đồ: `electrical/bang-di-day-evt.md`. Phiếu kiểm: IPQC-03, IPQC-09, IPQC-10, IPQC-11.

**Cụm này đi dây khi hệ thống chưa nối pin.** Giắc J0 để rời. Riêng bước 1 và 2 làm việc với pack thật: cell để ở trạng thái chưa sạc đầy, cất nơi an toàn; đầu cực và đầu dây cân bằng bọc cách điện ngay khi lắp, không để vật dẫn chạm vào; không nối pack vào J0 hay bất kỳ tải nào cho tới khi qua kiểm điện (FQC). Hàn dây cân bằng từng sợi một, bọc xong sợi này mới sang sợi khác. Dán nhãn mã dây ở hai đầu mỗi sợi.

| Bước | Linh kiện (mã chi tiết) | Ốc vít |
|:-:|---|---|
| 1 | Vỏ pin: đặt pack 12 cell, BMS, cầu chì tổng F1 (sát cực dương, dây ≤ 10 cm, có boot cách điện) và giắc J0 trong vỏ: M1-PW-001, M1-PW-002, M1-PW-003; vỏ M1-PW-013, 014, 015 | Hạt chèn nhiệt M4 của vỏ; ốc nắp: chờ EVT |
| 2 | Hàn dây cân bằng từ tap pack vào BMS | Không có |
| 3 | Tấm điện trên đế: 3 cọc M8 (BAT_F, SERVO+, GND), contactor K1, cầu chì nhánh tay F3, F4, ray DIN với 3 aptomat, mạch hạ áp 5 V, module opto, domino: M1-PW-004, M1-PW-012 | Vít đầu dù M8×35 ×3, đai ốc M8 và long đen ×3; ốc M3 bắt module (bộ ốc chung) |
| 4 | Đi trục chính theo thứ tự mã dây BW03, BW04R, BW05, BW06, BW07A, BW08A (2 AWG và 4 AWG) | Đầu cos SC35-8, SC25-8 lỗ M8: siết cờ lê M8, ghi mô-men |
| 5 | Nhánh BAT_F: nhánh Pi (qua mạch hạ áp tới cổng USB-C), nhánh điều khiển, nhánh sạc tới cổng sạc | Đầu cos theo cột `dau_tu`, `dau_den` của `electrical/bang-day.csv` |
| 6 | Mạch dừng: E-stop 2 tiếp điểm NC nối tiếp trong cuộn K1, nút Reset, rơ-le tự giữ, diode và zener song song cuộn K1: M1-PW-005, M1-PW-009, M1-PW-010 | M3 bắt hộp nút ở đỉnh cột (bộ ốc chung); đầu cos RV1.25-4 cho nút |
| 7 | Dây nguồn servo bánh qua đế cầu chì inline tới servo bánh (chưa cắm) | Đầu cos theo bảng dây |
| 8 | Máy tính Pi 5, tản nhiệt, thẻ nhớ, bo bus đế BUS_B (cắm Pi qua cáp USB): M1-CP-001, M1-CP-002, M1-CP-003, M1-EL-003 | Ốc theo lỗ bắt của Pi: chờ EVT. Cắm nguồn Pi **chỉ sau khi** đo mạch hạ áp ra 5 V |
| 9 | Đi dây vòng dịch vụ: trục nâng (chữ U neo giữa cột), vai, khuỷu, gập, cuộn cổ tay; dây vòng E-stop đi trong ống bọc riêng từng sợi | Kẹp giảm lực ≤ 50 mm trước nửa giắc: cỡ chờ EVT |
| 10 | Cấp điện lần đầu trên nguồn bàn, từng nhóm servo | Không có |

Hình:

| | | |
|:-:|:-:|:-:|
| ![Cụm điện trên robot đủ](../../media/hardware/hl/cum-dien.png)<br>Cụm điện trên robot đủ | chưa có hình bước 3 (tấm điện) | chưa có hình bước 6 (mạch dừng) |

**An toàn, phải giữ:**

- **Vòng E-stop của trạm.** E-stop và hai công tắc khoá trạm nằm nối tiếp trong cuộn contactor K1. Thiếu trạm, hở một khoá hoặc nhấn E-stop thì K1 nhả. Nhả E-stop không tự chạy lại: phải nhấn Reset.
- **Không đấu pin thật trước khi qua kiểm tra chất lượng.** Làm đủ các bước 1–5 của mục "Cấp điện lần đầu" trên nguồn bàn, ghi số đo; chỉ đấu pin khi cổng kiểm tra điện đạt và kỹ sư điện đã ký bảng đi dây.
- Dây vòng E-stop và dây 12V_AUX đi riêng từng sợi trong ống bọc, không buộc chung với dây nguồn, USB hay dây khác.

---

## Cụm 3. Trục nâng

Trụ kiểu AlohaMini2 có thanh răng, hành trình 0,45 m, có chốt hãm. Từ cụm này còn gắn **đế ngàm** của trạm lên xe nâng. Nguồn: `product/06-lap-rap.md` mục 3, 4A, 9. Phiếu kiểm: IPQC-04, IPQC-05A.

Dụng cụ: epoxy, lục giác M3, M5, M6, cờ lê lực, đồng hồ vạn năng.

| Bước | Linh kiện (mã chi tiết) | Ốc vít |
|:-:|---|---|
| 1 | Bắt đế nối trụ xuống mặt trên tấm thép: M1-LF-055 | Cỡ ốc chờ EVT (CAD chưa vẽ lỗ bắt) |
| 2 | Dựng trụ: đoạn 4, 3, 2, 1 theo thứ tự; bôi epoxy mỗi mặt tiếp xúc, thanh răng hướng về phía chốt hãm: M1-LF-054, 053, 052, 051 | Keo epoxy; không dùng ốc |
| 3 | Ép 8 vòng bi 624ZZ vào khung T giữa: M1-LF-057 | Không có |
| 4 | Bắt bánh răng vào đĩa ra servo trục nâng, luồn chốt Φ12×25: M1-LF-002, M1-LF-004, servo M1-SV-002 (ID 16) | M3×10 ×4 (bánh răng vào đĩa servo) |
| 5 | Luồn khung T giữa vào ray trụ; xe nâng phải trượt êm | Không có |
| 6 | Lắp giá chốt, solenoid, đầu chốt, công tắc báo chốt rút: M1-LF-060, M1-LF-050, M1-LF-041, M1-LF-061 | Lỗ bắt chưa vẽ: khoan theo mẫu thật, ghi vị trí (chờ EVT) |
| 7 | Lắp 2 công tắc hành trình dưới, trên và đệm PU chặn hai đầu | Chờ EVT |
| 8 | Bắt đế ngàm chữ U lên khung T giữa, chỉnh 3 đệm tựa cao bằng nhau: M1-TR-020 | Vít M6 đệm tựa thép ×3; con trượt T M5 ×4 (chốt côn và móc bắt vào con trượt thép, không vặn vít vào nhựa) |
| 9 | Bắt 2 chốt côn Φ8 và 2 móc giữ khoá kéo: M1-TR-021, M1-TR-022 | Con trượt T M5 (như bước 8) |
| 10 | Lắp nửa giắc 21W4 gắn nổi, xê dịch được ±0,8 mm bằng tay: M1-TR-023 | Vít vai qua lỗ rộng, vòng cao su hai bên |
| 11 | Hàn cầu chắn GND giữa các chân 2, 4, 6, 8 và 16 của giắc; đi dây vòng và dây phụ vào nửa giắc. **Bố trí hàng chân hiện là giả định**, chưa có bản vẽ hãng: đối chiếu giắc thật và datasheet trước khi hàn, rồi đo thông mạch theo IPQC-05A (chân 2, 4, 6, 8 thông chân 16, không thông chân 1, 3, 5, 7, 9). Cầu chắn này bảo vệ vòng E-stop | Đầu cos SC10-6 cho dây nguồn 8 AWG |
| 12 | Che nửa giắc bằng **nắp che thường ngày** khi chưa có trạm: nắp in (đề xuất M1-TR-024) | Không có |

Hình:

| | | |
|:-:|:-:|:-:|
| ![Trụ nâng](../../media/lap-rap/buoc-03.png)<br>Hình 03: trụ nâng | ![Xe nâng, bánh răng, chốt hãm, công tắc](../../media/lap-rap/buoc-04.png)<br>Hình 04: xe nâng | ![Cụm trục nâng trên robot đủ](../../media/hardware/hl/cum-truc-nang.png)<br>Cụm trục nâng trên robot đủ |
| chưa có hình bước 8 (đế ngàm) | chưa có hình bước 10 (nửa giắc) | |

**An toàn, phải giữ:**

- **Nửa giắc trên xe luôn có điện** ở chân 1 (12V_CTRL) và chân 9 (12V_AUX) khi tháo trạm: không chạm, không để vật dẫn rơi vào.
- **Nắp che thường ngày** (không cầu nối) che kín các tiếp điểm nguồn và chân 1, 9; dùng khi cất xe hoặc đổi trạm. Vòng E-stop vẫn hở nên contactor không đóng được.
- **Nắp bảo trì** (có cầu nối chân 1–3, bỏ qua vòng trạm) chỉ kỹ thuật viên dùng và giữ; màu khác nắp thường, nhãn "BẢO TRÌ – BỎ QUA VÒNG TRẠM", không để trên xe. Chỉ dùng khi chạy đế không có trạm; SDK chỉ cho chạy với cờ `allow_no_dock=True`.
- **Khoá DOCK.** Hai khoá kéo ép trạm xuống đệm và đẩy giắc vào hết; mỗi khoá có công tắc báo đóng nối tiếp vòng E-stop. Mở khoá là cắt vòng trước khi giắc tách. Không rút trạm khi đang chạy.
- Trước khi tháo trạm: cài chốt hãm trục nâng và nhìn xác nhận chốt đã cài; sau đó mới nhấn E-stop (servo trục nâng mất điện, xe nâng chỉ còn chốt giữ).
- Trạm có hai tay nặng khoảng 6 kg [ước]: nhấc thẳng đứng bằng hai tay cầm.

---

## Cụm 4. Trạm hai tay

Khung chữ U tháo lắp, khoảng vai 0,475 m, mang hai tay, camera đầu, E-stop và hộp điện. **Dựng và kiểm trên bàn, ngoài robot**; xong mới gắn lên đế ngàm của xe nâng (cụm 3). Nguồn: `product/06-lap-rap.md` mục 4, 5, 6, 7, 9A. Phiếu kiểm: IPQC-05, IPQC-06, IPQC-07, IPQC-08.

Dụng cụ: lục giác M3 và M5, kìm bấm cos 8 AWG, thước kẹp, thước đo góc, mỏ hàn chèn nhiệt.

| Bước | Linh kiện (mã chi tiết) | Ốc vít |
|:-:|---|---|
| 1 | Khung chữ U: ray trước nhôm 2040 dài 0,60 m, 2 tai 2040 dài 0,20 m bắt vào mặt sau ray ở y ±0,11 m: M1-TR-001, M1-TR-002 (×2) | Ke góc ×2, con trượt T M5 ×4 |
| 2 | Gá vai: 2 gá đáy tay lên ray, tâm cách nhau 0,475 m; siết sơ, đo, siết hẳn: M1-TR-003 (×2) | Con trượt T M5 (cùng bộ bước 1) |
| 3 | Bạc chốt: ép bạc thép vào khối bạc in, trái lỗ tròn, phải lỗ dẹt: M1-TR-004 (×2), M1-TR-004b (×2) | Ốc bắt khối bạc dưới tai: chờ EVT |
| 4 | Khoá kéo và công tắc báo khoá đóng: M1-TR-008 (×2), M1-TR-009 (×2) | Ốc bắt khoá và công tắc: chờ EVT |
| 5 | Giắc: nửa đực 21W4 trên giá in dưới tai trái, chân quay xuống: M1-TR-010, M1-TR-011 | Ốc bắt giá: chờ EVT |
| 6 | E-stop trạm: hộp ở mặt ngoài tai trái, nút quay ra ngoài; nối tiếp hai tiếp điểm NC: M1-TR-012, M1-TR-012b | Ốc bắt hộp: chờ EVT |
| 7 | Cột camera: cầu in qua cụm chốt hãm, cột 2020 dài khoảng 0,46 m, gá pan + tilt (pan ±30°, tilt 20–70°, chốt định vị 50° và 35°): M1-TR-005, M1-TR-006, M1-TR-007, M1-TR-007b | Núm khoá M5 ×2 và đệm ma sát; con trượt T |
| 8 | Hộp điện trạm: cầu chì 2 nhánh tay, 2 module cách ly USB, 2 bo bus BUS_L và BUS_R, hub USB, mạch hạ áp 5 V: M1-TR-013, M1-EL-003 | M3×10 (bộ ốc chung) |
| 9 | Vòng E-stop trên trạm: nối giắc, khoá trái, khoá phải, E-stop, giắc; nhánh đọc khoá DOCK_L, DOCK_R | Đầu cos theo bảng dây |
| 10 | Tay trái và tay phải: đế J1, khung xoay pan, vai, khuỷu, giá và vỏ cổ tay; 3 STS3095 (pan, vai, khuỷu), 3 STS3215 (gập, yaw, cuộn). Mỗi tay: M1-SH-005, M1-SH-019, M1-SH-006, M1-AR-001, M1-AR-002, M1-AR-003, M1-AR-004, M1-AR-005, M1-AR-011 | M3×10 (148 ốc cho hai tay), hạt chèn nhiệt M3 (8, [giả định]: số thật có thể cao hơn, đếm lỗ trên bản in) |
| 11 | Bắt hai tay lên hai gá đáy tay của trạm | M3×18 + đai ốc M3 ×8 |
| 12 | Kẹp gốc AM-ARM200 mỗi tay: thân kẹp, hàm động, servo kẹp (ID 6 và 12): M1-AR-007, M1-AR-008, M1-SV-001 | M3×10 (nằm trong số ốc bước 10) |
| 13 | Camera: 1 camera đầu trên gá pan + tilt, 2 camera cổ tay trên giá camera cổ tay: M1-CM-001 (×3), M1-AR-027 (×2) | M2×12 Phillips: 4 (cổ tay) + 2 (đầu) |
| 14 | Đi dây nguồn và DATA hai tay theo bảng dây, chừa vòng dịch vụ ở mỗi khớp | Không có |

Hình (cộng dồn):

| | | |
|:-:|:-:|:-:|
| ![Trạm hai tay](../../media/lap-rap/buoc-05.png)<br>Hình 05: khung trạm, gá, khoá, giắc, hộp, cột | ![Tay trái](../../media/lap-rap/buoc-06.png)<br>Hình 06: tay trái | ![Tay phải](../../media/lap-rap/buoc-07.png)<br>Hình 07: tay phải |
| ![Kẹp](../../media/lap-rap/buoc-08.png)<br>Hình 08: kẹp (thân, hàm, servo) | ![Camera](../../media/lap-rap/buoc-09.png)<br>Hình 09: camera | ![Cụm trạm trên robot đủ](../../media/hardware/hl/cum-tram.png)<br>Cụm trạm trên robot đủ |

**Lưu ý khi lắp tay:**

- Tay là 6+1 như AM-ARM200 gốc. **Lắp servo yaw** (ID 21 tay trái, 22 tay phải) vào hốc servo yaw trong vỏ cổ tay, không dùng khoá yaw. Yaw quay trơn, không rơ khi lắc tay.
- Cầu chì 3 A ×11: 4 trong mỗi hộp cầu chì của tay (4 servo STS3215 mỗi tay) và 3 trên nhánh bánh.
- **Vùng cấm cuộn cổ tay −147…−87°.** Ở vùng này giá camera cổ tay chạm đáy vỏ cổ tay. Khi quay tay bằng tay, cuộn ±160° trơn, trừ vùng này; cách xử lý chờ EVT. Xem `docs/vi/specifications.md`.
- Bắt ốc lỏng trước, căn xong mới siết. Quay tay từng khớp khi servo chưa cấp điện: không kẹt, không kéo căng dây.
- Cáp 3 chân giữa hai servo chỉ giữ lõi DATA; cắt V+ và GND ở giữa, mỗi nửa nối vào cặp nguồn của servo đầu đó.

**An toàn, phải giữ:**

- **Vòng E-stop trạm.** Công tắc khoá trạm (DOCK_L, DOCK_R) và E-stop của trạm nối tiếp trong vòng dừng đi qua giắc. Vòng thông khi hai khoá đóng, hở khi mở một khoá hoặc nhấn E-stop; kiểm bằng đồng hồ trước khi gắn trạm lên xe (IPQC-05A).
- **Khoá DOCK.** Chỉ gắn trạm khi E-stop đang nhấn và hai khoá kéo đang mở; gắn xong đóng hai khoá, cài chốt an toàn của khoá.
- Trước khi gắn hoặc tháo trạm, không ai trong tầm tay robot; hai tay robot ở tư thế nghỉ trong lòng trạm.
- Chưa cấp pin thật cho trạm trước khi qua kiểm tra chất lượng; thử trên nguồn bàn có giới hạn dòng.

---

## Sau khi lắp

Làm FQC theo `product/phieu-kiem/FQC.md`. Cân robot và từng khâu, đo thời gian gắn và tháo trạm, ghi vào phiếu. Số đo được thay cho các giá trị [ước] và "chờ EVT" trong `docs/vi/specifications.md`.
