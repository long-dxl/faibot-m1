# Tính toán điện M1 bản gọn v0.5: chuỗi nguồn L, cấu hình P-L, trạm hai tay (chưa duyệt)

Ngày tính: 23/11/2026 (chốt P-L tối 23/11); tính lại 24/11/2026 cho S2 trạm hai tay. Sinh bằng `tinh_toan.py` (khối `GL`; số servo P-L và ngân sách dòng firmware
đọc từ `sdk/faibotic/spec.py` `PL_SERVO_MODELS` và `limits.current_budget`). P-L: 2 tay AM-ARM200 nguyên, vai và khuỷu đơn, 6+1 (24/11/2026: mở khớp yaw như upstream, thay 5+1):
**7 STS3095** (2 pan, 2 vai, 2 khuỷu, 1 trục nâng) + **11 STS3215** (2 gập, 2 yaw, 2 cuộn, 2 kẹp, 3 bánh).

Chuỗi L: pack 3S4P → BMS Daly 100 A → F1 MEGA 80 A → giắc cách ly bảo trì J0 → BAT_F → contactor K1 100 A cuộn 12 V → SERVO+.
Mạch dừng: nút E-stop 2NC nối thẳng trong mạch cuộn K1, nút Reset và **một** rơ-le tự giữ KH để nhả E-stop không tự chạy lại;
bỏ K2/K3/KT (và vì thế bỏ nạp trước K4). Buck 5 V TPS54560 5 A (BOM V04) cho Pi, hai hộp cầu chì lưỡi trên trạm (FB-L, FB-R),
đế cầu chì inline cho bánh, Pi và mạch dừng, bộ sạc 3S. J0 = giắc Anderson SB120 cắt cả hai cực, thay công tắc ngắt pin
riêng: **cần kỹ sư điện xác nhận.** Datasheet contactor kiểu SW80 và BMS Daly 100 A **chưa có**: DK-N1, DK-N2 còn mở, không bảng nào dưới đây nói chúng đạt.

S2 (trạm hai tay): hai tay, hộp FB-L/FB-R, 2 bo bus tay, cách ly, hub và camera đầu nằm trên trạm, nối với xe nâng qua **một giắc D-sub combo
21W4** (bảng chân ở L6b). F3/F4 hạ **60 → 40 A** cho tiếp điểm 40 A và đuôi 8 AWG. Servo trục nâng sang **BUS_B** (bus đế), nguồn
và cuộn chốt lấy ở tấm điện qua đế cầu chì inline, không còn qua FB-L. E-stop trạm và 2 công tắc khoá nối tiếp E-stop đỉnh cột trong
mạch cuộn K1. Đế bàn là bộ tuỳ chọn, mạch riêng ở L9 và `bang-day-de-ban.csv`.

## L1. Dòng (P-L)

| Mục | Giá trị |
|---|---:|
| Dòng chặn servo: 7 × 9,8 + 11 × 2,7 A [trang bán] | 98,3 A |
| + Pi 2,6 A | **100,9 A** |
| + mạch điều khiển 1,2 A + cuộn chốt lift 1,0 A [giả định] (= dòng chặn theo SDK) | 103,1 A |
| + 12V_AUX lên trạm 0,6 A (hub, 3 camera, bo bus; [giả định R3b 5.4]) = đỉnh trục chính | **103,7 A** |
| Dòng chặn mỗi tay (3 STS3095 + 4 STS3215) qua F3/F4 và 21W4 | **40,2 A** > 40 A (VƯỢT nhẹ khi mọi servo một tay cùng chặn, thêm servo yaw; xem L6b) |
| Dòng thiết kế BMS/contactor (đỉnh × 1,25) | 129,6 A |
| Liên tục (servo × 0,4 [giả định] + phụ tải) | 44,7 A |
| **Ngân sách firmware** (L2, SDK) | **56,1 A** (+ 12V_AUX 0,6 A ngoài SDK = 56,7 A) |

Số của các cấu hình trước P-L: `archive/v0.4/electrical/tinh-toan-chuoi-A.md`.

## L2. Ngân sách dòng firmware (R2 7.4, P-L)

Dòng cho phép mỗi servo = dòng chặn × TORQUE_LIMIT/1000 [giả định dòng tỉ lệ mô-men, chưa đo, DK-N19]. TORQUE_LIMIT theo nhóm ở
`spec.TORQUE_LIMIT_PCT`; vai đơn: min(60%, 60 kg·cm / 95) = 600‰; lift = min(70% định mức, 0,5 × F_đo × r) =
97‰ với F_đo 120 N **[giả định, chờ đo DK-N18]**. SDK ghi TORQUE_LIMIT cho mọi servo và từ chối
`enable()` khi tổng vượt 80 A. Bảng SDK `SERVO_IDS` là 18 servo P-L 6+1 (ID 1–16, yaw 21/22);
ngân sách P-L tính bằng `current_budget(PL_SERVO_MODELS, models=PL_SERVO_MODELS)`.

| Nhóm | SL | Loại | Chặn (A) | ‰ | Cho phép (A) |
|---|---:|---|---:|---:|---:|
| Pan | 2 | STS3095 | 19,6 | 500 | 9,80 |
| Vai | 2 | STS3095 | 19,6 | 600 | 11,76 |
| Khuỷu | 2 | STS3095 | 19,6 | 600 | 11,76 |
| Gập cổ tay | 2 | STS3215 | 5,4 | 600 | 3,24 |
| Yaw cổ tay | 2 | STS3215 | 5,4 | 800 | 4,32 |
| Cuộn cổ tay | 2 | STS3215 | 5,4 | 500 | 2,70 |
| Kẹp | 2 | STS3215 | 5,4 | 500 | 2,70 |
| Bánh | 3 | STS3215 | 8,1 | 500 | 4,05 |
| Trục nâng | 1 | STS3095 | 9,8 | 97 | 0,95 |
| Pi + điều khiển + cuộn chốt | | | 4,8 | | 4,8 |
| **Tổng** | | | **103,1** | | **56,1** |

56,1 A ≤ 80 A (F1) ≤ 100 A (BMS). Pan là STS3095 ở AM-ARM200 nên nhóm pan nặng như vai, khuỷu. Tổng chỉ đúng khi
dòng tỉ lệ mô-men và servo tôn trọng TORQUE_LIMIT khi kẹt: chưa đo (DK-N19). Mỗi tay ở giới hạn mô-men: 23,14 A.

**Trục nâng sau** (bánh 20 răng, r 15 mm; dòng tỉ lệ mô-men [giả định]):

| Mục | Giá trị |
|---|---:|
| Lực giữ tải thiết kế 8,28 kg × (g + 0,5 m/s²) | 85,4 N |
| Mô-men trên bánh | 1,28 N·m = 58% định mức, 13,7% chặn |
| Dòng servo ở tải thiết kế, η 1 / η 0,9 [giả định] | 1,35 / 1,50 A |
| TORQUE_LIMIT lift (răng PETG F_đo 120 N giữ chỗ) | 97‰ → 0,95 A, lực răng 60,2 N |
| Dòng chặn (cầu chì phải chịu) | 9,8 A → FLIFT 7,5 A, 16 AWG (10,7 A) |

Ở mức răng giữ chỗ, lift **không** nâng đủ tải thiết kế (60,2 < 85,4 N, `LIFT_LIMIT_COVERS_DESIGN_LOAD` = False): chờ DK-N18 đo răng
≥ 171 N. Phần điện không đổi theo kết quả đó: dòng lift ≤ 1,5 A khi nâng, cầu chì và dây chọn theo dòng chặn.

## L3. Pack 3S4P, cell, ngắn mạch, đóng K1

| Mục | Giá trị | Ghi chú |
|---|---|---|
| Năng lượng | 172,8 Wh = 12 × 4,0 Ah × 3,6 V | Samsung INR21700-40T; P45B 194,4 Wh |
| Chạy 2 giờ | dòng TB ≤ **6,62 A** | dùng được 0,85 [giả định]; chưa có số đo teleop (SOR-014) |
| Dòng cell: đỉnh 103,7 / thiết kế 129,6 / ngân sách 56,1 A | 25,9 / 32,4 / 14,0 A | 74% / 93% / 40% của 35 A (40T bản tentative) |
| BMS 100 A / đỉnh | 0,96 lần; / thiết kế 0,77 lần | dưới mức × 1,25: dựa vào ngân sách firmware + F1 80 A (DK-N2) |
| Contactor 100 A phải ngắt | ≥ 130 A ở 12–14 V DC | hạ về mức ngân sách chỉ khi kỹ sư điện chấp nhận và đã đo (DK-N1) |
| Isc cận trên lý tưởng (chỉ cell AC 12 mΩ) | **1400 A** | MEGA 2 kA: biên 1,43 lần |
| Isc sau K1 (SERVO+), 40T / P45B / giả định hiện hành | 820 / 1084 / 462 A | **cầu chì lưỡi chỉ cắt 1 kA @ 32 VDC** [snippet]: đạt sát với 40T, **KHÔNG ĐẠT** với P45B ở Isc lớn nhất. Bắt buộc đo R pack + đường; không dùng P45B với cầu chì lưỡi |
| Isc tại BAT_F (aptomat Pi, mạch dừng, cổng sạc) | 939 / 1304 / 550 A | cầu chì lưỡi 1 kA sẽ sát dưới với 40T (94%) và vượt với P45B, nên ba nhánh BAT_F dùng **aptomat DC CHINT NB1-63DC 1P, 6 kA** (C6 Pi, C4 mạch dừng, C6 cổng sạc; [trang hãng]): biên 4,6 lần. Aptomat có phân cực: đấu theo chiều dòng sự cố (BAT_F → tải) |
| Đóng K1 không nạp trước | tụ bus ~4,0 mF (18 × 220 µF [giả định]) | xung ~462 A, tau ~108 µs, 0,31 J (R vòng 27,3 mΩ, giả định hiện hành): định mức đóng của contactor và tụ servo: DK-N30 (datasheet + đo xung ở EVT) |

## L4. Sụt áp trục chính

| Đoạn | R (mΩ) | Nguồn |
|---|---:|---|
| dây trục chính 2 AWG (BW00–BW06, đi + về, 1,46 m) | 0,89 | tính, 0,61 mΩ/m nóng |
| cầu chì F1 MEGA 80 A | 2,00 | [giả định] |
| giắc J0 SB120, hai cực (cách ly bảo trì) | 1,00 | [giả định] |
| contactor K1 100 A kiểu SW80 | 3,00 | [giả định] |
| 8 mối cos M8 (F1 ra, J0 ×2, BAT_F ×2, K1 ×2, GND) | 2,40 | [giả định] |
| **Tổng** | **9,29** | |

Ở dòng liên tục 44,7 A: 416 mV = **3,74%** (VƯỢT 3% của SOR-016, mong muốn).
Không hạ được dưới 3% bằng cỡ dây: phần lớn R nằm ở contactor, F1, J0 và mối cos [giả định]; dây 2 AWG chỉ chiếm 10%. Giữ DK-N24, đo ở EVT.
Nhánh (từ SERVO+, mỗi sợi đi và về cộng lại, dòng liên tục, `bang-day.csv` cột `sut_ap_lien_tuc_pct`):
- SERVO+ → hộp FB-L trên trạm (4 AWG 1,04 m, đuôi 8 AWG, 8 AWG trạm, đi + về): 0,72%; hai tiếp điểm 21W4
  (A1 + A2, 1,0 mΩ mỗi tiếp điểm [giả định, DK-N55]) ở 16,1 A: **0,29%** (32 mV;
  ở dòng chặn 40,2 A: 80 mV, 1,62 W mỗi tiếp điểm).
- Tới kẹp trái (xa nhất, 22 AWG): 2,93%; tới khuỷu trái (STS3095): 2,09%; tới bánh 0,46%; tới trục nâng
  (16 AWG qua vòng dịch vụ, 0,96 m) 1,06% ở dòng 3,92 A (0,4 × chặn; khi nâng thật ~1,5 A).
- Trục chính + nhánh xa nhất: 6,67%.

## L5. Mạch dừng chuỗi L

```
+12V_CTRL (aptomat C4) ─ 21W4.1 ─[trạm: khoá trái ─┬─ khoá phải ─┬─ EST-NC1 ─ EST-NC2]─ 21W4.3 ─ TB3 ─ ES-NC1 ─ ES-NC2 ─ K5 ─┐
                                                   └─ 21W4.5     └─ 21W4.7                                                    │
                                                      DOCK_L        DOCK_R                                                    │
  ┌───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
  └─┬─ RST-NO1 ─┬─ HOLD ─┬─ cuộn KH ─ GND
    └─ KH.1 NO ─┘        └─ cuộn K1 (diode + Zener) ─ GND
HOLD ─ opto ─ ESTOP_OK ;  SERVO+ ─ opto ─ ARM_SENSE ;  +12V_CTRL ─ RST-NO2 ─ opto ─ RESET_SENSE ;  DOCK_L/R ─ opto ─ BCM 16/26 [đề xuất]
```

Nhấn E-stop (đỉnh cột hoặc trạm) hoặc mở một khoá: vòng hở, KH và K1 mất cuộn cùng lúc; đóng lại: KH không tự hút lại, chờ Reset.
Khoá đặt trước E-stop trạm để DOCK_L/DOCK_R chỉ phụ thuộc +12V_CTRL và hai khoá: DOCK_L = khoá trái, DOCK_R = cả hai khoá (khoá trái
mở thì chưa biết khoá phải; Studio báo khoá trái trước). Mở lẫy là hở vòng, K1 nhả (~20 ms) trước khi giắc tách (≥ 0,5 s thao tác tay).
Không có trạm: vòng hở, K1 không đóng; bảo trì đế dùng nắp giắc có cầu nối 1–3, nắp phải che kín A1–A4 (khi tháo trạm, chân 1
(+12V_CTRL) và 9 (12V_AUX) ở nửa cái trên xe luôn có điện; sau Reset A1/A3 có SERVO+).

**Chống chập vòng (QC2 M1).** E-stop trạm là một kênh (NC1 nối tiếp NC2 chống dính tiếp điểm, không chống chập dây). Chập hai nút
bất kỳ của vòng (vào 1, về 3, DOCK_L 5, DOCK_R 7) hoặc với 12V_AUX (9) là bỏ qua khoá hoặc E-stop trạm mà C4 không cắt. Chặn bằng:
(1) bố trí chân: 1, 3, 5, 7, 9 cách nhau một chân chắn nối GND (2, 4, 6, 8, cầu về chân 16 GND_AUX ngay ở nửa cái), chập sang chân
chắn thì C4/C2 cắt hoặc K1 nhả; chân kề 3/5/7 ở hàng dưới chỉ là để trống, D+/D−, GND; VBUS (10) chỉ kề chân 1 (nguồn) và chắn 2;
(2) E01, E09, E10, E11, AX01 đi ống bọc riêng từng sợi qua vòng dịch vụ (cách điện kép); (3) phép thử nghiệm thu và định kỳ trong
DK-N56: nhấn E-stop trạm, tháo từng khoá, nhấn E-stop cột: K1 phải nhả. Bố trí hàng chân là giả định theo D-sub chuẩn (DK-N55).

Vòng trạm thêm 3,47 m dây 22 AWG và 2 chân tín hiệu 21W4 vào mạch cuộn: sụt toàn mạch cuộn K1 + KH (1,2 A) ~0,49 V
[tự tính, chân 5 mΩ giả định], cuộn K1 còn ~8,5 V khi pin 9,0 V: điện áp hút của contactor chưa có datasheet (DK-N1, DK-N56).
Bảng trạng thái và bảng nối dây: `bang-di-day-evt.md` mục L.

## L6. Bảng dây từng sợi (`bang-day.csv`, sinh từ `day_L.py`)

204 sợi, mỗi dòng một sợi: mã, điểm đầu → điểm cuối (đầu nối.chân), tín hiệu, AWG/mm², chiều dài đường đi và chiều dài cắt
(mm), nguồn chiều dài, vòng dịch vụ, màu, đầu cos/giắc hai đầu, cầu chì, dòng liên tục/đỉnh, chịu dòng ×0,82, sụt áp. Hình học ở
khối `GEO` của `day_L.py` (mỗi số ghi nguồn; [ước từ CAD] là kích thước lấy từ CAD/BOM/R1b, chưa đo trên robot). Vòng dịch vụ trục
nâng: chữ U neo giữa cột, dài hành trình/2 + π·r + dư = 513 mm (hành trình 0,45 m, r 60 mm).
Vật tư cắt/bấm: `vat-tu-day.csv`.

| Nhóm | Số sợi | AWG | Tổng cắt (m) |
|---|---:|---|---:|
| trục chính | 8 | 2 | 1,86 |
| nhánh tay | 14 | 2, 4, 8 | 7,42 |
| nguồn servo | 36 | 16, 22 | 32,72 |
| đuôi servo | 28 | 22 | 2,80 |
| đế cầu chì inline | 8 | 16 | 0,80 |
| aptomat BAT_F | 4 | 16 | 0,68 |
| Pi | 4 | 16, 20 | 1,10 |
| sạc | 2 | 14 | 0,50 |
| 12V_AUX | 6 | 18, 22 | 3,48 |
| cân bằng | 4 | 24 | 0,60 |
| mạch dừng | 21 | 16, 22 | 7,02 |
| vòng trạm | 15 | 22, 24 | 6,74 |
| GPIO | 21 | 24 | 6,66 |
| chốt trục nâng | 3 | 22 | 2,17 |
| dữ liệu BUS_L | 8 | 22, 24 | 1,87 |
| dữ liệu BUS_R | 8 | 22, 24 | 1,59 |
| dữ liệu BUS_B | 4 | 24 | 3,00 |
| USB | 10 | USB2.0 | 5,55 |

Cầu chì (mỗi servo một cầu chì; quy tắc: ≥ 1,25 × dòng cho phép firmware và ≤ chịu dòng ×0,82 của dây nhỏ nhất phía sau):

| Cầu chì | A | Loại | Cắt (A @ 32 VDC) | Vị trí | Bảo vệ |
|---|---:|---|---:|---|---|
| F1 | 80 | MEGA 298 bắt bu-lông M8 | 2000 | cực + pack, trong vỏ pin | BOM N01 (+ đế) |
| F3 | 40 | MEGA 298 bắt bu-lông M8 | 2000 | tấm điện, cạnh SERVO+ | tay trái qua 21W4 A1/A2 (≤ tiếp điểm 40 A, đuôi 8 AWG); BOM N18 |
| F4 | 40 | MEGA 298 bắt bu-lông M8 | 2000 | tấm điện, cạnh SERVO+ | tay phải qua 21W4 A3/A4; như F3 |
| FL1 | 7,5 | ATO lưỡi chuẩn | 1000 | hộp FB-L mạch 1 (hộp điện trạm) | left_pan |
| FL2 | 7,5 | ATO lưỡi chuẩn | 1000 | hộp FB-L mạch 2 (hộp điện trạm) | left_shoulder |
| FL3 | 7,5 | ATO lưỡi chuẩn | 1000 | hộp FB-L mạch 3 (hộp điện trạm) | left_elbow |
| FL4 | 3 | ATO lưỡi chuẩn | 1000 | hộp FB-L mạch 4 (hộp điện trạm) | left_wrist_flex |
| FL5 | 3 | ATO lưỡi chuẩn | 1000 | hộp FB-L mạch 5 (hộp điện trạm) | left_wrist_yaw |
| FL6 | 3 | ATO lưỡi chuẩn | 1000 | hộp FB-L mạch 6 (hộp điện trạm) | left_wrist_roll |
| FL7 | 3 | ATO lưỡi chuẩn | 1000 | hộp FB-L mạch 7 (hộp điện trạm) | left_gripper |
| FR1 | 7,5 | ATO lưỡi chuẩn | 1000 | hộp FB-R mạch 1 (hộp điện trạm) | right_pan |
| FR2 | 7,5 | ATO lưỡi chuẩn | 1000 | hộp FB-R mạch 2 (hộp điện trạm) | right_shoulder |
| FR3 | 7,5 | ATO lưỡi chuẩn | 1000 | hộp FB-R mạch 3 (hộp điện trạm) | right_elbow |
| FR4 | 3 | ATO lưỡi chuẩn | 1000 | hộp FB-R mạch 4 (hộp điện trạm) | right_wrist_flex |
| FR5 | 3 | ATO lưỡi chuẩn | 1000 | hộp FB-R mạch 5 (hộp điện trạm) | right_wrist_yaw |
| FR6 | 3 | ATO lưỡi chuẩn | 1000 | hộp FB-R mạch 6 (hộp điện trạm) | right_wrist_roll |
| FR7 | 3 | ATO lưỡi chuẩn | 1000 | hộp FB-R mạch 7 (hộp điện trạm) | right_gripper |
| FW1 | 3 | ATO lưỡi chuẩn trong đế inline FHAC | 1000 | cọc SERVO+ | wheel_0 |
| FW2 | 3 | ATO lưỡi chuẩn trong đế inline FHAC | 1000 | cọc SERVO+ | wheel_1 |
| FW3 | 3 | ATO lưỡi chuẩn trong đế inline FHAC | 1000 | cọc SERVO+ | wheel_2 |
| FLIFT | 7,5 | ATO lưỡi chuẩn trong đế inline FHAC | 1000 | cọc SERVO+ | lift (S2: rời FB-L, nguồn ở tấm điện) |
| FLAT | 2 | ATO lưỡi chuẩn trong đế inline FHAC | 1000 | cọc SERVO+ | cuộn chốt trục nâng (S2: rời FB-L) |
| FPI | 6 | MCB DC CHINT NB1-63DC 1P C6 | 6000 | ray DIN cạnh BAT_F | buck 5 V Pi |
| FCTRL | 4 | MCB DC CHINT NB1-63DC 1P C4 | 6000 | ray DIN cạnh BAT_F | +12V_CTRL mạch dừng (gồm vòng E-stop trạm) |
| FCHG | 6 | MCB DC CHINT NB1-63DC 1P C6 (đấu theo chiều dòng sự cố: BAT_F → cổng) | 6000 | ray DIN cạnh BAT_F | cổng sạc |
| FAUX | 2 | MCB DC CHINT NB1-63DC 1P C2 | 6000 | ray DIN cạnh BAT_F | 12V_AUX lên trạm (hub, camera, bo bus); không qua K1 |

Isc tại thanh cái hộp FB-L/FB-R trên trạm (L6b), 40T / P45B / giả định hiện hành: 517 /
612 / 325 A: cầu chì lưỡi trong hộp dưới 1 kA cả với P45B. Cầu chì lưỡi ở
cọc SERVO+ (FW1–3, FLIFT, FLAT) vẫn theo L3 (đạt với 40T, KHÔNG ĐẠT với P45B): giữ DK-N29. Nhánh BAT_F (Pi, mạch dừng, sạc, 12V_AUX) dùng aptomat 6 kA.

STS3095 dùng 7,5 A thay 10 A của bản trước: dòng cho phép lớn nhất 5,88 A × 1,25 =
7,35 A ≤ 7,5 A, và hai đuôi 22 AWG song song (2 × 4,1 A) chịu được 7,5 A.
STS3215 giữ 3 A (2,70 A ≤ 3 A ≤ 4,1 A). Dòng chặn > cầu chì: kẹt lâu thì cầu chì chảy, đúng ý.

Dây chính:

| Dây | Từ → đến | AWG | Dài / cắt (mm) | Đầu cos | Cầu chì | Liên tục / đỉnh (A) | Chịu ×0,82 (A) |
|---|---|---:|---|---|---|---|---:|
| BW00 | PACK.B- → BMS.B- | 2 | 100 / 150 | HAN / BMS-COC | F1 80 | 44,72 / 103,7 | 94,3 |
| BW01 | PACK.B+ → F1.in | 2 | 60 / 110 | HAN / SC35-8 | F1 80 | 44,72 / 103,7 | 94,3 |
| BW02 | F1.out → J0P.+ | 2 | 100 / 150 | SC35-8 / SB120 | F1 80 | 44,72 / 103,7 | 94,3 |
| BW03 | J0R.+ → BATF.stud | 2 | 400 / 450 | SB120 / SC35-8 | F1 80 | 44,72 / 103,7 | 94,3 |
| BW04 | BMS.P- → J0P.- | 2 | 100 / 150 | BMS-COC / SB120 | F1 80 | 44,72 / 103,7 | 94,3 |
| BW04R | J0R.- → GNDS.stud | 2 | 400 / 450 | SB120 / SC35-8 | F1 80 | 44,72 / 103,7 | 94,3 |
| BW05 | BATF.stud → K1.in | 2 | 150 / 200 | SC35-8 / SC35-8 | F1 80 | 40,32 / 99,3 | 94,3 |
| BW06 | K1.out → SERVOP.stud | 2 | 150 / 200 | SC35-8 / SC35-8 | F1 80 | 40,32 / 99,3 | 94,3 |
| BW07A | SERVOP.stud → F3.in | 2 | 80 / 130 | SC35-8 / SC35-8 | F1 80 | 16,08 / 40,2 | 94,3 |
| BW07 | F3.out → CDK.L+ | 4 | 1040 / 1090 | SC25-8 / SC25-6 | F3 40 | 16,08 / 40,2 | 69,7 |
| BW07N | CDK.L- → GNDS.stud | 4 | 1040 / 1090 | SC25-6 / SC25-8 | F3 40 | 16,08 / 40,2 | 69,7 |
| BW07T | CDK.L+ → J21F.A1 | 8 | 150 / 200 | SC10-6 / 21W4-P | F3 40 | 16,08 / 40,2 | 41,0 |
| BW07TN | J21F.A2 → CDK.L- | 8 | 150 / 200 | 21W4-P / SC10-6 | F3 40 | 16,08 / 40,2 | 41,0 |
| BW07S | J21M.A1 → FBL.in | 8 | 450 / 500 | 21W4-P / SC10-6 | F3 40 | 16,08 / 40,2 | 41,0 |
| BW07SN | FBL.neg → J21M.A2 | 8 | 450 / 500 | SC10-6 / 21W4-P | F3 40 | 16,08 / 40,2 | 41,0 |
| BW08A | SERVOP.stud → F4.in | 2 | 80 / 130 | SC35-8 / SC35-8 | F1 80 | 16,08 / 40,2 | 94,3 |
| BW08 | F4.out → CDK.R+ | 4 | 1040 / 1090 | SC25-8 / SC25-6 | F4 40 | 16,08 / 40,2 | 69,7 |
| BW08N | CDK.R- → GNDS.stud | 4 | 1040 / 1090 | SC25-6 / SC25-8 | F4 40 | 16,08 / 40,2 | 69,7 |
| BW08T | CDK.R+ → J21F.A3 | 8 | 150 / 200 | SC10-6 / 21W4-P | F4 40 | 16,08 / 40,2 | 41,0 |
| BW08TN | J21F.A4 → CDK.R- | 8 | 150 / 200 | 21W4-P / SC10-6 | F4 40 | 16,08 / 40,2 | 41,0 |
| BW08S | J21M.A3 → FBR.in | 8 | 450 / 500 | 21W4-P / SC10-6 | F4 40 | 16,08 / 40,2 | 41,0 |
| BW08SN | FBR.neg → J21M.A4 | 8 | 450 / 500 | SC10-6 / 21W4-P | F4 40 | 16,08 / 40,2 | 41,0 |
| P.LEFT_PAN+ | FBL.c1 → SP.left_pan.+ | 16 | 530 / 580 | FDD6.3 / HAN | FL1 7,5 | 3,92 / 9,8 | 10,7 |
| P.LEFT_GRIPPER+ | FBL.c7 → SV.left_gripper.A.V | 22 | 1560 / 1610 | FDD6.3 / 5264-3P | FL7 3 | 1,08 / 2,7 | 4,1 |
| P.W1+ | SP.wheel_0.+ → SV.wheel_0.A.V | 22 | 380 / 430 | HAN / 5264-3P | FW1 3 | 1,08 / 2,7 | 4,1 |
| FH.LIFT.i | SERVOP.stud → HFLIFT.in | 16 | 100 / 100 | RV5.5-8@SERVOP2 / FH-LEAD | FLIFT 7,5 | 3,92 / 9,8 | 10,7 |
| P.LIFT+ | HFLIFT.out → SP.lift.+ | 16 | 960 / 1010 | HAN / HAN | FLIFT 7,5 | 3,92 / 9,8 | 10,7 |
| PW01 | QFPI.2 → BUCK.in+ | 16 | 150 / 200 | PTV1.25-18 / HAN | FPI 6 | 2,60 / 2,6 | 10,7 |
| PW02 | BUCK.in- → GNDS.stud | 16 | 150 / 200 | HAN / RV5.5-8@GNDS | FPI 6 | 2,60 / 2,6 | 10,7 |
| PW03 | BUCK.out+ → PI.USBC.V | 20 | 300 / 350 | HAN / USB-C | BUCK-LIM 6 | 2,50 / 5,0 | 6,1 |
| PW04 | PI.USBC.G → BUCK.out- | 20 | 300 / 350 | USB-C / HAN | BUCK-LIM 6 | 2,50 / 5,0 | 6,1 |
| CH01 | QFCHG.2 → CHG.+ | 14 | 200 / 250 | PTV1.25-18 / XT60E-F | FCHG 6 | 5,00 / 5,0 | 16,4 |
| CH02 | CHG.- → GNDS.stud | 14 | 200 / 250 | XT60E-F / RV5.5-8@GNDS | FCHG 6 | 5,00 / 5,0 | 16,4 |
| AX01 | QFAUX.2 → J21F.9 | 22 | 1040 / 1090 | PTV1.25-18 / 21W4-S | FAUX 2 | 0,60 / 2,0 | 4,1 |
| AX03 | J21M.9 → BUCKT.in+ | 22 | 450 / 500 | 21W4-S / HAN | FAUX 2 | 0,60 / 2,0 | 4,1 |
| C02 | TB3.1 → ES.NC1a | 22 | 720 / 770 | E7508 / RV1.25-4 | FCTRL 4 | 1,20 / 1,2 | 4,1 |
| C13 | TB1.4 → K1.A1 | 22 | 150 / 200 | E7508 / RV1.25-4 | FCTRL 4 | 1,00 / 1,0 | 4,1 |
| E01 | TB1.1 → J21F.1 | 22 | 1040 / 1090 | E7508 / 21W4-S | FCTRL 4 | 1,20 / 1,2 | 4,1 |
| E09 | J21F.3 → TB3.1 | 22 | 1040 / 1090 | 21W4-S / E7508 | FCTRL 4 | 1,20 / 1,2 | 4,1 |
| G13 | PI.p18 → K10.IN | 24 | 150 / 180 | DUPONT / E7508 | -  | 0,01 / 0,1 | 2,9 |
| L01 | HFLAT.out → K10.COM | 22 | 100 / 150 | HAN / E7508 | FLAT 2 | 1,00 / 1,0 | 4,1 |
| L03 | K10.NO → LATCH.+ | 22 | 960 / 1010 | E7508 / HAN | FLAT 2 | 1,00 / 1,0 | 4,1 |
| L04 | LATCH.- → GNDS.stud | 22 | 960 / 1010 | HAN / RV5.5-8@GNDS2 | FLAT 2 | 1,00 / 1,0 | 4,1 |
| D.LIFT | BUS_B.S2.D → SV.lift.A.D | 24 | 960 / 990 | 5264-3P / 5264-3P | -  | 0,01 / 0,1 | 2,9 |
| U03 | PI.USB3 → J21F.13 | USB2.0 | 1070 / 1070 | USB-A / 21W4-S | -  | 0,50 / 0,9 |  |

Đi dây servo: nguồn hình sao, mỗi servo một cặp V+/GND từ hộp cầu chì; DATA nối chuỗi theo bus (bo bus → pan → vai → khuỷu →
gập → cuộn → kẹp; bánh 0 → 1 → 2; **trục nâng ở cổng S2 của bo bus đế BUS_B**, D.LIFT 24 AWG qua vòng dịch vụ). Cáp 3 chân giữa hai servo chỉ giữ DATA đi suốt, V+ và
GND cắt giữa, mỗi nửa nối vào cặp nguồn của servo ở đầu đó: không có đường nguồn nối tiếp qua cáp, và STS3095 nhận điện ở cả
hai cổng. USB: Pi → chân 10, 13–15 giắc 21W4 (qua vòng trục nâng) → hub 7 cổng trong hộp điện trạm (tự cấp nguồn 5 V từ 12V_AUX qua buck trạm);
hub nuôi camera đầu, 2 camera cổ tay và, qua 2 module cách ly USB ADuM3160, 2 bo bus tay; bo bus đế cắm thẳng Pi. Hộp điện trạm ở ngoài
tai phải nên dây tay trái dài hơn tay phải 0,28 m.

Đi qua vòng dịch vụ trục nâng (chữ U 513 mm): BW07/BW07N/BW08/BW08N 4 AWG; P.LIFT± 16 AWG; L03/L04 cuộn chốt, AX01/AX02 12V_AUX,
E01/E09 vòng trạm, E10/E11 DOCK (22 AWG; cùng AX01 mỗi sợi một ống bọc riêng); D.LIFT, G15/G16 (24 AWG); U03 USB. Không cáp nào nối trạm với xe: hai nửa giắc gắn cứng.
Bán kính uốn 4 AWG chỗ vào đế ngàm ≥ 50 mm, kẹp giảm lực ≤ 50 mm trước giắc (R3c mục 4).

**Mất dây về GND nhánh tay (QC2 M1).** Nếu BW07N/BW08N (hoặc tiếp điểm A2/A4) hở mà còn đường GND khác giữa servo tay và Pi (GND USB,
GND_AUX, DATA), tới 40,2 A sẽ về qua cáp USB hoặc chân 11 22 AWG: cháy, không phát hiện. Chọn **cắt mọi đường GND thứ hai**
thay vì thêm dây về thứ hai: (1) bo bus tay nằm sau module ADuM3160 (cách ly 1,5 kV, 12 Mbps đủ cho bus 1 Mbps), GND bus nối
GND servo của hộp mình (GB.L, GB.R); hub, buck trạm, camera thuộc miền GND_AUX = GND Pi, không nối GND servo trên trạm;
(2) cuộn chốt đóng bằng rơ-le K10 (nay trên tấm điện), dây cuộn về GNDS; (3) servo trục nâng sang BUS_B, nguồn và GND về thẳng tấm điện.
Dây về tay hở làm servo tay mất nguồn (an toàn khi hỏng) và SDK thấy mất bus. **BUS_B không cách ly** và nay mang thêm trục nâng: đứt
P.LIFT− thì dòng lift về qua DATA/GND USB của BUS_B (giới hạn bởi FLIFT 7,5 A). Còn đo ở EVT hoặc thêm ADuM3160 cho BUS_B: DK-N36.

## L6b. Giắc trạm D-sub combo 21W4

| Chân | Tín hiệu | Bảo vệ | Dây phía xe / phía trạm |
|---|---|---|---|
| A1 | SERVO+ tay trái | F3 40 A | BW07T / BW07S |
| A2 | GND tay trái | F3 | BW07TN / BW07SN |
| A3 | SERVO+ tay phải | F4 40 A | BW08T / BW08S |
| A4 | GND tay phải | F4 | BW08TN / BW08SN |
| 1 | +12V_CTRL vào vòng trạm | FCTRL C4 | E01 / E02 |
| 2 | chắn GND | - | GD1 |
| 3 | vòng trạm về (TRAM_OUT) | FCTRL C4 | E09 / E06 |
| 4 | chắn GND | - | GD1, GD2 |
| 5 | DOCK_L (sau khoá trái) | FCTRL C4 | E10 / E07 |
| 6 | chắn GND | - | GD2, GD3 |
| 7 | DOCK_R (sau khoá phải) | FCTRL C4 | E11 / E08 |
| 8 | chắn GND | - | GD3, GD4 |
| 9 | 12V_AUX | FAUX C2 | AX01 / AX03 |
| 10 | USB VBUS (chỉ nhận biết; hub trạm tự cấp nguồn); chỉ kề chân 1 (nguồn vòng) và chắn 2 | - | U03 / U03B |
| 11 | dự phòng (để trống: kề chân vòng về 3) | - | - |
| 12 | dự phòng (để trống: kề chân vòng về 3) | - | - |
| 13 | USB D+ | - | U03 / U03B |
| 14 | USB D− | - | U03 / U03B |
| 15 | USB GND | - | U03 / U03B |
| 16 | GND_AUX (chân chắn 2, 4, 6, 8 nối về đây) | FAUX C2 | AX02 / AX04, GD4 |
| 17 | dự phòng | - | - |

- Tiếp điểm nguồn 40 A [R3b 5.4, chưa kiểm datasheet]: dòng chặn mỗi tay 40,2 A (101%), liên tục ~16,1 A,
  ở giới hạn mô-men 23,1 A. F3/F4 MEGA 40 A ≥ 1,25 × 23,1 = 28,9 A và THẤP HƠN dòng chặn đồng thời 40,2 A (+1 STS3215 mỗi tay; chỉ xảy ra khi cả 7 servo cùng kẹt và bỏ qua TORQUE_LIMIT; cầu chì 40 A khi đó chảy chậm, bảo vệ dây 8 AWG vẫn đạt; tiếp điểm 21W4 ở 101% định mức: chờ kỹ sư điện xác nhận hoặc nâng F3/F4 lên nấc 50 A nếu tiếp điểm cho phép, DK-N55), ≤ tiếp điểm 40 A và ≤ 8 AWG ×0,82
  (41,0 A). Tiếp điểm chỉ nhận dây ≤ 8 AWG: 4 AWG dừng ở khối cọc CDK trên đế ngàm, đuôi 8 AWG 0,15 m tới tiếp điểm.
- Chân tín hiệu 5 A: vòng trạm và DOCK dưới FCTRL 4 A, 12V_AUX dưới FAUX 2 A; USB 2.0 qua chân 10, 13–15 là rủi ro (DK-N45).
- Bố trí hàng [giả định]: trên 1–9, dưới 10–17 so le. Không cặp chân kề nào nằm trong nhóm vòng + 12V_AUX (1, 3, 5, 7, 9); test
  `test_giac_21W4_khong_ke_chan_vong` kiểm theo `day_L.dsub_adjacent()`.
- Không cắt dòng tải bằng giắc: mở lẫy khoá là hở vòng E-stop, K1 nhả trước khi tiếp điểm tách.
- Isc tại thanh cái hộp FB trên trạm (sau F3/F4, 4 AWG 1,04 m, 8 AWG 0,60 m, 2 tiếp điểm, đi + về), 40T / P45B / giả định hiện hành:
  517 / 612 / 325 A.

## L7. Vỏ pin 12 cell (ghi chú cho CAD, chưa sửa `cad/`)

- 3S4P xếp 4 × 3: mặt cắt 86,2 × 64,7 mm, dài ~76 mm gồm nickel [tự tính]; bản 15 cell cũ 107,8 × 64,7 mm.
- BMS Daly 100 A ~184 × 65 × 33 mm [snippet, bản FAN]; bản Smart Bluetooth đo khi có hàng (DK-N2).
- Bỏ khoang công tắc ngắt pin, LOTO M1-PW-018, cọc M8 xuyên nắp; thêm chỗ giắc J0 Anderson SB120 (vỏ ~ 68 × 51 mm [nhớ từ catalogue, kiểm khi có hàng]) và
  cổng sạc XT60E-F trên thành vỏ. Khối cell tối đa 840 g.

## L9. Đế bàn (bộ tuỳ chọn; `bang-day-de-ban.csv`, `vat-tu-de-ban.csv`)

Không thuộc robot chuẩn. Adapter kín 12 V 15 A → thanh +12 V → rơ-le chính KB 40 A → 2 cầu chì ATO 15 A (mỗi tay một) → 21W4 A1/A3;
GND về A2/A4. Vòng trạm (khoá + E-stop trạm, chân 1–3) nối tiếp cuộn rơ-le tự giữ KHB (JQ1P) và cuộn KB; nút Reset song song tiếp điểm
tự giữ, nên nhả E-stop xong phải bấm Reset. 12V_AUX (chân 10/11) lấy trước KB qua cầu chì 2 A: laptop vẫn thấy camera khi nhấn E-stop.
USB chân 10, 13–15 ra laptop. Nửa cái trên đế bàn cũng có chân chắn GND 2, 4, 6, 8 (BGD1–BGD4). Thêm KHB so với R3b (một rơ-le 40 A không tự giữ được qua vòng mà không cho dòng tay đi qua chân tín hiệu).
Hai tụ 4.700 µF 25 V đặt sau KB trên đế bàn, không trên trạm, để không cộng 9,4 mF vào xung đóng K1 ở chế độ di động (DK-N30).

| Mục | Giá trị |
|---|---:|
| Adapter | 12 V × 15 A = 180 W |
| Mỗi tay ở giới hạn mô-men `di_dong` | 23,14 A; hai tay 46,3 A |
| Ngân sách servo `tram_ban` hiện ở `spec.ROBOT_CONFIGS` | 12,0 A + 12V_AUX 0,6 A = 84% adapter |
| **Đề xuất** (sửa `spec` ở lượt SDK) | **11,4 A** + 0,6 A = 12,0 A = 80%; đo bằng thanh ghi dòng STS, quá 0,5 s thì hạ mô-men (R3b mục 6) |
| Cầu chì / dây | BF1, BF2 15 A ATO, 12 AWG (20,5 A); vòng 1 A và 12V_AUX 2 A, 22 AWG; adapter phải có quá dòng ≤ 16,4 A (dây ra 14 AWG) |
| Sụt áp adapter → tay (1 m 14 AWG + 0,4 m 12 AWG, đi + về, 12 A) | 0,30 V [tự tính] |
| Rơ-le KB đóng vào tụ | ~12 mF (2 × 4.700 µF + tụ servo hai tay [giả định]): xung ~400 A vài chục µs, nguy cơ dính tiếp điểm, bàn không có ARM_SENSE (DK-N57) |

Ngân sách 12 A là 26% mức `di_dong`: chế độ bàn chỉ dùng cho tải nhẹ, SDK phải giới hạn động (DK-N50). Quá dòng adapter,
rơ-le 40 A cắt DC: DK-N57.

## L8. Điều kiện còn mở

Từ trước: DK-N1, DK-N2, DK-N3, DK-N7, DK-N17, DK-N19, DK-S2 (không K1_AUX), DK-N24 (sụt áp), DK-N28,
DK-N30 (đóng K1 không nạp trước). Bảng dây sửa hoặc thêm:

- **DK-N21**: Chốt trục nâng: linh kiện đã chọn (EML-0530B 0,6 A BOM N09, rơ-le K10 nay trên tấm điện, cầu chì FLAT 2 A trên cọc SERVO+, 1N5408, KW12-N); còn mở: lực khoá 6 N so thiết kế chốt, bật liên tục, module rơ-le kích chắc ở 3,3 V (đo), một cảm biến không phân biệt chốt tì đỉnh răng.
- **DK-N22**: Kỹ sư điện xác nhận bỏ công tắc ngắt pin riêng; J0 = Anderson SB120 120 A (cắt cả hai cực, không rút khi có tải).
- **DK-N25**: DC-13 của LA38P-11ZS (2NC, nay 2 nút: đỉnh cột và trạm), LA38-11BN và công tắc khoá KW12 (T04) với cuộn K1 + KH ~1,1 A @ 12 V: cần datasheet hãng.
- **DK-N29**: Cầu chì lưỡi 1 kA còn ở FW1–3, FLIFT, FLAT (cọc SERVO+, Isc 820 A với 40T, 1.084 A với P45B > 1 kA) và trong hộp FB trên trạm (517–612 A, tinh-toan.md L6b): FLIFT/FLAT (S2) chỉ chấp nhận với cell 40T; đo R pack + đường ở EVT. Nhánh BAT_F (cả 12V_AUX) dùng aptomat DC 6 kA.
- **DK-N31**: Buck đã chọn: TPS54560 5 A (BOM V04) thay buck LeKiwi; còn đo: 5,1 V tại cổng USB-C Pi khi tải 3 A qua PW03/PW04 và cấu hình PSU_MAX_CURRENT.
- **DK-N33**: Đầu nối servo: dòng mỗi chân 5264 của STS3095/STS3215 (giả định ~3 A), STS3095 có cổng nguồn riêng hay không, cỡ dây cáp N14; bo bus A07 chạy DATA khi không cấp V+ vào bo và qua ADuM3160 (12 Mbps, bus 1 Mbps); DATA trục nâng ~1,2 m qua vòng trục nâng về BUS_B cổng S2 (S2): đo dạng sóng 1 Mbps ở EVT.
- **DK-N34**: Hộp cầu chì V07: cầu chì ATO chuẩn hay mini, cỡ cọc vào (đầu cos SC10-6 cho 8 AWG), kiểu ngõ ra và thanh âm, định mức thanh cái ≥ 40 A: cần mẫu thật.
- **DK-N35**: Vòng dịch vụ trục nâng: dây 4 AWG silicon bán kính uốn 60 mm [giả định], mỏi uốn ở hành trình 0,45 m; S2 thêm 16 AWG (lift), 22 AWG (chốt, 12V_AUX, vòng trạm, DOCK), 24 AWG (DATA lift, cảm biến chốt) và cáp USB vào cùng bó; vòng cuộn cổ tay ±160°: thử chu kỳ ở EVT.
- **DK-N24**: Sụt áp trục chính + nhánh xa nhất 6,37% ở dòng liên tục 42,6 A (trục chính 3,56%, S2 thêm ~0,95%), vượt mong muốn 3% của SOR-016; điện áp tại servo ở đỉnh ngân sách 51,8 A khi pin 9,0 V chưa tính so với điện áp tối thiểu STS: đo ở EVT.
- **DK-N36**: Đường GND: bo bus tay sau ADuM3160 (GND bus = GND servo của hộp FB trên trạm), cuộn chốt qua rơ-le K10. Đứt BW07N/BW08N (hoặc tiếp điểm A2/A4) thì servo tay mất nguồn, SDK báo mất bus; nếu còn đường GND thứ hai (camera cổ tay, khung nhôm, GND_AUX) thì tới 37,5 A đi qua dây 22 AWG, C2 không bảo vệ. Phép thử động ở EVT: rút BW07N rồi BW08N khi servo có tải nhẹ, dòng qua cáp USB và chân 16 < 10 mA; điện trở GND servo tay ↔ GND Pi > 1 MΩ kể cả qua vỏ servo, khung nhôm, camera cổ tay. **Bus đế BUS_B chưa cách ly và nay mang cả trục nâng (S2)**: đứt P.LIFT− thì dòng lift (≤ 7,5 A, FLIFT) về qua GND USB của BUS_B. Đo hoặc thêm ADuM3160 cho BUS_B.
- **DK-N55**: Giắc 21W4: tiếp điểm nguồn 40 A (định mức, điện trở, nhiệt ở 37,5 A chặn), cỡ dây nhận ≤ 8 AWG, lực cắm, số lần cắm rút, bố trí hàng chân tín hiệu (đang giả định hàng trên 1–9, dưới 10–17 so le) và thứ tự tiếp xúc (chân vòng 1/3 phải tách trước A1–A4 khi rút; nếu không, dựa vào lẫy khoá + công tắc khoá) chưa có datasheet; đang giả định 1 mΩ/tiếp điểm. F3/F4 MEGA 40 A mã Littelfuse 0298040 chưa xác minh; MEGA cho ~135% lâu, cao hơn tiếp điểm và dây 41 A: chấp nhận vì firmware chặn 21 A/tay. Đo sụt áp và nhiệt tiếp điểm ở 15 A liên tục 30 phút.
- **DK-N56**: Vòng E-stop qua trạm: thêm 3,47 m dây 22 AWG (E01–E06, E09) và 2 chân 21W4 vào mạch cuộn K1 + KH (~1,2 A); sụt toàn mạch cuộn ~0,5 V, cuộn K1 còn ~8,5 V khi pin 9,0 V: kiểm điện áp hút contactor (DK-N1), DC-13 của KW12/LA38P (DK-N25). Chống chập (QC2 M1): chân vòng 1, 3, 5, 7 và 12V_AUX 9 không kề nhau, xen chân chắn GND 2, 4, 6, 8; E01, E09, E10, E11, AX01 đi ống bọc riêng từng sợi qua vòng dịch vụ. Phép thử nghiệm thu và định kỳ: nhấn E-stop trạm, tháo từng khoá trái/phải, nhấn E-stop đỉnh cột: mỗi lần K1 nhả (ARM_SENSE = 0), Reset mới đóng lại. Thử chập (giắc rời, đồng hồ đo): chập từng cặp chân kề nhau theo bố trí hàng, không cặp nào bỏ qua khoá hoặc E-stop (chập xuống chân chắn phải làm C4/C2 cắt hoặc K1 nhả).
- **DK-N57**: Đế bàn (bộ tuỳ chọn): mua adapter 12 V 15 A có quá dòng ≤ 16,4 A (chịu dòng dây ra 14 AWG ×0,82), ghi kiểu cắt và đáp ứng đỉnh; BF1/BF2 hạ 20 → 15 A; ngân sách servo đề xuất 11,4 A (+ 12V_AUX 0,6 = 12,0 A = 80%), `spec.ROBOT_CONFIGS['tram_ban']` còn 12,0 A: sửa ở lượt SDK. Rơ-le KB 40 A đóng vào ~12 mF (2 × 4.700 µF + tụ servo hai tay): xung ~400 A vài chục µs, dễ dính tiếp điểm, bàn không có ARM_SENSE để phát hiện: thêm NTC/điện trở nạp trước hoặc đo xung ở EVT (mở rộng DK-N30); rơ-le cắt DC 12 V ở 15 A.
- **DK-N58**: SDK `DockSwitches`: với nối dây hiện tại DOCK_R = cả hai khoá, nên khoá trái mở thì DOCK_R cũng 0: Studio phải báo 'khoá trái (khoá phải chưa biết)' và báo lỗi nối dây khi DOCK_R = 1 mà DOCK_L = 0. Việc kế tiếp (QC2 M1 c): opto TRAM_OUT (nút 21W4.3, kênh trống OPTO2.IN2) để Studio tự chạy bài thử E-stop trạm. Chỉ chẩn đoán, không an toàn.

---

Phần A cũ (chuỗi A, v0.3, lưu đối chiếu): `archive/v0.4/electrical/tinh-toan-chuoi-A.md`.
