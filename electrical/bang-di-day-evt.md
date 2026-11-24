# Bảng đi dây EVT (thay PCB nguồn cho bộ #0): chuỗi L cho P-L

> **v0.5 chuỗi L (chốt 23/11/2026 tối, cấu hình P-L).** Số dòng, cỡ dây: `tinh-toan.md` phần L; từng sợi: `bang-day.csv` (mã BW..).
> Datasheet contactor và BMS chưa có: DK-N1, DK-N2 mở. Bỏ công tắc ngắt pin riêng, chỉ cách ly bằng J0: chưa chốt (DK-N22, chờ kỹ sư điện).
> Bảng EVT trước (mạch K2/K3/KT/K4, mục 1–6, chế độ `bang_di_day` của SDK còn mô phỏng): `archive/v0.4/electrical/bang-di-day-evt.md`.
> **S2 trạm hai tay (24/11):** E-stop đỉnh cột nay nối sau vòng trạm (21W4 chân 5–6: 2 khoá + E-stop trạm, `bang-day.csv` E01–E11); FB-L/FB-R, hub lên trạm; lift và cuộn chốt lấy nguồn ở tấm điện (FLIFT, FLAT), K10 trên tấm điện. Khi bảng này lệch, `bang-day.csv` và `tinh-toan.md` L5, L6b là bản đúng.

## L. Chuỗi L cho P-L (v0.5)

P-L: 2 tay AM-ARM200 nguyên (vai, khuỷu đơn), 6+1: 7 STS3095 (2 pan, 2 vai, 2 khuỷu, 1 trục nâng) + 11 STS3215 (2 gập, 2 yaw, 2 cuộn,
2 kẹp, 3 bánh). Dòng chặn cả robot cộng điều khiển và cuộn chốt [giả định] 103,1 A; mỗi tay 40,2 A (vượt nhẹ F3/F4 40 A khi cả 7 servo một tay cùng chặn); ngân sách
firmware 56,1 A (`tinh-toan.md` L1, L2; số đúng luôn lấy ở đó).

### L.1 Sơ đồ

Từng sợi (204 sợi, đếm theo `bang-day.csv`: mã, điểm đầu/cuối theo chân, AWG, chiều dài đường đi và cắt, màu, đầu cos hai đầu, cầu chì, vòng dịch vụ):
`bang-day.csv`, sinh từ `day_L.py` qua `tinh_toan.py`; vật tư cắt/bấm `vat-tu-day.csv`; hình `so-do-khoi.png`, `di-day.png`.

```
+pack ─ BW01 ─ F1 MEGA 80 A ─ BW02 ─ J0 SB120 (+) ══ BW03 2 AWG ══ BAT_F (cọc M8)        (pack, BMS, F1, J0 trong/trên vỏ pin)
BMS P− ─ BW04 ─ J0 SB120 (−) ══ BW04R 2 AWG ══ GND (cọc M8)                               (J0 cắt cả hai cực)
BAT_F ─┬─ BW05 ─ K1 100 A kiểu SW80 ─ BW06 ─ SERVO+ (cọc M8)
       ├─ aptomat DC FPI C6 6 kA ─ PW01 ─ buck TPS54560 5 A ─ PW03/PW04 USB-C ─ Pi 5
       ├─ aptomat DC FCTRL C4 6 kA ─ C01 ─ +12V_CTRL (mạch dừng)
       └─ aptomat DC FCHG C6 6 kA ─ CH01 ─ cổng sạc XT60E-F ─ CH02 ─ GND        (sạc qua J0 + F1: rút J0 là cách ly cả cổng sạc)
SERVO+ ─┬─ BW07A ─ F3 MEGA 40 A ─ BW07 4 AWG (vòng trục nâng) ─ CDK.L ─ BW07T 8 AWG ─ 21W4 A1/A2 ─ BW07S ─ FB-L trên trạm:
        │                                                        3 × 7,5 A STS3095, 4 × 3 A STS3215 ─ BW07N 4 AWG về GND
        ├─ BW08A ─ F4 MEGA 40 A ─ BW08 4 AWG (vòng trục nâng) ─ CDK.R ─ BW08T 8 AWG ─ 21W4 A3/A4 ─ BW08S ─ FB-R: tay phải ─ BW08N về GND
        ├─ FLIFT 7,5 A inline ─ P.LIFT 16 AWG ─ servo trục nâng (tấm điện, rời FB-L từ S2)
        ├─ FLAT 2 A inline ─ L01 ─ rơ-le K10 trên tấm điện (Q10; cuộn ← Pi BCM 24 qua G13) ─ L03 ─ cuộn chốt EML-0530B ─ L04 ─ GND
        └─ FW1–FW3 3 A inline ─ P.W1–3 22 AWG ─ 3 servo bánh
Hub (hộp điện trạm) ─ ADuM3160 ─ bo bus trái ; hub ─ ADuM3160 ─ bo bus phải ; servo trục nâng trên BUS_B (bus đế)   (GND Pi không chạm GND servo tay)

+12V_CTRL ─ ES-NC1 ─ ES-NC2 ─ K5 (Pi cho phép) ─┬─ RST-NO1 ─┬─ HOLD ─┬─ cuộn KH ─ GND           (KH: rơ-le tự giữ JQ1P)
                                                 └─ KH.1 NO ─┘        └─ cuộn K1 (1N5408 + Zener 14 V) ─ GND
HOLD ─ opto ─ ESTOP_OK ;  SERVO+ (sau FW1) ─ opto ─ ARM_SENSE ;  +12V_CTRL ─ RST-NO2 ─ opto ─ RESET_SENSE
công tắc chốt NO xuống GND ─ BCM 25 LIFT_LATCH_RETRACTED ;  LIFT_BOT, LIFT_TOP NC xuống GND ─ BCM 5, BCM 6
```

Nguồn servo hình sao: mỗi servo một cặp V+/GND từ hộp cầu chì (STS3095 16 AWG tới mối nối rồi hai đuôi 22 AWG vào cả hai cổng;
STS3215 22 AWG thẳng vào cổng A). DATA nối chuỗi theo bus; cáp 3 chân giữa hai servo chỉ giữ DATA đi suốt, V+ và GND cắt giữa.

Mã cũ (bảng 21 dây trước 23/11) sang mã mới: BW01–BW06 giữ; BW07/BW08 giữ (nay tới cọc CDK trên đế ngàm, rồi qua 21W4 lên FB-L/FB-R trên trạm) + BW07A/BW08A, BW07N/BW08N;
BW09 bỏ (trục nâng dùng FLIFT trên tấm điện, bánh dùng FW1–3); BW10/BW11 → P.* và đuôi P.*.A/B; BW12 → FH.PI.i, PW01–PW04; BW13 → FH.CHG.i, CH01, CH02;
BW14 → BL0–BL3; BW15–BW17 → C01–C20; BW18 → L01, L03, L04; BW19 → G13 (+ G13V, G13G); BW20 → G15, G16; BW21 → G01–G12. (`sdk/faibotic/gpio_pi.py`
còn ghi BW19/BW20 trong chú thích: đổi ở luồng SDK.)

Bỏ so với bảng EVT trước: K2 (thay bằng KH), K3, KT, K4 + R_pc (không nạp trước), MCB DC, cầu chì sứ ABC. Hai NC của nút E-stop nằm
nối tiếp ngay trong mạch cuộn K1 và KH: nhấn là cả hai cuộn mất điện; nhả nút không tự đóng lại (KH đã nhả, chỉ RST-NO1 mới
cấp lại). K5 vẫn nối tiếp để Pi có thể cắt nhưng không thể tự đóng.

### L.2 Cách chạy

1. Cắm J0: BAT_F có điện, Pi khởi động. K1 hở, servo và cuộn chốt không có điện: chốt cài, lift nằm trên chốt.
2. Pi bật K5 khi phần mềm sẵn sàng. Nhấn Reset: KH hút và tự giữ qua KH.1, K1 hút **ngay khi nhấn** (không trễ, không nạp
   trước: xung nạp tụ servo ~460 A trong ~0,1 ms [tự tính, giả định], `tinh-toan.md` L3). SDK vẫn chỉ cho chạy sau khi nhả Reset
   và ARM_SENSE giữ cao `arm_confirm_s`.
3. Dừng: E-stop (cả hai NC), Pi hạ K5, cầu chì 3 A đứt, mất 12 V: KH và K1 nhả, SERVO+ về 0, cuộn chốt mất điện.
4. Trục nâng: như SDK `safety.LiftLatch` (R2 7.2): servo giữ → BCM 24 lên → cảm biến "rút" → chạy; dừng: servo giữ → BCM 24
   xuống → cảm biến "cài" → tắt mô-men lift. Giới hạn răng hiện chưa đủ tải thiết kế: SDK từ chối lệnh lift trừ khi
   `allow_lift_underpowered=True` (thử có giám sát).

### L.3 Bảng trạng thái (chuỗi L)

| # | Trạng thái | KH | K1 | SERVO+ | Chốt lift | Đúng ý? |
|---:|---|---|---|---|---|---|
| 1 | J0 cắm, chưa Reset | hở | hở | 0 | cài | có |
| 2 | Nhấn Reset (K5 cao, E-stop nhả) | hút | đóng ngay | đủ (xung nạp tụ) | cài tới khi SDK rút | có; xung đóng chưa đo |
| 3 | Nhả Reset, chạy | giữ qua KH.1 | đóng | đủ | theo SDK | có |
| 4 | Nhấn E-stop khi chạy | hở | hở (cả hai NC cắt cuộn) | 0 | cuộn mất điện, lift rơi ≤ 1 nấc | có |
| 5 | Nhả E-stop | hở | hở | 0, chờ Reset | cài | có (không tự chạy) |
| 6 | Reset kẹt nhấn | hút | đóng | đủ | | **KHÔNG chặn bằng phần cứng** (bảng EVT trước có KT/RST-NC gương): SDK thấy RESET_SENSE cao, từ chối reset và trip; nhấn E-stop vẫn cắt |
| 7 | Pi hạ K5 | hở | hở | 0 | cài | có |
| 8 | Pi treo, chân giữ mức | giữ | giữ | giữ | giữ | chỉ dừng bằng E-stop (DK-S4) |
| 9 | Cầu chì 3 A đứt / mất 12 V | hở | hở | 0 | cài | có |
| 10 | K1 dính tiếp điểm, nhấn E-stop | hở | **dính** | **đủ** | cuộn chốt **vẫn có điện** nếu Q10 còn bật; SDK hạ BCM 24 khi trip | KHÔNG: chỉ ARM_SENSE báo |
| 11 | KH dính | dính | đóng sau khi nhả E-stop mà không cần Reset | đủ | | KHÔNG: mất chốt tự giữ; dừng E-stop vẫn còn (NC cắt cuộn K1 trực tiếp). SDK: ESTOP_OK cao khi K5 hạ → lỗi "dính" |
| 12 | Chập hai kênh E-stop (chung cáp) | | | | | KHÔNG phát hiện |
| 13 | Q10 chập khi K1 đóng | | đóng | đủ | luôn rút | phát hiện ở lần cài (cảm biến không báo "cài": SDK giữ mô-men, báo lỗi); E-stop vẫn cắt cuộn qua K1 |
| 14 | Cảm biến chốt đứt dây | | | | đọc "không rút" | SDK không cho lift chạy (an toàn khi hỏng) |
| 15 | Rút J0 khi đang chạy | hở | hở | 0; Pi tắt đột ngột | cài | có, nhưng cấm rút có tải (hồ quang): quy trình |
| 16 | Cầu chì lưỡi nhánh gặp Isc > 1 kA | | | | | **có thể không cắt được**: đo R pack + đường; không dùng P45B |

Ca 6 và 11 là nơi chuỗi L kém hơn bảng EVT trước (mất RST-NC gương và KT). Mức an toàn vẫn Category B, PL b, không giao khách.

### L.4 Vật tư chuỗi L

Danh sách đủ (dây theo cỡ/màu, đầu cos, giắc, cầu chì, đế, module) kèm link: `vat-tu-day.csv`. Linh kiện chính:

| Mã | Tên | Chọn | SL | Ghi chú |
|---|---|---|---|---|
| BOM A04 | Cell 21700 | 3S4P, 12 cell; P45B **không** dùng với cầu chì lưỡi | 12 | |
| BOM A03 | BMS | Daly 3S 100 A | 1 | DK-N2 |
| BOM N01, N18 | F1, F3, F4 | MEGA bắt bu-lông 80 A (F1), 40 A (F3, F4) + đế | 3 | N01: F1; N18: 2 bộ 40 A (`vat-tu-day.csv` MEGA-40) |
| BOM A08 | K1 | contactor 100 A cuộn 12 V kiểu SW80 | 1 | DK-N1, DK-N30 |
| SB120 | J0 | Anderson SB120 120 A, 2 vỏ, 4 tiếp điểm 2 AWG | 1 cặp | DK-N22 |
| BOM V05 / N03 | E-stop 2NC, Reset | LA38P-11ZS; LA38-11BN + khối tiếp điểm NO gắn thêm | 1 + 1 | DK-N25 |
| BOM N04 | KH rơ-le tự giữ | JQ1P 12 V 10 A | 1 | |
| K5-MOD, OPTO-4 | K5, opto | module 1 rơ-le 5 V opto; module PC817 4 kênh | 1 + 1 | |
| BOM V07 | Hộp cầu chì lưỡi | 12 mạch có thanh âm, FB-L và FB-R trong hộp điện trạm | 2 | DK-N34 |
| FHAC | Đế cầu chì inline ATO | Littelfuse FHAC0001ZXJ (FW1–3) | 3 | DK-N29 |
| MCB-DC | Aptomat DC nhánh BAT_F | CHINT NB1-63DC 1P 6 kA: C6 (Pi), C4 (mạch dừng), C6 (cổng sạc) + ray DIN | 3 | Isc BAT_F 939–1.304 A |
| ISO-USB | Cách ly USB bo bus tay | module ADuM3160 (icdayroi) | 2 | DK-N36 |
| ATO | Cầu chì lưỡi chuẩn | 8 × 7,5 A, 12 × 3 A, 1 × 2 A | 21 | |
| BOM V04 | Buck 5 V cho Pi 5 | TPS54560 5 A (thay buck LeKiwi) | 1 | DK-N31 |
| BOM A10 | Bộ sạc 3S 12,6 V 5 A | + đuôi XT60 đực; cổng XT60E-F trên vỏ đế | 1 | |
| BOM N09, K10, N08 | Chốt solenoid, Q10, cảm biến | EML-0530B 0,6 A; module 1 rơ-le 5 V opto (K10); KW12-N ×3 (2 hành trình + chốt) | 1 + 1 + 3 | DK-N21 |
| HUB-4, USB | Hub USB trong hộp điện trạm (`tinh-toan.md` L6b), cáp A→C ×3, nối dài 1,5 m | | | DK-N35 (vòng trục nâng) |
| M1-PW-008 | Công tắc ngắt pin | **bỏ** | 0 | J0 thay (DK-N22) |

Không dùng ở chuỗi L: M1-WB-001..009 (MCB, K2/K4, KT), M1-WB-012 (R_pc), M1-WB-018/021/022 (đế và cầu chì sứ ABC) trừ khi kỹ sư
điện yêu cầu giữ khả năng cắt 10 kA.
