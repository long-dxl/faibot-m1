# Bảng vật tư (BOM)

Danh sách vật tư cho 1 robot Faibot M1, cấu hình chuẩn (2 tay, đế 3 bánh, trục nâng, trạm hai tay), bản v0.5 trước EVT. Mỗi hạng mục một dòng, số lượng là tổng cho 1 robot. Link kiểm ngày 23/11/2026 đến 25/11/2026. Dòng ghi "Đang cập nhật" là chưa tìm được trang mua đã kiểm. Bộ đế bàn tuỳ chọn và gói leader nằm ở hai bảng cuối trang.

Chú thích: † dòng chưa xác minh (biến thể hoặc nhà bán còn phải kiểm trước khi đặt).

## Servo

18 servo bus Feetech: 7 STS3095 (pan, vai, khuỷu mỗi tay và trục nâng) và 11 STS3215 (cổ tay gập, cổ tay xoay, cổ tay yaw, kẹp mỗi tay và 3 bánh). Bảng ID servo nằm ở mục "Servo ID" trong hướng dẫn lắp ráp. ID theo `sdk/faibotic/spec.py`.

| Hạng mục | Model / ghi chú | SL | Link quốc tế | Link VN |
|---|---|--:|---|---|
| A01 · STS3095 12 V C002 | pan, vai, khuỷu ×2 tay + trụ nâng | 7 | [Alibaba](https://www.alibaba.com/product-detail/FEETECH-STS3095-12V-95KG-servo-AI_1601045980686.html) | — |
| A02 · STS3215 12 V C018 1:345 | cổ tay gập ×2, xoay ×2, yaw ×2, kẹp ×2, bánh ×3 | 11 | [Alibaba](https://www.alibaba.com/product-detail/Lerobot-360-Degree-Smart-Servo-for_1601563310522.html) · [DFRobot](https://www.dfrobot.com/product-2962.html) · [Evelta](https://evelta.com/st3215-c018-12v-30kg-cm-1-345-dual-shaft-ttl-serial-servo-metal-gears-magnetic-encoder/) | — |

## Điện tử, camera và nguồn

Bo điều khiển, camera, pin và BMS, bảo vệ, dây, đầu cos, giắc và vật tư nhỏ.

| Hạng mục | Model / ghi chú | SL | Link quốc tế | Link VN |
|---|---|--:|---|---|
| A03 · BMS Daly 3S Li-ion 100 A | bán tối thiểu 2 cái, cái thứ hai là dự phòng | 1 | [Alibaba](https://www.alibaba.com/product-detail/Daly-Li-ion-3S-bms-12_1600112464909.html) | — |
| A04† · Cell Li-ion 21700 xả ≥ 30 A (40T/P42A), 3S4P = 12 cell | cell dùng thật xả ≥ 30 A, mua qua nhà phân phối trong nước | 12 | — | [kimpower.vn](https://kimpower.vn/san-pham/cell-pin-samsung-40t-21700-xa-35a-dung-luong-4000mah/) |
| A06† · Camera USB 720p góc rộng ×3 |  | 3 | [DFRobot](https://www.dfrobot.com/product-2537.html) | — |
| A07 · Bo điều khiển bus servo ×3 | Feetech FE-URT-1 hoặc Waveshare Bus Servo Adapter A | 3 | [Alibaba](https://www.alibaba.com/product-detail/Feetech-FE-URT-1-Multi-function_1601003839109.html) · [Waveshare](https://www.waveshare.com/bus-servo-adapter-a.htm) | — |
| A08† · Contactor 100 A cuộn 12 V | loại Albright SW80, ZJW100A | 1 | [Alibaba](https://www.alibaba.com/product-detail/SW80-NO-type-Coil-Voltage-12V_62329150139.html) | — |
| A10 · Bộ sạc 3S 12,6 V | bản tìm được 2 A; thiết kế ghi 5 A | 1 | — | [Thegioiic](https://www.thegioiic.com/adapter-sac-pin-3s-12-6v-2a-dau-dc-5-5x2-5mm-dai-1m-co-led) |
| V01 · Raspberry Pi 5 2 GB |  | 1 | [Kiwi Electronics](https://www.kiwi-electronics.com/en/raspberry-pi-5-computers-accessories-415/raspberry-pi-5-2gb-20129) · [Adafruit](https://www.adafruit.com/product/6007) | [hshop](https://hshop.vn/may-tinh-raspberry-pi-5-made-in-uk) |
| V02 · Tản nhiệt Raspberry Pi 5 |  | 1 | [Adafruit](https://www.adafruit.com/products/5815) | [hshop](https://hshop.vn/tan-nhiet-raspberry-pi-5-heatsink-pwm-fan) |
| V03† · Thẻ microSD 32 GB |  | 1 | [SanDisk](https://www.sandisk.com/products/memory-cards/microsd-cards/sandisk-ultra-uhs-i-microsd?sku=SDSQUA4-032G-GN6MA) | [Thegioiic](https://www.thegioiic.com/the-nho-micro-sdhc-32gb-class10-ghi-10mb-s-doc-48mb-s) |
| V04 · Mạch hạ áp 12 V → 5 V 5 A | TPS54560, chọn 5 V bằng pad | 1 | — | [hshop](https://hshop.vn/mach-giam-ap-dc-dc-buck-tps54560-60vdc-5a-600khz) |
| V05† · Nút E-stop 22 mm 2NC | LA38P-11ZS | 1 | — | [Thegioiic](https://www.thegioiic.com/la38p-11zs-nut-nhan-dung-khan-cap-660v-10a-2nc-22mm-nap-do) |
| V06† · Cầu chì lá 10 A ×7 | trong bộ 120 cái | 1 | — | [Thegioiic](https://www.thegioiic.com/bo-cau-chi-xe-120-cai-5a-10a-15a-20a-25a-30a) |
| V07† · Hộp cầu chì lá 12 mạch ×2 |  | 2 | [Nilight](https://www.nilight.com/collections/most-bought-icart/products/nilight-12-way-blade-fuse-block-12-circuits-with-negative-bus-fuse-box-holder-with-led-indicator-ato-atc-fuse-panel-waterproof-cover-for-12v-automotive-cars-marine-boats-rvs-trailers-2-years-warranty) | — |
| N01† · Cầu chì tổng MEGA/bắt bu lông 80 A + đế |  | 1 | Đang cập nhật | Đang cập nhật |
| N02† · Cầu chì lá 3 A ×11 | mini blade, mỗi STS3215 một cái | 1 bộ (11 cái) | Đang cập nhật | Đang cập nhật |
| N03† · Nút Reset nhả 22 mm | LA38-11BN | 1 | — | [Thegioiic](https://www.thegioiic.com/la38-11bn-nut-nhan-nha-mau-xanh-duong-22mm-10a) |
| N04 · Rơ-le 12 V tự giữ | JQ1P-12V-F, 10 A SPDT | 1 | — | [Thegioiic](https://www.thegioiic.com/jq1p-12v-f-ro-le-12vdc-10a-spdt-5-chan) |
| N05† · Dây 4 AWG đỏ/đen, nhánh tay |  | 4,8 m | — | [Thegioiic](https://www.thegioiic.com/4awgx1-day-dien-silicon-chiu-nhiet-sieu-mem-mau-den-loai-tot) |
| N06† · Dây 2 AWG, trục chính | nhà bán theo bội số 50; nên mua cắt lẻ trong nước | 2,5 m | [Alibaba](https://www.alibaba.com/product-detail/High-temperature-2-AWG-Copper-Electrical_1601118536933.html) | — |
| N07 · Đầu cos đồng mạ thiếc 25 mm² lỗ M8 SC25-8 | cho dây 4 AWG | 6 | — | [Thegioiic](https://www.thegioiic.com/sc25-8-dau-cos-tran-bang-dong-ma-thiec-co-day-25mm2-vit-m8) |
| N13† · Cáp USB-A sang USB-C 1 m ×3 | bo bus → Pi, cổng A của Pi | 3 | — | [UGREEN](https://ugreen.vn/cap-sac-nhanh-type-c-ugreen-us288) |
| N14† · Cáp nối servo 3 chân, 50 cm | thay cho cáp Feetech 26 cm ×16 + 90 cm ×2 | 18 | — | [Thegioiic](https://www.thegioiic.com/cap-noi-dai-dong-co-rc-servo-dai-50cm) |
| N18† · Cầu chì MEGA 40 A + đế | F3, F4 | 2 | Đang cập nhật | Đang cập nhật |
| N19 · Cầu chì ATO lưỡi chuẩn 7,5 A ×7 | trang ghi hết hàng | 7 | — | [Thegioiic](https://www.thegioiic.com/cau-chi-xe-may-o-to-mau-nau-ato-blade-7-5a-32v) |
| N20† · Đế cầu chì inline ATO 16 AWG có nắp | FW1-3, FLIFT, FLAT | 5 | Đang cập nhật | Đang cập nhật |
| N21† · Giắc Anderson SB120 2 cực 120 A | J0; bán tối thiểu 20 | 4 | [Alibaba](https://www.alibaba.com/product-detail/Battery-Connector-50A-120A-175A-350A_1600792110908.html) · [Digikey](https://www.digikey.ca/en/products/detail/anderson-power-products-inc/6800G1/28398323) | — |
| N22† · Module cách ly USB ADuM3160 ×2 |  | 2 | — | [ICdayroi](https://icdayroi.com/mach-chuyen-doi-usb-usb-co-cach-ly-adum3160) |
| N23† · Aptomat DC CHINT NB1-63DC 1P | C6 ×2, C4 ×1, C2 ×1 | 4 | [CHINT](https://www.chintglobal.com/sg/en/products/low-voltage/iec/final-power-distribution/nb1-63dc.html) | [baoanjsc.com.vn](https://baoanjsc.com.vn/products/dc-circuit-breaker-chint-NB1-63DC-1P-C40A-DC250V-6kA_814508_en.aspx) · [baoanjsc.com.vn](https://baoanjsc.com.vn/products/dc-circuit-breaker-chint-NB1-63DC-1P-C6A-DC250V-6kA_814511_en.aspx) |
| S01 · Đầu cos đồng trần 35 mm² lỗ M8 SC35-8 | dây 2 AWG | 14 | — | [Thegioiic](https://www.thegioiic.com/sc35-8-dau-cos-tran-bang-dong-ma-thiec-co-day-35mm2-vit-m8) |
| S02 · Đầu cos đồng mạ thiếc 25 mm² lỗ M6 SC25-6 | cọc hộp cầu chì V07 | 6 | — | [Thegioiic](https://www.thegioiic.com/sc25-6-dau-cos-tran-bang-dong-ma-thiec-co-day-25mm2-vit-m6) |
| S03 · Đầu cos cái 6,3 mm bấm dây FDD mạ bạc |  | 29 | — | [Thegioiic](https://www.thegioiic.com/dau-cos-6-3mm-bam-day-cai-fdd-ma-bac) |
| S04 · Đầu cos tròn bọc nhựa RV5.5-8 lỗ M8 | gom 2-3 dây 16/22 AWG | 8 | — | [Thegioiic](https://www.thegioiic.com/rv5-5-8-dau-cos-tron-48a-co-day-4-6-6-64mm2-vit-m8-mau-xanh-duong) |
| S05 · Đầu cos pin đặc bọc nhựa PTV1.25-18 | cọc kẹp MCB | 10 | — | [Thegioiic](https://www.thegioiic.com/ptv1-25-18-dau-cos-pin-dac-19a-co-day-0-5-1-5mm2-mau-vang) |
| S06 · Giắc XT60E-F cái bắt vít | cổng sạc trên vỏ | 3 | — | [Thegioiic](https://www.thegioiic.com/xt60e-f-jack-cam-cai-500vdc-35a-bat-vit) |
| S07 · Đầu cos pin rỗng E7508 0,75 mm² | domino, module vít | 50 | — | [Thegioiic](https://www.thegioiic.com/e7508-cos-pin-rong-co-day-0-75mm2-mau-den) |
| S08 · Đầu cos tròn bọc nhựa RV1.25-4 lỗ M4 | nút LA38, cuộn K1; bán tối thiểu 20 | 16 | — | [Thegioiic](https://www.thegioiic.com/rv1-25-4-dau-cos-tron-19a-co-day-0-25-1-65mm2-vit-m4-mau-xanh-duong) |
| S09 · Dây bus cái-cái 4 tiếp điểm 2,54 mm dài 50 cm | cắt lấy đầu Dupont cái | 3 | — | [Thegioiic](https://www.thegioiic.com/day-bus-cai-cai-4-tiep-diem-2-54mm-dai-50cm) |
| S11 · Domino 4 cực TB-2504 25 A | TB1, TB2, TB3 | 3 | — | [Thegioiic](https://www.thegioiic.com/tb-2504-cau-dau-dien-terminal-khoi-tu-dien-4-cuc-25a-600v-co-day-0-5-2-5mm2) |
| S12 · Ray DIN nhôm 35×7,5 mm dài 1 m | cắt 0,16 m cho 4 aptomat; trang ghi hết hàng | 1 | — | [Thegioiic](https://www.thegioiic.com/thanh-ray-din-nhom-35x7-5mm-dai-1m) |
| S13 · Module 1 rơ-le 5 V kích mức cao/thấp, opto | K5, K10 | 2 | — | [Thegioiic](https://www.thegioiic.com/module-1-relay-5v-kich-muc-cao-thap) |
| S14 · Module opto PC817 4 kênh | ESTOP_OK, ARM_SENSE, RESET_SENSE | 2 | — | [Thegioiic](https://www.thegioiic.com/pc817-mach-opto-cach-ly-4-kenh) |
| S15 · Diode 1N5408 3 A 1 kV | dập cuộn K1, cuộn chốt; bán tối thiểu 5 | 2 | — | [Thegioiic](https://www.thegioiic.com/1n5408-diode-chinh-luu-3a-1kv-do-201ad) |
| S16 · Zener 1N5351B 14 V 5 W | nối tiếp diode cuộn K1 | 1 | — | [Thegioiic](https://www.thegioiic.com/1n5351b-diode-zener-14v-5w) |
| S17 · Khối tiếp điểm phụ LA38/39 2NO cho nút nhấn | RST NO2; trang ghi hết hàng | 1 | — | [Thegioiic](https://www.thegioiic.com/la38-39-tiep-diem-phu-2no-cho-nut-nhan-mau-xanh-la-10a) |
| S18 · Hub USB 2.0 7 cổng có cổng nguồn 5 V | hộp điện trạm: 2 cách ly + 3 camera | 1 | — | [HACOM](https://hacom.vn/bo-chia-usb-2.0-tu-1-ra-7-cong-chinh-hang-ugreen-30374) |
| S19 · Cáp USB-C → USB-C 1 m | buck → Pi, PW03/PW04; trang ghi hết hàng | 1 | — | [Thegioiic](https://www.thegioiic.com/cap-usb-type-c-noi-usb-type-c-mau-trang-dai-1m) |
| S20 · Cáp XT60 đực 10 cm cho dây sạc | phía bộ sạc | 1 | — | [Thegioiic](https://www.thegioiic.com/cap-xt60-m-duc-dung-cho-pin-lipo-rc-dai-10cm) |
| S21 · Ống co nhiệt 10 mm 600 V |  | 1 m | — | [Thegioiic](https://www.thegioiic.com/ong-co-nhiet-10mm-600v) |
| S22 · Dây silicon 14 AWG đen 3239 | bán tối thiểu 1 | 0,3 m | — | [Thegioiic](https://www.thegioiic.com/3239-14awgx1-day-dien-silicon-chiu-nhiet-mau-den) |
| S23 · Dây silicon 14 AWG đỏ 3239 | bán tối thiểu 1 | 0,3 m | — | [Thegioiic](https://www.thegioiic.com/3239-14awgx1-day-dien-silicon-chiu-nhiet-mau-do) |
| S24 · Dây silicon 16 AWG đen siêu mềm |  | 5,7 m | — | [Thegioiic](https://www.thegioiic.com/16awgx1-day-dien-silicon-chiu-nhiet-sieu-mem-mau-den-loai-tot) |
| S25 · Dây silicon 16 AWG đỏ siêu mềm |  | 6,3 m | — | [Thegioiic](https://www.thegioiic.com/16awgx1-day-dien-silicon-chiu-nhiet-sieu-mem-mau-do-loai-tot) |
| S26 · Dây silicon 22 AWG đen siêu mềm | gồm trắng 4,4 m + cam 1,8 m thay cho màu trắng/cam | 21,4 m | — | [Thegioiic](https://www.thegioiic.com/22awgx1-day-dien-silicon-chiu-nhiet-sieu-mem-mau-den-loai-tot) |
| S27 · Dây silicon 22 AWG đỏ siêu mềm |  | 22,5 m | — | [Thegioiic](https://www.thegioiic.com/22awgx1-day-dien-silicon-chiu-nhiet-sieu-mem-mau-do-loai-tot) |
| S28 · Dây silicon 24 AWG đỏ siêu mềm | gồm vàng 10,3 m + đỏ 0,8 m | 11,1 m | — | [Thegioiic](https://www.thegioiic.com/24awgx1-day-dien-silicon-chiu-nhiet-sieu-mem-mau-do-loai-tot) |
| S29 · Dây silicon 24 AWG đen siêu mềm |  | 3,2 m | — | [Thegioiic](https://www.thegioiic.com/24awgx1-day-dien-silicon-chiu-nhiet-sieu-mem-mau-den-loai-tot) |
| S30† · Dây silicon 8 AWG đỏ/đen | đen 1,6 + đỏ 1,6 | 3,2 m | — | [Thegioiic](https://www.thegioiic.com/8awgx1-day-dien-silicon-chiu-nhiet-sieu-mem-mau-den-loai-tot) |
| S31† · Dây silicon 18 AWG đỏ/đen | đen 0,2 + đỏ 0,2 | 0,4 m | — | [Thegioiic](https://www.thegioiic.com/18awgx1-day-dien-silicon-chiu-nhiet-sieu-mem-mau-den-loai-tot) |
| S32† · Đầu cos 10 mm² lỗ M6 SC10-6 | dây 8 AWG: khối cọc đế ngàm, cọc vào hộp cầu chì V07 | 10 | — | [Thegioiic](https://www.thegioiic.com/sc10-6-dau-cos-tran-bang-dong-ma-thiec-co-day-10mm2-vit-m6) |
| S33† · Tiếp điểm nguồn 40 A của giắc D-sub combo 21W4 | cốc hàn hoặc bấm, dây ≤ 8 AWG | 10 | [NorComp](https://www.norcomp.net/series/680m-crimp-and-poke) | — |
| S34† · Khối cọc M6 4 cực trên đế cách điện in | 4 AWG ↔ đuôi 8 AWG, trên đế ngàm; kèm bu-lông, đai ốc, long đen M6 | 1 | — | [Thegioiic](https://www.thegioiic.com/san-pham/dau-cos-noi-day) |
| S35† · Cầu chì ATO lưỡi chuẩn 2 A |  | 1 | — | [ICdayroi](https://icdayroi.com/cau-chi-o-to-4s-2a) |
| S36 · Cáp USB 2.0 A đực 1,5 m cắt một đầu | Pi → chân 1-4 giắc 21W4 qua vòng trục nâng | 1 | — | [HACOM](https://hacom.vn/cap-usb-2.0-noi-dai-1.5m-vention-vas-a44-b150-black) |
| T01a · Giắc D-sub combo 21W4 40 A, 2 nửa (đực trên trạm, cái trên xe) + tiếp điểm 40 A | bán tối thiểu 50 | 2 | [Alibaba](https://www.alibaba.com/product-detail/D-Sub-21W4-Male-Mixed-Contact_1600756823919.html) | — |
| T07a† · Nút E-stop 22 mm 2NC LA38P-11ZS | thứ hai, trên trạm | 1 | — | [Thegioiic](https://www.thegioiic.com/la38p-11zs-nut-nhan-dung-khan-cap-660v-10a-2nc-22mm-nap-do) |
| T11 · Buck 5 V 5 A TPS54560 cho hub/camera trên trạm |  | 1 | — | [hshop](https://hshop.vn/mach-giam-ap-dc-dc-buck-tps54560-60vdc-5a-600khz) |
| T12† · Tụ 4.700 µF 25 V ×2 |  | 2 | — | [Thegioiic](https://www.thegioiic.com/tu-hoa-4700uf-25v-16x25mm-xuyen-lo) |

## Cơ khí

Bánh, nhôm định hình, vòng bi, khoá trạm, công tắc hành trình và chi tiết cắt gia công.

| Hạng mục | Model / ghi chú | SL | Link quốc tế | Link VN |
|---|---|--:|---|---|
| A05† · Bánh omni 100 mm (4 in) kiểu LeKiwi ×3 |  | 3 | Đang cập nhật | Đang cập nhật |
| V12† · Nhôm định hình 2040, 5 đoạn |  | 1 | — | [Thegioiic](https://www.thegioiic.com/nhom-dinh-hinh-qt-20x40-dai-1-2m) |
| V09† · Vòng bi 624ZZ 4×13×5 ×8 | cột nâng | 8 | — | [Thegioiic](https://www.thegioiic.com/624zz-bac-dan-vong-bi-trong-4mm-ngoai-13mm-day-5mm) |
| V10† · Vòng bi 6701ZZ 12×18×4 ×5 | 1 cột nâng + 2/tay ×2 | 5 | — | [Thegioiic](https://www.thegioiic.com/6701zz-bac-dan-vong-bi-trong-12mm-ngoai-18mm-day-4mm) |
| N08† · Công tắc hành trình KW12-N ×3 | 2 giới hạn + cảm biến chốt | 3 | — | [Thegioiic](https://www.thegioiic.com/kw12-n-cong-tac-hanh-trinh-250vac-5a) |
| N09† · Chốt khoá điện từ 12 V | EML-0530B, kéo-đẩy 600 mA, 6 N, hành trình 10 mm | 1 | — | [Thegioiic](https://www.thegioiic.com/eml-0530b-khoa-chot-dien-12v) |
| N10 · Chốt thép Φ12×25 | cắt từ thanh inox 304 Φ12 | 1 | — | [thepmanhtienphat.com](https://thepmanhtienphat.com/san-pham/gia-lap-inox-phi-12/) |
| N12† · Keo epoxy AB 2 thành phần |  | 1 | — | [Banlinhkien](https://banlinhkien.com/keo-epoxy-ab-2-thanh-phan-mau-trong-p6648195.html) |
| X02† · Đối trọng thép ~8 kg |  | 1 | Đang cập nhật | Đang cập nhật |
| T02a† · Khoá kéo 100 kg có chốt an toàn | trạm-xe | 2 | [Alibaba](https://www.alibaba.com/product-detail/Draw-Latch-Pull-Action-Latch-Type_1600754435395.html) | — |
| T04† · Công tắc hành trình KW12-N | khoá trạm | 2 | — | [Thegioiic](https://www.thegioiic.com/kw12-n-cong-tac-hanh-trinh-250vac-5a) |
| T05a† · Nhôm 2040 dài 1,2 m | 1 thanh cho trạm, cắt 1,0 m | 1 | — | [Thegioiic](https://www.thegioiic.com/nhom-dinh-hinh-qt-20x40-dai-1-2m) |
| T03 · Chốt côn thép Ø8 ×4, bạc khoan Ø8 ×2 |  | 6 | Đang cập nhật | Đang cập nhật |
| T05xa† · Nhôm 2020 dài 0,6 m | trạm và đế bàn | 1 | — | [Thegioiic](https://www.thegioiic.com/nhom-dinh-hinh-qt-20x20-dai-60cm) |
| T05xb† · Ke góc, con trượt T M5 cho nhôm 2020/2040 |  | 1 | — | [Thegioiic](https://www.thegioiic.com/ke-goc-chiu-luc-90-do-dung-cho-nhom-dinh-hinh-qt-2020) |
| T06 · Núm khoá M5 ×2 + đệm ma sát | gá pan-tilt in | 1 | Đang cập nhật | Đang cập nhật |
| K01† · Bộ giữ đồ khay đế: 2 dây đai velcro 25 mm × 0,6 m có khoá + 4 dải cao su silicone tự dính chống trượt |  | 1 | Đang cập nhật | Đang cập nhật |
| K02† · Gá khay đế không dụng cụ: 2 núm vặn M5 + 2 đai ốc T M5 cho 2040, heat-set M3 ×4 + M4 ×8 | lỗ Jetson/VESA | 1 | — | [Thegioiic](https://www.thegioiic.com/dai-oc-truot-chu-t-m5-dung-cho-nhom-dinh-hinh-qt-2020) |

## Ốc vít

Số lượng từng loại ở bảng "Chi tiết ốc vít" ngay dưới.

| Hạng mục | Model / ghi chú | SL | Link quốc tế | Link VN |
|---|---|--:|---|---|
| V08† · Ốc lục giác M3×10 inox 304 | 200 cái/robot, giả định: 74/tay ×2 + ~50 | 200 | — | [Nshop](https://nshopvn.com/product/combo-100-oc-luc-giac-inox-304-dau-tron-m3x10/) |
| N17† · Bộ ốc M2/M3/M4 + đai ốc | M2×12 Phillips ×4 cho giá camera cổ tay, 2 ốc/camera ×2 tay; M2/M4 cho đế | 1 | — | [Thegioiic](https://www.thegioiic.com/bo-880-mon-oc-luc-giac-dai-oc-long-den-304-m2-m3-m4-m5) |
| X01† · Heat-set insert M3 | số lượng chưa xác định | chưa xác định | [Alibaba](https://www.alibaba.com/product-detail/Threaded-Heat-M3-M4-M5-M6_1601022610315.html) · [Adafruit](https://www.adafruit.com/product/4255) | [Lazada](https://www.lazada.vn/products/dai-oc-cay-ren-insert-nuts-tone-dong-m2-m25-m3-m4-m5-m6-i2231690411.html) |
| S10 · Vít đầu dù M8×35 | cọc BAT_F, SERVO+, GND; thiếu đai ốc + long đen; trang ghi hết hàng | 3 | — | [Thegioiic](https://www.thegioiic.com/vit-dau-du-tm-m8-x-35mm) |

### Chi tiết ốc vít

| Mã | Loại ốc vít | SL | Cụm | Tính trong dòng | Ghi chú |
|---|---|--:|---|---|---|
| F01.t | M3×10 lục giác inox (tay follower) | 148 cái | Trạm | V08 | số lượng giả định |
| F01.c | M3×10 lục giác inox (còn lại) | 52 cái | Điện chung | V08 | số lượng giả định |
| F02 | M2×12 Phillips (camera cổ tay) | 4 cái | Trạm | — |  |
| F03 | M2×12 Phillips (camera đầu) | 2 cái | Trạm | — | số lượng giả định |
| F04 | Bộ ốc M2/M3/M4 + đai ốc | theo bộ | Điện chung | N17 |  |
| F05 | Heat-set insert M3 | 8 cái | Trạm | X01 | số lượng giả định |
| F06 | Vít đầu dù M8×35 | theo bộ | Đế | S10 |  |
| F07 | Đai ốc M8 + long đen | 3 bộ | Đế | — |  |
| F08a | Ke góc 2020/2040 | 2 cái | Trạm | — | số lượng giả định |
| F08.t | Con trượt T M5 (nhôm 2020/2040) (trạm) | 4 cái | Trạm | — | số lượng giả định |
| F08.n | Con trượt T M5 (nhôm 2020/2040) (đế ngàm) | 4 cái | Trục nâng | — | số lượng giả định |
| F09 | Núm khoá M5 + đệm ma sát | 2 cái | Trạm | — |  |
| F10 | Vít M6 đệm tựa thép | 3 cái | Trục nâng | — | số lượng giả định |
| F12 | Vít M3×18 + đai ốc M3 | 8 cái | Trạm | — | số lượng giả định |
| F15 | M3×20 lục giác (ghép 4 mảnh khay đế) | 7 cái | Đế | — | số lượng giả định |

## Vật liệu in

Filament PETG cho chi tiết in.

| Hạng mục | Model / ghi chú | SL | Link quốc tế | Link VN |
|---|---|--:|---|---|
| V11† · Filament PETG 8 kg | cần khoảng 8 cuộn | 8 | [Creality](https://store.creality.com/products/cr-petg-3d-printing-filament-1kg) | [Lazada](https://www.lazada.vn/products/nhua-in-3d-creality-cr-petg-khong-mui-chong-am-it-cong-venh-1kgcuon-i2655649911.html) |

### Khối lượng in theo danh sách chi tiết

| Cụm | Số chi tiết | Khối lượng (g) | Giờ in |
|---|--:|--:|--:|
| Tay | 9 | 2.373 | 51,9 |
| Kẹp | 3 | 220 | 4,8 |
| Trụ nâng | 7 | 3.248 | 71,0 |
| Khung T | 1 | 439 | 9,6 |
| Đế | 3 | 480 | 10,5 |
| Khay | 2 | 564 | 12,3 |
| **Cộng** | **25** | **7.323** | **160,2** |

Chưa tính vào khối lượng này: 11 chi tiết của trạm (chưa ước tính khối lượng) và các chi tiết thuộc bộ đế bàn tuỳ chọn.

Danh sách khuyến nghị cần khoảng 7,3 kg (6 thành, 20 % infill), tức 8 cuộn 1 kg.

## Bộ đế bàn tuỳ chọn

Bán riêng, không nằm trong 1 robot chuẩn.

| Hạng mục | Model / ghi chú | SL | Link quốc tế | Link VN |
|---|---|--:|---|---|
| T01b · Giắc D-sub combo 21W4 40 A, nửa thứ 3 | cái trên đế bàn; bán tối thiểu 50 | 1 | [Alibaba](https://www.alibaba.com/product-detail/D-Sub-21W4-Male-Mixed-Contact_1600756823919.html) | — |
| T02b† · Móc khoá kéo thêm cho đế bàn |  | 2 | [Alibaba](https://www.alibaba.com/product-detail/Draw-Latch-Pull-Action-Latch-Type_1600754435395.html) | — |
| T09† · Adapter kín 12 V 15 A 180 W | đế bàn | 1 | [Alibaba](https://www.alibaba.com/product-detail/180W-12V-15A-DC-Power-Adapter_1600280511295.html) | — |
| T05b† · Nhôm 2040 dài 1,2 m | 1 thanh đế bàn, 1,16 m | 1 | — | [Thegioiic](https://www.thegioiic.com/nhom-dinh-hinh-qt-20x40-dai-1-2m) |
| T07b† · Nút Reset nhả 22 mm LA38-11BN | đế bàn | 1 | — | [Thegioiic](https://www.thegioiic.com/la38-11bn-nut-nhan-nha-mau-xanh-duong-22mm-10a) |
| T08† · Kẹp chữ F | 2 cái, đế bàn | 2 | — | [Meta](https://meta.vn/kep-chu-f-jorgensen-p119929) |
| T10a† · Cầu chì lá Mini Blade 20 A 32 V | 2 cái, hộp đế bàn | 2 | — | [Thegioiic](https://www.thegioiic.com/cau-chi-xe-may-o-to-mau-vang-mini-blade-20a-32v) |
| T10b† · Rơ-le 12 V 40 A + đế | hộp đế bàn | 1 | — | [Thegioiic](https://www.thegioiic.com/san-pham/relay-khong-chot) |
| T10c† · Đế cầu chì lá inline, cọc vào, cáp USB ra panel | hộp đế bàn | 1 | — | [Thegioiic](https://www.thegioiic.com/de-cau-chi-mini-v2-5mm) |

## Gói leader (2 tay)

Bán riêng, không nằm trong 1 robot chuẩn. Số lượng theo bộ leader 2 tay.

| Hạng mục | Model / ghi chú | SL | Link quốc tế | Link VN |
|---|---|--:|---|---|
| L01 · Servo STS3215 5 V | 7 mỗi tay (6 khớp + tay cầm kẹp, như AM-ARM200); chưa rõ có bản 5 V | 14 | [Alibaba](https://www.alibaba.com/product-detail/Lerobot-360-Degree-Smart-Servo-for_1601563310522.html) | — |
| L02 · Bo bus servo | 1 bo mỗi tay | 2 | [Alibaba](https://www.alibaba.com/product-detail/Feetech-FE-URT-1-Multi-function_1601003839109.html) · [Waveshare](https://www.waveshare.com/bus-servo-adapter-a.htm) | — |
| L03 · Nguồn 5 V | adapter DC 5 V dùng chung | 1 | — | [Caka](https://caka.vn/nguon-adapter-dc-5v-3a-jack-dc-5-5x2-1mm) |
| L04 · Cáp USB-C | 1 cáp mỗi tay; số lượng giả định | 2 | — | [UGREEN](https://ugreen.vn/cap-sac-nhanh-type-c-ugreen-us288) |
| L05 · Heat-set insert M3 | số lượng giả định | 72 | [Alibaba](https://www.alibaba.com/product-detail/Threaded-Heat-M3-M4-M5-M6_1601022610315.html) · [Adafruit](https://www.adafruit.com/product/4255) | [Lazada](https://www.lazada.vn/products/dai-oc-cay-ren-insert-nuts-tone-dong-m2-m25-m3-m4-m5-m6-i2231690411.html) |
| L06 · Ốc M3 | số lượng giả định | 72 | — | [Nshop](https://nshopvn.com/product/combo-100-oc-luc-giac-inox-304-dau-tron-m3x10/) |
| L07 · Chi tiết in 3D | 1 bộ mỗi tay, tự in từ file STL; số lượng giả định | 2 | — | — |
