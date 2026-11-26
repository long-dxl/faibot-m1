# Bill of materials (BOM)

Parts list for one Faibot M1 robot, standard configuration (2 arms, 3-wheel base, lift column, two-arm station), v0.5 before EVT. One row per item; the quantity is the total for one robot. Links checked 23/11/2026 to 25/11/2026. A row marked "Updating" has no verified purchase page yet. The optional tabletop base kit and the leader pack are in the last two tables.

Note: † row not yet verified (the variant or the seller must still be checked before ordering).

## Servo

18 Feetech bus servos: 7 STS3095 (pan, shoulder and elbow on each arm, plus the lift column) and 11 STS3215 (wrist flex, wrist roll, wrist yaw and gripper on each arm, plus 3 wheels). The servo ID table is in the "Servo ID" section of the assembly guide.

| Item | Model / notes | Qty | International | Vietnam |
|---|---|--:|---|---|
| A01 · Servo STS3095 12 V C002 | pan, shoulder, elbow on each arm + lift column | 7 | [Alibaba](https://www.alibaba.com/product-detail/FEETECH-STS3095-12V-95KG-servo-AI_1601045980686.html) | — |
| A02 · Servo STS3215 12 V C018 1:345 | wrist flex x2, wrist roll x2, wrist yaw x2, gripper x2, wheels x3 | 11 | [Alibaba](https://www.alibaba.com/product-detail/Lerobot-360-Degree-Smart-Servo-for_1601563310522.html) · [DFRobot](https://www.dfrobot.com/product-2962.html) · [Evelta](https://evelta.com/st3215-c018-12v-30kg-cm-1-345-dual-shaft-ttl-serial-servo-metal-gears-magnetic-encoder/) | — |

## Electronics, cameras and power

Controller boards, cameras, battery and BMS, protection, wire, lugs, connectors and small parts.

| Item | Model / notes | Qty | International | Vietnam |
|---|---|--:|---|---|
| A03 · BMS Daly 3S Li-ion 100 A | sold in packs of 2 minimum, the second is a spare | 1 | [Alibaba](https://www.alibaba.com/product-detail/Daly-Li-ion-3S-bms-12_1600112464909.html) | — |
| A04† · Li-ion 21700 cell >= 30 A discharge (40T / P42A) for 3S4P = 12 cells | real cells must sustain 30 A or more; buy through a domestic distributor | 12 | — | [kimpower.vn](https://kimpower.vn/san-pham/cell-pin-samsung-40t-21700-xa-35a-dung-luong-4000mah/) |
| A06† · USB camera 720p wide-angle x3 |  | 3 | [DFRobot](https://www.dfrobot.com/product-2537.html) | — |
| A07 · Servo bus controller board x3 | Feetech FE-URT-1 or Waveshare Bus Servo Adapter A | 3 | [Alibaba](https://www.alibaba.com/product-detail/Feetech-FE-URT-1-Multi-function_1601003839109.html) · [Waveshare](https://www.waveshare.com/bus-servo-adapter-a.htm) | — |
| A08† · Contactor 100 A with 12 V coil | Albright SW80 or ZJW100A type | 1 | [Alibaba](https://www.alibaba.com/product-detail/SW80-NO-type-Coil-Voltage-12V_62329150139.html) | — |
| A10 · 3S 12.6 V battery charger | the one found is 2 A; the design calls for 5 A | 1 | — | [Thegioiic](https://www.thegioiic.com/adapter-sac-pin-3s-12-6v-2a-dau-dc-5-5x2-5mm-dai-1m-co-led) |
| V01 · Raspberry Pi 5 2 GB |  | 1 | [Kiwi Electronics](https://www.kiwi-electronics.com/en/raspberry-pi-5-computers-accessories-415/raspberry-pi-5-2gb-20129) · [Adafruit](https://www.adafruit.com/product/6007) | [hshop](https://hshop.vn/may-tinh-raspberry-pi-5-made-in-uk) |
| V02 · Raspberry Pi 5 heatsink |  | 1 | [Adafruit](https://www.adafruit.com/products/5815) | [hshop](https://hshop.vn/tan-nhiet-raspberry-pi-5-heatsink-pwm-fan) |
| V03† · microSD card 32 GB |  | 1 | [SanDisk](https://www.sandisk.com/products/memory-cards/microsd-cards/sandisk-ultra-uhs-i-microsd?sku=SDSQUA4-032G-GN6MA) | [Thegioiic](https://www.thegioiic.com/the-nho-micro-sdhc-32gb-class10-ghi-10mb-s-doc-48mb-s) |
| V04 · Step-down module 12 V to 5 V 5 A | TPS54560, 5 V set by pad | 1 | — | [hshop](https://hshop.vn/mach-giam-ap-dc-dc-buck-tps54560-60vdc-5a-600khz) |
| V05† · E-stop button 22 mm 2NC | LA38P-11ZS | 1 | — | [Thegioiic](https://www.thegioiic.com/la38p-11zs-nut-nhan-dung-khan-cap-660v-10a-2nc-22mm-nap-do) |
| V06† · Blade fuse 10 A x7 | from a 120-piece kit | 1 | — | [Thegioiic](https://www.thegioiic.com/bo-cau-chi-xe-120-cai-5a-10a-15a-20a-25a-30a) |
| V07† · 12-way blade fuse box x2 |  | 2 | [Nilight](https://www.nilight.com/collections/most-bought-icart/products/nilight-12-way-blade-fuse-block-12-circuits-with-negative-bus-fuse-box-holder-with-led-indicator-ato-atc-fuse-panel-waterproof-cover-for-12v-automotive-cars-marine-boats-rvs-trailers-2-years-warranty) | — |
| N01† · Main fuse MEGA / bolt-down 80 A + holder |  | 1 | [Digikey](https://www.digikey.ca/en/products/detail/littelfuse-inc/0298080-ZXEH/778253) · [Digikey](https://www.digikey.co.uk/en/products/detail/littelfuse-commercial-vehicle-products/02981036ZXT/3996948) | — |
| N02† · Blade fuse 3 A x11 | mini blade, one per STS3215 | 1 set (11 pcs) | [Digikey](https://www.digikey.ca/en/products/detail/littelfuse-inc/0297003-WXNV/146575) | — |
| N03† · Latching reset button 22 mm | LA38-11BN | 1 | — | [Thegioiic](https://www.thegioiic.com/la38-11bn-nut-nhan-nha-mau-xanh-duong-22mm-10a) |
| N04 · 12 V latching relay | JQ1P-12V-F, 10 A SPDT | 1 | — | [Thegioiic](https://www.thegioiic.com/jq1p-12v-f-ro-le-12vdc-10a-spdt-5-chan) |
| N05† · Wire 4 AWG red/black for the arm branches |  | 4.8 m | — | [Thegioiic](https://www.thegioiic.com/4awgx1-day-dien-silicon-chiu-nhiet-sieu-mem-mau-den-loai-tot) |
| N06† · Wire 2 AWG for the main trunk | seller's minimum is a multiple of 50; better to buy cut lengths locally | 2.5 m | [Alibaba](https://www.alibaba.com/product-detail/High-temperature-2-AWG-Copper-Electrical_1601118536933.html) | — |
| N07 · Tinned copper lug 25 mm2 M8 hole SC25-8 | for 4 AWG wire | 6 | — | [Thegioiic](https://www.thegioiic.com/sc25-8-dau-cos-tran-bang-dong-ma-thiec-co-day-25mm2-vit-m8) |
| N13† · USB-A to USB-C cable 1 m x3 | bus board to Pi, Pi USB-A port | 3 | — | [UGREEN](https://ugreen.vn/cap-sac-nhanh-type-c-ugreen-us288) |
| N14† · 3-pin servo extension cable 50 cm | replaces Feetech cables 26 cm x16 + 90 cm x2 | 18 | — | [Thegioiic](https://www.thegioiic.com/cap-noi-dai-dong-co-rc-servo-dai-50cm) |
| N18† · MEGA fuse 40 A + holder | F3 and F4 | 2 | [Digikey](https://www.digikey.ca/en/products/detail/littelfuse-inc/0298040-ZXB/778251) · [Digikey](https://www.digikey.co.uk/en/products/detail/littelfuse-commercial-vehicle-products/02981036ZXT/3996948) | — |
| N19 · ATO standard blade fuse 7.5 A x7 | listing shows out of stock | 7 | — | [Thegioiic](https://www.thegioiic.com/cau-chi-xe-may-o-to-mau-nau-ato-blade-7-5a-32v) |
| N20† · Inline ATO fuse holder 16 AWG with cover | FW1-3, FLIFT, FLAT | 5 | [littelfuse.com](https://www.littelfuse.com/products/fuse-blocks-fuseholders-and-fuse-accessories/automotive-and-commercial-vehicle-fuse-holders/ato-fhac/fhac0001zxj.aspx) · [parts-express.com](https://www.parts-express.com/In-line-ATO-ATC-Fuse-Holder-16-AWG-070-670) | — |
| N21† · Anderson SB120 2-pole connector 120 A | J0; minimum order 20 | 4 | [Alibaba](https://www.alibaba.com/product-detail/Battery-Connector-50A-120A-175A-350A_1600792110908.html) · [Digikey](https://www.digikey.ca/en/products/detail/anderson-power-products-inc/6800G1/28398323) | — |
| N22† · USB isolator module ADuM3160 x2 |  | 2 | — | [ICdayroi](https://icdayroi.com/mach-chuyen-doi-usb-usb-co-cach-ly-adum3160) |
| N23† · DC circuit breaker CHINT NB1-63DC 1P | C6 x2, C4 x1, C2 x1 | 4 | [CHINT](https://www.chintglobal.com/sg/en/products/low-voltage/iec/final-power-distribution/nb1-63dc.html) | [baoanjsc.com.vn](https://baoanjsc.com.vn/products/dc-circuit-breaker-chint-NB1-63DC-1P-C40A-DC250V-6kA_814508_en.aspx) · [baoanjsc.com.vn](https://baoanjsc.com.vn/products/dc-circuit-breaker-chint-NB1-63DC-1P-C6A-DC250V-6kA_814511_en.aspx) |
| S01 · Bare copper lug 35 mm2 M8 hole SC35-8 | 2 AWG wire | 14 | — | [Thegioiic](https://www.thegioiic.com/sc35-8-dau-cos-tran-bang-dong-ma-thiec-co-day-35mm2-vit-m8) |
| S02 · Tinned copper lug 25 mm2 M6 hole SC25-6 | fuse box studs (V07) | 6 | — | [Thegioiic](https://www.thegioiic.com/sc25-6-dau-cos-tran-bang-dong-ma-thiec-co-day-25mm2-vit-m6) |
| S03 · Female spade terminal 6.3 mm crimp FDD silver-plated |  | 29 | — | [Thegioiic](https://www.thegioiic.com/dau-cos-6-3mm-bam-day-cai-fdd-ma-bac) |
| S04 · Insulated ring terminal RV5.5-8 M8 hole | joins 2-3 wires 16/22 AWG | 8 | — | [Thegioiic](https://www.thegioiic.com/rv5-5-8-dau-cos-tron-48a-co-day-4-6-6-64mm2-vit-m8-mau-xanh-duong) |
| S05 · Insulated pin terminal PTV1.25-18 | MCB clamp posts | 10 | — | [Thegioiic](https://www.thegioiic.com/ptv1-25-18-dau-cos-pin-dac-19a-co-day-0-5-1-5mm2-mau-vang) |
| S06 · Panel-mount female XT60E-F connector | charging port on the enclosure | 3 | — | [Thegioiic](https://www.thegioiic.com/xt60e-f-jack-cam-cai-500vdc-35a-bat-vit) |
| S07 · Ferrule E7508 0.75 mm2 | terminal blocks and screw modules | 50 | — | [Thegioiic](https://www.thegioiic.com/e7508-cos-pin-rong-co-day-0-75mm2-mau-den) |
| S08 · Insulated ring terminal RV1.25-4 M4 hole | LA38 buttons, relay K1 coil; minimum order 20 | 16 | — | [Thegioiic](https://www.thegioiic.com/rv1-25-4-dau-cos-tron-19a-co-day-0-25-1-65mm2-vit-m4-mau-xanh-duong) |
| S09 · Female-female bus cable 4-pin 2.54 mm 50 cm | cut to get a female Dupont end | 3 | — | [Thegioiic](https://www.thegioiic.com/day-bus-cai-cai-4-tiep-diem-2-54mm-dai-50cm) |
| S11 · 4-pole terminal block TB-2504 25 A | TB1, TB2, TB3 | 3 | — | [Thegioiic](https://www.thegioiic.com/tb-2504-cau-dau-dien-terminal-khoi-tu-dien-4-cuc-25a-600v-co-day-0-5-2-5mm2) |
| S12 · Aluminium DIN rail 35x7.5 mm 1 m | cut 0.16 m for the 4 circuit breakers; listing shows out of stock | 1 | — | [Thegioiic](https://www.thegioiic.com/thanh-ray-din-nhom-35x7-5mm-dai-1m) |
| S13 · 1-channel 5 V relay module high/low trigger with opto | K5, K10 | 2 | — | [Thegioiic](https://www.thegioiic.com/module-1-relay-5v-kich-muc-cao-thap) |
| S14 · Opto module PC817 4-channel | ESTOP_OK, ARM_SENSE, RESET_SENSE | 2 | — | [Thegioiic](https://www.thegioiic.com/pc817-mach-opto-cach-ly-4-kenh) |
| S15 · Diode 1N5408 3 A 1 kV | flyback for relay K1 and the latch coil; minimum order 5 | 2 | — | [Thegioiic](https://www.thegioiic.com/1n5408-diode-chinh-luu-3a-1kv-do-201ad) |
| S16 · Zener 1N5351B 14 V 5 W | in series with the K1 coil diode | 1 | — | [Thegioiic](https://www.thegioiic.com/1n5351b-diode-zener-14v-5w) |
| S17 · Auxiliary contact block LA38/39 2NO for push buttons | RST NO2; listing shows out of stock | 1 | — | [Thegioiic](https://www.thegioiic.com/la38-39-tiep-diem-phu-2no-cho-nut-nhan-mau-xanh-la-10a) |
| S18 · USB 2.0 hub 7 ports with 5 V power port | station electronics box: 2 isolators + 3 cameras | 1 | — | [HACOM](https://hacom.vn/bo-chia-usb-2.0-tu-1-ra-7-cong-chinh-hang-ugreen-30374) |
| S19 · USB-C to USB-C cable 1 m | buck converter to Pi, PW03/PW04; listing shows out of stock | 1 | — | [Thegioiic](https://www.thegioiic.com/cap-usb-type-c-noi-usb-type-c-mau-trang-dai-1m) |
| S20 · XT60 male cable 10 cm for the charge lead | charger side | 1 | — | [Thegioiic](https://www.thegioiic.com/cap-xt60-m-duc-dung-cho-pin-lipo-rc-dai-10cm) |
| S21 · Heat-shrink tube 10 mm 600 V |  | 1 m | — | [Thegioiic](https://www.thegioiic.com/ong-co-nhiet-10mm-600v) |
| S22 · Silicone wire 14 AWG black 3239 | minimum order 1 | 0.3 m | — | [Thegioiic](https://www.thegioiic.com/3239-14awgx1-day-dien-silicon-chiu-nhiet-mau-den) |
| S23 · Silicone wire 14 AWG red 3239 | minimum order 1 | 0.3 m | — | [Thegioiic](https://www.thegioiic.com/3239-14awgx1-day-dien-silicon-chiu-nhiet-mau-do) |
| S24 · Silicone wire 16 AWG black extra-flexible |  | 5.7 m | — | [Thegioiic](https://www.thegioiic.com/16awgx1-day-dien-silicon-chiu-nhiet-sieu-mem-mau-den-loai-tot) |
| S25 · Silicone wire 16 AWG red extra-flexible |  | 6.3 m | — | [Thegioiic](https://www.thegioiic.com/16awgx1-day-dien-silicon-chiu-nhiet-sieu-mem-mau-do-loai-tot) |
| S26 · Silicone wire 22 AWG black extra-flexible | includes white 4.4 m + orange 1.8 m in place of white/orange | 21.4 m | — | [Thegioiic](https://www.thegioiic.com/22awgx1-day-dien-silicon-chiu-nhiet-sieu-mem-mau-den-loai-tot) |
| S27 · Silicone wire 22 AWG red extra-flexible |  | 22.5 m | — | [Thegioiic](https://www.thegioiic.com/22awgx1-day-dien-silicon-chiu-nhiet-sieu-mem-mau-do-loai-tot) |
| S28 · Silicone wire 24 AWG red extra-flexible | includes yellow 10.3 m + red 0.8 m | 11.1 m | — | [Thegioiic](https://www.thegioiic.com/24awgx1-day-dien-silicon-chiu-nhiet-sieu-mem-mau-do-loai-tot) |
| S29 · Silicone wire 24 AWG black extra-flexible |  | 3.2 m | — | [Thegioiic](https://www.thegioiic.com/24awgx1-day-dien-silicon-chiu-nhiet-sieu-mem-mau-den-loai-tot) |
| S30† · Silicone wire 8 AWG red/black | black 1.6 + red 1.6 | 3.2 m | — | [Thegioiic](https://www.thegioiic.com/8awgx1-day-dien-silicon-chiu-nhiet-sieu-mem-mau-den-loai-tot) |
| S31† · Silicone wire 18 AWG red/black | black 0.2 + red 0.2 | 0.4 m | — | [Thegioiic](https://www.thegioiic.com/18awgx1-day-dien-silicon-chiu-nhiet-sieu-mem-mau-den-loai-tot) |
| S32† · Lug 10 mm2 M6 hole SC10-6 | 8 AWG wire: docking-base terminal block, fuse box studs (V07) | 10 | — | [Thegioiic](https://www.thegioiic.com/sc10-6-dau-cos-tran-bang-dong-ma-thiec-co-day-10mm2-vit-m6) |
| S33† · 40 A power contacts for the D-sub combo 21W4 connector | solder-cup or crimp; wire up to 8 AWG | 10 | [NorComp](https://www.norcomp.net/series/680m-crimp-and-poke) | — |
| S34† · 4-pole M6 terminal block on a printed insulating base | 4 AWG to 8 AWG tail, on the docking base; with M6 bolts, nuts, washers | 1 | — | [Thegioiic](https://www.thegioiic.com/san-pham/dau-cos-noi-day) |
| S35† · ATO standard blade fuse 2 A |  | 1 | — | [ICdayroi](https://icdayroi.com/cau-chi-o-to-4s-2a) |
| S36 · USB 2.0 A male cable 1.5 m cut at one end | Pi to pins 1-4 of the 21W4 connector through the lift-column ring | 1 | — | [HACOM](https://hacom.vn/cap-usb-2.0-noi-dai-1.5m-vention-vas-a44-b150-black) |
| T01a · D-sub combo 21W4 40 A connector, 2 halves (male on the station, female on the base) + 40 A contacts | minimum order 50 | 2 | [Alibaba](https://www.alibaba.com/product-detail/D-Sub-21W4-Male-Mixed-Contact_1600756823919.html) | — |
| T07a† · E-stop button 22 mm 2NC LA38P-11ZS | second one on the station | 1 | — | [Thegioiic](https://www.thegioiic.com/la38p-11zs-nut-nhan-dung-khan-cap-660v-10a-2nc-22mm-nap-do) |
| T11 · Buck converter 5 V 5 A TPS54560 for the hub and cameras on the station |  | 1 | — | [hshop](https://hshop.vn/mach-giam-ap-dc-dc-buck-tps54560-60vdc-5a-600khz) |
| T12† · Capacitor 4700 uF 25 V x2 |  | 2 | — | [Thegioiic](https://www.thegioiic.com/tu-hoa-4700uf-25v-16x25mm-xuyen-lo) |

## Mechanical

Wheels, aluminium extrusion, bearings, station lock, limit switches and machined parts.

| Item | Model / notes | Qty | International | Vietnam |
|---|---|--:|---|---|
| A05† · Omni wheel 100 mm (4 in) LeKiwi style x3 |  | 3 | [vxb.com](https://vxb.com/products/omni-wheel-100mm) | — |
| V12† · Aluminium extrusion 2040 (5 sections) |  | 1 | — | [Thegioiic](https://www.thegioiic.com/nhom-dinh-hinh-qt-20x40-dai-1-2m) |
| V09† · Bearing 624ZZ 4x13x5 x8 | lift column | 8 | — | [Thegioiic](https://www.thegioiic.com/624zz-bac-dan-vong-bi-trong-4mm-ngoai-13mm-day-5mm) |
| V10† · Bearing 6701ZZ 12x18x4 x5 | 1 in the lift column + 2 per arm x2 | 5 | — | [Thegioiic](https://www.thegioiic.com/6701zz-bac-dan-vong-bi-trong-12mm-ngoai-18mm-day-4mm) |
| N08† · Limit switch KW12-N x3 | 2 end stops + latch sensor | 3 | — | [Thegioiic](https://www.thegioiic.com/kw12-n-cong-tac-hanh-trinh-250vac-5a) |
| N09† · 12 V solenoid lock pin | EML-0530B, push-pull 600 mA, 6 N, 10 mm stroke | 1 | — | [Thegioiic](https://www.thegioiic.com/eml-0530b-khoa-chot-dien-12v) |
| N10 · Steel pin dia 12x25 | cut from dia 12 stainless 304 rod | 1 | — | [thepmanhtienphat.com](https://thepmanhtienphat.com/san-pham/gia-lap-inox-phi-12/) |
| N12† · Two-part epoxy adhesive (AB) |  | 1 | — | [Banlinhkien](https://banlinhkien.com/keo-epoxy-ab-2-thanh-phan-mau-trong-p6648195.html) |
| X02† · Steel counterweight about 8 kg |  | 1 | Updating | Updating |
| T02a† · Draw latch 100 kg with safety catch | station to base | 2 | [Alibaba](https://www.alibaba.com/product-detail/Draw-Latch-Pull-Action-Latch-Type_1600754435395.html) | — |
| T04† · Limit switch KW12-N | station lock | 2 | — | [Thegioiic](https://www.thegioiic.com/kw12-n-cong-tac-hanh-trinh-250vac-5a) |
| T05a† · Aluminium 2040 1.2 m | 1 bar for the station; cut to 1.0 m | 1 | — | [Thegioiic](https://www.thegioiic.com/nhom-dinh-hinh-qt-20x40-dai-1-2m) |
| T03 · Steel taper pin dia 8 x4 and drill bushing dia 8 x2 |  | 6 | Updating | Updating |
| T05xa† · Aluminium 2020 0.6 m | station and tabletop base | 1 | — | [Thegioiic](https://www.thegioiic.com/nhom-dinh-hinh-qt-20x20-dai-60cm) |
| T05xb† · Corner bracket, M5 T-nut for aluminium 2020/2040 |  | 1 | — | [Thegioiic](https://www.thegioiic.com/ke-goc-chiu-luc-90-do-dung-cho-nhom-dinh-hinh-qt-2020) |
| T06† · M5 clamping knob x2 + friction pad | printed pan-tilt mount | 1 | — | [Lazada](https://www.lazada.vn/products/num-van-ren-ngoai-bu-long-van-tay-inox-304-size-m3-m4-m5-m6-m8-i2383584913.html) |
| K01† · Tray hold-down kit: 2 velcro straps 25 mm x 0.6 m with buckle + 4 self-adhesive anti-slip silicone strips |  | 1 | Updating | Updating |
| K02† · Tool-free tray mount: 2 M5 knobs + 2 M5 T-nuts for 2040, M3 heat-set inserts x4 + M4 x8 | Jetson/VESA holes | 1 | — | [Thegioiic](https://www.thegioiic.com/dai-oc-truot-chu-t-m5-dung-cho-nhom-dinh-hinh-qt-2020) |

## Screws and fasteners

Quantity per type is in the "Screw details" table right below.

| Item | Model / notes | Qty | International | Vietnam |
|---|---|--:|---|---|
| V08† · Hex socket screw M3x10 stainless 304 | 200 pcs per robot (estimate: 74 per arm x2 + ~50) | 200 | — | [Nshop](https://nshopvn.com/product/combo-100-oc-luc-giac-inox-304-dau-tron-m3x10/) |
| N17† · M2/M3/M4 screw and nut set | M2x12 Phillips x4 for the wrist camera mount, 2 per camera x2 arms; M2/M4 for the base | 1 | — | [Thegioiic](https://www.thegioiic.com/bo-880-mon-oc-luc-giac-dai-oc-long-den-304-m2-m3-m4-m5) |
| X01† · M3 heat-set insert | quantity not yet fixed | TBD | [Alibaba](https://www.alibaba.com/product-detail/Threaded-Heat-M3-M4-M5-M6_1601022610315.html) · [Adafruit](https://www.adafruit.com/product/4255) | [Lazada](https://www.lazada.vn/products/dai-oc-cay-ren-insert-nuts-tone-dong-m2-m25-m3-m4-m5-m6-i2231690411.html) |
| S10 · Mushroom-head screw M8x35 | posts BAT_F, SERVO+, GND; nuts and washers still needed; listing shows out of stock | 3 | — | [Thegioiic](https://www.thegioiic.com/vit-dau-du-tm-m8-x-35mm) |

### Screw details

| ID | Screw type | Qty | Group | Counted in row | Notes |
|---|---|--:|---|---|---|
| F01.t | M3×10 hex socket, stainless (follower arms) | 148 pcs | Station | V08 | assumed quantity |
| F01.c | M3×10 hex socket, stainless (remaining) | 52 pcs | Shared electrical | V08 | assumed quantity |
| F02 | M2×12 Phillips (wrist camera) | 4 pcs | Station | — |  |
| F03 | M2×12 Phillips (head camera) | 2 pcs | Station | — | assumed quantity |
| F04 | M2/M3/M4 screw and nut set | per set | Shared electrical | N17 |  |
| F05 | M3 heat-set insert | 8 pcs | Station | X01 | assumed quantity |
| F06 | Mushroom-head screw M8×35 | per set | Base | S10 |  |
| F07 | M8 nut + washer | 3 sets | Base | — |  |
| F08a | Corner bracket 2020/2040 | 2 pcs | Station | — | assumed quantity |
| F08.t | M5 T-nut for aluminium 2020/2040 (station) | 4 pcs | Station | — | assumed quantity |
| F08.n | M5 T-nut for aluminium 2020/2040 (docking base) | 4 pcs | Lift column | — | assumed quantity |
| F09 | M5 knob + friction pad | 2 pcs | Station | — |  |
| F10 | M6 screw, steel backing plate | 3 pcs | Lift column | — | assumed quantity |
| F12 | M3×18 screw + M3 nut | 8 pcs | Station | — | assumed quantity |
| F15 | M3×20 hex socket (joins the 4 tray pieces) | 7 pcs | Base | — | assumed quantity |

## Print material

PETG filament for the printed parts.

| Item | Model / notes | Qty | International | Vietnam |
|---|---|--:|---|---|
| V11† · PETG filament 8 kg | about 8 spools | 8 | [Creality](https://store.creality.com/products/cr-petg-3d-printing-filament-1kg) | [Lazada](https://www.lazada.vn/products/nhua-in-3d-creality-cr-petg-khong-mui-chong-am-it-cong-venh-1kgcuon-i2655649911.html) |

### Print mass by part list

| Group | Parts | Mass (g) | Print time (h) |
|---|--:|--:|--:|
| Arm | 9 | 2,373 | 51.9 |
| Gripper | 3 | 220 | 4.8 |
| Lift column | 7 | 3,248 | 71.0 |
| T frame | 1 | 439 | 9.6 |
| Base | 3 | 480 | 10.5 |
| Tray | 2 | 564 | 12.3 |
| **Total** | **25** | **7,323** | **160.2** |

Not included in this mass: 11 station parts (no mass estimate yet) and the parts of the optional tabletop base kit.

The recommended part list needs about 7.3 kg (6 walls, 20 % infill), that is 8 spools of 1 kg.

## Optional tabletop base kit

Sold separately, not part of one standard robot.

| Item | Model / notes | Qty | International | Vietnam |
|---|---|--:|---|---|
| T01b · D-sub combo 21W4 40 A connector 3rd half | female on the tabletop base; minimum order 50 | 1 | [Alibaba](https://www.alibaba.com/product-detail/D-Sub-21W4-Male-Mixed-Contact_1600756823919.html) | — |
| T02b† · Extra draw-latch hooks for the tabletop base |  | 2 | [Alibaba](https://www.alibaba.com/product-detail/Draw-Latch-Pull-Action-Latch-Type_1600754435395.html) | — |
| T09† · Sealed adapter 12 V 15 A 180 W | tabletop base | 1 | [Alibaba](https://www.alibaba.com/product-detail/180W-12V-15A-DC-Power-Adapter_1600280511295.html) | — |
| T05b† · Aluminium 2040 1.2 m | 1 bar for the tabletop base; 1.16 m | 1 | — | [Thegioiic](https://www.thegioiic.com/nhom-dinh-hinh-qt-20x40-dai-1-2m) |
| T07b† · Latching reset button 22 mm LA38-11BN | tabletop base | 1 | — | [Thegioiic](https://www.thegioiic.com/la38-11bn-nut-nhan-nha-mau-xanh-duong-22mm-10a) |
| T08† · F-clamp | 2 pcs for the tabletop base | 2 | — | [Meta](https://meta.vn/kep-chu-f-jorgensen-p119929) |
| T10a† · Mini blade fuse 20 A 32 V | 2 pcs for the tabletop base box | 2 | — | [Thegioiic](https://www.thegioiic.com/cau-chi-xe-may-o-to-mau-vang-mini-blade-20a-32v) |
| T10b† · Relay 12 V 40 A + socket | tabletop base box | 1 | — | [Thegioiic](https://www.thegioiic.com/san-pham/relay-khong-chot) |
| T10c† · Inline blade fuse holder, studs in, USB cable out to the panel | tabletop base box | 1 | — | [Thegioiic](https://www.thegioiic.com/de-cau-chi-mini-v2-5mm) |

## Leader pack (2 arms)

Sold separately, not part of one standard robot. Quantities are for a 2-arm leader set.

| Item | Model / notes | Qty | International | Vietnam |
|---|---|--:|---|---|
| L01 · Servo STS3215 5 V | 7 per arm (6 joints + gripper handle, as in the inherited arm design); a 5 V version is unconfirmed | 14 | [Alibaba](https://www.alibaba.com/product-detail/Lerobot-360-Degree-Smart-Servo-for_1601563310522.html) | — |
| L02 · Servo bus controller board | 1 board per arm | 2 | [Alibaba](https://www.alibaba.com/product-detail/Feetech-FE-URT-1-Multi-function_1601003839109.html) · [Waveshare](https://www.waveshare.com/bus-servo-adapter-a.htm) | — |
| L03 · 5 V power supply | DC 5 V adapter, shared | 1 | — | [Caka](https://caka.vn/nguon-adapter-dc-5v-3a-jack-dc-5-5x2-1mm) |
| L04 · USB-C cable | 1 per arm; assumed quantity | 2 | — | [UGREEN](https://ugreen.vn/cap-sac-nhanh-type-c-ugreen-us288) |
| L05 · M3 heat-set insert | assumed quantity | 72 | [Alibaba](https://www.alibaba.com/product-detail/Threaded-Heat-M3-M4-M5-M6_1601022610315.html) · [Adafruit](https://www.adafruit.com/product/4255) | [Lazada](https://www.lazada.vn/products/dai-oc-cay-ren-insert-nuts-tone-dong-m2-m25-m3-m4-m5-m6-i2231690411.html) |
| L06 · M3 screw | assumed quantity | 72 | — | [Nshop](https://nshopvn.com/product/combo-100-oc-luc-giac-inox-304-dau-tron-m3x10/) |
| L07 · 3D-printed parts | 1 set per arm; printed from the STL files; assumed quantity | 2 | — | — |
