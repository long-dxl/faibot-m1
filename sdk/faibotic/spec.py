"""Thông số M1 v0: nguồn sự thật duy nhất cho mô hình sim, động học và backend Feetech.

Mọi kích thước tự đặt từ bảng "Thông số v0" của SPEC F016 (không lấy từ file CAD/URDF
của dự án khác). Đơn vị SI: m, kg, rad, N·m.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

G = 9.81
KGCM = 0.0980665  # 1 kg·cm = 0.0980665 N·m


@dataclass(frozen=True)
class Servo:
    model: str
    stall_kgcm: float  # mô-men chặn (đỉnh)
    rated_kgcm: float | None  # mô-men định mức; None = chưa có số hãng
    voltage: float
    mass_kg: float | None
    source: str
    stall_a: float | None = None   # dòng chặn ở 12 V (A) [trang bán, electrical/tinh-toan.md mục 1]; None = chưa có số

    @property
    def stall_nm(self) -> float:
        return self.stall_kgcm * KGCM

    @property
    def rated_nm(self) -> float | None:
        return None if self.rated_kgcm is None else self.rated_kgcm * KGCM


# Nguồn chi tiết + ngày tra: docs/vi/servo-datasheet.md
STS3215 = Servo("STS3215-C018 (12 V)", stall_kgcm=30.0, rated_kgcm=None, voltage=12.0,
                mass_kg=0.0705, source="docs/servo-datasheet.md#sts3215", stall_a=2.7)
STS3095 = Servo("STS3095-C002 (12 V)", stall_kgcm=95.0, rated_kgcm=22.5, voltage=12.0,
                mass_kg=0.1875, source="docs/servo-datasheet.md#sts3095", stall_a=9.8)

# ---------------- Đế 3 bánh omni ----------------
# P-L: đế kiểu LeKiwi 12 V mở bán kính bánh 0,25 m, một bánh hướng ra trước, đối trọng thép lùi sau, khay trước trên đế.
# S1G: hình va chạm và khối lượng thân lấy từ CAD cad/faibotic/assembly_pl.py qua cad/faibotic/pl_sim_dims.py -> cad/export/M1-PL-sim-dims.json
# (tests/test_pl_sim_dims.py so khớp <= 0,5 mm, 0,01 kg). Khối lượng in PETG theo infill giả định, khoang điện tử [giả định]; chờ cân (DK).
BASE_RADIUS = 0.185         # sàn in M1-BS-037 R185
BASE_Z0, BASE_Z1 = 0.039, 0.064  # mặt dưới nan (= mặt trên tấm thép) / mặt trên sàn
BASE_MASS = 2.147           # sàn, giá servo bánh, khoang điện tử (pin, Pi, E-stop), 3 servo bánh; không gồm đối trọng, nan, khay, bánh, trụ
WHEEL_RADIUS = 0.0508       # omni 4 in (kit LeKiwi, CAD P-L); trước S1G 0,0635
WHEEL_WIDTH = 0.04
WHEEL_MOUNT_RADIUS = 0.25   # tâm bánh cách tâm đế (R0,25; v0.4: 0,17)
WHEEL_ANGLES = (0.0, math.radians(120), math.radians(240))  # một bánh ra trước (R1b mục 4.2); v0.4: 60/180/300
SPOKE_R = (0.095, 0.188)    # nan nhôm 20x40 M1-BS-031: bán kính trong/ngoài (nan trước bắt từ 0,06 trên lưỡi tấm thép)
SPOKE_Z = (0.039, 0.059)
SPOKE_MASS = 0.066          # mỗi nan [giả định 0,6 kg/m]
COUNTERWEIGHT_MASS = 8.229  # thép M1-BS-032 8,5 mm + M1-BS-039 6 mm (DK-N40, Q8 mặc định (a) ~8 kg); trước S1G 5,0
COUNTERWEIGHT_HALF = (0.1825, 0.13, 0.00725)       # hộp bao 2 tấm (gồm lưỡi trước): x -0,245..0,120, y ±0,13, z 0,0245..0,039
COUNTERWEIGHT_POS = (-0.0625, 0.0, 0.03175)
TRAY_MASS = 0.798           # khay đế 4 mảnh 0,425 + cột đỡ 0,259 + thanh 2040 0,114 kg (cad/export/M1-PL-masses.json), tải khay <= 1 kg không mô hình
TRAY_HALF = (0.128, 0.19, 0.0142)                  # khay đế M1-ST-012 (gồm gờ 10 mm, vấu dưới 14 mm): x 0,064..0,320, y ±0,19
TRAY_POS = (0.192, 0.0, 0.2058)                    # mặt khay 0,21 m; mép trước 0,32 m: chui dưới bàn, đứng cách mép bàn 0,19
# Khay đế (25/11/2026): sàn thiết bị (laptop HOẶC Jetson + phụ kiện), không cho tay đặt vật. Hình học: cad/faibotic/khay_de.py,
# yêu cầu docs/M1-TRAM/R5-khay-de.md. Người dùng khai báo vật trên khay (khay.KhayTai); vùng tâm cho phép theo R4 4a.
KHAY_DE_X = (0.064, 0.320)  # m: mép sau (cách chân trụ x 0,058 khe 6 mm) .. mép trước (giữ như M1-ST-010)
KHAY_DE_Y = 0.19            # m: nửa bề rộng (laptop 16 in rộng tới 0,36 m nằm theo y, gờ 3 mm)
KHAY_DE_TAI_MAX = 3.0       # kg: tải định mức phân bố (thử 1,5x = 4,5 kg)
KHAY_DE_TAM_X = (0.08, 0.20)    # m: vùng tâm khối vật cho phép (R4 4a: x <= 0,20; x >= 0,08 để tâm còn trên khay)
KHAY_DE_TAM_Y = 0.04        # m: |y| tâm khối vật <= 0,04
KHAY_DE_LIFT_MIN = 0.04     # m: lift thấp nhất khi có khai báo vật (sàn cố định; vật cao hơn thì tính thêm theo đáy trạm, khay.lift_min)
KHAY_DE_KHE = 0.02          # m: khe tối thiểu giữa đỉnh vật và điểm thấp nhất của trạm/xe nâng
BASE_MAX_V = 0.4            # m/s: trần ĐỘ LỚN vận tốc tịnh tiến |(vx, vy)| (trước chỉ kẹp từng trục, đi chéo ra 0,566)
BASE_MAX_W = 1.5            # rad/s: trần phần cứng/kiểm QC; SDK còn kẹp theo chống lật BASE_MAX_W_TIP
# (E2, chống lật động >= 1,5): ca xấu nhất lift đỉnh, khay 1,3 kg góc xa, hai tay duỗi ngang tải 1 kg/tay, khối tay cận trên
# ARM_MASS_MOVING + BASE_TIP_ARM_EXTRA_KG, pan xấu nhất (tests/tipover_dong.py, mô hình cad/faibotic/tipover.py). Bốn trần cùng lúc:
BASE_MAX_ACC = 0.05         # m/s²: dốc tăng/giảm tốc tịnh tiến (độ lớn vector), phần mềm limits.BaseRamp
BASE_MAX_ALPHA = 0.15       # rad/s²: dốc gia tốc góc
BASE_MAX_W_TIP = 0.25       # rad/s: ly tâm ω²r lên tay duỗi
BASE_TURN_ACC_MAX = 0.01    # m/s²: |v|·|ω| (gia tốc hướng tâm khi đi cong); 0,4 m/s thì ω <= 0,025 rad/s
BASE_TIP_ARM_EXTRA_KG = 0.0705   # kg/tay dự phòng. servo yaw đã nằm trong ARM_MASS_MOVING (1,693 kg), giữ 70,5 g làm biên (cáp, ốc): hệ số 1,503; bỏ biên 1,517
BASE_TIP_MIN = 1.5          # hệ số chống lật động tối thiểu ở biên các trần trên

# ---------------- Cột + trục nâng ----------------
# Trụ AlohaMini2 (Post1-4 + đế nối, M1-LF-051..055, thanh răng) đứng trên tấm thép. S2 (G1b): xe nâng = khung T giữa + đế ngàm M1-TR-020;
# trạm hai tay (khung chữ U 2040, gá tay, cột camera đầu) lắp nhanh lên đế ngàm (cad/faibotic/tram_hai_tay.py, tram_de.py). Khung T trái/phải,
# xà ngang 2040, giá camera ngực đã bỏ.
COLUMN_XR = (-0.14288, 0.02603)    # hộp va chạm trụ trên (Post1-2), z BASE_Z1..COLUMN_TOP
COLUMN_YH = 0.05111
COLUMN_TOP = 0.92283               # đỉnh Post1
COLUMN_X = (COLUMN_XR[0] + COLUMN_XR[1]) / 2
COLUMN_SECTION = (COLUMN_XR[1] - COLUMN_XR[0], 2 * COLUMN_YH)
COLUMN_FOOT_XR, COLUMN_FOOT_YH, COLUMN_FOOT_Z = (-0.16069, 0.05829), 0.07542, (0.039, 0.32304)   # Post4 + đế nối (chân trụ rộng)
COLUMN_MASS = 2.227                # trụ in PETG (infill 0,40 giả định)
TMID_XR, TMID_YH, TMID_Z = (0.00566, 0.06888), 0.07084, (-0.15011, -0.00651)    # khung T giữa ôm trụ (khung xe nâng, gốc = đáy J1)
# Trạm hai tay (S2), khung xe nâng (x như robot, z = 0 ở đáy J1). Hộp va chạm sim theo CAD (pl_sim_dims.py, test_pl_sim_dims.py so khớp).
STATION_RAIL_XR, STATION_RAIL_YH, STATION_RAIL_Z = (0.0719, 0.1119), 0.3, (-0.027, -0.007)     # ray trước 2040 0,60 m
STATION_EAR_XR, STATION_EAR_Y, STATION_EAR_Z = (-0.1281, 0.0719), (0.1, 0.12), (-0.047, -0.007)  # tai 2040 0,20 m (tai trái; tai phải gương y)
HEAD_MAST_XR, HEAD_MAST_YH, HEAD_MAST_Z = (0.0499, 0.0699), 0.01, (0.029, 0.49)              # cột camera 2020
HEAD_GIMBAL_XR, HEAD_GIMBAL_YH, HEAD_GIMBAL_Z = (0.0439, 0.08083), 0.03, (0.49, 0.542)        # gá pan + tilt + camera đầu
DOCK_XR, DOCK_YH, DOCK_Z = (0.0299, 0.1219), 0.12, (-0.097, -0.067)                           # đế ngàm M1-TR-020 (thuộc xe nâng)
STATION_MASS = 2.094        # trạm chưa tay (khung, gá, cột + camera đầu, nửa ngàm, E-stop, hộp điện, dây) [CAD + giả định, tram_hai_tay.KG_FIXED]
STATION_COM = (0.0228, -0.0084, 0.0098)            # tâm khối trạm, khung xe nâng
H_J1_ABOVE_TABLE = 0.12     # chế độ bàn: đáy J1 trên mặt bàn (đế bàn tuỳ chọn + phần trạm dưới J1) [tự tính CAD]
HEAD_CAM_TILT_DEG = 50.0    # chúi mặc định (gá tay 20..70°, chốt định vị 35° và 50°)
HEAD_CAM_PAN_DEG = 0.0      # gá pan ±30°
HEAD_CAM_AXIS = (0.0619, 0.0, 0.53)             # trục tilt (0,65 m trên mặt bàn ở chế độ bàn), khung xe nâng
HEAD_CAM_POS = (0.07347, 0.0, 0.51621)             # đầu ống kính (gốc camera sim "head"), khung xe nâng
HEAD_CAM_XYAXES = (0.0, -1.0, 0.0, 0.76604, 0.0, 0.64279)
HEAD_CAM_FOVY = 70.0        # sim; H65V1 thật chưa đo (DK FOV H65V1: 80° chéo hay ngang)
# Vùng cấm trạm (luật mềm, limits.keepout_violations; R3b mục 4, đối chiếu quét mesh pl_lift_scan.py --tay)
KEEPOUT_HEAD_R = 0.14       # khuỷu, cổ tay, tâm kẹp ngoài cầu quanh trục tilt đầu camera (R3b 0,12; quét mesh 24/11: kẹp rộng 128 mm chạm gá ở 0,129 -> 0,14)
KEEPOUT_DECK_X, KEEPOUT_DECK_Z = 0.14, 0.08    # cổ tay, tâm kẹp ở x < 0,14 m trước trục pan thì z >= 0,08 m trên đáy J1 (ray, đế ngàm, khung T giữa);
                                                # R3b đề xuất 0,10/0,07 cho tâm kẹp; quét mesh 24/11 thấy kẹp đập ray/đế ngàm -> nới, áp cả cổ tay
KEEPOUT_TRAY_MARGIN = 0.03  # di_dong: tâm kẹp trên khay (hộp khay + 3 cm viền) phải cao hơn mặt khay >= 3 cm (quét mesh: kẹp xuyên khay ở lift thấp)
KEEPOUT_TABLE_MARGIN = 0.01  # tram_ban: tâm kẹp >= mặt bàn + 1 cm
KEEPOUT_MAST_R = 0.13       # S2 (quét mesh 24/11: kẹp chạm cột camera tới -10 mm, tâm kẹp khi đó cách trục cột 109 mm): khuỷu, cổ tay, tâm kẹp
                            # ngoài trụ đứng r 0,13 m quanh trục cột camera, dưới trục tilt. Kiểm lại bằng quét mesh (M1-PL-tram-scan-tay.json, DK-N53)
LIFT_RANGE = (0.0, 0.45)   # S1G: hành trình CAD P-L -160..+290 mm (bánh răng còn ăn dải răng; khung T trái/phải còn tựa trụ 1); trước: 0..0,50
CARRIAGE_MASS = 0.915       # S2: xe nâng giữ lại theo CAD (khung T giữa, bánh răng 20 răng, STS3095, chốt hãm, công tắc, đế ngàm); không gồm trạm (STATION_MASS); S1G 1,057 có khung T trái/phải + camera ngực
# v0.5 (23/11/2026): thanh răng + bánh răng in AlohaMini2, 1 STS3095 truyền động trực tiếp (KHÔNG tự hãm), chốt solenoid 12 V cài vào
# răng khi cuộn mất điện, 2 công tắc hành trình, cảm biến "chốt đã rút". Thay vitme T8x2 + tăng tốc 16:1 của v0.4.
LIFT_SPEED_MAX = 0.0145     # m/s: GIỮ số của v0.4 (vitme) tới khi đo thử rơi DK-N4; thanh răng cho tới ~65 mm/s [R2 7.2, tự suy ra], chưa nới
LIFT_SERVO_LIMIT_PCT = 70   # % định mức liên tục 22,5 kg·cm: trần trên của giới hạn mô-men lift (xem LIFT_TORQUE_LIMIT_PERMILLE)
LIFT_PINION_PITCH_D = 0.030    # m: (R3d mục 5): bánh 20 răng m1,5, vòng chia Ø30 (trước 0,0428 theo bánh 26 răng AlohaMini2); không lò xo
LIFT_PINION_R = LIFT_PINION_PITCH_D / 2      # 15 mm: cánh tay đòn lực răng
LIFT_RACK_MODULE = 1.5         # mm [giả định: chưa đếm răng file AlohaMini2]
LIFT_NOTCH_PITCH = math.pi * LIFT_RACK_MODULE / 1000   # m: bước răng = khoảng nấc chốt, ~4,71 mm; rơi tối đa khi mất điện = 1 nấc
LIFT_DESIGN_LOAD_KG = 8.28     # kg treo trên bánh răng có trạm (R3c): xe nâng + trạm + hai tay + tải 0,5 kg/tay; chờ cân thật (DK-N6). Trước trạm: 6,1
LIFT_HARD_STOP_MARGIN = (0.012, 0.011)   # m từ công tắc tới đệm PU chặn cứng (dưới, trên): SỐ CỦA v0.4 (ray MGN12, cad/faibotic/lift.py), chờ CAD thanh răng v0.5
LIFT_RAIL_LENGTH = 0.700               # m, ray MGN12H của v0.4: chờ CAD v0.5 (test hình học ray đọc cad/export, chưa dựng lại)
# 1 vòng servo = pi x 30 mm = 94,2 mm -> 4096 / 0,0942 = 43.460 bước/m (bánh 26 răng: 30.463; v0.4 vitme: 128.000). 0,45 m = 19.557 bước, ~4,8 vòng: cần đa vòng.
LIFT_STEPS_PER_M = round(4096 / (math.pi * LIFT_PINION_PITCH_D))
# raw 0 = đáy hành trình (công tắc LIFT_BOT, homing). 0..0,5 m = 0..21.730 bước, vừa 15 bit có dấu (±32.767) mà không cần dời gốc;
# v0.4 phải đặt raw 0 giữa hành trình (128.000 bước/m x 0,5 m = 64.000 > 32.767), nay bỏ. Dải thật của chế độ bước STS: CHƯA kiểm bảng thanh ghi.
LIFT_RAW_ZERO_M = 0.0
# Giới hạn mô-men lift (R2 7.2 điều (ii)): lực răng tại giới hạn <= 50% lực gãy răng đo được F_đo, nên
#   TORQUE_LIMIT = min(70% định mức, 0,5 x F_đo x r).  TORQUE_LIMIT servo là ‰ của mô-men CHẶN 95 kg·cm (không phải định mức).
# F_đo CHƯA ĐO (DK-N18: >= 5 răng in cùng hướng, tải tĩnh: thử cả bánh 20 răng). 120 N là chỗ giữ [giả định, chờ đo]: R2 7.2 ghi 60 N cho
# răng PETG, đọc là 50% của 120 N. Với 120 N và r 15 mm: lực giới hạn 60 N < 85,4 N giữ tải thiết kế 8,28 kg + gia tốc 0,5 m/s²: lift không
# nâng đủ tải thiết kế ở giới hạn này (LIFT_LIMIT_COVERS_DESIGN_LOAD False) tới khi DK-N18 đo răng >= 171 N.
LIFT_TOOTH_F_MEASURED_N = 120.0     # [giả định, chờ đo] lực tĩnh làm gãy răng PETG (DK-N18); đổi bằng số đo thật
LIFT_TOOTH_SAFETY = 0.5             # R2 7.2 (ii): lực răng tại giới hạn <= 50% F_đo


def lift_torque_limit_permille(f_measured_n: float | None = None) -> int:
    """TORQUE_LIMIT servo lift (‰ mô-men chặn) = min(70% định mức, 0,5 x F_đo x r) theo R2 7.2 (ii)."""
    f = LIFT_TOOTH_F_MEASURED_N if f_measured_n is None else f_measured_n
    rated_cap_nm = LIFT_SERVO_LIMIT_PCT / 100 * STS3095.rated_kgcm * KGCM
    tooth_cap_nm = LIFT_TOOTH_SAFETY * f * LIFT_PINION_R
    return int(round(1000 * min(rated_cap_nm, tooth_cap_nm) / STS3095.stall_nm))


LIFT_TORQUE_LIMIT_PERMILLE = lift_torque_limit_permille()
LIFT_FORCE_LIMIT_N = LIFT_TORQUE_LIMIT_PERMILLE / 1000 * STS3095.stall_nm / LIFT_PINION_R   # lực răng lớn nhất servo đẩy được
LIFT_HOLD_FORCE_N = LIFT_DESIGN_LOAD_KG * (G + 0.5)                                         # tải thiết kế + gia tốc 0,5 m/s²
LIFT_LIMIT_COVERS_DESIGN_LOAD = LIFT_FORCE_LIMIT_N >= LIFT_HOLD_FORCE_N                      # False với F_đo giữ chỗ 120 N: báo, không giấu
# Chốt hãm (R2 7.2, DK-N17): thời gian chờ cảm biến và ngưỡng kẹt [giả định, đo ở EVT]
LIFT_LATCH_RETRACT_TIMEOUT_S = 0.5   # cuộn có điện -> cảm biến "rút"
LIFT_LATCH_ENGAGE_TIMEOUT_S = 0.5    # cuộn mất điện -> cảm biến "cài"
LIFT_LATCH_CONFIRM_S = 0.05          # cảm biến phải giữ mức mới tính
LIFT_LATCH_IDLE_S = 5.0              # lift đứng yên lâu hơn -> cài chốt, tắt mô-men lift [R2 7.2 đề xuất]
LIFT_JAM_FRACTION = 0.9              # tải lift >= 90% TORQUE_LIMIT khi chốt chưa rút (hoặc liên tục khi chạy) -> kẹt, dừng
LIFT_JAM_CYCLES = 3                  # số chu kỳ step() liên tiếp khi đang chạy
COLUMN_SECTION = (0.04, 0.04)            # nhôm định hình 40x40 (x, y) m [thay hộp 0,06 x 0,08 của sim; sim chưa đổi]
# S1G (CAD thân P-L, cad/faibotic/assembly_pl.py): gốc tay = khớp URDF AlohaMini2 left/right_base_joint (±187,26; -59,9; 477,5 mm trên
# vertical_link, vertical_link cao 96,196 mm ở vertical_move 0) quy về hệ robot (x trước = -y URDF). Lift SDK 0 = lift CAD -160 mm (đáy dải
# răng còn ăn khớp, cad/export/AlohaMini2_placement.json lift_range_mm -160..290). Trước S1G: SHOULDER_X 0,12, SHOULDER_Y 0,17, trục vai 0,55-1,05.
LIFT_CAD_OFFSET = -0.160    # lift CAD (assembly_pl --lift, m) = lift SDK + LIFT_CAD_OFFSET
ARM_MOUNT_CAD_Z = 0.096196 + 0.4775   # gốc tay (đáy J1) ở lift CAD 0, URDF AlohaMini2
SHOULDER_X = 0.0599         # trục pan (gốc tay) trước tâm đế (URDF: -y 59,9 mm); trục vai lệch thêm SHOULDER_DR ra trước
SHOULDER_Y = 0.2375         # S2: nửa khoảng vai trạm 0,475 m (ray rãnh T cho dời ±25 mm khi lắp); S1G theo URDF AlohaMini2 0,18726

# ---------------- Tay AM-ARM200 gốc (P-L, 23/11/2026) ----------------
# Chốt P-L: 2 tay AM-ARM200 follower NGUYÊN (vai, khuỷu một servo, không lò xo, không chi tiết nhôm).
# (24/11/2026): 6+1 ĐÚNG NHƯ AM-ARM200 gốc: mở khớp wrist_yaw (thay/Q4 "5+1", bỏ khoá yaw M1-AR-025), 7 servo mỗi tay.
# Kích thước khâu từ URDF upstream cad/upstream/AM-ARM/am-arm200/urdf/am-arm200.urdf (commit 5b03af1e), suy bằng sim/tools/am_arm200_pl.py,
# tests/test_pl_arm_urdf.py so lại. Bản ghép đôi M1 (P-A, v0.3-v0.4.6: link Faibotic L1 0,140, D0 0,045, lò xo) chỉ còn giá trị lịch sử.
# Khung tay (tay trái): gốc ở đáy đế J1 trên trục pan, x trước, y trái, z lên. Tay phải lấy gương y.
ARM_VARIANT = "P-L"
D0 = 0.14823          # đáy J1 -> trục vai, theo phương đứng (URDF: 39,67 + 108,56 mm)
SHOULDER_DR = 0.05799  # trục vai lệch ra trước trục pan (URDF: 57,99 mm); P-A: 0
# Gốc tay (đáy J1) nằm dưới trục vai D0: trục vai = SHOULDER_Z0 + lift.
ARM_BASE_Z0 = ARM_MOUNT_CAD_Z + LIFT_CAD_OFFSET   # 0,4137 m: độ cao gốc tay khi lift SDK = 0 (CAD P-L, S1G)
SHOULDER_Z0 = ARM_BASE_Z0 + D0                     # 0,5619 m: trục vai khi lift = 0; tới 1,0119 m ở lift 0,45
L1 = 0.13491     # trục vai -> trục khuỷu (URDF: hypot(128,49; 41,12) mm)
# Quy ước SDK: tư thế 0 = đường vai-khuỷu nằm ngang ra trước. Ở tư thế 0 của URDF cánh tay ngả ra sau, ngẩng 17,75 độ: đường vai-khuỷu ở 162,25 độ.
UPPER_URDF_DEG = 162.2524
# Đổi góc URDF upstream (cũng là góc tay leader AM-ARM200 khi hiệu chuẩn cùng tư thế 0) -> góc SDK: q_sdk = dấu x q_urdf + lệch (rad).
# pan: trục URDF -z; vai, khuỷu: trục URDF -x của hệ đế (= -y SDK); gập: +x; cuộn: dọc cẳng tay ra trước (= +x SDK).
# yaw: trục URDF left_wrist_yaw_joint -z (khung SDK, tư thế 0) -> SDK quay quanh +z của body cổ tay (dương = ngược chiều kim đồng hồ nhìn từ trên khi cổ tay 0).
URDF_TO_SDK = {"pan": (-1.0, 0.0), "shoulder": (-1.0, -math.radians(UPPER_URDF_DEG)), "elbow": (-1.0, math.radians(UPPER_URDF_DEG)),
               "wrist_flex": (1.0, 0.0), "wrist_yaw": (-1.0, 0.0), "wrist_roll": (1.0, 0.0)}
# Cổ tay theo URDF: trục gập, trục cuộn.
L2 = 0.19841     # khuỷu -> trục gập dọc cẳng tay
WF_DZ = -0.00316  # trục gập thấp hơn đường trục khuỷu
LW = 0.08912     # trục gập -> gốc khớp cuộn dọc trục cuộn
ROLL_DZ = -0.02332  # trục cuộn thấp hơn trục gập
ROLL_DY = -0.00381  # trục cuộn lệch ngang khỏi mặt phẳng tay (tay trái)
# (6+1): khớp yaw giữa giá J5 (left_Wrist_Pitch_Roll) và vỏ J56 (left_wrist_yaw), theo URDF upstream; gốc khớp so với trục gập (khung cổ tay):
YAW_DX = 0.02813   # dọc cẳng tay
YAW_DZ = -0.00130  # thấp hơn trục gập; lệch ngang = ROLL_DY. Trục yaw = z khung cổ tay (vuông góc trục gập và trục cuộn). Ở yaw 0 chuỗi trùng 5+1 cũ.
# Kẹp gốc AM-ARM200 (24/11/2026; thay kẹp cũ): thân J7 + giá camera (link left_Fixed_Jaw) cố định theo khớp cuộn, hàm J7 (left_Moving_Jaw)
# quay quanh trục khớp left_gripper (song song z khung tay khi cuộn 0). Số đo CAD: cad/faibotic/gripper_am.py --scan -> cad/export/AM_gripper_PL.json
# (STEP follower đăng ký vào mesh URDF, am_j7_register.py). Tâm kẹp = trung điểm hai đầu ngón khi khép lùi TIP_LEN dọc trục cuộn; lệch ngang về phía ngón cố định.
LT = 0.18294     # trục gập -> tâm kẹp dọc trục cuộn = LW + 0,09382 (kẹp cũ: 0,21459, dài hơn 31,7 mm)
TCP_DY = 0.01013   # tâm kẹp lệch ngang khỏi trục cuộn (tay trái, cuộn 0), về phía ngón cố định
TCP_DZ = -0.00016  # tâm kẹp thấp hơn trục cuộn (cuộn 0)
TIP_LEN = 0.010  # đầu ngón trước tâm kẹp
FIXED_FACE_Y = 0.0100   # mặt trong ngón cố định (khung tay, tay trái, cuộn 0); vật kẹp tựa vào mặt này (kinematics.grasp_tcp)
JAW_PIVOT = (0.02161, -0.02021, -0.00203)   # trục khớp kẹp trong khung tay (tay trái; sim/tools/am_arm200_pl.py)
GRIPPER_CLOSE_URDF = 1.12757                # góc URDF left_gripper khi hai ngón chạm (CAD 64,605 độ)
GRIPPER_OPEN_RAD = math.radians(64.0)       # độ mở mềm từ chỗ khép (= tư thế 0 URDF upstream) [giả định, CAD không có chặn cứng tới 180 độ]
GRIPPER_MAX_GAP = 0.0973    # khoảng cách hai đầu ngón khi mở mềm (CAD 97,25 mm); lúc khép 7,65 mm giữa tâm hai đầu ngón (ngón chạm)
GRIPPER_TIP_R = 0.0847      # trục kẹp -> đầu hàm động (CAD) [tự tính]
# Lực kẹp ở đầu ngón = mô-men servo kẹp / GRIPPER_TIP_R. Chặn STS3215 12 V 30 kg.cm (2,94 N.m) -> 34,7 N; giới hạn mô-men 50% -> 17,4 N (sim).

# Khối lượng, trọng tâm, quán tính từng body (kg, m, kg·m² trong khung body ở tư thế 0, trục thẳng hàng khung tay).
# Phần in: URDF upstream, khối đặc 1,0 g/cm³ (SolidWorks; đối chiếu OB_F_J7_Jaw 28,18 cm³ = 28,2 g). In thật PETG 4 thành 15% nhẹ hơn
# (~0,5-0,6 g/cm³ hiệu dụng [tự suy ra từ cad/export/arm_mass.json]) nên số này là cận trên cho phần in. URDF KHÔNG có servo: cộng ở ARM_POINT_MASSES.
LINK_PRINT = {   # body: (khối lượng, trọng tâm, (ixx iyy izz ixy ixz iyz)) [URDF upstream, sim/tools/am_arm200_pl.py]
    "mount": (0.31988, (-0.03147, -0.00028, 0.03357), (6.37405e-4, 6.55681e-4, 6.54163e-4, -1.392e-6, 1.55474e-4, 3.035e-6)),
    "shoulder": (0.17036, (0.04158, -0.00089, 0.08061), (2.41774e-4, 3.17987e-4, 1.28238e-4, -7.79e-7, -6.575e-6, -5.565e-6)),
    "upper": (0.26386, (0.06931, 0.00003, 0.01695), (2.62321e-4, 5.01588e-4, 6.29157e-4, -1.138e-6, -1.25855e-4, -5.84e-7)),
    "fore": (0.27006, (0.08050, -0.00296, -0.00407), (2.07982e-4, 6.76984e-4, 8.36241e-4, 3.816e-6, 7.564e-6, -1.34e-7)),
    "wrist": (0.03045, (0.01479, -0.00273, 0.00073), (1.1295e-5, 8.225e-6, 1.4509e-5, -5.07e-7, -2.43e-7, -1.6e-8)),   # giá J5 (left_Wrist_Pitch_Roll), gốc = trục gập
    "yaw": (0.11421, (0.03137, -0.00018, 0.00894), (7.2612e-5, 7.4945e-5, 4.7047e-5, -4.9e-8, 1.8591e-5, 6.7e-8)),   # vỏ J56 (left_wrist_yaw), gốc = khớp yaw
    "hand": (0.07435, (0.02505, 0.01205, 0.00929), (2.9444e-5, 5.4755e-5, 5.5044e-5, -9.076e-6, 1.597e-6, 5.9e-7)),   # thân kẹp J7 + giá camera (left_Fixed_Jaw)
    "jaw": (0.02816, (0.03073, 0.00893, 0.00157), (6.801e-6, 1.8295e-5, 1.517e-5, -3.995e-6, -5e-9, -3e-9)),   # hàm J7 ở góc khép, gốc = trục kẹp
}
# Khối điểm thêm vào body (tay trái; tay phải gương y): servo theo docs/vi/servo-datasheet.md.
# Servo đặt tại trục khớp nó quay [giả định, thân servo thật lệch ~20 mm]; servo pan và vai nằm ở đế J1/thân xoay, không vào mô-men vai.
ARM_POINT_MASSES = (
    ("mount", "servo pan STS3095", 0.1875, (0.0, 0.0, 0.03)),
    ("shoulder", "servo vai STS3095", 0.1875, (SHOULDER_DR, 0.0, D0)),
    ("upper", "servo khuỷu STS3095", 0.1875, (L1, 0.0, 0.0)),
    ("upper", "cáp + ốc [giả định]", 0.030, (L1 / 2, 0.0, 0.0)),
    ("fore", "servo gập STS3215", 0.0705, (L2, 0.0, WF_DZ)),
    ("fore", "cáp + ốc [giả định]", 0.030, (L2 / 2, 0.0, 0.0)),
    # servo cuộn và servo yaw nằm trong vỏ J56 (link wrist_yaw upstream) -> body yaw; khoá yaw M1-AR-025 bỏ ở 6+1
    ("yaw", "servo cuộn STS3215", 0.0705, (LW - 0.020 - YAW_DX, 0.0, ROLL_DZ - YAW_DZ)),
    ("yaw", "servo yaw STS3215", 0.0705, (0.0125, 0.0, 0.03647)),   # tâm hộp bao chồng servo y < 20 mm của mesh wrist_yaw (sim/tools/am_arm200_pl.py YAW_SERVO_*)
    # kẹp AM-ARM200: servo kẹp đặt theo khớp kẹp URDF (tâm hộp bao, cad/export/AM_gripper_PL.json); camera theo khung left_camera URDF [vị trí đề xuất]
    ("hand", "servo kẹp STS3215", 0.0705, (0.02162, -0.00772, -0.00127)),
    ("hand", "camera cổ tay H65V1 + cáp [giả định 25 g]", 0.025, (0.00085, -0.00181, 0.06877)),
)
# Hệ số khối lượng in thật / khối đặc URDF theo body (PETG 1,27 g/cm³, 4 thành, 15%: cad/faibotic/print_mass_pl.py, đo trên STEP 23/11/2026) [tự tính].
# Dùng cho cột "ước tính in" của bảng tải (demos/static_torque.py); mô hình sim mặc định giữ khối đặc (cận trên).
PRINT_FACTOR_EST = {"mount": 0.876, "shoulder": 1.0, "upper": 0.634, "fore": 0.676, "wrist": 0.863, "yaw": 0.863}   # yaw lấy hệ số cụm J5+J56 cũ
JAW_MASS = LINK_PRINT["jaw"][0]   # hàm J7 28,2 g (URDF, khối đặc)
GRIPPER_MASS = 0.17301  # cả kẹp gồm servo và giá camera, chưa có camera: 74,35 + 28,16 + 70,5 g (khối đặc); kẹp cũ 193,5 g


def _body_mass_com(body: str) -> tuple[float, tuple[float, float, float]]:
    m, c = 0.0, [0.0, 0.0, 0.0]
    if body in LINK_PRINT:
        mp, cp, _ = LINK_PRINT[body]
        m, c = mp, [mp * v for v in cp]
    for b, _, mi, ci in ARM_POINT_MASSES:
        if b == body:
            m += mi
            c = [a + mi * v for a, v in zip(c, ci)]
    return m, tuple(v / m for v in c)


LINK_MASS = {b: round(_body_mass_com(b)[0], 5) for b in ("mount", "shoulder", "upper", "fore", "wrist", "yaw", "hand", "jaw")}
LINK_COM3 = {b: tuple(round(v, 5) for v in _body_mass_com(b)[1]) for b in LINK_MASS}
LINK_COM = {b: LINK_COM3[b][0] for b in ("upper", "fore", "wrist", "yaw", "hand")}   # thành phần dọc khâu (cho mã cũ)
ARM_MASS_MOVING = sum(LINK_MASS[b] for b in ("shoulder", "upper", "fore", "wrist", "yaw", "hand", "jaw"))   # phần quay theo pan
# (24/11/2026): xuất lại CAD 6+1 (cad/faibotic/assembly_pl.py --masses -> M1-PL-masses.json "tay_cad"): chi tiết in = thể tích STEP follower x 1,0 g/cm3,
# 7 servo + camera + cáp như trên. CAD nhẹ hơn tổng URDF 0,073 kg/tay (STEP J1-J4 nhẹ hơn khối URDF 5-12 %; phần yaw lệch 5 g). Mô hình động lực/sim
# giữ LINK_PRINT (URDF, cận trên); số công bố khối lượng lấy CAD. tests/test_tipover.py kiểm khớp masses.json.
ARM_MASS_CAD = 2.128         # kg, một tay đủ 7 servo (spec cộng tay: ARM_MASS_MOVING + LINK_MASS["mount"] = 2,201)
ARM_COM_R_CAD = 0.189        # m, tâm khối lượng tay duỗi ngang cách trục pan (CAD); tipover.arm_pl URDF không servo: 0,181
ROBOT_MASS_CAD = 21.17       # kg = thân không tay 16,91 (masses.json tong_khong_tay_kg) + 2 x ARM_MASS_CAD

SHOULDER_SOFT_MAX_DEG = 48.5   # v0.4.3: 49,5 -> 48,5; đệm chặn dưới chạm ở 50 độ (FAILSAFE_PADS["shoulder_up"]), biên 1,5 độ = 1,18 mm tại r 45 >= dung sai dày đệm PU +-0,5 mm (v0.4.2: 0,5 độ = 0,39 mm)
ELBOW_SOFT_MAX_DEG = 127.0     # v0.4.3: 130 -> 127; đệm khuỷu M1-AR-022 chạm ở 128 (FAILSAFE_PADS["elbow_high"]), biên 1 độ = 1,06 mm tại r 61 >= 0,5 mm
ARM_JOINTS = ("pan", "shoulder", "elbow", "wrist_flex", "wrist_yaw", "wrist_roll")   # thứ tự chuỗi URDF upstream (gập -> yaw -> cuộn)
# Quy ước: zero = tay duỗi thẳng ngang phía trước. shoulder/elbow/wrist_flex dương = gập xuống.
PAN_SOFT_MAX_DEG = 70.0
ARM_LIMITS = {
    "pan": (math.radians(-PAN_SOFT_MAX_DEG), math.radians(PAN_SOFT_MAX_DEG)),   # v0.5 (QC2 cột nâng): ±110 -> ±70, tay chạm tay đỡ bên kia từ |pan| >= 90 và cột từ 30 khi gập (cad/export/M1-LF-scan-*.json)
    #
    "shoulder": (math.radians(-80), math.radians(SHOULDER_SOFT_MAX_DEG)),      # v0.4.3: +49,5 -> +48,5; v0.4.2 (-2): +90 -> +49,5, link chạm đệm dưới ở 50 (mất 0 tầm tool-down bàn 0,60-0,90 m); v0.4.1 vòng 2: -85 -> -80, đệm vai trên chạm ở -80,5 (thêm 5 độ hành trình); v0.4: -90 -> -85: chặn đệm chạm ở -85,5 (mất điện, CHẶN 2); v0.3: -100 -> -90
    "elbow": (math.radians(-10), math.radians(ELBOW_SOFT_MAX_DEG)),         # v0.4.3: 130 -> 127 (chặn cứng 136 -> 133, đệm dời chạm ở 128); v0.4.1 vòng 2: -20 -> -10 (chặn khuỷu ngược có hành trình);         # v0.3: 150 -> 130 (khe 8,2 mm tới link; 140 độ chạm); xem ELBOW_MAX_BY_SHOULDER
    "wrist_flex": (math.radians(-77.5), math.radians(87.5)),   # v0.4.4: -100..120 -> -77,5..87,5: tự va chạm cụm cổ tay với đầu cẳng tay (ngàm M1-AR-026, servo gập) quét vòng 2: chạm thật trong (90; 90,5], phía âm sạch tới -77,5 (chỉ quét tới đó); biên dương 3 độ (cad/export/couple_v044.json, failsafe_v044_wrist_roll_scan.json)
    # URDF upstream ghi ±3,14 (WRIST_YAW_URDF_LIMIT) cho mọi khớp, không phải tầm thật; SDK tạm ±90 [giả định].
    # (DK-N66, 24/11/2026): quét mesh STEP (gripper_am.py --yaw-scan -> cad/export/AM_wrist_yaw_scan.json): vỏ J56 chạm giá J5 ở yaw URDF
    # +84,6 / -83,7 độ (SDK -84,6 / +83,7) ở mọi gập, cuộn, độ mở kẹp; biên 3 độ -> SDK -81..+80. Gập > 0 thì đáy/vỏ J56 chạm cẳng tay J4
    # sớm hơn ở yaw SDK dương: giới hạn ghép wrist_yaw_range(gập), bảng WRIST_YAW_COUPLE_*. Ở yaw 0 mọi bảng quét cũ vẫn đúng.
    "wrist_yaw": (math.radians(-81), math.radians(80)),
    "wrist_roll": (math.radians(-160), math.radians(160)),  # giá camera OB_F_Camera chạm đáy vỏ cổ tay J56 khi cuộn URDF trong (-147; -87) độ (tới 235 mm3,
                                                            # cad/export/AM_gripper_roll_scan.json). Chưa thu hẹp (thu về -82..160 làm demo lab chọn nghiệm cuộn khác,
                                                            # trụ đổ): DK-N54 mở, chốt trước EVT (vùng cấm cuộn hoặc cắt giá camera)
}
# v0.4.2 (-2): chặn dưới dời (link chạm đệm ở vai 50, cứng 62, giá chữ Z dưới đáy má, r 45 mm): quét CAD vai -80..62 x khuỷu -32..133: 0 mm3 giao gá khuỷu
# (cad/export/failsafe_v042_low_scan.json). Khuỷu 136 (giới hạn cứng, sau đệm khuỷu 131-136) giao đáy giá 69-464 mm3 ở vai 40..62: chỉ khi cả hai đệm cùng chạm đáy (ma sát thấp), CÒN HỞ.
# Bảng v0.4.1 (thanh r 90 ở vai 97,5, khuỷu <= 95-130 theo vai) đã bỏ.
LOWSTOP_ELBOW_CLEAR_DEG = ((38.0, 133.0), (62.0, 133.0))   # khuỷu lớn nhất còn 0 mm3 giao (vai < 38: quét 0 mm3 tới 136, hàm trả 140); v0.4.3: chặn cứng khuỷu 133 nên bảng không còn chặn gì thêm


# v0.4.3: giới hạn ghép cổ tay-khuỷu-vai. Quét CAD kẹp J7 + vỏ cổ tay J56 (AM-ARM200) với giá chặn M1-SH-017 + đệm M1-SH-018, thanh chặn trên + đệm, gá vai M1-SH-003
# (cad/faibotic/scan_wrist_v043.py, cad/export/failsafe_v043_wrist_scan.json): giao chỉ xảy ra ở cổ tay > 45 độ khi khuỷu >= 100 và vai >= 0; vai -80 sạch cả dải.
# Vòng 2 (QC2): vật cản thêm link, chốt cân bằng, hai lò xo (chỉ chốt giao, ở cổ tay 120); cuộn cổ tay ±90, ±160 quét tại giới hạn ghép (giả định trục cuộn = tâm vỏ J56).
# Bảng: giới hạn cổ tay lớn nhất (độ) ở từng nút (vai, khuỷu) = (mức giao đầu tiên tinh tới 2,5 độ) - 5 độ biên. Tra bằng nhỏ nhất bốn nút quanh tư thế (bảo thủ).
# Cổ tay < 0: quét tới -100, không giao. Cuộn cổ tay 0, trục cổ tay suy từ chốt J5 [giả định như arm_fit]: cuộn khác 0 chưa có số liệu.
# bảng trên quét với cổ tay quay quanh chốt J5 (hình cũ). v0.4.4 (22/11/2026) thay bằng quét lại trên CAD theo cuộn:
# cad/faibotic/scan_wrist_v044.py --sweep -> cad/export/failsafe_v044_wrist_roll_scan.json, bảng ở cad/export/couple_v044.json. Vật cản: giá chặn dưới + đệm,
# thanh chặn trên + đệm, gá vai, link, chốt, lò xo, đệm/tab trên link, tự va chạm với cẳng tay (gá khuỷu, ống, ngàm, servo gập); chi tiết quay: giá J5, chốt, vỏ/đáy/nắp J56,
# khoá yaw, kẹp J7 + ngón (mesh), camera cổ tay (vị trí đề xuất). Cuộn -160, -135, -90, -45, 0, 45, 90, 135, 160; cổ tay thô -77,5..90,5 rồi tinh 2,5.
# Ô = nhỏ nhất qua mọi cuộn của mức sạch lớn nhất (liền từ 0) - 3 độ. Ô 87 = chỉ bị tự va chạm (đáy J56 với ngàm ở 90,5). Cổ tay âm tới -77,5: sạch.
# Chạm quyết định: camera cổ tay (vị trí đề xuất) ở cuộn ±160 với giá chặn dưới/gá vai (9 ô, vai >= 0, khuỷu >= 110); kẹp ở cuộn ±45; đáy J56 với giá chặn dưới ở vai 40/48,5 khuỷu 127 (mọi cuộn).
# v0.4.5 (DK-W4, 22/11/2026): camera cổ tay dời ra trước mặt đầu vỏ J56 + giá M1-AR-009 (assembly_full WCAM_DX/DZ 19/-24), quét lại cùng cách
# (cad/export/failsafe_v045_wrist_roll_scan.json). So bảng v0.4.4: nới 9 ô (vai 0 khuỷu 120/127: 57->62, 37->47; vai 30: 57/42/42 -> 59,5/54,5/49,5;
# vai 40: 49,5/47 -> 59,5/57; vai 48,5: 52/49,5 -> 62/57), siết 3 ô (vai -80 khuỷu 127: 84,5->74,5; vai 0 khuỷu 110: 87->82; vai 48,5 khuỷu 100: 79,5->67),
# ô nhỏ nhất giữ 22. Chạm quyết định vẫn là camera/giá với chặn dưới ở cuộn ±160; dời camera ngang ±15/25 hoặc ra trước 30/45 đều làm xấu hơn (cad/bao-cao-cad.md v0.4.5).
WRIST_COUPLE_SHOULDER_DEG = (-80.0, 0.0, 30.0, 40.0, 48.5)
WRIST_COUPLE_ELBOW_DEG = (-10.0, 70.0, 100.0, 110.0, 120.0, 127.0)
WRIST_COUPLE_MAX_DEG = (
    (87.0, 87.0, 87.0, 87.0, 87.0, 74.5),
    (87.0, 87.0, 87.0, 82.0, 62.0, 47.0),
    (87.0, 87.0, 87.0, 59.5, 54.5, 49.5),
    (87.0, 87.0, 82.0, 59.5, 57.0, 39.5),
    (87.0, 87.0, 67.0, 62.0, 57.0, 22.0),
)


# v0.5 (QC2 cột nâng): giới hạn khuỷu theo vai khi đã lắp cột (quét chuỗi tay duỗi bán kính 30 mm với cột, ray, xe, đầu; pan ±70; lift 0 và 0,50 m;
# cad/faibotic/lift_scan.py derive -> M1-LF-scan-derive.json: không chạm tới 127 (vai <= 0), 116 (16), 107 (24), 98 (32), 89 (40), 80 (48,5)); trừ biên 3 độ, làm lõm để vùng cho phép lồi (nội suy tuyến tính an toàn).
# Cáp chưa quét [chưa có số liệu]. Gộp với giới hạn ghép cổ tay WRIST_COUPLE (lấy nhỏ nhất).
# v0.4.4: quét lại bằng đám điểm mesh thật (link, gá khuỷu, cẳng tay, ngàm, servo gập, cụm cổ tay, kẹp, camera; lưới 5 mm) với hộp vật cản nới 10 mm,
# pan ±70 bước 5, lift 0 và 0,50 m, hai tay, khuỷu bước 3, cổ tay -90..120 bước 15 (cad/faibotic/lift_scan.py derive_mesh -> cad/export/M1-LF-scan-derive_mesh.json):
# khuỷu không chạm nhỏ nhất qua mọi góc cổ tay, trừ 3 độ (đề xuất 127, 127, 119, 113, 107, 101, 92, 86, 86, 83, 74, 62). Chạm đầu: kẹp/vỏ J56 với dầm, tấm bên xe nâng,
# cột, tấm đế cột ở lift 0. Bảng cũ (chuỗi 434 mm r 30) bỏ. Làm lõm (chỉ HẠ, không nâng nút nào) để vùng cho phép lồi như bảng cũ: vai 0..40 theo đường dốc -0,8 độ/độ
# từ (0; 111); hạ 2..4 độ ở vai 0, 8, 16, 32, 36, 40 so với số quét.
ELBOW_COUPLE_SHOULDER_DEG = (-80.0, -32.0, -16.0, 0.0, 8.0, 16.0, 24.0, 32.0, 36.0, 40.0, 44.0, 48.5)
ELBOW_COUPLE_MAX_DEG = (127.0, 127.0, 119.0, 111.0, 104.6, 98.2, 91.8, 85.4, 82.2, 79.0, 74.0, 62.0)


def elbow_max(q_shoulder: float) -> float:
    """Giới hạn trên của khuỷu (rad) theo vai (rad): nội suy tuyến tính ELBOW_COUPLE_*, không vượt ARM_LIMITS khuỷu."""
    d = math.degrees(q_shoulder)
    xs, ys = ELBOW_COUPLE_SHOULDER_DEG, ELBOW_COUPLE_MAX_DEG
    if d <= xs[0]:
        v = ys[0]
    elif d >= xs[-1]:
        v = ys[-1]
    else:
        i = max(k for k in range(len(xs) - 1) if xs[k] <= d)
        v = ys[i] + (ys[i + 1] - ys[i]) * (d - xs[i]) / (xs[i + 1] - xs[i])
    return min(math.radians(v), ARM_LIMITS["elbow"][1])


def _bracket(nodes, x):
    """Chỉ số hai nút kề (kẹp ở hai đầu) quanh x (độ)."""
    if x <= nodes[0]:
        return 0, 0
    if x >= nodes[-1]:
        return len(nodes) - 1, len(nodes) - 1
    for i in range(len(nodes) - 1):
        if nodes[i] <= x <= nodes[i + 1]:
            return i, i + 1
    return len(nodes) - 1, len(nodes) - 1


# (DK-N66): giới hạn ghép yaw theo gập cổ tay (độ SDK). Nút gập; yaw SDK nhỏ nhất/lớn nhất = (chạm đầu tiên, chia đôi 0,25 độ,
# cad/faibotic/yaw_couple_scan.py -> cad/export/AM_wrist_yaw_couple.json) trừ biên 3 độ, không vượt ARM_LIMITS. Tra: hẹp nhất hai nút kề (bảo thủ).
# Gập >= 73,4 độ: đáy J56 đã chạm cẳng tay J4 ngay ở yaw 0 (DK-N68, giới hạn gập 87,5 lấy từ CAD cẳng tay cũ) -> nút cuối khoá yaw 0.
# Chạm yaw URDF âm (SDK dương, đáy J56 x cẳng tay J4) theo gập 0/10/20/30/40/50/60/65/70/72,5: -83,75 (giá J5)/-83,75/-65,8/-45,0/-30,9/-22,2/-14,1/-9,5/-4,5/-1,7;
# phía URDF dương (SDK âm): giá J5 +84,7 tới gập 60, rồi vỏ J56 x J4 83,3/81,9/81,3 ở gập 65/70/72,5. Nút 72,5 (biên < 3 độ) và 87,5: khoá yaw 0.
WRIST_YAW_COUPLE_FLEX_DEG = (-77.5, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 65.0, 70.0, 72.5, 87.5)
WRIST_YAW_COUPLE_MIN_DEG = (-81.0, -81.0, -81.0, -81.0, -81.0, -81.0, -81.0, -80.0, -78.5, -78.0, 0.0)
WRIST_YAW_COUPLE_MAX_DEG = (80.0, 80.0, 62.5, 42.0, 27.5, 19.0, 11.0, 6.5, 1.5, 0.0, 0.0)


def wrist_yaw_range(q_wrist_flex: float) -> tuple[float, float]:
    """(nhỏ nhất, lớn nhất) yaw cổ tay (rad SDK) ứng với gập cổ tay (rad): hẹp nhất hai nút bảng kề, trong ARM_LIMITS yaw."""
    i0, i1 = _bracket(WRIST_YAW_COUPLE_FLEX_DEG, math.degrees(q_wrist_flex))
    lo = max(WRIST_YAW_COUPLE_MIN_DEG[i0], WRIST_YAW_COUPLE_MIN_DEG[i1])
    hi = min(WRIST_YAW_COUPLE_MAX_DEG[i0], WRIST_YAW_COUPLE_MAX_DEG[i1])
    L = ARM_LIMITS["wrist_yaw"]
    return max(math.radians(lo), L[0]), min(math.radians(hi), L[1])


def wrist_max(q_shoulder: float, q_elbow: float) -> float:
    """Giới hạn trên của cổ tay (rad) ứng với vai và khuỷu (rad): nhỏ nhất bốn nút bảng quanh tư thế, không vượt ARM_LIMITS cổ tay."""
    ds, de = math.degrees(q_shoulder), math.degrees(q_elbow)
    i0, i1 = _bracket(WRIST_COUPLE_SHOULDER_DEG, ds)
    j0, j1 = _bracket(WRIST_COUPLE_ELBOW_DEG, de)
    m = min(WRIST_COUPLE_MAX_DEG[i][j] for i in (i0, i1) for j in (j0, j1))
    return min(math.radians(m), ARM_LIMITS["wrist_flex"][1])


def lowstop_elbow_clear_deg(q_shoulder_deg: float) -> float:
    """Khuỷu lớn nhất (độ) không giao chặn dưới ở vai q (nội suy tuyến tính bảng quét CAD; vai < 38: không ràng buộc, 140)."""
    t = LOWSTOP_ELBOW_CLEAR_DEG
    if q_shoulder_deg <= t[0][0]:
        return 140.0
    for (a, ea), (b, eb) in zip(t, t[1:]):
        if a <= q_shoulder_deg <= b:
            return ea + (eb - ea) * (q_shoulder_deg - a) / (b - a)
    return t[-1][1]


# ---------------- v0.4: an toàn mất điện vai (cad/bao-cao-cad.md "v0.4", cad/faibotic/failsafe.py) ----------------
# Giới hạn CƠ KHÍ (chặn cứng/đệm), độc lập với giới hạn mềm ARM_LIMITS: khi mất điện chỉ có các chặn này còn tác dụng.
# v0.4.1: mỗi chặn là một ĐỆM PU có đường lực-ép (poweroff.pad_force), chạm ở contact_deg, giới hạn cứng hard_deg (kết cấu/CAD).
# Mọi kích thước, độ cứng, giá trị lực là [giả định / tự tính], chưa có số đo đệm (EVT: đo đường lực-ép, tiêu chí 5).
FAILSAFE_SHOULDER_CONTACT_DEG = -80.5   # link chạm chân đệm PU M1-SH-015 (đệm dời v0.4)
FAILSAFE_SHOULDER_STOP_DEG = -88.0      # hard: gá khuỷu giao cradle từ vai -89 ở khuỷu <= -40 hoặc >= 100 (failsafe_scan.json v0.4); -88 sạch; đệm ép 3,93 mm = 25% ở -88
FAILSAFE_SHOULDER_UP_DEG = 62.0         # hard (v0.4.2-2): chặn dưới M1-SH-017 v3 + đệm M1-SH-018 v3 (dưới đáy má, r 45 mm); v0.4.1: 105 (thanh r 90 ở 97,5, gá khuỷu đập thanh)
FAILSAFE_ELBOW_STOP_DEG = 133.0         # hard (v0.4.3: 136 -> 133, biên 2 độ của quét cũ quá mỏng; 133 sạch tới mọi vai theo failsafe_v042_low_scan.json); đệm M1-AR-022 (nay PU 7 trên đế lót 5) chạm ở 128, ép ~76% ở 133; gá khuỷu chạm link ở 138-140
FAILSAFE_ELBOW_LOW_DEG = -32.0          # hard: chặn khuỷu ngược M1-AR-023 (đệm chạm -27,5 (CAD: giao đệm từ ~-28), ép ~20% ở -33; z đệm >= 15 vì thấp hơn giao gá khuỷu khi gập; dưới -33 gờ servo trượt qua đệm: KHÔNG có chặn)
# Đệm: shore (độ cứng Shore A, E theo Gent), area_mm2 (diện tích tiếp xúc), t_mm (dày tự do), r_m (cánh tay đòn quanh trục khớp),
# contact_deg/hard_deg (độ). Tác động lên mô-men khớp: tau = F(ép) * r. Số liệu theo bao-cao-cad.md v0.4.1.
FAILSAFE_PADS = {
    "shoulder_low": dict(contact_deg=-80.5, hard_deg=-88.0, r_m=0.090, shore=50, area_mm2=48.0, t_mm=20.0),
    "shoulder_up": dict(contact_deg=50.0, hard_deg=62.0, r_m=0.045, shore=50, area_mm2=96.0, t_mm=20.0),   # v0.4.2: 8 x 12 tiếp xúc, r 45 (v0.4.1: 97,5/105, r 90, 48 mm2)
    "elbow_high": dict(contact_deg=128.0, hard_deg=133.0, r_m=0.061, shore=50, area_mm2=32.0, t_mm=7.0),
    "elbow_low": dict(contact_deg=-23.5, hard_deg=-32.0, r_m=0.017, shore=70, area_mm2=156.0, t_mm=10.0),
}
FAILSAFE_T_GEAR_NM = 9.3                # [giả định] 1/2 mô-men chặn cặp servo (18,6): giới hạn sốc bánh răng; chưa có số liệu: dữ liệu hãng
FAILSAFE_LINK_ALU = True                # link M1-SH-004 bằng nhôm 6061-T6 8 mm (PETG: cổ link 7,8 N·m không chịu nổi mô-men chặn)

WRIST_YAW_URDF_LIMIT = (-3.14, 3.14)   # giới hạn trong URDF upstream left_wrist_yaw_joint
# P-L + pan, vai, khuỷu STS3095; gập, yaw, cuộn, kẹp STS3215 12 V (đúng BOM upstream AM-ARM200: 3 STS3095 + 4 STS3215 mỗi tay).
# Trước P-L (P-A, v0.3-v0.4.6): pan STS3215, vai/khuỷu ghép đôi 2 x STS3095.
ARM_SERVO = {"pan": STS3095, "shoulder": STS3095, "elbow": STS3095,
             "wrist_flex": STS3215, "wrist_yaw": STS3215, "wrist_roll": STS3215, "gripper": STS3215}
SIDES = ("left", "right")
ARM_SERVO_JOINTS = ARM_JOINTS + ("gripper",)   # 7 servo mỗi tay
# Cả robot P-L 6+1: 18 servo = 7 STS3095 (2 pan, 2 vai, 2 khuỷu, 1 lift) + 11 STS3215 (2 gập, 2 yaw, 2 cuộn, 2 kẹp, 3 bánh). Dùng cho ngân sách dòng
# (limits.current_budget(models=PL_SERVO_MODELS)); từ S1C trùng với ARM_SERVO.
PL_SERVO_MODELS = {**{f"{s}_{j}": ARM_SERVO[j] for s in ("left", "right") for j in ARM_SERVO_JOINTS},
                   "wheel_0": STS3215, "wheel_1": STS3215, "wheel_2": STS3215, "lift": STS3095}

# ---------------- v0.3: cân bằng trọng lực vai + khuỷu ghép đôi (cad/bao-cao-cad.md v0.3) ----------------
# Mỗi tay: 2 lò xo kéo (một bên má gá vai mỗi lò xo) nối trụ neo trên gá vai (A) với chốt M6 trên link (B).
# Hệ toạ độ mặt phẳng x-z của vai, gốc ở trục vai: A = (BAL_XA, BAL_H), B = (BAL_A cos q, -BAL_A sin q).
# v0.3 vòng 2: lò xo THỰC (thép đàn hồi ASTM A228/EN 10270-1 SM, d 1,4 mm, D trung bình 15,4 mm, 20,8 vòng, đặt theo thông số,
# cad/bao-cao-cad.md v0.3 mục 1b): lực F = F0 + k (L - L0), F0 = 5,1 N nằm trong dải lực căng đầu thực hiện được (Shigley);
# bộ F0 = 24 N, k = 0,23 N/mm của vòng 1 (snippet RS PRO 0751865) không khả thi vật lý, đã bỏ.
# P-L: KHÔNG lò xo (BAL_N_PER_SIDE = 0). Số lò xo dưới đây giữ cho biến thể ghép đôi P-A (model.robot_mjcf(springs=True), test lịch sử).
BAL_N_PER_SIDE = 0
BAL_K = 500.0            # N/m (0,50 N/mm) mỗi lò xo
BAL_F0 = 5.1             # N
BAL_L0 = 0.0585          # m (trong hai móc)
BAL_E = BAL_L0 - BAL_F0 / BAL_K        # chiều dài tự do hiệu dụng cho tendon springlength (0,0483 m) [tự suy ra]
BAL_A, BAL_H, BAL_XA = 0.035, 0.105, -0.025   # m: chốt B cách trục vai 35 mm trên link; neo A cao 105 mm, lùi 25 mm
BAL_Y = 0.062            # m: mặt phẳng lò xo cách mặt phẳng đối xứng (ngoài má gá vai 43,9 mm)
BAL_MAX_F = 47.3         # N lực lớn nhất mỗi lò xo trong dải vai (balance_strength.py)
# Số servo mỗi khớp. P-L: mỗi khớp một servo. P-A (lịch sử): vai, khuỷu 2 servo đồng trục chia tải.
ARM_SERVO_COUNT = {"pan": 1, "shoulder": 1, "elbow": 1, "wrist_flex": 1, "wrist_yaw": 1, "wrist_roll": 1, "gripper": 1}
PAYLOAD_TARGET = 0.5        # kg/tay: tải dùng cho bảng mô-men mặc định; tải công bố xem docs/M1-LITE/tai-cong-bo-PL.md

# Tư thế gốc (H): cánh tay dựng đứng, cẳng tay nằm ngang, kẹp chĩa xuống
# (TCP cách gốc tay ~0,16 m về trước, thấp hơn gốc tay ~0,03 m; trước 0,15 m trước, cao hơn 0,075 m; sẵn sàng cho teleop Descartes)
# v0.4.4: giới hạn gập 87,5 và ô bảng ghép vai -80 là 87, nên khuỷu 90 -> 93, cổ tay 90 -> 87 (tổng góc dụng cụ giữ 100 độ, kẹp chĩa xuống như cũ)
HOME_ARM = {"pan": 0.0, "shoulder": math.radians(-80), "elbow": math.radians(93),
            "wrist_flex": math.radians(87), "wrist_yaw": 0.0, "wrist_roll": 0.0}


def joint_name(side: str, j: str) -> str:
    return f"{side}_{j}"


def all_joint_names() -> list[str]:
    names = []
    for s in SIDES:
        names += [joint_name(s, j) for j in ARM_JOINTS]
        names.append(joint_name(s, "gripper"))
    names.append("lift")
    return names


def limits(name: str) -> tuple[float, float]:
    if name == "lift":
        return LIFT_RANGE
    side, _, j = name.partition("_")
    if j == "gripper":
        return (0.0, 100.0)
    return ARM_LIMITS[j]


# ---------------- Bản đồ servo Feetech (ID trên bus) ----------------
# P-L 6+1: 18 servo, mỗi khớp một servo. ID 1-16 giữ nguyên (BUS_B 13-16 không đổi, tự nhận cấu hình không đổi);
# servo yaw mới lấy 21 (trái, BUS_L), 22 (phải, BUS_R): tránh 17-20 (PA_PAIR_IDS lịch sử). Upstream đánh 1-7 theo từng tay trên bus riêng.
# P-L 5+1 cũ: 16 servo. ID 17-20 (servo B vai/khuỷu ghép đôi của P-A) bỏ; giữ đường mã ghép đôi có tham số
# (FeetechBackend(paired_joints=PA_PAIRED_JOINTS, ids={**SERVO_IDS, **PA_PAIR_IDS})) cho test lịch sử, mặc định P-L không ghép.
SERVO_IDS = {
    "left_pan": 1, "left_shoulder": 2, "left_elbow": 3, "left_wrist_flex": 4,
    "left_wrist_roll": 5, "left_gripper": 6,
    "right_pan": 7, "right_shoulder": 8, "right_elbow": 9, "right_wrist_flex": 10,
    "right_wrist_roll": 11, "right_gripper": 12,
    "wheel_0": 13, "wheel_1": 14, "wheel_2": 15,
    "lift": 16,
    "left_wrist_yaw": 21, "right_wrist_yaw": 22,
}
# S2: ba bus. BUS_L, BUS_R nằm trên trạm hai tay (đi theo trạm khi tháo); BUS_B trên tấm điện đế: 3 bánh + servo lift
# (lift chuyển từ BUS_L cổng S2 sang BUS_B vì BUS_L nay ở trên trạm). ID giữ nguyên.
SERVO_BUS = {**{f"left_{j}": "BUS_L" for j in ARM_SERVO_JOINTS},
             **{f"right_{j}": "BUS_R" for j in ARM_SERVO_JOINTS},
             "wheel_0": "BUS_B", "wheel_1": "BUS_B", "wheel_2": "BUS_B", "lift": "BUS_B"}
BUS_B_IDS = tuple(SERVO_IDS[n] for n, b in SERVO_BUS.items() if b == "BUS_B")    # 13, 14, 15, 16
# Hai cấu hình robot (R3c mục 8). Tự nhận: thấy đủ BUS_B -> di_dong; không thấy servo nào của BUS_B -> tram_ban; thấy một phần -> lỗi.
ROBOT_CONFIGS = {
    "tram_ban": dict(buses=("BUS_L", "BUS_R"), servos=tuple(n for n, b in SERVO_BUS.items() if b != "BUS_B"), base=False, lift=False,
                     current_budget_a=12.0,      # adapter 12 V 15 A, 80% (R3b mục 6) [giả định tới khi đo]
                     keepout=("dau_camera", "be_tram", "mat_ban")),
    "di_dong": dict(buses=("BUS_L", "BUS_R", "BUS_B"), servos=tuple(SERVO_BUS), base=True, lift=True,
                    current_budget_a=80.0,       # = CURRENT_BUDGET_A (F1 MEGA 80 A)
                    keepout=("dau_camera", "be_tram"), dock_required=True),
}
DOCK_PINS = ("DOCK_L", "DOCK_R")   # GPIO Pi (qua opto như ESTOP_OK): True = khoá kéo trái/phải đã đóng (công tắc KW12, nối tiếp vòng E-stop)


def detect_config(ids_present) -> str:
    """Cấu hình theo servo BUS_B trả lời ping. ids_present: tập ID đã thấy."""
    seen = [i for i in BUS_B_IDS if i in set(ids_present)]
    if len(seen) == len(BUS_B_IDS):
        return "di_dong"
    if not seen:
        return "tram_ban"
    raise ValueError(f"BUS_B thiếu servo: thấy {seen}, cần {list(BUS_B_IDS)} (hoặc không có servo nào ở chế độ bàn)")


def dataset_meta(config: str, shoulder_y: float | None = None, head_tilt_deg: float | None = None, head_pan_deg: float | None = None) -> dict:
    """Metadata ghi kèm LeRobotDataset: chế độ, khoảng vai, tư thế camera đầu (giá trị đặt tay trên gá, mặc định theo spec)."""
    if config not in ROBOT_CONFIGS:
        raise ValueError(f"cấu hình không biết: {config!r}")
    return {"robot_type": "faibot_m1_lite", "che_do": config,
            "khoang_vai_m": round(2 * (SHOULDER_Y if shoulder_y is None else shoulder_y), 4),
            "camera_dau": {"chui_deg": HEAD_CAM_TILT_DEG if head_tilt_deg is None else head_tilt_deg,
                           "pan_deg": HEAD_CAM_PAN_DEG if head_pan_deg is None else head_pan_deg,
                           "cao_tren_ban_m": round(H_J1_ABOVE_TABLE + HEAD_CAM_AXIS[2], 4)}}


# Thứ tự vị trí đọc/ghi trên bus (backend feetech): 14 khớp tay + lift; bánh chạy chế độ vận tốc.
POS_NAMES = [f"{s}_{j}" for s in SIDES for j in ARM_JOINTS] + [f"{s}_gripper" for s in SIDES] + ["lift"]
PAIRED_JOINTS: tuple[str, ...] = ()     # P-L: không khớp nào ghép đôi
PA_PAIRED_JOINTS = ("shoulder", "elbow")
PA_PAIR_IDS = {"left_shoulder_b": 17, "right_shoulder_b": 18, "left_elbow_b": 19, "right_elbow_b": 20}
PAIR_SUFFIX = "_b"


def pair_name(joint: str) -> str:
    """Tên servo B của khớp ghép đôi, ví dụ left_shoulder -> left_shoulder_b."""
    return joint + PAIR_SUFFIX


# Giám sát cặp ghép đôi (backend feetech). [giả định] trừ chênh tải 20% (Q-FQC-C4, product/phieu-kiem/IPQC.md).
PAIR_MAX_POS_DIFF_DEG = 3.0     # chênh góc khớp đọc từ hai servo (đã quy về khung khớp); IPQC mục 3 chưa có ngưỡng
PAIR_MAX_LOAD_DIFF = 0.20       # |La - Lb| / trung bình |L|
PAIR_LOAD_FLOOR = 0.10          # dưới 10% mô-men chặn thì so chênh tải theo mức sàn này (tránh báo nhầm khi gần không tải)
PAIR_FAULT_CYCLES = 3           # số chu kỳ liên tiếp vượt ngưỡng mới dừng (50 Hz: 60 ms)

# Giới hạn mô-men nạp vào servo vai (product/06-lap-rap.md bước 9.2: vai <= 60 kg·cm tổng cả cặp).
SHOULDER_TORQUE_LIMIT_KGCM = 60.0

# v0.5 (R2 7.4, DK-N19): TORQUE_LIMIT cho MỌI servo theo nhóm, % mô-men chặn [đề xuất; giả định dòng tỉ lệ mô-men, chưa đo].
# Vai lấy nhỏ hơn của % nhóm và SHOULDER_TORQUE_LIMIT_KGCM chia đều số servo của khớp; lift lấy LIFT_TORQUE_LIMIT_PERMILLE.
# Gập cổ tay 60%: tải 1 kg kẹp ngang cần 74% chặn (R2 3.1) nên bị chặn ở mức này: chủ ý, giữ dòng.
# Sim (model.arm_kp): forcerange actuator = mức này; kp đạt forcerange khi lệch ARM_KP_SAT_DEG.
# yaw 80%: trục yaw cách tâm kẹp ~155 mm, lực ngang ở kẹp đè thẳng lên servo yaw; demo lab mở nắp hộp (sim) cần >= 2,35 N.m (50% = 1,47: yaw lệch 16,6 độ,
# nắp chỉ mở 0,79 rad; 60%: 1,09 rad; 80%: 1,64 rad như 5+1). Ngân sách dòng +2 x 2,7 x 0,3 A.
TORQUE_LIMIT_PCT = {"shoulder": 60, "elbow": 60, "pan": 50, "wrist_yaw": 80, "wrist_roll": 50, "gripper": 50, "wrist_flex": 60, "wheel": 50}
ARM_MAX_STEP_DEG = 1.5      # bước đích khớp tay lớn nhất mỗi chu kỳ 50 Hz (75 độ/s) của Robot.move_*; servo STS3215 không tải ~270 độ/s [trang bán]
ARM_KP_SAT_DEG = 1.5        # [giả định] servo STS hộp số 1:345 giữ cứng; chưa đo độ võng dưới tải
CURRENT_BUDGET_A = 80.0     # tổng dòng cho phép (servo ở giới hạn mô-men + Pi) <= cầu chì chính F1 MEGA 80 A (BMS 100 A cắt sau) [R2 7.4]
PI_CURRENT_A = 2.6          # nhánh Pi phía pin: 5,09 V x 5 A / 0,9 / 10,8 V (electrical/tinh-toan.md)
CTRL_CURRENT_A = 1.2        # mạch điều khiển: cuộn contactor K1 kiểu SW80 ~1 A + rơ-le tự giữ và K5 ~0,2 A [giả định: datasheet K1 chưa có số liệu, DK-N1]
LIFT_LATCH_COIL_A = 1.0     # cuộn chốt solenoid 12 V lấy từ SERVO+ [giả định: chưa chọn mã, DK-N17]

# Mạch dừng khẩn cấp mà safety.SafetyLatch đọc (cấu hình mặc định; truyền hw= hoặc GPIO có thuộc tính hw để đổi).
# "bang_di_day": bộ #0 EVT, electrical/bang-di-day-evt.md (K5 cho phép, K2 tự giữ, KT 0,5 s, ESTOP_OK, ARM_SENSE).
# "pcb": PCB nguồn sau EVT, electrical/quyet-dinh-bms-estop.md 2.6 (RELAY_EN cạnh lên cho U4, ESTOP_SENSE, LATCH_SENSE).
SAFETY_HW = "bang_di_day"
EVT_KT_S = 0.5             # rơ-le trễ đóng KT (H3Y-2, dải chưa kiểm)
EVT_ARM_CONFIRM_S = 1.0    # từ mốc K1 phải đóng (sau KT và sau khi nhả Reset), ARM_SENSE phải cao liên tục chừng này mới cho chạy
EVT_SERVO_DISCHARGE_S = None   # thời gian ARM_SENSE rớt sau K4 nhả khi K1 không đóng, có servo, mô-men tắt: CHƯA ĐO (estop_jig.py --che-do xa-tu). Cần <= EVT_ARM_CONFIRM_S / 2
EVT_STUCK_S = 0.2          # RUN còn cao (hoặc SERVO+ còn điện, cộng thời gian xả tụ) quá chừng này sau khi K5 hạ / K2 nhả -> báo dính (bang-di-day-evt.md mục 2 bước 5)

# ---------- Tầng 1b: giám sát khi chạy (24/11/2026) ----------
# Điện áp servo (PRESENT_VOLTAGE, 0,1 V): đọc mọi servo mỗi chu kỳ trong cùng gói sync_read của step() (không thêm giao dịch), lấy
# trung vị các servo, lọc V_LOC_CHU_KY chu kỳ liền (tránh dừng nhầm do sụt áp tức thời khi dòng đỉnh). Pack 3S Li-ion (D3/D7):
# 12,6 V đầy, ~9,0 V cạn. Dưới V_CANH: cảnh báo một lần (log WARNING); dưới V_CAT: TTAT + DienApThap, enable() từ chối.
# [ước] chờ kỹ sư điện chốt theo BMS và đường cong xả cell 40T.
V_CANH = 10.0
V_CAT = 9.0
V_LOC_CHU_KY = 5
# Watchdog trong luồng chốt E-stop: đế đang có lệnh bánh khác 0 mà không có step() quá WATCHDOG_S giây -> ghi GOAL_SPEED bánh 0,
# tắt mô-men mọi servo, lệnh kế ném WatchdogHet. 200 ms = 10 chu kỳ 50 Hz.
WATCHDOG_S = 0.2
# Trần thời gian một lần đọc bus (sync_read cả robot hoặc read một servo), thay cho timeout x (2 + n) = 0,34 s.
# 18 servo x gói 8-12 byte ở 1 Mbps + trễ trả lời ~0,5 ms/servo ~ 10 ms [ước]; 30 ms chừa biên.
BUS_READ_DEADLINE_S = 0.03
# Số chu kỳ liền một servo im lặng hoặc trả gói hỏng thì về TTAT (ServoMat). Lỗi lẻ (1-2 chu kỳ) bỏ qua, không dừng nhầm.
SERVO_MISS_CYCLES = 3
# Trần thời lượng một lệnh chờ xong (move_joints/move_tcp/move_base/wait) và bước dt lớn nhất của step() (theo chu kỳ điều khiển).
# QC2 M1: dt lớn nhất (5 x 20 ms = 0,1 s) + một lần đọc bus phải nhỏ hơn hẳn WATCHDOG_S, nếu không step(dt_max) tự làm watchdog dừng nhầm.
MAX_MOVE_DURATION_S = 120.0
MAX_STEP_DT_CYCLES = 5
# Đường dừng khẩn lấy khoá bus có hạn (QC2 M2): hết hạn thì bỏ ghi TORQUE_ENABLE=0 qua bus, chốt phần cứng đã cắt nguồn servo.
ESTOP_BUS_LOCK_S = 0.1
# MODEL_NUMBER (địa chỉ 3-4) đã biết của dòng STS; đọc lúc kết nối, số lạ -> cảnh báo. STS3215 = 777 (bảng model của LeRobot);
# STS3095 [CẦN-XÁC-MINH] chưa tra được số, nên servo STS3095 tạm nhận mọi số đã biết.
SERVO_MODEL_NUMBERS = {"STS3215": 777}
