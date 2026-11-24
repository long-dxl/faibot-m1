# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""Trạm hai tay tháo lắp (G1b): khung chữ U nhôm 2040, gá đáy tay, cột camera đầu + gá pan/tilt, nửa trạm của giao diện
lắp nhanh (bạc chốt, khoá kéo, công tắc khoá, giắc D-sub 21W4 đực), E-stop trạm, hộp điện trạm. Đế ngàm (giống nhau trên xe nâng và trên đế
bàn) và đế bàn (BỘ TUỲ CHỌN, không thuộc robot chuẩn) ở tram_de.py. build123d.

Khung trạm S (mm): gốc ở giữa hai trục pan, ngang đáy J1; x trước, y trái, z lên. Ở chế độ di động S = W dời (SHOULDER_X, 0, gốc tay ở lift).
Kích thước là [giả định] thiết kế (R3b), chờ in/lắp thử ở EVT. Bố trí theo vật cản thật của CAD P-L (probe 24/11):
- khung T giữa M1-LF-057 (giữ làm xe nâng): x -54..9, |y| <= 70,8, z -150..-6,5;
- chốt hãm + giá M1-LF-050/041/060/061 trên mặt khung T giữa: x -32..52, y -27..19, z -6..19  -> ray trước nằm dưới giá chốt (z <= -7),
  cột camera đứng trên một "cầu" in bắc qua cụm chốt;
- đế J1 AM-ARM200 thật: x -76,8..49,8, rộng 138,6 mm theo y (R3b giả định 120) -> trạm rộng 0,475 + 0,139 = 0,614 m (ray 0,60 m, J1 thò 7 mm mỗi đầu).
Chạy: python tram_hai_tay.py   (in hộp bao, khối lượng ước từng chi tiết)"""
import math
import numpy as np
from build123d import Box, Cylinder, Pos, Rot, Align, Location, Plane, Compound

MIN = (Align.MIN,) * 3
SHOULDER_HALF = 237.5          # khoảng vai 0,475 m
RAIL_Y = 300.0                 # ray trước 2040 dài 0,60 m (rãnh T cho dời gá tay ±25 mm)
RAIL_X, RAIL_Z = (12.0, 52.0), (-27.0, -7.0)      # 2040 nằm (40 theo x, 20 theo z), trước khung T giữa 3 mm, dưới giá chốt 1 mm
EAR_X, EAR_Y, EAR_Z = (-188.0, 12.0), (100.0, 120.0), (-47.0, -7.0)   # 2 tai 2040 dài 0,20 m, đứng (40 theo z), ôm hai bên cột (|y| cột <= 51)
PIN_X, PIN_Y = -15.0, 110.0    # tâm chốt côn Ø8 (trái: lỗ tròn, phải: lỗ dẹt dọc y)
BUSH_Z = (-67.0, -47.0)        # khối bạc dưới tai; mặt đáy = mặt tựa trên đế ngàm (z -67)
DOCK_TOP = BUSH_Z[0]
H_J1_TREN_BAN = 120.0          # chế độ bàn: đáy J1 cao hơn mặt bàn (đế ngàm 30 + khung 2040 20 + chân 3 + phần trạm dưới J1 67) [tự tính từ CAD]
CAM_TREN_BAN = 650.0           # / yêu cầu của dự án: camera đầu 0,65 m trên mặt bàn (trục tilt)
CAM_Z = CAM_TREN_BAN - H_J1_TREN_BAN   # 530: trục tilt trên đáy J1 (R3b giả định 600 với J1 cao 50 trên bàn; vị trí so với bàn giữ nguyên)
CAM_TILT = 50.0                # chúi mặc định 50° (chỉnh tay 20..70°, chốt định vị 35° và 50°)
CAM_PAN = 0.0                  # gá pan ±30°, mặc định 0
MAST_XY = (0.0, 0.0)           # cột camera 2020 trên đường trục pan
BRIDGE_TOP = 29.0              # mặt trên cầu in bắc qua cụm chốt hãm
MAST_TOP = CAM_Z - 40.0      # đĩa pan + đế má dưới mô-đun camera đã chúi (khe >= 3 mm ở 50°)
TILT_X = 2.0                   # trục tilt lệch ra trước tâm cột
CAM_MOD = (36.0, 36.0, 8.0)    # mô-đun H65V1 (như assembly_full.CAM), ống kính Ø15 x 10
CAM_LENS = (15.0, 10.0)


def box(x, y, z):
    return Pos(x[0], y[0], z[0]) * Box(x[1] - x[0], y[1] - y[0], z[1] - z[0], align=MIN)


def cyl_z(cx, cy, z, r):
    return Pos(cx, cy, z[0]) * Cylinder(r, z[1] - z[0], align=(Align.CENTER, Align.CENTER, Align.MIN))


def mirror_y(y, s):
    return (y[0], y[1]) if s > 0 else (-y[1], -y[0])


def cam_look(tilt=CAM_TILT, pan=CAM_PAN):
    t, p = math.radians(tilt), math.radians(pan)
    return np.array([math.cos(t) * math.cos(p), math.cos(t) * math.sin(p), -math.sin(t)])


def cam_axis_point():
    """Điểm trên trục tilt (giữa hai má), khung trạm, mm."""
    return np.array([MAST_XY[0] + TILT_X, MAST_XY[1], CAM_Z])


def cam_lens_point(tilt=CAM_TILT, pan=CAM_PAN):
    """Đầu ống kính (dùng làm gốc camera sim), khung trạm, mm: mặt sau mô-đun ở trục tilt, + thân 8 + ống kính 10 dọc trục nhìn."""
    return cam_axis_point() + cam_look(tilt, pan) * (CAM_MOD[2] + CAM_LENS[1])


def camera_module():
    return Pos(0, 0, CAM_MOD[2] / 2) * Box(*CAM_MOD) + Pos(0, 0, CAM_MOD[2] + CAM_LENS[1] / 2) * Cylinder(CAM_LENS[0] / 2, CAM_LENS[1])


def station_parts(tilt=CAM_TILT):
    """(tên, vật liệu, hình) của trạm trong khung trạm. Không gồm hai tay (assembly_pl.arm)."""
    out = [("M1-TR-001_ray_truoc_2040", "alu", box(RAIL_X, (-RAIL_Y, RAIL_Y), RAIL_Z))]
    for s, sd in ((1, "L"), (-1, "R")):
        yc = s * SHOULDER_HALF
        out.append((f"M1-TR-002_tai_2040_{sd}", "alu", box(EAR_X, mirror_y(EAR_Y, s), EAR_Z)))
        # gá đáy tay in: tấm 7 mm dưới đế J1 (x -80..55, rộng 144) bắt ray bằng con trượt T; 2 gân cạnh + gân sau đỡ phần hẫng sau ray
        ga = box((-80.0, 55.0), (yc - 72.0, yc + 72.0), (-7.0, 0.0))
        for g in ((yc - 72.0, yc - 62.0), (yc + 62.0, yc + 72.0)):
            ga += box((-80.0, RAIL_X[0]), g, (RAIL_Z[0], -7.0))
        ga += box((-80.0, -70.0), (yc - 72.0, yc + 72.0), (RAIL_Z[0], -7.0))
        out.append((f"M1-TR-003_ga_day_tay_{sd}", "navy", ga))
        # khối bạc dưới tai: lỗ Ø12 chứa bạc thép (trái lỗ tròn H7, phải lỗ dẹt)
        kb = box((-35.0, 5.0), mirror_y(EAR_Y, s), BUSH_Z) - cyl_z(PIN_X, s * PIN_Y, (BUSH_Z[0] - 1, BUSH_Z[1] + 1), 6.0)
        out.append((f"M1-TR-004_khoi_bac_{sd}", "navy", kb))
        bac = cyl_z(PIN_X, s * PIN_Y, BUSH_Z, 6.0) - cyl_z(PIN_X, s * PIN_Y, (BUSH_Z[0] - 1, BUSH_Z[1] + 1), 4.0)
        out.append((f"M1-TR-004b_bac_thep_{sd}", "steel", bac))
        # khoá kéo có chốt an toàn (thân trên mặt ngoài tai, móc xuống móc giữ trên đế ngàm) + công tắc khoá KW12 vào vòng E-stop
        kk = box((-28.0, 2.0), mirror_y((120.0, 136.0), s), (-45.0, -20.0)) + box((-14.0, -6.0), mirror_y((124.0, 130.0), s), (DOCK_TOP, -45.0))
        out.append((f"M1-TR-008_khoa_keo_{sd}", "steel", kk))
        out.append((f"M1-TR-009_cong_tac_khoa_{sd}", "servo", box((-50.0, -30.0), mirror_y((120.0, 130.0), s), (-32.0, -22.0))))   # sau thân khoá, lẫy nhấc là hở
        out.append((f"M1-TR-014_tay_cam_{sd}", "navy", box((-185.0, -150.0), mirror_y(EAR_Y, s), (-7.0, 25.0))
                    - box((-178.0, -157.0), mirror_y((EAR_Y[0] - 1, EAR_Y[1] + 1), s), (-7.0, 18.0))))
    # giắc D-sub 21W4 nửa đực (cố định, quay xuống) trên giá in dưới tai trái + thanh đỡ dưới ray
    out.append(("M1-TR-010_giac_21W4_duc", "steel", box((7.0, 60.0), (103.0, 117.0), (DOCK_TOP, -55.0))))
    out.append(("M1-TR-011_gia_giac", "navy", box((6.0, 62.0), EAR_Y, (-55.0, -47.0)) + box((RAIL_X[0], RAIL_X[1]), EAR_Y, (-47.0, RAIL_Z[0]))))
    # cầu in bắc qua cụm chốt hãm (2 chân trên ray, tấm trên z 21..29), cột 2020, gá pan + tilt, camera đầu
    br = box((-14.0, RAIL_X[1]), (-52.0, 52.0), (21.0, BRIDGE_TOP))
    for y in ((32.0, 52.0), (-52.0, -32.0)):
        br += box(RAIL_X, y, (RAIL_Z[1], 21.0))
    out.append(("M1-TR-005_cau_cot_camera", "navy", br))
    mx, my = MAST_XY
    out.append(("M1-TR-006_cot_camera_2020", "alu", box((mx - 10, mx + 10), (my - 10, my + 10), (BRIDGE_TOP, MAST_TOP))))
    A = cam_axis_point()
    head = cyl_z(mx, my, (MAST_TOP, MAST_TOP + 12.0), 16.0)                                     # đĩa pan + núm khoá
    head += box((mx - 8, mx + 12), (my - 24, my + 24), (MAST_TOP + 12, MAST_TOP + 18))
    for y in ((20.0, 24.0), (-24.0, -20.0)):
        head += box((mx - 8, mx + 12), (my + y[0], my + y[1]), (MAST_TOP + 18, A[2] + 12))       # hai má tilt
    out.append(("M1-TR-007_ga_pan_tilt", "navy", head))
    for s in (1, -1):
        out.append((f"M1-TR-007b_num_khoa_M5_{'L' if s > 0 else 'R'}", "servo",
                    Pos(A[0], my + s * 27.0, A[2]) * Rot(X=90) * Cylinder(7.0, 6.0)))
    look = cam_look(tilt); xd = np.array([0.0, -1.0, 0.0])
    L = Location(Plane(origin=tuple(A), x_dir=tuple(xd), z_dir=tuple(look)))
    out.append(("M1-CM-001_head_cam", "servo", L * camera_module()))
    # E-stop trạm (hộp + nấm) ở mặt ngoài tai trái; hộp điện trạm (cầu chì 2 nhánh, 2 bo bus, cách ly, hub USB, buck 5 V)
    # S2 quét mesh 24/11: hộp E-stop trước ray (x 52..112) bị kẹp đập khi tay gập xuống giữa -> dời ra mặt ngoài tai trái, sau J1, nấm quay sang trái
    out.append(("M1-TR-012_estop_hop", "navy", box((-150.0, -100.0), (EAR_Y[1] + 2.0, EAR_Y[1] + 45.0), (EAR_Z[0], RAIL_Z[1]))))
    out.append(("M1-TR-012b_estop_nam", "servo", Pos(-125.0, EAR_Y[1] + 45.0, -27.0) * Rot(X=-90) * Pos(0, 0, 7.5) * Cylinder(20.0, 15.0)))
    out.append(("M1-TR-013_hop_dien_tram", "navy", box((-150.0, -55.0), (-162.0, -EAR_Y[1] - 2.0), EAR_Z)))   # ngoài tai phải, cách gân gá tay 3,5 mm, sau công tắc khoá
    return out


# khối lượng cố định (kg) theo tiền tố tên [giả định trừ khi ghi]; phần còn lại tính theo thể tích (tram_mass)
KG_FIXED = {"M1-TR-008": 0.05, "M1-TR-009": 0.005, "M1-TR-010": 0.04, "M1-TR-012b": 0.06, "M1-TR-012_": 0.06, "M1-TR-013": 0.30,
            "M1-TR-007b": 0.005, "M1-CM-001_head": 0.03}
# hộp điện trạm 0,30 (R3b: cầu chì, bus, hub, buck); E-stop LA38 + hộp 0,12; khoá kéo 100 kg ~50 g; nửa giắc 21W4 có 4 tiếp điểm 40 A ~40 g
KG_PER_MM = {"M1-TR-001": 0.6e-3, "M1-TR-002": 0.6e-3, "M1-TR-006": 0.3e-3}    # 2040 ~0,6 kg/m, 2020 ~0,3 kg/m [giả định, điển hình]
# không vẽ: ke góc + con trượt T + ốc (0,10 kg ở hai góc ray-tai), dây trên trạm (0,25 kg) [giả định, R3b STATION_PARTS]
EXTRA = [("ke_goc_oc", 0.10, (0.0, 0.0, -20.0)), ("day_tram", 0.25, (-60.0, 0.0, -30.0))]
RHO_PETG, RHO_STEEL, INFILL = 1.27e-6, 7.85e-6, 0.5      # kg/mm3; in PETG infill hiệu dụng 0,5 như M1-BS-036 [giả định]


def part_kg(n, s):
    for k, v in KG_FIXED.items():
        if n.startswith(k): return v
    for k, v in KG_PER_MM.items():
        if n.startswith(k):
            b = s.bounding_box(); return v * max(b.max.X - b.min.X, b.max.Y - b.min.Y, b.max.Z - b.min.Z)
    if n.startswith(("M1-TR-004b", "M1-TR-021", "M1-TR-022")): return s.volume * RHO_STEEL
    if n.startswith("M1-TR-023"): return 0.04
    return s.volume * RHO_PETG * INFILL


def tram_mass(parts):
    """(kg, tâm mm khung trạm, bảng từng chi tiết) gồm EXTRA."""
    m, c, tab = 0.0, np.zeros(3), {}
    for n, _, s in parts:
        k = part_kg(n, s); p = s.center(); p = np.array([p.X, p.Y, p.Z])
        tab[n] = (round(k, 4), [round(v, 1) for v in p]); m += k; c += k * p
    for n, k, p in EXTRA:
        tab[n] = (k, list(p)); m += k; c += k * np.array(p)
    return m, c / m, tab


if __name__ == "__main__":
    P = station_parts()
    m, c, tab = tram_mass(P)
    for n, (k, p) in tab.items(): print(f"{n:34s} {k:7.3f} kg  {p}")
    b = Compound([s for _, _, s in P]).bounding_box()
    print("trạm chưa tay %.3f kg, tâm %s mm; hộp bao x %.0f..%.0f y %.0f..%.0f z %.0f..%.0f" % (m, np.round(c, 1), b.min.X, b.max.X, b.min.Y, b.max.Y, b.min.Z, b.max.Z))
    print("đầu ống kính camera (mm):", np.round(cam_lens_point(), 1), "trục tilt:", cam_axis_point())
