# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Cột nâng M1: cột 4040 + 2 ray MGN12H + xe nâng (Al) + vitme T8x2 treo + tăng tốc 16:1 (bánh răng in 4:1 + đai GT2 4:1).
build123d. Hệ toạ độ như sdk (m -> mm): x trước, y trái, z lên, gốc sàn, tâm đế x=0.
Chạy: python lift.py -> ../export/M1-LF-*.step + M1-LF-build.json (khối lượng, giao cắt).
Không dẫn xuất từ hình học upstream (AlohaMini2 dùng thanh răng + bánh răng in; Faibotic chọn vitme, vẽ mới). Số có cờ trong ../../archive/v0.4/cad/bao-cao-cot-nang.mdmd."""
import json, math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); EXP = os.path.join(HERE, "../export")
sys.path.insert(0, os.path.join(HERE, "../../sdk"))
from build123d import Box, Cylinder, Pos, Rot, Align, Compound, export_step, import_step
from faibotic import spec
MIN = (Align.MIN,) * 3; C = (Align.CENTER,) * 3
RHO = {"Al": 2.70e-3, "PETG": 1.27e-3, "steel": 7.85e-3, "POM": 1.41e-3}   # g/mm3 [điển hình]

CX = spec.COLUMN_X * 1000            # -100
Z_BASE = spec.BASE_Z1 * 1000         # 170
Z_TOP = spec.COLUMN_TOP * 1000       # 1200
Z_AX0 = spec.SHOULDER_Z0 * 1000      # 550 (trục vai, lift 0)
STROKE = (spec.LIFT_RANGE[1] - spec.LIFT_RANGE[0]) * 1000
SX, SY = spec.SHOULDER_X * 1000, spec.SHOULDER_Y * 1000
COL = 40.0                           # nhôm định hình 40x40
SCX = CX + 40                        # trục vitme x = -60
D0 = spec.D0 * 1000
# --- ray MGN12H [kích thước catalog điển hình, chưa kiểm datasheet]: rộng 12, cao 8; khối 27 x 45,4, cao tổng 13
RAIL_W, RAIL_H, BLK_L, BLK_W, BLK_H = 12.0, 8.0, 45.4, 27.0, 13.0
S_BLK = 100.0                        # khoảng cách hai khối cùng ray (z)
PAD_DX = 0.0                         # R0 (23/11): tâm tấm giao diện đặt đúng trục pan (cũ -15). Mặt tấm hạ PAN_DROP nên link vai ở 50 độ (đệm chặn) chỉ xuống tới z -75 tại x +43 > mép +30
Z_BEAM = -(D0 + 8 + 20)              # tâm dầm so với trục vai: -73
PAN_DROP = 40.0                      # R0 v2 (23/11): 30 -> 40 vì đệm khuỷu M1-AR-022 (đáy z -84,2 ở vai 62°) chạm tay đỡ ở vai >= 50° (M1-LF-R0-scan.json). Mặt trên tay đỡ M1-LF-023 + tấm giao diện hạ 40 mm dưới mặt chuẩn pan của SDK (trục vai - D0) để giá chặn dưới
                                     # M1-SH-017 (đáy z -69,8 so với trục vai) quay pan không quệt; đế xoay M1-LF-024 cao 45 bù lại. Trục pan vẫn thẳng đứng qua trục vai: động học SDK không đổi
Z_RAIL0, Z_RAIL1 = 375.0, 1075.0     # ray MGN12 dài 700 (chuẩn 700) sau QC2 vòng 2: khối còn >= 14 mm trong ray ở hai đầu khi chạm đệm
RAIL_L = Z_RAIL1 - Z_RAIL0

def alu(b, name): b.label = name; return b

def column():
    # 4040 giản lược: ống đặc 40x40 có lỗ giữa Ø8,3 (khối lượng ~1,4 kg/m dạng đặc -> khoét để ~1,0 kg/m): dùng hộp rỗng thành 6 [giản lược]
    h = Z_TOP - Z_BASE
    p = Pos(CX, 0, Z_BASE + h / 2) * Box(COL, COL, h)
    p -= Pos(CX, 0, Z_BASE + h / 2) * Box(COL - 12, COL - 12, h + 1)
    return p

def flange():
    """Tấm đế cột Al 8 mm 120x120, 4 lỗ M6 xuống đế (PCD 100 trong đế: GIAO DIỆN với M1-BS-001/khung đế, cần chốt) + chân cột."""
    p = Pos(CX, 0, Z_BASE + 4) * Box(120, 120, 8)
    for sx in (-50, 50):
        for sy in (-50, 50):
            p -= Pos(CX + sx, sy, Z_BASE + 4) * Cylinder(3.3, 10)
    return p

def rails():
    out = []
    for s in (-1, 1):   # ray gắn mặt bên cột (y = ±20), trục ray dọc z
        out.append(Pos(CX, s * (COL / 2 + RAIL_H / 2), (Z_RAIL0 + Z_RAIL1) / 2) * Box(RAIL_W, RAIL_H, RAIL_L))
    return out

def blocks(z_ax):
    out = []
    for s in (-1, 1):
        for dz in (-S_BLK / 2, S_BLK / 2):
            zc = z_ax + Z_BEAM + dz
            out.append(Pos(CX, s * (COL / 2 + BLK_H - 0.5 * (BLK_H - RAIL_H) + 0), zc) * Box(BLK_W, BLK_H - RAIL_H + RAIL_H, BLK_L))
    return out

def carriage(z_ax):
    zb = z_ax + Z_BEAM
    parts = {}
    # tấm bên Al 5 mm, y = ±(33..38), x -128..-44, z = zb ± 75
    for s in (-1, 1):
        parts[f"side{'L' if s > 0 else 'R'}"] = Pos((-128 + -44) / 2, s * (COL / 2 + BLK_H + 2.5), zb) * Box(84, 5, 150)
    # tấm cầu Al 6 mm ở x -50..-44, nối hai tấm bên (đỡ đai ốc vitme + dầm)
    parts["bridge"] = Pos(-47, 0, zb) * Box(6, 2 * (COL / 2 + BLK_H + 5), 150)
    # dầm ngang ống Al 40(z) x 25(x) x 2,5, x -44..-19, y ±200 (liền tấm cầu)
    beam = Pos(-31.5, 0, zb) * Box(25, 400, 40)
    beam -= Pos(-31.5, 0, zb) * Box(20, 401, 35)
    parts["beam"] = beam
    # hai tay đỡ vai: tấm Al 6 x 60 từ x -19 đến +150, y quanh ±170, mặt trên = trục vai - D0 - PAN_DROP (R0)
    top = z_ax - D0 - PAN_DROP
    for s in (-1, 1):
        parts[f"out{'L' if s > 0 else 'R'}"] = Pos((-19 + SX + PAD_DX + 30) / 2, s * SY, top - 3) * Box(SX + PAD_DX + 30 + 19, 60, 6)
        parts[f"pad{'L' if s > 0 else 'R'}"] = Pos(SX + PAD_DX, s * SY, top - 3) * Box(60, 60, 6)   # tấm giao diện: 4 x M5 PCD 42 [giả định] cho vòng bi pan dưới M1-LF-024, tâm = trục pan (R0)
    # đai ốc T8 mặt bích Ø22 dày 30 (POM/đồng) trục x -60
    nut = Pos(SCX + 0, 0, zb) * Cylinder(11, 30)
    parts["nut"] = nut
    # đai ốc an toàn dự phòng (QC2 vòng 2 mục NÊN): đai ốc T8 thứ hai không tải, cách đai ốc chính 10 mm dưới, bắt cùng tấm cầu;
    # chỉ chịu tải khi ren đai ốc chính mòn hết (độ rơ dọc trục tăng): tải rơi xuống đai ốc an toàn thay vì rơi tự do
    parts["safe_nut"] = Pos(SCX, 0, zb - 35) * Cylinder(11, 20)
    return parts

def screw_parts():
    # vitme T8x2 treo từ vòng bi chặn trên; từ z 205 đến 1085 (dài 880: mua 900 chuẩn?) trục x=-60, y=0
    z0, z1 = 205.0, 1085.0
    s = Pos(SCX, 0, (z0 + z1) / 2) * Cylinder(4, z1 - z0)
    return s

def top_block():
    """Gối vitme trên Al/PETG 28x40x30: vòng bi chặn + chặn cứng trên (đệm PU dày 5 dưới đáy) + giá công tắc."""
    return Pos(-60, 0, 1068 + 16) * Box(40, 40, 32)    # x -80..-40, bắt mặt trước cột; giữ vòng bi chặn vitme

def stops():
    top = Pos(-60, 0, 1068 - 2.5) * Box(40, 40, 5)          # đệm PU trên (đáy gối trên 1068), x -80..-40 (mặt trước cột, trục vitme x -60)
    bot_block = Pos(-60, 0, 380) * Box(40, 40, 10)           # chặn dưới: khối nhôm bắt mặt trước cột, mặt trên 385
    bot_pad = Pos(-60, 0, 387.5) * Box(40, 40, 5)
    return top, bot_block, bot_pad

def gearbox():
    """Hộp tăng tốc 16:1 đặt trên tấm đế cột (z 170..290): servo STS3095 trục thẳng đứng (còi lên), bánh Z48 mod 1,5 (Ø72) trên servo ->
    Z12 (Ø18) cùng trục trung gian với puly GT2 64T (Ø40,7) -> đai 6 mm -> puly 16T (Ø10,2) ở đầu vitme. Khoảng cách trục: servo-trung gian 45, trung gian-vitme 32 (dọc y)."""
    xs = SCX
    I_ = (xs, 32.0); O_ = (xs, 77.0)
    z = 235.0
    g = Pos(O_[0], O_[1], z) * Cylinder(36, 14)               # Z48 (thể tích đặc ~ cận trên khối lượng)
    g2 = Pos(I_[0], I_[1], z) * Cylinder(9, 14)               # Z12
    p64 = Pos(I_[0], I_[1], z + 16) * Cylinder(20.35, 8)
    p16 = Pos(xs, 0, z + 16) * Cylinder(5.1, 8)
    servo = Pos(O_[0] + 0, O_[1] + 20, z - 20) * Box(48, 65, 30)    # thân servo [65x30x48: trục ra nằm trên mặt 48x65, giản lược]
    plate = Pos(xs, 40, 175 + 3) * Box(110, 190, 6)             # tấm gá hộp PETG/Al 6 mm trên chân cột
    belt = Pos(xs, 16, z + 16) * Box(6, 32, 6)
    return dict(zgear=g, pinion=g2, pulley64=p64, pulley16=p16, servo=servo, plate=plate, belt=belt)

def vol(s): return s.volume

def mass_report():
    car = carriage(Z_AX0)
    m = {}
    for k, s in car.items():
        m[k] = s.volume * (RHO["POM"] if k in ("nut", "safe_nut") else RHO["Al"])
    m["blocks_4x"] = 4 * 90.0 * 2 / 2    # 4 khối/xe ~ 90 g mỗi khối ... 2 ray x 2 khối = 4 khối [giả định 90 g/khối H-type, chưa kiểm]
    m["bu_long_vit_gia"] = 60.0
    m["ray_2x700"] = 2 * 0.70 * 650.0    # g: ray MGN12 ~0,65 kg/m [điển hình, chưa kiểm datasheet], không thuộc xe
    car_g = sum(v for k, v in m.items() if k != "ray_2x700")
    col_g = column().volume * RHO["Al"] * (1.0 / 1.0)
    return dict(ray_g=round(m["ray_2x700"]), xe_g=round(car_g), chi_tiet_xe_g={k: round(v) for k, v in m.items()}, cot_g=round(col_g))

def _aabb(shape, grow=0.0):
    bb = shape.bounding_box()
    return (bb.min.X - grow, bb.min.Y - grow, bb.min.Z - grow, bb.max.X + grow, bb.max.Y + grow, bb.max.Z + grow)

def interference():
    """(1) cụm vai thật (shoulder_balance_assembly.step) quét pan -110..110 mỗi 10 độ, cả hai tay, lift 0 và 500: giao thể tích với cột, ray, xe, đầu.
    (2) chuỗi tay (vai 140 + cẳng 434 duỗi, bán kính 30 mm [giả định]) quét pan x vai -80..48,5 x khuỷu -10..127 : khoảng hở nhỏ nhất tới từng chi tiết (hộp bao)."""
    import numpy as np
    sba = import_step(os.path.join(EXP, "shoulder_balance_assembly.step"))
    head = Pos(CX + 10, 0, spec.COLUMN_TOP * 1000 - 30) * Box(70, 140, 60)     # đầu camera theo sdk/model.py
    res = {}
    for lift in (0.0, STROKE):
        z_ax = Z_AX0 + lift
        car = carriage(z_ax)
        stat = {"cot": column(), "de_cot": flange(), "dau": head, "gối_trên": top_block()}
        for i, b in enumerate(blocks(z_ax)): stat[f"khoi{i}"] = b
        for i, r in enumerate(rails()): stat[f"ray{i}"] = r
        for k, v in car.items(): stat["xe_" + k] = v
        # (1)
        bad = []
        for side in (-1, 1):
            own = "L" if side > 0 else "R"
            for pan in range(-110, 111, 10):
                sh_ = Pos(SX, side * SY, z_ax) * Rot(Z=pan) * sba
                for k, st in stat.items():
                    if k.startswith("xe_") and k.endswith(own) and k[3:6] in ("pad", "out"):
                        continue          # giá chính của tay này
                    v = (sh_ & st).volume
                    if v > 1.0: bad.append((side, pan, k, round(v)))
        # (2)
        boxes = {k: np.array(_aabb(v, 30.0)) for k, v in stat.items()}
        boxes_own = {k: np.array(_aabb(v, 45.0)) for k, v in stat.items()}
        hits = {}; ncfg = 0; ncol = 0
        for side in (-1, 1):
            own = "L" if side > 0 else "R"
            for pan in range(-110, 111, 10):
                for qs in np.linspace(-80, 48.5, 9):
                    for qe in np.linspace(-10, 127, 9):
                        ncfg += 1
                        a1 = math.radians(qs); a2 = math.radians(qs + qe)
                        pts = []
                        for t in np.arange(0, 140, 10.0): pts.append((t * math.cos(a1), -t * math.sin(a1)))
                        p1 = (140 * math.cos(a1), -140 * math.sin(a1))
                        for t in np.arange(0, 434, 10.0): pts.append((p1[0] + t * math.cos(a2), p1[1] - t * math.sin(a2)))
                        pts = np.array(pts)
                        # lấy mẫu từ 60 mm ra ngoài vai (bỏ phần cụm vai đã xét ở (1))
                        pts = pts[pts[:, 0] ** 2 + pts[:, 1] ** 2 > 60 ** 2]
                        cp, sp = math.cos(math.radians(pan)), math.sin(math.radians(pan))
                        X = SX + pts[:, 0] * cp; Y = side * SY + pts[:, 0] * sp; Z = z_ax + pts[:, 1]
                        hit_any = False
                        for k, bx in boxes.items():
                            if k.startswith("xe_") and k.endswith(own) and k[3:6] in ("pad", "out") and False: continue
                            inside = (X > bx[0]) & (X < bx[3]) & (Y > bx[1]) & (Y < bx[4]) & (Z > bx[2]) & (Z < bx[5])
                            if inside.any():
                                hit_any = True; hits.setdefault(k + ('|cua_tay_nay' if (k.startswith('xe_') and k.endswith(own) and k[3:6] in ('pad', 'out')) else ''), []).append((side, pan, round(float(qs)), round(float(qe))))
                        ncol += hit_any
        res[f"lift_{int(lift)}"] = dict(cum_vai_giao_cat=dict(so_cap=len(bad), vi_du=bad[:6]),
            chuoi_tay=dict(so_tu_the=ncfg, so_cham=ncol, theo_chi_tiet={k: dict(n=len(v), pan_min_abs=min(abs(h[1]) for h in v), vai=[min(h[2] for h in v), max(h[2] for h in v)], khuyu=[min(h[3] for h in v), max(h[3] for h in v)], vi_du=v[:2]) for k, v in hits.items()}))
    return res

def rail_check():
    """Kiểm hình học khối-ray (QC2 vòng 2 mục 3-4): khối nằm trọn trong ray, có biên, ở cả hai đầu hành trình DANH ĐỊNH và khi chạm đệm.
    Đệm chạm cầu xe (z cầu = trục vai - 73 -+ 75); khối là hộp chữ nhật dài 45,4 (H-type) [catalog điển hình, chưa kiểm]."""
    car0 = carriage(Z_AX0)["bridge"].bounding_box(); zb0 = Z_AX0 + Z_BEAM
    t_pad, b_blk, b_pad = stops()
    gap_dn = car0.min.Z - b_pad.bounding_box().max.Z                                   # cầu -> đệm dưới ở lift 0
    gap_up = t_pad.bounding_box().min.Z - (carriage(Z_AX0 + STROKE)["bridge"].bounding_box().max.Z)   # cầu -> đệm trên ở lift 500
    out = {"ray_z": [Z_RAIL0, Z_RAIL1], "ray_dai_mm": Z_RAIL1 - Z_RAIL0, "qua_dem_duoi_mm": round(gap_dn, 2), "qua_dem_tren_mm": round(gap_up, 2)}
    margins = {}
    for tag, lift, over in (("lift0", 0.0, -gap_dn), ("lift0_cham_dem", 0.0, -gap_dn), ("lift500", STROKE, 0.0), ("lift500_cham_dem", STROKE, gap_up)):
        blks = blocks(Z_AX0 + lift + (over if lift == 0 else over))
        lo = min(b.bounding_box().min.Z for b in blks); hi = max(b.bounding_box().max.Z for b in blks)
        margins[tag] = dict(khoi_z=[round(lo, 1), round(hi, 1)], bien_duoi_mm=round(lo - Z_RAIL0, 1), bien_tren_mm=round(Z_RAIL1 - hi, 1))
    out["bien_khoi"] = margins
    out["dat"] = all(v["bien_duoi_mm"] >= 10 and v["bien_tren_mm"] >= 10 for v in margins.values())
    return out

def link_vs_pad():
    """Link vai thật (balance.link_v2) quay Rot(Y=q) trong dải vai -80..48,5: điểm x nhỏ nhất của link nằm dưới mặt trên tấm vai (z = -D0 - PAN_DROP) so với mép trước tấm."""
    import balance as b
    l2 = b.link_v2(0.0); box = Pos(150, 0, -D0 - PAN_DROP - 100) * Box(600, 200, 200)
    q = spec.SHOULDER_SOFT_MAX_DEG
    s_ = (Rot(Y=q) * l2) & box
    xmin = s_.bounding_box().min.X
    return dict(q_deg=q, x_link_duoi_mat_tam_mm=round(xmin, 1), mep_truoc_tam_mm=PAD_DX + 30, ho_mm=round(xmin - (PAD_DX + 30), 1))

def export_all():
    os.makedirs(EXP, exist_ok=True)
    car = carriage(Z_AX0)
    gb = gearbox()
    top = top_block(); t_pad, b_blk, b_pad = stops()
    files = {
     "M1-LF-010_column_4040.step": column(),
     "M1-LF-011_base_flange.step": flange(),
     "M1-LF-020_carriage_side.step": car["sideL"],
     "M1-LF-021_carriage_bridge.step": car["bridge"],
     "M1-LF-022_carriage_beam.step": car["beam"],
     "M1-LF-023_shoulder_outrigger.step": car["outL"],
     "M1-LF-030_top_block.step": top,
     "M1-LF-040_gearbox_plate.step": gb["plate"],
    }
    for n, s in files.items(): export_step(s, os.path.join(EXP, n))
    asm = Compound([column(), flange(), screw_parts(), top, t_pad, b_blk, b_pad] + rails() + blocks(Z_AX0) + list(car.values()) + list(gb.values()))
    export_step(asm, os.path.join(EXP, "M1-LF-assembly_lift0.step"))
    asm5 = Compound([column(), flange(), screw_parts(), top] + rails() + blocks(Z_AX0 + STROKE) + list(carriage(Z_AX0 + STROKE).values()) + list(gb.values()))
    export_step(asm5, os.path.join(EXP, "M1-LF-assembly_lift500.step"))
    return list(files)

if __name__ == "__main__":
    out = dict(khoi_luong=mass_report(), link_vs_tam_vai=link_vs_pad(), ray=rail_check(), giao_cat=interference(), files=export_all())
    json.dump(out, open(os.path.join(EXP, "M1-LF-build.json"), "w"), indent=1, ensure_ascii=False)
    print(json.dumps(out, indent=1, ensure_ascii=False))
