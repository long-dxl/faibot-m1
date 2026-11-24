# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""Lắp ráp Faibot M1 v0.5 cấu hình P-L (chốt 23/11/2026 tối): 2 tay AM-ARM200 follower nguyên bản (6+1; trước: 5+1, yaw khoá) trên trạm hai tay
tháo lắp (S2, G1b 24/11, tram_hai_tay.py, vai 0,475 m, cột camera đầu) gắn lên đế ngàm (tram_de.py) trên khung T giữa AlohaMini2
(xe nâng; bỏ khung T trái/phải, xà ngang, giá camera ngực), trụ AlohaMini2 (file bản thường), trục nâng thanh răng AlohaMini2 (1 STS3095) +
chốt hãm solenoid, đế kiểu LeKiwi 12 V (3 STS3215, omni 4 in) nối nan nhôm 20x40 ra R 0,25 m + tấm thép đối trọng, khay cố định.
build123d.

Hệ robot W (mm): x trước, y trái, z lên, gốc sàn tại tâm tam giác bánh (như sdk). Hệ URDF AlohaMini2 U (base_link): trước là -y.
W = Rz(+90) U: x_W = -y_U, y_W = x_U. Chi tiết upstream AlohaMini2 đặt bằng phép biến đổi trong cad/export/AlohaMini2_placement.json
(đăng ký mesh bố trí in vào mesh URDF base_link / vertical_link: am2_place.py). Tay đặt theo URDF AlohaMini2 (left/right_base_joint)
và chuỗi khớp URDF AM-ARM200 (amarm.py); 12 STEP follower gán link theo amarm.PART_LINK.
Không gia công nhôm: chỉ nhôm định hình cắt (thanh ngang T 2040, nan 20x40) và thép tấm cắt; còn lại in PETG hoặc mua sẵn.

Kẹp (24/11/2026): mặc định kẹp gốc AM-ARM200 (thân J7 M1-AR-007, hàm J7 M1-AR-008, giá camera OB_F_Camera M1-AR-027, Apache-2.0;
gripper_am.py), đặt theo link left_Fixed_Jaw / left_Moving_Jaw của URDF upstream nên đi theo khớp cuộn và khớp kẹp. M1_GRIPPER khác "am": kẹp
song song cũ từ lưu trữ nội bộ (tuỳ chọn tương lai, chờ giấy phép; chỉ nội bộ).
Đế bàn (bộ tuỳ chọn, tram_de.de_ban_parts) không thuộc build(); kiểm riêng bằng --ban (trạm + 2 tay + đế bàn, khung trạm đặt trên bàn).
Chạy: python assembly_pl.py [--lift 0] [--pose ready] [--grip 0] [--roll 0] [--check] [--mesh DIR] [--masses] [--ban]
(--grip: độ mở kẹp % 0 khép..100 mở mềm; --roll: cuộn cổ tay độ, quy ước SDK)
"""
import json, math, os, sys, argparse, itertools
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); EXP = os.path.join(HERE, "../export")
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, "../../sdk"))
from build123d import Box, Cylinder, Pos, Rot, Align, Compound, export_step, export_stl, import_step, Location, Plane
import servos as SV
import amarm as AM
GRIPPER = os.environ.get("M1_GRIPPER", "am")     # "am" (mặc định) | kẹp song song cũ (lưu trữ nội bộ)
ROBONINE_DIR = os.path.join(HERE, "..", "..", "archive", "robonine", "cad")  # public-scan: ok

MIN = (Align.MIN,) * 3
AM2 = os.path.join(HERE, "..", "upstream", "AlohaMini", "AlohaMini2", "hardware", "mobile_base2", "stp")
PLACE = json.load(open(os.path.join(EXP, "AlohaMini2_placement.json")))
# (R3d, 24/11): bánh răng lift 20 răng m1,5 (vòng chia r 15 mm) thay bánh 26 răng STEP AlohaMini2 (r 19,5), KHÔNG lò xo.
# Tâm bánh (và servo đồng trục) dời về phía thanh răng 4,5 mm để giữ đường chia thanh răng (x_W = tâm - r); x_W = -y_U.
GEAR_Z, GEAR_M, GEAR_R_OLD = 20, 1.5, 19.5
GEAR_R = GEAR_Z * GEAR_M / 2
GEAR_SHIFT = GEAR_R_OLD - GEAR_R
PLACE["gear_center_U_step26"] = list(PLACE["gear_center_U"])
PLACE["gear_center_U"] = [PLACE["gear_center_U"][0], PLACE["gear_center_U"][1] + GEAR_SHIFT, PLACE["gear_center_U"][2]]
RHO = {"Al": 2.70e-3, "PETG": 1.27e-3, "steel": 7.85e-3}
INFILL_UP = 0.40        # chi tiết in AlohaMini2: 4-6 vòng thành, infill 15-25 % (print_guide) -> khối lượng ~0,40 thể tích đặc [giả định]

# ---------------- hệ toạ độ ----------------
RW = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], float)     # U -> W


def M4(R=np.eye(3), t=(0, 0, 0)):
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t; return M


def loc(M):
    return Location(Plane(origin=tuple(M[:3, 3]), x_dir=tuple(M[:3, 0]), z_dir=tuple(M[:3, 2])))


def U2W(p):
    return RW @ np.asarray(p, float)


# ---------------- upstream AlohaMini2 ----------------
STATIC = ("OB_Main_Assembly_Post1", "OB_Main_Assembly_Post2", "OB_Main_Assembly_Post3", "O_Main_Assembly_Post4", "O_POST4_Connector_Base")
# OB_POST4_Mount_Adapter (giá sau cho Pi/màn hình/pin/đối trọng của AlohaMini2) không dùng: P-L để điện tử trên sàn đế (M1-BS-038)
# S2: xe nâng chỉ giữ khung T giữa (+ chốt định vị 12x25 nằm trong nó); khung T trái/phải, xà ngang 2040, giá camera ngực bỏ (trạm thay)
MOVING = ("OB_T_Connector_Middle", "O_T_Connector_Dowel_Pin_12_25")
CODE = {"OB_Main_Assembly_Post1": "M1-LF-051", "OB_Main_Assembly_Post2": "M1-LF-052", "OB_Main_Assembly_Post3": "M1-LF-053", "O_Main_Assembly_Post4": "M1-LF-054",
        "O_POST4_Connector_Base": "M1-LF-055", "OB_POST4_Mount_Adapter": "M1-LF-056", "OB_T_Connector_Middle": "M1-LF-057", "OB_T_Connector_Left": "M1-LF-058",
        "OB_T_Connector_Right": "M1-LF-059", "O_T_Connector_Cross_Bar": "M1-BS-006", "O_T_Connector_Dowel_Pin_12_25": "M1-LF-004", "OB_T_Camera_Mount": "M1-CM-004",
        "OB_Z_Axis_Servo_Gear": "M1-LF-002"}
_UP = {}


def up_shape(name):
    """Chi tiết upstream trong hệ W ở lift 0 (vertical_move = 0)."""
    if name not in _UP:
        s = import_step(os.path.join(AM2, name + ".stp")); p = PLACE[name]
        sol = s.solids()
        if len(sol) > 1:      # STEP Post1 có thực thể lạc ở gốc toạ độ bố trí: chỉ giữ solid có thể tích đáng kể
            big = [x for x in sol if x.volume > 0.01 * max(y.volume for y in sol)]
            s = Compound(big) if len(big) > 1 else big[0]
        M = np.array(p.get("post_W", np.eye(4).tolist())) @ M4(RW) @ M4(np.array(p["R"]), p["t"]) @ M4(np.eye(3), -np.array(p.get("step_offset", (0, 0, 0))))
        if name == "OB_Z_Axis_Servo_Gear":    # STEP bánh răng (bản 'repair') không ở hệ bố trí như STL: đặt tâm theo STL, trục dọc x_U
            bb = s.bounding_box(); c = bb.center()
            M = M4(RW) @ M4(np.eye(3), PLACE["gear_center_U"]) @ M4(np.eye(3), (-c.X, -c.Y, -c.Z))
        _UP[name] = loc(M) * s
    return _UP[name]


def T_frame(lift):
    out = [(f"{CODE[n]}_{n}", "ivory" if "Bar" not in n and "Pin" not in n else ("alu" if "Bar" in n else "steel"), Pos(0, 0, lift) * up_shape(n)) for n in MOVING]
    out.append(("M1-LF-002_lift_gear_z20", "ivory", Pos(0, 0, lift) * gear20()))
    return out


_GEAR = []


def gear20(width=14.0, hub_r=9.0, hub_w=7.0):
    """Bánh răng lift 20 răng m1,5 (đỉnh r 16,5, chân r 13,125), răng hình thang (gần đúng thân khai, đủ cho kiểm hình), rộng 14 + moay-ơ
    Ø18 x 7 phía còi servo; trục dọc y_W, tâm PLACE['gear_center_U'] (W, lift 0). [giả định: bề rộng, moay-ơ; R3d mục 3]"""
    if not _GEAR:
        from build123d import Polygon, extrude, Plane as Pl
        ra, rf = GEAR_R + GEAR_M, GEAR_R - 1.25 * GEAR_M; pts = []
        for k in range(GEAR_Z):
            a0 = 2 * math.pi * k / GEAR_Z; p = 2 * math.pi / GEAR_Z
            for ang, r in ((a0 - 0.30 * p, rf), (a0 - 0.13 * p, ra), (a0 + 0.13 * p, ra), (a0 + 0.30 * p, rf)):
                pts.append((r * math.cos(ang), r * math.sin(ang)))
        g = extrude(Polygon(*pts, align=None), width)                                    # trục z cục bộ
        g += Pos(0, 0, width) * Cylinder(hub_r, hub_w, align=(Align.CENTER, Align.CENTER, Align.MIN))
        c = U2W(PLACE["gear_center_U"]); s = PLACE.get("servo_side", 1.0)
        R = np.array([[1, 0, 0], [0, 0, s], [0, -s, 0]], float)          # z cục bộ -> s*y_W: moay-ơ về phía còi servo (lift_servo_pose: còi ở tâm + s*14)
        M = M4(R, c - R @ np.array([0, 0, width / 2]))
        _GEAR.append(loc(M) * g)
    return _GEAR[0]


def column():
    return [(f"{CODE[n]}_{n}", "ivory", up_shape(n)) for n in STATIC]


# ---------------- tay ----------------
import tram_hai_tay as TR, tram_de as TD
# gốc tay: x (trước) và z giữ như khớp URDF AlohaMini2 left/right_base_joint (+ vertical_move gốc); y = ±nửa khoảng vai trạm (0,475 m; URDF ±187,26)
ARM_BASE_U = {"L": (TR.SHOULDER_HALF, -59.9, 96.196 + 477.5), "R": (-TR.SHOULDER_HALF, -59.9, 96.196 + 477.5)}
T_STEP = None


def _t_step():
    global T_STEP
    if T_STEP is None:
        import assembly_full as A
        T_STEP = np.array(A.wrist_ref()["t"])
    return T_STEP


_ARM = {}


def arm_parts_local():
    """STEP follower trong khung left_Base (mm) ở tư thế 0, kèm link."""
    if not _ARM:
        t = _t_step()
        for f, link in AM.PART_LINK.items():
            if f in ("OB_F_J7_Gripper", "OB_F_J7_Jaw", "OB_F_Camera"):
                continue        # kẹp J7 + giá camera: STEP ở bố cục trưng bày, đặt riêng bằng đăng ký (gripper_am.step_parts_base)
            s = import_step(os.path.join(AM.STP, f + ".stp"))
            if f in PLACE:      # J1, J2: STEP bố trí khác, đăng ký vào mesh link URDF
                p = PLACE[f]; s = loc(M4(np.array(p["R"]), p["t"])) * s
            else:
                s = Pos(*(-t)) * s
            _ARM[f] = (link, s)
    return _ARM


_GRIP = []


def grip_step():
    """Chi tiết gắn link wrist_yaw (cuộn 0) trong hệ STEP follower: với M1_GRIPPER khác "am" kẹp cũ mở 84 mm (không còn khoá yaw M1-AR-025)
    + gá M1-GR-002 (lưu trữ nội bộ). Kẹp AM-ARM200 không ở đây (đi theo link kẹp, xem grip_am_local)."""
    if not _GRIP:
        import assembly_full as A
        if GRIPPER == "robonine":  # public-scan: ok
            sys.path.insert(0, ROBONINE_DIR); import gripper_robonine as GR  # public-scan: ok
            ra = A.wrist_ref()["fore"](A.wrist_ref()["roll"]); ax = (float(ra[1]), float(ra[2]))
            Lg = SV.m2loc(GR.g_to_step(A, ax))
            _GRIP.extend((n, m, Lg * s) for n, m, s in GR.parts_g(84.0))
        # (6+1): khoá yaw M1-AR-025 BỎ; servo yaw upstream đặt ở arm() (yaw_servo_pose_link)
    return _GRIP


_GAM = []


def grip_am_local():
    """Kẹp AM-ARM200: list (tên, vật liệu, link URDF, shape khung left_Base mm ở tư thế 0 URDF, góc kẹp URDF 0). Gồm thân, hàm, giá camera,
    camera H65V1 (M1-CM-001, vị trí đề xuất) và servo kẹp STS3215 (M1-SV-001_gripper, chỉ hiển thị)."""
    if GRIPPER != "am": return []
    if not _GAM:
        import gripper_am as GA
        _GAM.extend(GA.parts_base())
        _GAM.append(("M1-SV-001_gripper", "servo_body", "left_Fixed_Jaw", SV.solid("sts3215", GA.grip_servo_pose_base())))
    return _GAM


def yaw_servo_pose_link():
    """servo yaw STS3215 trong khung link URDF left_wrist_yaw (mm): trục ra = trục khớp yaw (z link, tại x = y = 0), còi hướng -z (về giá J5),
    thân dọc +y. Mặt tựa còi z 22,6 = đáy hộp bao chồng servo y < 20 mm trong mesh URDF upstream (18,6, cad/upstream/.../derived/derived.json
    bo_servo_yaw) + đầu vít 4,0 (servos.S15_SCREW_Z1 - S15_SEAT_Z)."""
    return SV.pose("sts3215", (0.0, 0.0, 18.62 + (SV.S15_SCREW_Z1 - SV.S15_SEAT_Z)), (0, 0, -1), (0, 1, 0))


def grip_urdf(pct):
    """Độ mở kẹp % (0 khép, 100 mở mềm) -> góc URDF left_gripper (rad), theo cad/export/AM_gripper_PL.json."""
    g = json.load(open(os.path.join(EXP, "AM_gripper_PL.json")))
    return math.radians(g["goc_khep_urdf_deg"] - g["chieu_khep_urdf"] * g["mo_mem_deg"] * pct / 100.0)


def arm(side, q, lift):
    """List (tên, vật liệu, hình) của một tay trong hệ W."""
    T0 = AM.fk({}); Tq = AM.fk(q)
    B = M4(RW) @ M4(np.eye(3), np.array(ARM_BASE_U[side]) + (0, 0, lift))
    out = []
    mat = {"left_Base": "ivory", "left_Rotation_Pitch": "navy", "left_Upper_Arm": "ivory", "left_Lower_Arm": "ivory", "left_Wrist_Pitch_Roll": "navy", "left_wrist_yaw": "ivory"}
    code = {"O_F_J1_Base": "M1-SH-005", "O_F_J2_Rotation": "M1-SH-019", "OB_F_J3_Shoulder": "M1-SH-006", "OB_F_J4_Elbow": "M1-AR-001", "OB_F_J5_Elbow_Wrist_Bracket": "M1-AR-002",
            "OB_F_J5_Pin_Bearing": "M1-AR-011", "OB_F_J56_Wrist_Housing": "M1-AR-003", "OB_F_J56_Bottom": "M1-AR-004", "OB_F_J56_Top": "M1-AR-005"}
    for f, (link, s) in arm_parts_local().items():
        D = Tq[link] @ np.linalg.inv(T0[link]); D[:3, 3] *= 1000
        out.append((f"{code[f]}_{f}_{side}", mat[link], loc(B @ D) * s))
    # kẹp cũ nếu M1_GRIPPER khác "am": cố định theo link wrist_yaw khi cuộn 0, đặt bằng phép đặt vỏ J56
    D = Tq["left_wrist_yaw"] @ np.linalg.inv(T0["left_wrist_yaw"]); D[:3, 3] *= 1000
    L = loc(B @ D) * Pos(*(-_t_step()))
    for n, m, sh in grip_step():
        out.append((f"{n}_{side}", m, L * sh))
    # (6+1, như AM-ARM200 gốc): servo yaw STS3215 trong vỏ J56, đi theo link wrist_yaw (thay khoá M1-AR-025)
    Ty = np.array(T0["left_wrist_yaw"], float).copy(); Ty[:3, 3] *= 1000
    out.append((f"M1-SV-001_wrist_yaw_{side}", "servo_body", loc(B @ D) * SV.solid("sts3215", Ty @ yaw_servo_pose_link())))
    # kẹp AM-ARM200 theo link left_Fixed_Jaw (cuộn) / left_Moving_Jaw (cuộn + kẹp)
    for n, m, link, sh in grip_am_local():
        D = Tq[link] @ np.linalg.inv(T0[link]); D[:3, 3] *= 1000
        out.append((f"{n}_{side}", m, loc(B @ D) * sh))
    return out


# ---------------- trục nâng: servo, chốt hãm, công tắc, đệm ----------------
def gear_axis_W(lift):
    c = U2W(PLACE["gear_center_U"]) + (0, 0, lift); return c, np.array([0.0, 1.0, 0.0])      # trục bánh dọc x_U = y_W


def lift_servo_pose(lift):
    """STS3095 trong khung T giữa: còi chạm mặt bánh răng phía PLACE['servo_side'], thân theo PLACE['servo_body'] (chọn bằng servo_fit())."""
    c, a = gear_axis_W(lift); s = PLACE.get("servo_side", 1.0); h = -s * a
    P = c + s * a * (14.0 + SV.S95_HORN_T)
    return SV.pose("sts3095", P, h, PLACE.get("servo_body", (0, 0, 1)))


def latch_parts(lift):
    """Chốt hãm (M1-LF-050 solenoid khối bao 45 x 24 x 22 [giả định]; M1-LF-041 đầu chốt; M1-LF-061 công tắc chốt rút; M1-LF-060 giá in).
    Đặt trên mặt trên khung T giữa, đầu chốt hướng -x vào răng của trụ (mặt răng x_W = PLACE['rack_tip_xW']), cao hơn tâm bánh LATCH_DZ.
    Trạng thái vẽ: chốt rút, cách đỉnh răng 2 mm."""
    c, _ = gear_axis_W(lift); ztop = PLACE["Tmid_top_zW"] + 0.5 + lift; z = ztop + 3.0 + 11.0
    xt = PLACE["rack_tip_xW"]; yc = PLACE["rack_center_yW"]
    pawl_tip = xt + 2.0
    sol = Pos(pawl_tip + 26.0, yc - 12.0, z - 11.0) * Box(45.0, 24.0, 22.0, align=MIN)
    pawl = Pos(pawl_tip, yc - 4.0, z - 3.0) * Box(26.0, 8.0, 6.0, align=MIN)
    sensor = Pos(pawl_tip + 50.0, yc + 13.0, z - 5.0) * Box(20.0, 10.0, 10.0, align=MIN)
    bracket = Pos(pawl_tip + 24.0, yc - 20.0, ztop) * Box(60.0, 46.0, 3.0, align=MIN)
    return [("M1-LF-050_latch_solenoid", "servo", sol), ("M1-LF-041_latch_pawl", "steel", pawl), ("M1-LF-061_latch_sensor", "servo", sensor),
            ("M1-LF-060_latch_bracket", "navy", bracket)]


def switches(lift):
    """2 công tắc hành trình NC (M1-LF-041, khối bao 18 x 20 x 8 [giả định]) gắn dưới và trên khung T giữa, đi theo xe, phía trước-trái.
    Vật chặn cố định (khối in kẹp trụ + đệm PU) CHƯA vẽ: AlohaMini2 không có chặn cứng; đặt khi in thử (điều kiện mở)."""
    zb, zt = PLACE["Tmid_bot_zW"] - 0.5 + lift, PLACE["Tmid_top_zW"] + 0.5 + lift
    return [("M1-LF-041_switch_bot", "servo", Pos(48.0, 45.0, zb - 8.0) * Box(18.0, 20.0, 8.0, align=MIN)),
            ("M1-LF-041_switch_top", "servo", Pos(48.0, 45.0, zt) * Box(18.0, 20.0, 8.0, align=MIN))]


# ---------------- đế kiểu LeKiwi ----------------
WHEEL_R_MOUNT, WHEEL_ANG = 250.0, (0.0, 120.0, 240.0)     # R 0,25, một bánh ra trước
WR, WW = 50.8, 40.0                    # omni 4 in (LeKiwi BOM), rộng 40 [giả định, chưa có bản vẽ bên bán]
AXLE_Z = WR
SPOKE_W, SPOKE_H, SPOKE_R0, SPOKE_R1 = 40.0, 20.0, 95.0, 188.0      # nan 20x40 nằm trên tấm thép; nan trước bắt trên lưỡi tấm từ r 60
PLATE_T = 8.5
CW2_T = 6.0                                              # tấm thép đối trọng phụ M1-BS-039 bắt dưới tấm chính, cùng biên CW_MAIN (DK-N40, Q8 (a): tổng ~8 kg)
CW_MAIN = (-245.0, 22.0, -130.0, 130.0)                 # tấm thép chính (x0, x1, y0, y1)
CW_TONGUE = (22.0, 120.0, -25.0, 25.0)                  # lưỡi trước đỡ nan 0
DECK_R, DECK_T = 185.0, 5.0                              # sàn in kiểu LeKiwi trên nan
FOOT_CUT = (-62.0, 62.0, -71.0, 71.0)                    # khoét sàn quanh đế trụ O_POST4_Connector_Base (x -57..57, y -66..66)
EBAY = [((-80.0, 85.0), (100.0, 75.0, 70.0)), ((-80.0, -160.0), (100.0, 75.0, 70.0))]   # pin 12 V (trái), Pi 5 + 3 bus adapter + E-stop/contactor (phải) [giả định]


def base_heights():
    """Mặt tấm thép = đáy O_POST4_Connector_Base (z 39,0): giữ nguyên độ cao trụ, khung T, tay như URDF AlohaMini2."""
    zf = up_shape("O_POST4_Connector_Base").bounding_box().min.Z      # = PLACE['post_foot_zW'] (39,0), lấy đúng hình đã đặt để mặt tựa khít
    return dict(plate_top=zf, plate_bot=zf - PLATE_T, spoke_bot=zf, spoke_top=zf + SPOKE_H, deck_top=zf + SPOKE_H + DECK_T)


def omni(i):
    a = WHEEL_ANG[i]; parts = {}
    parts["hub"] = Rot(Y=90) * Cylinder(32.0, 2 * 17.0)
    for k, s in enumerate((-18.5, 18.5)): parts[f"plate{k}"] = Pos(s, 0, 0) * Rot(Y=90) * Cylinder(27.0, 3.0)
    rr, rl, rc, n = 8.0, 15.0, WR - 8.0, 9
    from build123d import fillet
    base = fillet(Cylinder(rr, rl).edges(), 4.0); k = 0
    for row, ax in enumerate((-10.0, 10.0)):
        for j in range(n):
            ang = 360.0 / n * j + (20.0 if row else 0.0)
            parts[f"roller{k}"] = Pos(ax, 0, 0) * Rot(X=ang) * Pos(0, 0, rc) * Rot(X=90) * base; k += 1
    return {f"M1-BS-035_omni{i}_{n}": Pos(0, 0, AXLE_Z) * Rot(Z=a) * Pos(WHEEL_R_MOUNT, 0, 0) * s for n, s in parts.items()}


SEAT_R = WHEEL_R_MOUNT - WW / 2 - (SV.S15_SCREW_Z1 - SV.S15_SEAT_Z)      # 226: còi + vít chạm mặt trong bánh (bánh lắp thẳng trên còi như LeKiwi)


def wheel_servo_poses():
    out = []
    for i, a in enumerate(WHEEL_ANG):
        u = np.array([math.cos(math.radians(a)), math.sin(math.radians(a)), 0.0]); tg = np.array([-u[1], u[0], 0.0])
        out.append((f"M1-SV-001_wheel{i}", "sts3215", SV.pose("sts3215", SEAT_R * u + np.array([0, 0, AXLE_Z]), u, tg)))      # thân nằm ngang theo tiếp tuyến
    return out


def base_parts():
    h = base_heights(); out = []
    for i in range(3):
        out += [(n, "hub" if ("hub" in n or "plate" in n) else "rubber", s) for n, s in omni(i).items()]
    for i, a in enumerate(WHEEL_ANG):
        R = Rot(Z=a); r0 = 60.0 if i == 0 else SPOKE_R0
        out.append((f"M1-BS-031_spoke{i}", "alu", R * Pos(r0, -SPOKE_W / 2, h["spoke_bot"]) * Box(SPOKE_R1 - r0, SPOKE_W, SPOKE_H, align=MIN)))
        # giá servo bánh in (kiểu LeKiwi): vách sau servo bắt đầu nan + nắp trên thân servo; bánh lắp trên còi, không có gối ngoài
        r_sv0 = SEAT_R - SV.S15_SEAT_Z - 18.3                      # mặt sau servo (vấu đáy) r 191,5
        wall = Pos(SPOKE_R1, -22.0, h["plate_bot"] + 2.0) * Box(r_sv0 - 0.5 - SPOKE_R1, 62.0, AXLE_Z + 26.0 - (h["plate_bot"] + 2.0), align=MIN)
        cap = Pos(r_sv0 - 0.5, -14.0, AXLE_Z + 12.9) * Box(SEAT_R - 1.0 - (r_sv0 - 0.5), 52.0, 10.0, align=MIN)
        out += [(f"M1-BS-036_wheel_mount_wall{i}", "navy", R * wall), (f"M1-BS-036_wheel_mount_cap{i}", "navy", R * cap)]
    x0, x1, y0, y1 = CW_MAIN; tx0, tx1, ty0, ty1 = CW_TONGUE
    plate = Pos(x0, y0, h["plate_bot"]) * Box(x1 - x0, y1 - y0, PLATE_T, align=MIN) + Pos(tx0, ty0, h["plate_bot"]) * Box(tx1 - tx0, ty1 - ty0, PLATE_T, align=MIN)
    out.append(("M1-BS-032_cw_hub_plate", "steel", plate))
    out.append(("M1-BS-039_cw_plate2", "steel", Pos(x0, y0, h["plate_bot"] - CW2_T) * Box(x1 - x0, y1 - y0, CW2_T, align=MIN)))
    deck = Pos(0, 0, h["spoke_top"]) * Cylinder(DECK_R, DECK_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
    deck -= Pos(FOOT_CUT[0], FOOT_CUT[2], h["spoke_top"] - 1) * Box(FOOT_CUT[1] - FOOT_CUT[0], FOOT_CUT[3] - FOOT_CUT[2], DECK_T + 2, align=MIN)
    out.append(("M1-BS-037_deck", "ivory", deck))
    for k, ((x, y), (dx, dy, dz)) in enumerate(EBAY):
        out.append((f"M1-BS-038_ebay{k}", "ivory_hi", Pos(x, y, h["deck_top"]) * Box(dx, dy, dz, align=MIN)))
    return out


# ---------------- khay ----------------
TRAY = dict(x0=120.0, x1=320.0, y=150.0, t=5.0, top=210.0, lip=8.0)


def tray_parts(display=False):
    """Khay đế (cad/faibotic/khay_de.py): 4 mảnh PETG ghép mộng đuôi én + M3, cột M1-ST-011 cắt ngắn, thanh 2040 đỡ khay.
    display=True: hình đơn giản cho sim/GLB (khay_de.build_parts(display=True))."""
    import khay_de
    mat = {"M1-ST-012": "ivory", "M1-ST-011": "navy", "2040": "alu"}
    out = []
    for n, s in khay_de.build_parts(display):
        k = next(m for m in mat if n.startswith(m))
        out.append((n if k != "2040" else "M1-BS-006_2040_do_khay", mat[k], s))
    return out


# ---------------- trạm hai tay + đế ngàm ----------------
def station_origin(lift):
    """Gốc khung trạm trong W (mm): giữa hai trục pan, ngang đáy J1."""
    p = U2W(ARM_BASE_U["L"]); return np.array([p[0], 0.0, p[2] + lift])


def station(lift, tilt=TR.CAM_TILT):
    o = station_origin(lift); return [(n, m, Pos(*o) * s) for n, m, s in TR.station_parts(tilt)]


def dock(lift):
    o = station_origin(lift); return [(n, m, Pos(*o) * s) for n, m, s in TD.dock_parts()]


def table_base(lift=0.0):
    """Đế bàn tuỳ chọn đặt dưới trạm (cùng gốc trạm); chỉ cho --ban, không thuộc build()."""
    o = station_origin(lift); return [(n, m, Pos(*o) * s) for n, m, s in TD.de_ban_parts()]


# ---------------- lắp ráp ----------------
def _q(pose, pan, grip, roll):
    q = AM.pose(pose, pan)
    if GRIPPER == "am": q["grip"] = grip_urdf(grip)
    if roll: q["roll"] = math.radians(roll) / AM_ROLL_SIGN
    return q


AM_ROLL_SIGN = 1.0      # spec.URDF_TO_SDK["wrist_roll"] = (+1, 0)


def build(lift, pose, pan=0.0, grip=0.0, roll=0.0):
    q = _q(pose, pan, grip, roll)
    out = base_parts() + tray_parts() + column() + T_frame(lift) + latch_parts(lift) + switches(lift) + dock(lift) + station(lift)
    for n, model, M in wheel_servo_poses() + [("M1-LF-lift_servo", "sts3095", lift_servo_pose(lift))]:
        out.append((n, "servo_body", SV.solid(model, M)))
    out += arm("L", q, lift) + arm("R", q, lift)
    return out


def build_ban(pose, pan=0.0, grip=0.0, roll=0.0):
    """Chế độ bàn: trạm + 2 tay + đế bàn tuỳ chọn (không có đế, trụ, xe nâng). Khung trạm ở lift 0 của W."""
    q = _q(pose, pan, grip, roll)
    return station(0.0) + table_base(0.0) + arm("L", q, 0.0) + arm("R", q, 0.0)


# ---------------- khối lượng cho chống lật ----------------
KG_FIXED = {"M1-SV-001": 0.0705, "M1-LF-lift_servo": 0.1875, "M1-BS-038_ebay0": 1.10, "M1-BS-038_ebay1": 0.50, "M1-LF-050": 0.06, "M1-LF-061": 0.005}
# ebay0: 2 pin 12 V 11200 mAh (BOM AlohaMini2) ~0,55 kg/chiếc [giả định]; ebay1: Pi 5 + 3 bus adapter + E-stop, contactor, cầu chì, hộp ~0,5 kg [giả định]
OMNI_KG = 0.20          # omni 4 in, mỗi bánh [giả định, bên bán chưa ghi]
AL_KG_PER_MM = 0.6e-3   # nhôm định hình 20x40 ~0,6 kg/m [giả định, điển hình]


def part_mass(n, s):
    """(kg, nguồn) theo tên chi tiết; None nếu không tính (tay: tính riêng trong tipover theo URDF)."""
    if n.startswith(("M1-SH-", "M1-AR-", "M1-GR-", "M1-CM-001_wrist", "M1-SV-001_gripper", "M1-SV-001_wrist_yaw")): return None   # tay: tay_cad()
    if n.startswith(("M1-TR-", "M1-CM-001_head")): return TR.part_kg(n, s), "[tram_hai_tay.part_kg: giả định/tự tính]"
    for k, v in KG_FIXED.items():
        if n.startswith(k): return v, "[giả định]" if "ebay" in k or "050" in k or "061" in k else "[số hãng tự báo]"
    if n.startswith("M1-BS-035_omni") : return (OMNI_KG, "[giả định]") if n.endswith("_hub") else (0.0, "")
    if n.startswith(("M1-BS-031", "M1-BS-006")):
        bb = s.bounding_box(); L = max(bb.max.X - bb.min.X, bb.max.Y - bb.min.Y, bb.max.Z - bb.min.Z); return L * AL_KG_PER_MM, "[giả định 0,6 kg/m]"
    if n.startswith(("M1-BS-032", "M1-BS-039")): return s.volume * RHO["steel"] / 1000, "[tự tính]"
    if n.startswith(("M1-LF-041", "M1-LF-004", "M1-BS-033")): return s.volume * RHO["steel"] / 1000, "[tự tính]"
    if n.startswith(("M1-LF-05", "M1-LF-002", "M1-LF-060")) or "_OB_" in n or "_O_" in n: return s.volume * RHO["PETG"] * INFILL_UP / 1000, "[tự tính, infill 0,40 giả định]"
    if n.startswith("M1-ST-012"): return s.volume * RHO["PETG"] / 1000, "[tự tính, chi tiết mỏng 4 vách coi đặc]"
    if n.startswith(("M1-BS-036", "M1-BS-037", "M1-ST-01")): return s.volume * RHO["PETG"] * 0.5 / 1000, "[tự tính, infill 0,5 giả định]"
    return None


CW_KEY = "M1-BS-032+039_doi_trong"
ARM_LINK_BODY = {"left_Base": "mount", "left_Rotation_Pitch": "shoulder", "left_Upper_Arm": "upper", "left_Lower_Arm": "fore",
                 "left_Wrist_Pitch_Roll": "wrist", "left_wrist_yaw": "yaw", "left_Fixed_Jaw": "hand", "left_Moving_Jaw": "jaw"}


def tay_cad():
    """Một tay 6+1 từ CAD, tư thế duỗi ngang (amarm 'straight', như tipover.arm_pl): chi tiết in = thể tích STEP follower x 1,0 g/cm3
    (cùng quy ước khối đặc của URDF/spec.LINK_PRINT, cận trên); servo theo datasheet: yaw và kẹp đặt theo hình (assembly_pl.arm), pan/vai/khuỷu/gập/cuộn
    tại gốc khớp (như spec.ARM_POINT_MASSES); camera, cáp + ốc theo spec. Trả khối lượng, tâm cách trục pan (m, ngang), bảng theo body."""
    sys.path.insert(0, os.path.join(HERE, "../../sdk"))
    from faibotic import spec
    import gripper_am as GA
    from build123d import CenterOf
    q = AM.pose("straight"); q["grip"] = grip_urdf(0.0)
    T0 = AM.fk({}); Tq = AM.fk(q)
    def place(link, s):
        D = Tq[link] @ np.linalg.inv(T0[link]); D[:3, 3] *= 1000; return loc(D) * s
    rows, body = [], {b: [0.0, np.zeros(3)] for b in ARM_LINK_BODY.values()}
    def add(b, ten, m, p, nguon):
        p = np.asarray(p, float); body[b][0] += m; body[b][1] = body[b][1] + m * p
        rows.append(dict(ten=ten, body=b, kg=round(m, 5), xyz_m=[round(v, 4) for v in p], nguon=nguon))
    shapes = [(f, link, s) for f, (link, s) in arm_parts_local().items()]
    shapes += [(n, link, s) for n, _, link, s in GA.parts_base(grip_urdf(0.0)) if not n.startswith("M1-CM-001")]
    for n, link, s in shapes:
        sh = place(link, s); c = sh.center(CenterOf.MASS)
        add(ARM_LINK_BODY[link], n, float(sh.volume) * 1.0e-6, np.array([c.X, c.Y, c.Z]) / 1000, "STEP x 1,0 g/cm3 [tự tính]")
    Ty = np.array(T0["left_wrist_yaw"], float).copy(); Ty[:3, 3] *= 1000
    for n, link, s in (("servo yaw STS3215", "left_wrist_yaw", SV.solid("sts3215", Ty @ yaw_servo_pose_link())),
                       ("servo kẹp STS3215", "left_Fixed_Jaw", SV.solid("sts3215", GA.grip_servo_pose_base()))):
        c = place(link, s).center(CenterOf.MASS); add(ARM_LINK_BODY[link], n, 0.0705, np.array([c.X, c.Y, c.Z]) / 1000, "datasheet, vị trí theo hình")
    c = place("left_Fixed_Jaw", GA.cam_module_base()).center(CenterOf.MASS)
    add("hand", "camera cổ tay H65V1 + cáp", 0.025, np.array([c.X, c.Y, c.Z]) / 1000, "[giả định 25 g] như spec")
    org = lambda link: Tq[link][:3, 3]
    for n, b, link, m in (("servo pan STS3095", "mount", "left_Base", 0.1875), ("servo vai STS3095", "shoulder", "left_Rotation_Pitch", 0.1875),
                          ("servo khuỷu STS3095", "upper", "left_Lower_Arm", 0.1875), ("servo gập STS3215", "fore", "left_Wrist_Pitch_Roll", 0.0705),
                          ("servo cuộn STS3215", "yaw", "left_Fixed_Jaw", 0.0705)):
        add(b, n, m, org(link), "datasheet, tại gốc khớp [giả định như spec]")
    for n, b, l0, l1 in (("cáp + ốc cánh tay", "upper", "left_Upper_Arm", "left_Lower_Arm"), ("cáp + ốc cẳng tay", "fore", "left_Lower_Arm", "left_Wrist_Pitch_Roll")):
        add(b, n, 0.030, (org(l0) + org(l1)) / 2, "[giả định] như spec")
    m = sum(v[0] for v in body.values()); c = sum(v[1] for v in body.values()) / m
    mov = sum(v[0] for k, v in body.items() if k != "mount")
    theo_body = {b: dict(cad_kg=round(v[0], 4), spec_kg=spec.LINK_MASS[b], chenh_kg=round(v[0] - spec.LINK_MASS[b], 4)) for b, v in body.items()}
    return dict(ghi_chu=tay_cad.__doc__.split("\n")[0], kg=round(m, 4), quay_theo_pan_kg=round(mov, 4), r_m=round(float(-c[1]), 4), z_tam_m=round(float(c[2]), 4),
                spec_tong_kg=round(spec.ARM_MASS_MOVING + spec.LINK_MASS["mount"], 4), spec_quay_theo_pan_kg=round(spec.ARM_MASS_MOVING, 4),
                chenh_tong_kg=round(m - spec.ARM_MASS_MOVING - spec.LINK_MASS["mount"], 4), theo_body=theo_body, chi_tiet=rows)


def masses(lift=0.0):
    parts = build(lift, "rest")
    fixed = {}; xe = [0.0, np.zeros(3)]; tr = [0.0, np.zeros(3)]; tab = {}
    for n, _, s in parts:
        r = part_mass(n, s)
        if r is None or r[0] <= 0: continue
        m, src = r; c = s.center(); p = np.array([c.X, c.Y, c.Z]) / 1000
        if n.startswith("M1-BS-035_omni"):
            i = int(n.split("omni")[1].split("_")[0]); a = math.radians(WHEEL_ANG[i]); p = np.array([WHEEL_R_MOUNT * math.cos(a), WHEEL_R_MOUNT * math.sin(a), AXLE_Z]) / 1000
        in_tram = n.startswith(("M1-TR-0", "M1-CM-001_head")) and not n.startswith("M1-TR-02")
        moving = in_tram or (n.startswith(("M1-LF-057", "M1-LF-004", "M1-LF-002", "M1-LF-lift_servo", "M1-LF-05", "M1-LF-04", "M1-LF-06", "M1-TR-02"))
                             and not n.startswith(("M1-LF-051", "M1-LF-052", "M1-LF-053", "M1-LF-054", "M1-LF-055", "M1-LF-056")))
        tab[n] = dict(kg=round(m, 4), xyz_m=[round(v, 4) for v in p], nguon=src, xe_nang=moving and not in_tram, tram=in_tram)
        if in_tram: tr[0] += m; tr[1] = tr[1] + m * p
        elif moving: xe[0] += m; xe[1] = xe[1] + m * p
        else: fixed[n] = [m, *p.tolist()]
    o = station_origin(lift) / 1000
    for n, m, pp in TR.EXTRA:      # phần trạm không vẽ (ke góc, ốc, dây trên trạm)
        p = o + np.array(pp) / 1000; tab["M1-TR-extra_" + n] = dict(kg=m, xyz_m=[round(v, 4) for v in p], nguon="[giả định]", xe_nang=False, tram=True)
        tr[0] += m; tr[1] = tr[1] + m * p
    xe_c = xe[1] / xe[0]; fixed["xe_nang"] = [xe[0], *xe_c.tolist()]
    tr_c = tr[1] / tr[0]; fixed["tram_hai_tay"] = [tr[0], *tr_c.tolist()]
    cw = [fixed.pop(k) for k in ("M1-BS-032_cw_hub_plate", "M1-BS-039_cw_plate2")]      # đối trọng = tấm chính + tấm phụ (một khối cho tipover)
    mc = sum(v[0] for v in cw); fixed[CW_KEY] = [mc, *(sum(v[0] * np.array(v[1:]) for v in cw) / mc).tolist()]
    tot = sum(v[0] for v in fixed.values())
    out = dict(R=WHEEL_R_MOUNT / 1000, angles=list(WHEEL_ANG), pan_xy=[-ARM_BASE_U["L"][1] / 1000, ARM_BASE_U["L"][0] / 1000],
               khay_goc_xa=[0.30, 0.15, 0.21], doi_trong_key=CW_KEY, co_dinh=fixed, chi_tiet=tab,
               tong_khong_tay_kg=round(tot, 3), xe_nang_kg=round(xe[0], 3), tram_kg=round(tr[0], 3),
               tram_tam_khung_tram_m=[round(v, 4) for v in (tr_c - o)], tram_tam_khung_xe_m=[round(float(tr_c[0]), 4), round(float(tr_c[1]), 4), round(float(tr_c[2] - o[2]), 4)],
               de_kg=round(sum(v[0] for k, v in fixed.items() if k.startswith(("M1-BS-", "M1-SV-001", "M1-ST-"))), 3))
    return out


INTENDED = {}
_MESH = "bánh răng ăn khớp thanh răng trụ (răng lồng nhau; bánh và trụ đặt theo URDF AlohaMini2, sâu ăn khớp 3,1 mm)"
for _p in ("M1-LF-051_OB_Main_Assembly_Post1", "M1-LF-052_OB_Main_Assembly_Post2", "M1-LF-053_OB_Main_Assembly_Post3"):
    INTENDED[frozenset((_p, "M1-LF-002_lift_gear_z20"))] = _MESH
INTENDED[frozenset(("M1-LF-052_OB_Main_Assembly_Post2", "M1-LF-053_OB_Main_Assembly_Post3"))] = "mối ghép chồng 15 mm dán epoxy giữa hai đoạn trụ (vị trí lắp sẵn trong tệp bố trí upstream, cùng phép đặt)"
INTENDED[frozenset(("M1-LF-053_OB_Main_Assembly_Post3", "M1-LF-054_O_Main_Assembly_Post4"))] = "mối ghép chồng 15 mm giữa Post3 và Post4 (z 307,8..323, như Post1-2, 2-3); boolean OCC lỗi trên cặp này (-3,4), không phải giao lớn (pl_tfix.py)"
INTENDED[frozenset(("M1-LF-051_OB_Main_Assembly_Post1", "M1-LF-052_OB_Main_Assembly_Post2"))] = "mối ghép chồng 15 mm dán epoxy giữa hai đoạn trụ (vị trí lắp sẵn trong tệp bố trí upstream, cùng phép đặt)"
if GRIPPER == "robonine":  # public-scan: ok
    sys.path.insert(0, ROBONINE_DIR); import gripper_robonine as _GR  # public-scan: ok
    for _sd in "LR":    # cặp chủ ý trong kẹp song song cũ (lý do ở mô-đun kẹp cũ, INTENDED)
        INTENDED.update({frozenset((a + "_" + _sd, c + "_" + _sd)): why for (a, c), why in _GR.INTENDED.items()})
for _sd in "LR":    # camera cổ tay trong giá OB_F_Camera (khối bao camera giả định 36 x 36 x 8 + ống kính, đặt theo khung left_camera URDF)
    INTENDED[frozenset((f"M1-AR-027_OB_F_Camera_{_sd}", f"M1-CM-001_wrist_cam_{_sd}"))] = ("bo camera 36 x 36 lọt hốc giá camera upstream; chồng 29 mm3 ở mép do khối bao "
        "camera giả định (đẩy ra 8 mm dọc trục quang vẫn 0,1 mm3: hốc ôm sát bo), gripper_am.cam_module_base")
for _sd in "LR":    # như assembly_full._pairs (CAD gốc AM-ARM200)
    INTENDED[frozenset((f"M1-AR-002_OB_F_J5_Elbow_Wrist_Bracket_{_sd}", f"M1-AR-011_OB_F_J5_Pin_Bearing_{_sd}"))] = "chốt J5 xuyên lỗ giá J5 (CAD gốc AM-ARM200, cặp chủ ý của assembly_full)"
    INTENDED[frozenset((f"M1-AR-011_OB_F_J5_Pin_Bearing_{_sd}", f"M1-AR-004_OB_F_J56_Bottom_{_sd}"))] = "chốt J5 xuyên đáy J56 (CAD gốc AM-ARM200, cặp chủ ý của assembly_full)"


DISPLAY_ONLY = ("M1-LF-lift_servo", "M1-SV-001_gripper")   # servo trục nâng: chỗ trong khung T giữa chưa xác định (PLACE['servo_ghi_chu']);
# servo kẹp: khối bao servos.py đặt theo khớp kẹp URDF, chưa đo hốc thân kẹp J7 (như assembly_full DISPLAY_ONLY "gripper_servo")
DISPLAY_HITS = []


def check(parts, tol=0.5):
    import balance as b
    bbs = [(n, s, s.bounding_box()) for n, _, s in parts]; res = []; DISPLAY_HITS.clear()
    for (n1, s1, b1), (n2, s2, b2) in itertools.combinations(bbs, 2):
        if b1.max.X < b2.min.X or b2.max.X < b1.min.X or b1.max.Y < b2.min.Y or b2.max.Y < b1.min.Y or b1.max.Z < b2.min.Z or b2.max.Z < b1.min.Z:
            continue
        try:
            v = b.inter(s1, s2)[0] or 0.0
        except Exception as e:
            v = -1.0
        if v != v: v = -1.0
        if (v > tol or v < 0) and (n1.startswith(DISPLAY_ONLY) or n2.startswith(DISPLAY_ONLY)): DISPLAY_HITS.append((n1, n2, round(v, 2))); continue
        if v > tol or v < 0: res.append((n1, n2, round(v, 2), intended(n1, n2)))
    return res


def intended(n1, n2):
    k = frozenset((n1, n2))
    if k in INTENDED: return INTENDED[k]
    for (a, b_), why in INTENDED_PREFIX:
        if (n1.startswith(a) and n2.startswith(b_)) or (n2.startswith(a) and n1.startswith(b_)): return why
    return None


INTENDED_PREFIX = []


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--lift", type=float, default=0.0); ap.add_argument("--pose", default="ready"); ap.add_argument("--pan", type=float, default=0.0)
    ap.add_argument("--check", action="store_true"); ap.add_argument("--mesh"); ap.add_argument("--step"); ap.add_argument("--masses", action="store_true")
    ap.add_argument("--grip", type=float, default=0.0, help="độ mở kẹp %% (0 khép, 100 mở mềm)"); ap.add_argument("--roll", type=float, default=0.0)
    ap.add_argument("--ban", action="store_true", help="chế độ bàn: trạm + 2 tay + đế bàn tuỳ chọn"); a = ap.parse_args()
    if a.masses:
        import tipover as TP
        M = masses(); M["tay_cad"] = tay_cad(); M["tong_robot_kg"] = round(M["tong_khong_tay_kg"] + 2 * M["tay_cad"]["kg"], 3)
        p = os.path.join(EXP, "M1-PL-masses.json"); json.dump(M, open(p, "w"), indent=1, ensure_ascii=False)
        cw = M["co_dinh"][M["doi_trong_key"]][0]
        M["he_so_lat"] = {"doi_trong_kg": round(cw, 3), "co_khay_1kg": round(TP.pl_min(M)[0], 3), "khong_tai_khay": round(TP.pl_min(M, tray_load=False)[0], 3),
                          "khong_doi_trong_co_khay": round(TP.pl_min(M, cw=0.0)[0], 3), "doi_trong_can_cho_1_5_kg": TP.pl_cw_needed(M),
                          "gia_toc_0_5_lift_thap_cao": [round(TP.pl_min(M, acc=0.5, z_sh=z)[0], 3) for z in (0.62, 0.95)],
                          # 6+1: tay CAD đủ servo (tay_cad), cùng quy ước khay 1 kg góc xa, tĩnh
                          "tay_cad_kg": M["tay_cad"]["kg"], "tay_cad_r_m": M["tay_cad"]["r_m"],
                          "co_khay_1kg_tay_cad": round(TP.pl_min(M, arm=(M["tay_cad"]["kg"], M["tay_cad"]["r_m"]))[0], 3),
                          "khong_tai_khay_tay_cad": round(TP.pl_min(M, tray_load=False, arm=(M["tay_cad"]["kg"], M["tay_cad"]["r_m"]))[0], 3)}
        json.dump(M, open(p, "w"), indent=1, ensure_ascii=False); print(json.dumps({k: v for k, v in M.items() if k not in ("chi_tiet", "co_dinh")}, ensure_ascii=False)); sys.exit()
    parts = build_ban(a.pose, a.pan, a.grip, a.roll) if a.ban else build(a.lift, a.pose, a.pan, a.grip, a.roll)
    print("n parts", len(parts), "pose", a.pose, "lift", a.lift, flush=True)
    bb = Compound([s for _, _, s in parts]).bounding_box()
    print("BBOX", [round(v, 1) for v in (bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z)], flush=True)
    if a.step: export_step(Compound([s for _, _, s in parts]), a.step)
    if a.mesh:
        os.makedirs(a.mesh, exist_ok=True); man = []
        for n, m, s in parts:
            export_stl(s, os.path.join(a.mesh, n + ".stl"), tolerance=0.15, angular_tolerance=0.3); man.append((n, m))
        json.dump(man, open(os.path.join(a.mesh, "manifest.json"), "w"))
    if a.check:
        r = check(parts); bad = [x for x in r if not x[3]]
        print("KHONG CHU Y:", json.dumps(bad, ensure_ascii=False), "| chu y:", len(r) - len(bad), "| hien thi:", json.dumps(DISPLAY_HITS), flush=True)
        suf = "" if (a.grip == 0 and a.roll == 0) else f"_g{int(a.grip)}_r{int(a.roll)}"
        json.dump(dict(pose=a.pose, lift=a.lift, pan=a.pan, kep=GRIPPER, mo_kep_pct=a.grip, cuon_deg=a.roll, n_parts=len(parts), giao_cat=r, giao_hien_thi=DISPLAY_HITS),
                  open(os.path.join(EXP, (f"M1-PL-ban-check_{a.pose}_{int(a.pan)}" if a.ban else f"M1-PL_check_{a.pose}_{int(a.lift)}_{int(a.pan)}") + suf + ".json"), "w"), ensure_ascii=False, indent=1)
