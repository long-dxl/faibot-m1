# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Faibotic
"""Lắp ráp toàn robot Faibot M1 (v0.4.4 + cột nâng + thân đế nháp + cụm nguồn + camera): đế omni, vỏ pin, bảng đi dây EVT, cột 4040, xe nâng, 2 tay
(vai, khuỷu, cẳng tay M1-AR-021 + ngàm servo gập M1-AR-026, cổ tay J5/J56 + khoá yaw M1-AR-025, kẹp J7), camera đầu và cổ tay. build123d.
 Hệ toạ độ như sdk (mm): x trước, y trái, z lên, gốc sàn. Tay phải = gương tay trái qua mặt y = 0.
v0.4.4: chi tiết AM-ARM200 (Apache-2.0) đặt theo chuỗi khớp URDF upstream, không theo hộp bao J4: gốc = khuỷu URDF quy về hệ STEP (wrist_ref).
Cổ tay gập quanh trục wrist_flex thật (198,4 mm từ trục khuỷu, thấp hơn 3,2 mm); chốt J5 là wrist_yaw, khoá bằng M1-AR-025 (servo yaw bỏ).
Giá J5, chốt J5, vỏ J56, kẹp, ngón, khoá yaw, camera cổ tay quay cùng khớp gập; servo gập + ngàm + cẳng tay cố định theo cẳng tay. Khuỷu: Rot(Y=q) dương = gập xuống.
Chi tiết dựng mới/nháp: thân đế + bánh omni (base_shell.py), vỏ pin M1-PW-016 (battery_box.py, đặt theo fit trong M1-PW-016_battery_box_check.json),
bảng đi dây EVT + K1 (khối bao), M1-AR-021/025/026, M1-LF-024 đế xoay vai, camera M1-CM-001 (khối 36 x 36, chiều sâu [giả định]), giá camera đầu M1-CM-002 (vị trí đề xuất).
Kẹp J7/ngón: STEP upstream là bố cục trưng bày, nâng GRIP_UP/JAW_UP theo z để đồng trục cổ tay [giả định, giữ từ v0.4.3].
Chạy: python assembly_full.py [--lift 200] [--pose ready|reach|rest|straight] [--check] [--mesh DIR] [--step FILE]   (lift mm 0..500)"""
import json, math, os, sys, itertools, argparse, re
import xml.etree.ElementTree as ET
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); EXP = os.path.join(HERE, "../export")
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, "../../sdk"))
from build123d import Box, Cylinder, Pos, Rot, Align, Compound, export_step, export_stl, import_step, mirror, Plane, fillet, Location
import balance as b, failsafe as f, failsafe_v042 as g2, failsafe_v042_low as L, failsafe_v043 as g3, shoulder as sh, lift, base_shell
import servos as SV
from params import LINK_LEN
from faibotic import spec
C = (Align.CENTER,) * 3
UPA = os.path.join(HERE, "..", "upstream", "AM-ARM", "am-arm200")
STP = os.path.join(UPA, "stp", "follower") + os.sep
URDF = os.path.join(UPA, "urdf", "am-arm200.urdf")
SX, SY, D0 = lift.SX, lift.SY, lift.D0
GRIP_UP, JAW_UP, JAW_X, CAM_UP = 72.45, 0.0, 45.0, 0.0   # kẹp dời +72,45 z: chạm J56 Bottom (khoảng cách 0,0) và vỏ J56 (0,12), 0 giao; ngón giữ vị trí STEP; JAW_X: xem README
GRIP_SLIDE = 2.6                 # kẹp tiến sát mặt đầu vỏ J56
# ---- G2B (v0.5): kẹp song song cũ (mô-đun nội bộ) thay kẹp J7 + giá camera M1-AR-009. "am" = kẹp J7 AM-ARM200 (sim v0.4.6).
# (24/11/2026): mặc định "am"; kẹp song song cũ chuyển vào lưu trữ nội bộ (tuỳ chọn tương lai, chờ giấy phép). Bản P-L hiện hành: assembly_pl.py + gripper_am.py. ----
GRIPPER = os.environ.get("M1_GRIPPER", "am")
ROBONINE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "archive", "robonine", "cad")  # public-scan: ok
GRIP_OPEN, GRIP_ROLL = 84.0, 0.0  # khe ngón (mm, 0..84) và góc cuộn cổ tay (độ, ±160) của kẹp cũ; đổi bằng --grip/--roll
HORN_X = 287.5                    # mặt còi servo cuộn (khung cẳng tay) = mặt đầu vỏ J56, chỗ mặt sau thân J7 upstream tựa [tự suy ra]
# ---- hết G2B ----
# tư thế (độ): vai, khuỷu, cổ tay; đều trong ARM_LIMITS và giới hạn ghép (elbow_max, wrist_max)
POSES = {"straight": (0.0, 0.0, 0.0), "ready": (-30.0, 70.0, 10.0), "reach": (-5.0, 30.0, 10.0), "rest": (30.0, 100.0, 20.0)}
# servo STS3215 45,2 x 24,7 x 35 [chưa kiểm, như battery_box.py]; trục ra cách đầu thân 12 mm [giả định]
STS3215 = (45.2, 24.7, 35.0); SHAFT_E = 12.0

def valid(qs, qe, qw):
    r = math.radians; L_ = spec.ARM_LIMITS
    ok = L_["shoulder"][0] <= r(qs) <= L_["shoulder"][1] and L_["elbow"][0] <= r(qe) <= spec.elbow_max(r(qs)) and -100 <= qw <= math.degrees(spec.wrist_max(r(qs), r(qe)))
    return ok

# ---------------- mốc cổ tay theo URDF upstream ----------------
def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])

def _step_cyl(name):
    """Mặt trụ trong STEP (r, điểm trên trục) đọc thẳng từ text STEP."""
    E = {int(m.group(1)): m.group(2) for m in re.finditer(r"#(\d+)\s*=\s*(.*?);", open(STP + name, errors="ignore").read(), re.S)}
    pt = lambda i: np.array([float(x) for x in re.search(r"\(([^()]*)\)\s*\)$", E[i]).group(1).split(",")])
    out = []
    for v in E.values():
        if v.startswith("CYLINDRICAL_SURFACE"):
            ax = [int(x) for x in re.findall(r"#(\d+)", E[int(re.findall(r"#(\d+)", v)[0])])]
            out.append((float(v.rstrip(")").split(",")[-1]), pt(ax[0])))
    return out

_REF = {}
def wrist_ref():
    """Khung URDF upstream (tay trái, tư thế 0) quy về hệ STEP follower: STEP = URDF + t, t lấy từ chốt J5 (= wrist_yaw, x y) và lỗ trục cuộn r 11,52 của vỏ J56
    (= wrist_roll, z), cùng cách tests/test_wrist_upstream.py. Trả: elbow (khuỷu URDF trong hệ STEP = gốc đặt chi tiết upstream), flex, khung camera."""
    if not _REF:
        T = {"left_Base": (np.eye(3), np.zeros(3))}; org = {}
        for j in ET.parse(URDF).getroot().findall("joint"):
            o = j.find("origin"); R, p = T[j.find("parent").get("link")]
            Rj = R @ _rpy(*map(float, o.get("rpy").split())); pj = p + R @ np.array(list(map(float, o.get("xyz").split()))) * 1000
            org[j.get("name")] = pj; T[j.find("child").get("link")] = (Rj, pj)
        pin = _step_cyl("OB_F_J5_Pin_Bearing.stp")[0][1]
        bore = [c[1] for c in _step_cyl("OB_F_J56_Wrist_Housing.stp") if abs(c[0] - 11.523) < 0.01][0]
        yaw, roll = org["left_wrist_yaw_joint"], org["left_wrist_roll"]
        t = np.array([pin[0] - yaw[0], pin[1] - yaw[1], bore[2] - roll[2]])
        _REF.update(t=t, elbow=org["left_elbow_flex"] + t, flex=org["left_wrist_flex"] + t, yaw=yaw + t, roll=roll + t,
                    cam=(T["left_camera"][0], T["left_camera"][1] + t))
        _REF["fore"] = lambda P: np.array([-(P[1] - _REF["elbow"][1]), P[0] - _REF["elbow"][0], P[2] - _REF["elbow"][2]])   # STEP -> khung cẳng tay
    return _REF

_U = {}
def upstream():
    if not _U:
        _U["p"] = {n: import_step(STP + n + ".stp") for n in
                   ("OB_F_J5_Elbow_Wrist_Bracket", "OB_F_J5_Pin_Bearing", "OB_F_J56_Wrist_Housing", "OB_F_J56_Bottom", "OB_F_J56_Top", "OB_F_J7_Gripper", "OB_F_J7_Jaw")}
    return _U

def step_to_fore():
    """Hệ STEP follower -> khung cẳng tay (gốc trục khuỷu, x dọc cẳng tay, y trái, z lên) theo khuỷu URDF (wrist_ref). Thay mốc hộp bao J4 của v0.4.3,
    vốn cao hơn khuỷu URDF 9,6 mm và lệch 2,0 mm dọc, 1,3 mm ngang (nguồn của model.CAD_WRIST_SHIFT)."""
    E = wrist_ref()["elbow"]
    return Rot(Z=90) * Pos(-E[0], -E[1], -E[2])

def flex_axis():
    """(x, z) trục gập cổ tay trong khung cẳng tay: (198,41; -3,16) mm."""
    F = wrist_ref()["fore"](wrist_ref()["flex"]); return float(F[0]), float(F[2])

from params import SERVO_L, SHAFT_FROM_END, CLEAR, BACK_T
# v0.4.5 (22/11/2026): M1-AR-021 rev C kéo về tới mặt ngoài thành sau gá khuỷu M1-AR-020 (x 63,4 = SERVO_L - SHAFT_FROM_END + CLEAR + BACK_T, hệ khuỷu = khung cẳng tay):
# rev B bắt đầu ở x 92,9 nên cách gá khuỷu ~10 mm (lơ lửng, phát hiện khi render). Đầu ống có nút nhôm 8 mm (END_PLUG), bắt 4 x M4 xuyên thành sau gá vào nút [giả định].
FORE_X0, FORE_X1 = SERVO_L - SHAFT_FROM_END + CLEAR + BACK_T, 150.0    # M1-AR-021: từ mặt sau gá khuỷu tới ngàm servo (cắm 4 mm vào M1-AR-026)
END_PLUG = 8.0
FORE_YC, FORE_ZC = -1.6, 1.2      # tâm tiết diện giữ như v0.4.3 ở đầu khuỷu (đã kiểm giao với gá khuỷu)
SEAT_X0, SEAT_X1, SEAT_T = 146.0, 171.0, 3.0   # đuôi servo gập ở x 165,6; thành 3: thành 4 chạm đáy vỏ J56 ở gập 90 (quét tự va chạm)
def forearm_021():
    """M1-AR-021 rev B (NHÁP): nhôm hộp 6061 40 (y) x 30 (z), thành 2 [giả định], x 92,9 -> 150. Bản v0.4.3 kéo tới mặt giá J5 (x 184,8) và giữ giá J5 cứng;
    nay giá J5 quay cùng khớp gập nên cẳng tay dừng trước vùng quét của giá."""
    c = Pos((FORE_X0 + FORE_X1) / 2, FORE_YC, FORE_ZC)
    tube = c * Box(FORE_X1 - FORE_X0, 40.0, 30.0) - c * Box(FORE_X1 - FORE_X0 + 2, 36.0, 26.0)
    return tube + Pos(FORE_X0 + END_PLUG / 2, FORE_YC, FORE_ZC) * Box(END_PLUG, 36.0, 26.0)     # nút đầu ống (bắt gá khuỷu)

FLEX_YC = -3.0   # tâm ngang servo gập (khung cẳng tay); quét gập -90..90 với giá J5 + chốt: 0 mm3 ở y -3 (y -2 và -4 có giao)
def flex_servo_box():
    """Servo gập cổ tay STS3215 (khối bao, M1-SV-001): trục ra song song trục khuỷu tại trục gập URDF; đầu phía cổ tay bo tròn quanh trục (R 12,35 = nửa bề rộng)
    [giả định], thân lùi về khuỷu, dài tổng 45,2. Còi servo bắt giá J5 M1-AR-002 như upstream; khe 0 với giá J5 và chốt khi gập -90..90 (quét)."""
    fx, fz = flex_axis(); Ls, Ws, Hs = STS3215; lb = Ls - Ws / 2
    return Pos(fx - lb / 2, FLEX_YC, fz) * Box(lb, Hs, Ws) + Pos(fx, FLEX_YC, fz) * Rot(X=90) * Cylinder(Ws / 2, Hs)

def seat_026():
    """M1-AR-026 ngàm servo gập cổ tay (NHÁP, PETG): cốc ôm đuôi servo (x 165,6..171) và đầu ống nhôm (x 146..150), thành 3 mm; bắt servo 4 x M2 vào nắp sau,
    bắt ống 2 x M4 xuyên [giả định, cần bản vẽ STS3215]. Mặt trước x 171 cách trục gập 27,4 mm: ngoài vùng quét của giá J5 (kiểm bằng quét gập)."""
    sv = flex_servo_box().bounding_box(); fo = forearm_021().bounding_box()
    y0, y1 = min(sv.min.Y, fo.min.Y) - SEAT_T, max(sv.max.Y, fo.max.Y) + SEAT_T
    z0, z1 = min(sv.min.Z, fo.min.Z) - SEAT_T, max(sv.max.Z, fo.max.Z) + SEAT_T
    s = Pos((SEAT_X0 + SEAT_X1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(SEAT_X1 - SEAT_X0, y1 - y0, z1 - z0)
    c = Pos((FORE_X0 + FORE_X1) / 2, FORE_YC, FORE_ZC)
    return s - flex_servo_box() - c * Box(FORE_X1 - FORE_X0, 40.0, 30.0)

YAW_E, YAW_Z0 = 8.0, 163.7   # hốc servo yaw trong vỏ J56 (hệ STEP): thân dọc +y (về khuỷu), trục cách đầu 8 mm, đáy z 163,7 (mặt trên càng giá J5); tìm bằng quét giao
def yaw_lock_025():
    """M1-AR-025 khoá yaw cổ tay (MỚI, PETG): khối thay servo wrist_yaw trong hốc servo của vỏ J56, trừ đi vỏ J56, nắp J56, giá J5 và chốt (ôm theo hốc, khe 0).
    Bắt vỏ J56 bằng các lỗ bắt servo cũ và bắt càng trên giá J5 bằng 4 vít M3 ở vị trí còi servo [giả định]: giá J5 và vỏ J56 thành một khối, chốt J5 + ổ bi chỉ còn định vị.
    Hệ STEP follower (đặt cùng phép biến đổi với vỏ J56)."""
    U = upstream()["p"]; R = wrist_ref(); px, py = R["yaw"][0], R["yaw"][1]; Ls, Ws, Hs = STS3215
    blk = Pos(px, py + Ls / 2 - YAW_E, YAW_Z0 + Hs / 2) * Box(Ws, Ls, Hs)
    hb = U["OB_F_J56_Wrist_Housing"].bounding_box()
    blk = blk & Pos(0, (hb.min.Y + hb.max.Y) / 2, 0) * Box(1000, hb.max.Y - hb.min.Y, 1000)    # chỉ phần nằm trong vỏ J56 (phần lòi ra giao servo gập)
    for n in ("OB_F_J56_Wrist_Housing", "OB_F_J56_Top", "OB_F_J5_Elbow_Wrist_Bracket", "OB_F_J5_Pin_Bearing"):
        blk = blk - U[n]
    return blk

# camera USB 720p M1-CM-001 (H65V1): bo 36 x 36 [upstream bom.md "720p, 36×36 mm form factor"]; dày bo + linh kiện 8, ống kính Ø15 x 10 [giả định]
CAM, LENS = (36.0, 36.0, 8.0), (15.0, 10.0)
WCAM_K = 0.0   # dời mô-đun camera cổ tay dọc trục quang so với gốc khung left_camera URDF (mm); đặt bằng quét giao, xem cam_wrist
# v0.4.5 (DK-W4, 22/11/2026): camera cổ tay dời trong khung cẳng tay (cổ tay 0, cuộn 0): +WCAM_DX dọc trục cuộn (ra trước), +WCAM_DZ lên (đổi theo cuộn cùng kẹp).
# Vị trí cũ (gốc khung left_camera URDF) nằm trên vỏ J56 (x 274,9..301,8; z 24..60), cách thân kẹp 36 mm, không có giá: vỏ J56 không cuộn nên camera
# đặt ở đó quét quanh vỏ khi cuộn (chạm đáy J56 ở cuộn -110..-125, cam_bracket_check). Mới: cả camera + giá nằm trước mặt đầu vỏ J56 (x >= 290,5, khe 3 tới 287,5), ngay trên thân kẹp, giữ hướng nhìn (chúc 24°).
WCAM_DX, WCAM_DZ = 19.0, -24.0
WCAM_DY = 0.0     # dời ngang (khung cẳng tay +y)
WCAM_DX, WCAM_DY, WCAM_DZ = (float(os.environ.get(k, v)) for k, v in (("M1_WCAM_DX", WCAM_DX), ("M1_WCAM_DY", WCAM_DY), ("M1_WCAM_DZ", WCAM_DZ)))   # thử phương án (DK-W4, cam_cells)
WCAM_XMIN, WCAM_ZMIN = 290.5, -2.0     # ràng buộc dùng để chọn WCAM_DX/DZ (kiểm trong cam_bracket_check)
def camera_module():
    """Mặt sau ở z 0, nhìn theo +z (hệ cục bộ)."""
    return Pos(0, 0, CAM[2] / 2) * Box(*CAM) + Pos(0, 0, CAM[2] + LENS[1] / 2) * Cylinder(LENS[0] / 2, LENS[1])

def _cam_loc():
    Rc, O = wrist_ref()["cam"]
    O = O + Rc[:, 2] * WCAM_K + np.array([WCAM_DY, -WCAM_DX, WCAM_DZ])      # khung cẳng tay +x = STEP -y, +y = STEP +x, +z = STEP +z
    return Location(Plane(origin=tuple(O), x_dir=tuple(Rc[:, 0]), z_dir=tuple(Rc[:, 2])))

def cam_wrist():
    """Camera cổ tay (VỊ TRÍ ĐỀ XUẤT, hệ STEP): hướng theo khung left_camera của URDF upstream (gắn hàm cố định, chúc 24° xuống phía ngón),
    gốc dời WCAM_K dọc trục quang và (WCAM_DX, WCAM_DZ) trong khung cẳng tay. Giá M1-AR-009 Faibotic (cam_bracket_009) thay OB_F_Camera upstream."""
    return _cam_loc() * camera_module()

BRK_T, BRK_FOOT_Y, BRK_FOOT_DX, BRK_FOOT_Z0 = 3.0, (-6.0, 14.0), 16.0, -14.0
def cam_bracket_009():
    """M1-AR-009 giá camera cổ tay (MỚI, PETG, hệ STEP; quay cùng kẹp): tấm tựa 36 x 36 x 3 sau camera (2 x M2 bắt bo camera [giả định]) + chân khối
    từ mép sau-dưới tấm xuống mặt trên thân kẹp J7 (x WCAM_XMIN .. +16, y -6..14 khung cẳng tay), trừ thân kẹp và camera (ôm theo mặt kẹp, khe 0).
    Bắt thân kẹp 2 x M3 tự ren vào nhựa [giả định, cần lỗ trên OB_F_J7_Gripper]. Chân nằm ngoài vùng quay hàm động (y >= -6, hàm ở y <= -13,4 khi mở)."""
    U = upstream()["p"]; S = step_to_fore()
    plate = _cam_loc() * (Pos(0, 0, -BRK_T / 2) * Box(CAM[0], CAM[1], BRK_T))
    pb = (S * plate).bounding_box()
    x0 = max(WCAM_XMIN, pb.min.X); z1 = pb.min.Z + 6.0
    foot_f = Pos(x0, BRK_FOOT_Y[0] + WCAM_DY, BRK_FOOT_Z0) * Box(BRK_FOOT_DX, BRK_FOOT_Y[1] - BRK_FOOT_Y[0], z1 - BRK_FOOT_Z0, align=(Align.MIN, Align.MIN, Align.MIN))
    grip = Pos(0, -GRIP_SLIDE, GRIP_UP) * U["OB_F_J7_Gripper"]
    foot = S.inverse() * foot_f
    return plate + foot - grip - cam_wrist()

HEAD_TILT = 40.0   # chúc xuống, như camera "head" của sim (model.py)
def cam_head():
    """Camera đầu M1-CM-001 + giá M1-CM-002 (VỊ TRÍ ĐỀ XUẤT; SoR/08 chưa có vị trí): tâm mô-đun tại camera "head" của sim (x COLUMN_X + 50, y 0,
    z COLUMN_TOP - 30), nhìn trước, chúc 40°. Giá: tấm Al 3 mm trên đỉnh cột + quai đứng + tấm tựa sau camera (khối)."""
    t = math.radians(HEAD_TILT); look = np.array([math.cos(t), 0.0, -math.sin(t)])
    c = np.array([spec.COLUMN_X * 1000 + 50, 0.0, spec.COLUMN_TOP * 1000 - 30])
    back = c - look * CAM[2] / 2
    loc = Location(Plane(origin=tuple(back), x_dir=(0, -1, 0), z_dir=tuple(look)))
    cam = loc * camera_module()
    plate_back = loc * (Pos(0, 0, -1.5) * Box(CAM[0], CAM[1], 3.0))
    top = spec.COLUMN_TOP * 1000; cx = lift.CX
    edge = back + np.array([math.sin(t), 0, math.cos(t)]) * CAM[1] / 2      # mép trên tấm tựa
    mount = Pos((cx - 20 + edge[0] + 3) / 2, 0, top + 1.5) * Box(edge[0] + 3 - (cx - 20), 40, 3) \
        + Pos(edge[0] - 4.75, 0, (edge[2] - 1 + top) / 2) * Box(6.5, 30, top - edge[2] + 1) + plate_back
    return [("M1-CM-001_head_cam", "servo", cam), ("M1-CM-002_head_mount", "alu", mount)]

# (22/11/2026): bảng đi dây EVT (cx, cy, dài, rộng, góc) trong thân. fit["pcb"] của battery_box.fit_base (-66; 96; 60°) chỉ tính hộp vỏ pin + tai đế, không tính
# tai hạt chèn M4 lồi 9 mm ở thành sau vỏ: hai tai x 150/200 giao khối linh kiện 696 mm3. Quét 2D lại (góc 5°, lưới 2 mm; vỏ pin gồm tai đế + mọi tai hạt chèn;
# tránh vùng bánh + servo bánh như fit_base; cách mặt trong thành thân R 195 >= 5, cách vỏ pin >= 3, cách K1 >= 3): chọn điểm có biên nhỏ nhất lớn nhất.
# Giữ cỡ 150 x 75 x 75 (electrical/bang-di-day-evt.md), chỉ xoay 60° -> 40° và dời 16 mm. Khe 3D đo bằng --clear.
WB_FIT = (-80.0, 104.0, 150.0, 75.0, 40.0)
PW_Z = 40.0   # mặt trên đáy thân M1-BS-020 (z 35 + thành 5); battery_box đặt trên đĩa 4 mm (z 39): dời +1 mm
def power_parts():
    """Cụm nguồn trong thân đế theo archive/v0.4/cad/bao-cao-vo-pin.md (fit trong M1-PW-016_battery_box_check.json): vỏ pin M1-PW-013..016 + linh kiện gộp theo nút điện,
    bảng đi dây EVT (electrical/bang-di-day-evt.md: tấm 150 x 75 x 3 + khối linh kiện cao 75) và K1 (khối bao EV200)."""
    import battery_box as pw
    fit = json.load(open(os.path.join(EXP, "M1-PW-016_battery_box_check.json")))["fit"]
    pw.EARS = fit["ears"]     # tai bắt đế theo kết quả fit (battery_box.main tự chọn; mặc định module là 'y')
    T = Pos(0, 0, PW_Z - pw.BASE_Z0 - pw.T_DECK) * pw.to_base(tuple(fit["pack"])) * Pos(-pw.OUT[0] / 2, -pw.OUT[1] / 2, 0)
    comp = pw.components()
    comp["out_cable"] = comp["out_cable"] & Box(2000, 2000, 2 * (145.0 - PW_Z))   # cáp ra chỉ vẽ tới dưới nắp thân (z 145), hệ hộp: z thế giới - PW_Z
    grp = {"cells": ("cells", "busbar_ends", "mesh_inlet"), "bms": ("bms",), "f1": ("f1",), "d0": ("d0_body",),
           "N1": ("bms_stud_pos", "bar1_v", "bar1_h", "f1_stud_r", "f1_nut_r", "f1_spacer_r"),
           "N2": ("f1_stud_f", "f1_nut_f", "f1_spacer_f", "bar2_v", "bar2_a", "bar2_b", "d0_A_stud", "d0_A_washer", "d0_A_nut"),
           "N3": ("d0_B_stud", "d0_B_washer", "d0_B_nut", "bar3_v", "bar3_h", "ft_head", "ft_stud", "outlug_palm", "outlug_barrel", "out_cable", "ft_washer", "ft_nut"),
           "ins": ("ft_standoff", "ft_bush"), "ntc": ("ntc1", "ntc2")}
    matg = {"cells": "steel", "bms": "servo", "f1": "servo", "d0": "navy", "N1": "steel", "N2": "steel", "N3": "steel", "ins": "servo", "ntc": "servo"}
    out = [("M1-PW-013_battery_box", "ivory", T * pw.build_box()), ("M1-PW-014_battery_lid", "ivory", T * Pos(0, 0, pw.BODY_H) * pw.build_lid()),
           ("M1-PW-015_battery_bay_lid", "ivory_hi", T * Pos(0, 0, pw.BODY_H) * pw.build_bay_lid())]
    for g, names in grp.items():
        out.append((f"M1-PW-016_{g}", matg[g], T * Compound([comp[n] for n in names if n in comp])))
    cx, cy, w, h, th = WB_FIT
    out.append(("M1-WB-017_wiring_plate", "alu", Pos(cx, cy, PW_Z) * Rot(Z=th) * Box(w, h, 3.0, align=(Align.CENTER, Align.CENTER, Align.MIN))))
    out.append(("M1-WB_wiring_parts", "servo", Pos(cx, cy, PW_Z + 3.0) * Rot(Z=th) * Box(w, h, pw.WB_H - 3.0, align=(Align.CENTER, Align.CENTER, Align.MIN))))
    cx, cy, w, h, th = fit["k1"]
    out.append(("M1-PW-004_K1", "servo", Pos(cx, cy, PW_Z) * Rot(Z=th) * Box(w, h, pw.K1[2], align=(Align.CENTER, Align.CENTER, Align.MIN))))
    # v0.4.5: truy cập D0 + LOTO: bộ đổi hướng 90 độ + trục đứng qua nắp thân + núm, tấm khoá trên mặt nắp thân (battery_box.d0_loto)
    d, (sx, sy) = pw.d0_loto()
    out.append(("M1-PW-018_d0_drive", "servo", T * Compound([d["d0_drive"], d["d0_adapter"]]))); out.append(("M1-PW-018_d0_shaft", "steel", T * d["d0_shaft"]))
    ax = T * Pos(sx, sy, 0); X, Y = ax.position.X, ax.position.Y
    assert math.hypot(X - base_shell.D0_SHAFT_XY[0], Y - base_shell.D0_SHAFT_XY[1]) < 0.5, (X, Y, base_shell.D0_SHAFT_XY)
    top = base_shell.Z1
    plate = Pos(X, Y, top) * Box(56, 40, 3, align=(Align.CENTER, Align.CENTER, Align.MIN))
    for k in (-1, 1): plate -= Pos(X + k * 21, Y, top) * Cylinder(4.0, 8)      # 2 lỗ Ø8 cho khoá móc (núm có tai khoá trùng lỗ ở vị trí TẮT) [giả định]
    plate -= Pos(X, Y, top) * Cylinder(6.0, 8)
    out.append(("M1-PW-018_loto_plate", "alu", plate))
    knob = Pos(X, Y, top + 3) * Cylinder(20.0, 14, align=(Align.CENTER, Align.CENTER, Align.MIN)) + Pos(X, Y, top + 3) * Box(52, 8, 4, align=(Align.CENTER, Align.CENTER, Align.MIN))
    for k in (-1, 1): knob -= Pos(X + k * 21, Y, top) * Cylinder(4.0, 20)
    out.append(("M1-PW-018_d0_knob", "navy", knob))
    return out

HUB_R = 30.0
def pan_hub_024():
    """M1-LF-024 đế xoay vai (NHÁP R0, 23/11), hệ vai: trụ Al Ø60 đồng trục pan (x = 0, y = 0, thẳng đứng qua trục vai như SDK), z từ mặt trên tay đỡ
    (-D0 - PAN_DROP = -85, R0 v2) tới đáy chân gá vai (-30); quay cùng gá vai. Bắt vòng bi pan ở mặt dưới. Rãnh: (1) trước x >= -8, |y| <= 9 cho link vai, đệm M1-SH-018 khi vai gập xuống;
    (2) |y| <= 8, z -71..-50,6 cho dầm + tấm lưng giá M1-SH-017 (giá bắt vào rãnh, cùng quay); (3) vát trước x >= 10, z >= -36 cho chốt lò xo M1-SH-012 ở vai 48,5..62 độ."""
    z0, z1 = -D0 - lift.PAN_DROP, f.FOOT_Z0
    h = Pos(0, 0, (z0 + z1) / 2) * Cylinder(HUB_R, z1 - z0)
    h -= Pos((-8 + HUB_R + 1) / 2, 0, (z0 + z1) / 2) * Box(HUB_R + 9, 18, z1 - z0 + 2)
    h -= Pos(0, 0, (-71 + -50.6) / 2) * Box(2 * HUB_R + 2, 16, 71 - 50.6)
    h -= Pos((10 + HUB_R + 1) / 2, 0, (-36 + z1 + 1) / 2) * Box(HUB_R - 9, 2 * HUB_R + 2, z1 + 1 + 36)
    return h

# ---------------- v0.4.6: thân servo thật (servos.py) ----------------
# _SV_LOCAL[tên] = (loại, Location cha, ma trận đặt trong hệ cha, "solid"|"mesh"); SERVO[tên đầy đủ] = (loại, ma trận 4x4 thế giới mm) sau build().
# "solid": hình CAD = khối bao sạch đặt bằng đúng ma trận đó (dùng cho --check); "mesh": hình CAD giữ khối cũ, mesh chỉ hiển thị.
_SV_LOCAL = {}; SERVO = {}
def _sv(P, name, model, parent, M, kind="solid"):
    _SV_LOCAL[name] = (model, parent, M, kind); P.append((name, "servo_body", parent * SV.solid(model, M)))
# Servo chỉ để hiển thị trong --check (giao ghi riêng "giao_hien_thi", không tính lỗi): khối bao Menagerie đặt theo URDF upstream chạm hốc vỏ J56/thân kẹp ở mức
# bề mặt (<= 1 mm: rút khối bao 1 mm còn 91 mm3 vỏ J56, 70 mm3 thân kẹp) và chạm đáy J56/ngón vốn đặt lệch URDF trong CAD (đáy J56 lệch 23,7 mm theo y, ngón là bố cục trưng bày).
# Servo bánh [VỊ TRÍ ĐỀ XUẤT]: servo bánh 2 (phi 300) chạm bộ dẫn D0 M1-PW-018 (v0.4.5) 147 mm3; quét góc thân -20..210 độ không hướng nào hết chạm (ít nhất 63 mm3).
DISPLAY_ONLY = ("wrist_roll_servo", "gripper_servo", "M1-SV-001_wheel")

# Vị trí servo cổ tay/kẹp theo chuỗi khớp URDF AM-ARM200 (cùng commit 5b03af1e): mesh link URDF left_Lower_Arm / left_wrist_yaw có thân STS3215
# (đọc hộp bao từng thành phần trong khung cẳng tay, tư thế 0; xem archive/v0.4/cad/CHANGES.md v0.4.6). Khung cẳng tay: gốc trục khuỷu, x dọc cẳng tay.
FLEX_SEAT_Y = -18.5     # servo gập: mặt tựa còi (còi phía -y như URDF), trong hộp bao cũ (y -20,5..14,5). Mesh thật lệch hộp: còi 0,5 + đầu vít 2,0 ra phía -y,
                        # vấu đáy 1,5 ra phía +y; trục cách đầu 10,2 (hộp giả định 12,35) nên đuôi servo lùi 2,4 mm vào thành ngàm M1-AR-026 (chỉ hiển thị, nằm khuất)
def flex_mesh_pose():
    fx, fz = flex_axis(); return SV.pose("sts3215", (fx, FLEX_SEAT_Y, fz), (0, -1, 0), (-1, 0, 0))
# Servo cuộn đặt theo vỏ J56 như URDF (thân sát đáy vỏ, x 248..284): vỏ J56 CAD = vỏ trong mesh URDF dời (0; -0,8; -5,4) mm, nên servo dời cùng.
# Hệ quả: trục servo thấp hơn trục cuộn CAD (lỗ Ø23 của vỏ J56) 5,4 mm. Đặt đồng trục thì thân servo cắt thành vỏ 2.783 mm3 (rút khối bao 1 mm) nên không phải vị trí thật.
# STEP và URDF upstream lệch nhau ở đây: chờ đo bản in (archive/v0.4/cad/CHANGES.md v0.4.6).
ROLL_SEAT_X, ROLL_DY, ROLL_DZ = 281.1, -0.8, -5.4
def roll_servo_pose():
    """Servo cuộn: trục ra dọc trục cuộn, còi hướng ra kẹp (+x), thân lên trên trục (+z) như URDF (thân z -36,6..8,6 trong khung URDF)."""
    R = wrist_ref()["fore"](wrist_ref()["roll"])
    return SV.pose("sts3215", (ROLL_SEAT_X, R[1] + ROLL_DY, R[2] + ROLL_DZ), (1, 0, 0), (0, 0, 1))
GRIP_SV = (309.15, -24.03, -12.5)     # URDF: trục khớp kẹp (left_gripper) và mặt tựa còi, khung cẳng tay
GRIP_SV_D = (2.6, 0.0, -7.1)          # dời theo thân kẹp CAD so với left_Fixed_Jaw URDF (GRIP_SLIDE và GRIP_UP; hộp bao thân kẹp lệch (2,6; 0; -7,1))
def gripper_servo_pose():
    """Servo kẹp: trục ra dọc z (khớp kẹp URDF), còi lên, thân dọc +y về phía cổ tay như URDF (thân y -34,1..11,1)."""
    p = np.array(GRIP_SV) + np.array(GRIP_SV_D); return SV.pose("sts3215", p, (0, 0, 1), (0, 1, 0))

def base_servos(za):
    """Servo gắn đế/xe nâng (hệ thế giới, mm): trục nâng STS3095 (hộp số), pan STS3215 hai vai (dưới tấm giao diện M1-LF-023), 3 servo bánh STS3215."""
    out = []
    xs = lift.SCX; O_ = (xs, 77.0); zt = 235.0 - 20 + 15      # lift.gearbox: trục Z48 tại O_, mặt trên hộp servo z 230
    out.append(("M1-LF-gear_servo", "sts3095", SV.pose("sts3095", (O_[0], O_[1], zt - SV.S95_HORN_T), (0, 0, 1), (0, 1, 0))))
    top = za - D0 - lift.PAN_DROP       # mặt trên tay đỡ vai / tấm giao diện
    for s in (1, -1):   # pan: trục ra trùng trục pan, còi lên, đầu vít chạm mặt dưới tấm giao diện (z top - 6); thân dưới tay đỡ, hướng về cột [VỊ TRÍ ĐỀ XUẤT]
        out.append((f"M1-LF-carriage_pan_servo{'L' if s > 0 else 'R'}", "sts3215",
                    SV.pose("sts3215", (SX, s * SY, top - 6.0 - (SV.S15_SCREW_Z1 - SV.S15_SEAT_Z)), (0, 0, 1), (-1, 0, 0))))
    for i, a in enumerate(base_shell.WANG):    # bánh: trục ra hướng tâm, còi chạm tấm hub trong (r 150), tâm z = bán kính bánh [VỊ TRÍ ĐỀ XUẤT, vùng battery_box.SERVO_RAD]
        u = np.array([math.cos(math.radians(a)), math.sin(math.radians(a)), 0.0]); t = np.array([-u[1], u[0], 0.0])
        th = math.radians(WHEEL_SV_TH[i]); bb = math.cos(th) * t + math.sin(th) * np.array([0, 0, 1.0])
        r0 = base_shell.WRING - base_shell.WW / 2 - (SV.S15_SCREW_Z1 - SV.S15_SEAT_Z)
        out.append((f"M1-SV-001_wheel{i}", "sts3215", SV.pose("sts3215", r0 * u + np.array([0, 0, base_shell.WZ]), u, bb)))
    return out
WHEEL_SV_TH = (0.0, 0.0, 0.0)   # góc thân servo bánh trong mặt tiếp tuyến (0 = dọc tiếp tuyến, 90 = đứng)

def servo_display(name, detail="sim"):
    """Mesh hiển thị (V mm thế giới, F) của servo `name` trong lần build() gần nhất; None nếu không có mesh (STS3095: tessellate khối CAD)."""
    if name not in SERVO: return None
    model, M = SERVO[name]; return SV.mesh(model, M, detail)

def arm_left():
    """Trả list (tên, vật liệu, hình) trong hệ vai (gốc = trục vai, tay trái)."""
    qs, qe, qw = ARM_Q
    U = upstream()
    P = []
    P.append(("M1-SH-003_cradle_v043", "navy", f.cradle_v4())); P.append(("M1-LF-024_pan_hub", "alu", pan_hub_024()))
    # v0.4.6: thân servo thật (servos.py), vật liệu servo_body; vị trí giữ nguyên hộp bao cũ (sh.servo: còi giữa thân và tấm link)
    yi = sh.LINK_T / 2 + sh.HORN_T
    _sv(P, "servo_L", "sts3095", Location(), SV.pose("sts3095", (0, -yi, 0), (0, 1, 0), (-1, 0, 0)))
    _sv(P, "servo_R", "sts3095", Location(), SV.pose("sts3095", (0, yi, 0), (0, -1, 0), (-1, 0, 0)))
    P.append(("M1-SH-004_link", "alu", b.link_v2(qs)))
    P.append(("M1-SH-012_pin", "steel", b.pin(qs)))
    for i, s in enumerate(b.anchor_bolts()): P.append((f"M1-SH-014_anchor{i}", "steel", s))
    for i, s in enumerate((-1, 1)): P.append((f"spring{i}", "steel", b.spring(qs, s)))
    xt = g2.find_top_xf(); P.append(("M1-SH-016_top_bar", "alu", g2.top_bar(xt))); P.append(("M1-SH-015_top_pad", "pad", g2.top_pad(xt)))
    pad, br, _ = L.shapes(45.0, 50.0); P.append(("M1-SH-017_low_stop", "alu", br)); P.append(("M1-SH-018_low_pad", "pad", pad))
    lt = Rot(Y=qs)
    tab, rp = g2.elbow_rev(); P.append(("M1-AR-023_rev_tab", "alu", lt * tab))
    for i, s in enumerate(rp): P.append((f"M1-AR-023_rev_pad{i}", "pad", lt * s))
    import failsafe_v045 as g5      # v0.4.5: vít/đai ốc M3 của tab (M1-HW-008), mô hình kích thước chuẩn
    for k, s in g5.hardware().items(): P.append((f"M1-HW-008_{k}", "steel", lt * s))
    P.append(("M1-AR-022_elbow_pad", "pad", lt * g3.pad_pu())); P.append(("M1-AR-024_pad_base", "alu", lt * g3.pad_base()))
    Fm = Rot(Y=qs) * Pos(LINK_LEN, 0, 0) * Rot(Y=qe)
    ea = b.elbow_assembly()
    P.append(("M1-AR-020_elbow_cradle", "navy", Fm * ea["e_cradle"])); _sv(P, "elbow_servoL", "sts3095", Fm, SV.pose("sts3095", (0, -yi, 0), (0, 1, 0), (1, 0, 0)))     # = gương sh.servo qua mặt x = 0 (balance.elbow_assembly)
    _sv(P, "elbow_servoR", "sts3095", Fm, SV.pose("sts3095", (0, yi, 0), (0, -1, 0), (1, 0, 0)))
    P.append(("M1-AR-021_forearm", "alu", Fm * forearm_021())); P.append(("M1-AR-026_flex_seat", "navy", Fm * seat_026()))
    # servo gập: khối bao cũ flex_servo_box() giữ cho quét (quét cổ tay và ngàm M1-AR-026 dựng theo nó); mesh STS3215 chỉ để hiển thị (FLEX_MESH)
    P.append(("wrist_flex_servo", "servo_body", Fm * flex_servo_box())); _SV_LOCAL["wrist_flex_servo"] = ("sts3215", Fm, flex_mesh_pose(), "mesh")
    fx, fz = flex_axis()
    W = Fm * Pos(fx, 0, fz) * Rot(Y=qw) * Pos(-fx, 0, -fz)      # gập quanh trục wrist_flex URDF
    fr = W * step_to_fore()
    mat = {"OB_F_J5_Elbow_Wrist_Bracket": "ivory", "OB_F_J5_Pin_Bearing": "steel", "OB_F_J56_Wrist_Housing": "ivory", "OB_F_J56_Bottom": "navy", "OB_F_J56_Top": "navy",
           "OB_F_J7_Gripper": "ivory", "OB_F_J7_Jaw": "navy"}
    for n, s in U["p"].items():
        if GRIPPER == "robonine" and n.startswith("OB_F_J7_"): continue     # G2B: kẹp J7 (M1-AR-007/008) bỏ ở v0.5  public-scan: ok
        sh_ = Pos(0, -GRIP_SLIDE, GRIP_UP) * s if n == "OB_F_J7_Gripper" else (Pos(JAW_X, -GRIP_SLIDE, JAW_UP) * s if n == "OB_F_J7_Jaw" else s)
        P.append((n, mat[n], fr * sh_))     # giá J5 + chốt + vỏ J56 + kẹp + ngón: cùng quay quanh trục gập (yaw khoá)
    if "lock" not in _U: _U["lock"] = yaw_lock_025()
    P.append(("M1-AR-025_yaw_lock", "navy", fr * _U["lock"]))
    _sv(P, "wrist_roll_servo", "sts3215", W, roll_servo_pose())      # v0.4.6: servo cuộn trong vỏ J56 (đi cùng khớp gập, còi quay kẹp); DISPLAY_ONLY
    if GRIPPER == "robonine":       # G2B: M1-GR-001 (kẹp song song cũ) + gá M1-GR-002 + servo kẹp + camera H65V1 trên giá camera của kẹp cũ; quay theo cuộn GRIP_ROLL  public-scan: ok
        sys.path.insert(0, ROBONINE_DIR); import gripper_robonine as GR  # public-scan: ok
        ra = wrist_ref()["fore"](wrist_ref()["roll"]); ax = (float(ra[1]), float(ra[2]))
        Mg = GR.g_to_fore(ax, HORN_X); Wr = W * GR.roll_loc(GRIP_ROLL, ax); Lg = Wr * SV.m2loc(Mg)
        for n, m, s in GR.parts_g(GRIP_OPEN): P.append((n, m, Lg * s))
        _sv(P, "gripper_servo", "sts3215", Wr, Mg @ GR.servo_pose_g())    # DISPLAY_ONLY: khối bao servos.py chồng hốc khung/giá kẹp cũ (theo vỏ Feetech thật) 280/356 mm3
        return P
    _sv(P, "gripper_servo", "sts3215", W, gripper_servo_pose())      # v0.4.6: servo kẹp trong thân kẹp J7 (đi cùng khớp cuộn); DISPLAY_ONLY
    P.append(("M1-CM-001_wrist_cam", "servo", fr * cam_wrist()))
    if "brk" not in _U: _U["brk"] = cam_bracket_009()
    P.append(("M1-AR-009_wrist_cam_bracket", "navy", fr * _U["brk"]))
    return P

def build(lift_mm, q):
    global ARM_Q; ARM_Q = q
    za = lift.Z_AX0 + lift_mm
    out = list(base_shell.parts()) + power_parts() + cam_head()
    out += [("M1-LF-010_column", "alu", lift.column()), ("M1-LF-011_flange", "alu", lift.flange()), ("M1-LF-screw", "steel", lift.screw_parts()), ("M1-LF-030_top_block", "alu", lift.top_block())]
    t, bb, bp = lift.stops(); out += [("M1-LF-stop_top", "pad", t), ("M1-LF-stop_bot", "alu", bb), ("M1-LF-stop_botpad", "pad", bp)]
    for i, r in enumerate(lift.rails()): out.append((f"M1-LF-rail{i}", "steel", r))
    for i, r in enumerate(lift.blocks(za)): out.append((f"M1-LF-block{i}", "steel", r))
    for n, s in lift.carriage(za).items(): out.append((f"M1-LF-carriage_{n}", "alu", s))
    SERVO.clear()
    for n, s in lift.gearbox().items():
        if n != "servo": out.append((f"M1-LF-gear_{n}", {"plate": "alu", "belt": "servo"}.get(n, "steel"), s))
    for n, model, M in base_servos(za):         # v0.4.6: thân servo thật thay hộp servo trục nâng; thêm pan + bánh
        SERVO[n] = (model, M); out.append((n, "servo_body", SV.solid(model, M)))
    _SV_LOCAL.clear(); L_ = arm_left()
    for side in (1, -1):
        T = Pos(SX, side * SY, za); Tm = SV.loc2m(T)
        for n, m, s in L_:
            nn = f"{n}_{'L' if side > 0 else 'R'}"
            if n in _SV_LOCAL:
                model, par, M, kind = _SV_LOCAL[n]; Ml = SV.loc2m(par) @ M
                Mw = Tm @ (Ml if side > 0 else SV.mirror_proper(Ml)); SERVO[nn] = (model, Mw)
                if kind == "solid": out.append((nn, m, SV.solid(model, Mw))); continue
            s2 = mirror(s, about=Plane.XZ) if side < 0 else s
            out.append((nn, m, T * s2))
    # R0 (23/11): bỏ rãnh cấp lắp ráp cho M1-SH-017/018; đã sửa ở cấp chi tiết (tay đỡ M1-LF-023 hạ PAN_DROP trong lift.py, đế xoay M1-LF-024 có rãnh, xem pan_hub_024)
    # chỉnh cấp lắp ráp còn lại (ghi trong README): tấm hộp số M1-LF-040 dời lên 3 mm (nằm trên mặt tấm đế cột z 178) và khoét lỗ cột
    for i, (n, m, s) in enumerate(out):
        if n == "M1-LF-gear_plate":
            out[i] = (n, m, Pos(0, 0, 3.0) * s - Pos(lift.CX, 0, 180) * Box(40.2, 40.2, 20))
    return out

def _pairs():
    P = set()
    def add(a, bs):
        for x in bs: P.add(frozenset((a, x)))
    add("M1-LF-screw", ["M1-LF-030_top_block", "M1-LF-stop_top", "M1-LF-stop_bot", "M1-LF-stop_botpad", "M1-LF-carriage_nut", "M1-LF-carriage_safe_nut", "M1-LF-gear_pulley16", "M1-LF-gear_belt"])
    add("M1-LF-rail0", ["M1-LF-block0", "M1-LF-block1"]); add("M1-LF-rail1", ["M1-LF-block2", "M1-LF-block3"])
    add("M1-LF-carriage_sideR", ["M1-LF-block0", "M1-LF-block1", "M1-LF-carriage_bridge"]); add("M1-LF-carriage_sideL", ["M1-LF-block2", "M1-LF-block3", "M1-LF-carriage_bridge"])
    add("M1-LF-carriage_bridge", ["M1-LF-carriage_nut", "M1-LF-carriage_safe_nut"]); add("M1-LF-carriage_outR", ["M1-LF-carriage_padR"]); add("M1-LF-carriage_outL", ["M1-LF-carriage_padL"])
    add("M1-LF-010_column", ["M1-LF-011_flange"]); add("M1-LF-gear_zgear", ["M1-LF-gear_servo"]); add("M1-LF-gear_pulley64", ["M1-LF-gear_belt"]); add("M1-LF-gear_pulley16", ["M1-LF-gear_belt"])
    for sd in "LR":
        add(f"M1-SH-012_pin_{sd}", [f"spring0_{sd}", f"spring1_{sd}"]); add(f"M1-SH-014_anchor0_{sd}", [f"spring0_{sd}"]); add(f"M1-SH-014_anchor1_{sd}", [f"spring1_{sd}"])
        add(f"M1-SH-017_low_stop_{sd}", [f"M1-SH-018_low_pad_{sd}"])
        add(f"OB_F_J5_Elbow_Wrist_Bracket_{sd}", [f"OB_F_J5_Pin_Bearing_{sd}"]); add(f"OB_F_J5_Pin_Bearing_{sd}", [f"OB_F_J56_Bottom_{sd}"])
    return P
# v0.4.4: cặp mới, mỗi cặp một lý do (không nới cặp cũ). Tên đầy đủ.
NEW_INTENDED = {
    frozenset(("M1-PW-014_battery_lid", "M1-PW-015_battery_bay_lid")): "mép nắp chính chồng mép nắp khoang F1/D0 (cặp chủ ý ('lid','bay_lid') của battery_box.py, hai nắp in riêng, ép gioăng)",
}
# ---- G2B: cặp chủ ý trong kẹp song song cũ (lý do ở mô-đun kẹp cũ, INTENDED) ----
if GRIPPER == "robonine":  # public-scan: ok
    sys.path.insert(0, ROBONINE_DIR); import gripper_robonine as _GR  # public-scan: ok
    for _sd in "LR": NEW_INTENDED.update({frozenset((a + "_" + _sd, c + "_" + _sd)): why for (a, c), why in _GR.INTENDED.items()})
# ---- hết G2B ----
INTENDED_EXACT = _pairs()    # tên đầy đủ, không khớp tiền tố: chồng chủ ý của CAD gốc (khối liền, chốt xuyên lỗ, lò xo móc chốt, đệm lõm 4 mm trên dầm chặn dưới)
def intended(n1, n2):
    k = frozenset((n1, n2))
    return "chủ ý (CAD gốc)" if k in INTENDED_EXACT else NEW_INTENDED.get(k)

DISPLAY_HITS = []
def interference(parts, tol=0.5):
    bbs = [(n, s, s.bounding_box()) for n, _, s in parts]
    res = []; DISPLAY_HITS.clear()
    for (n1, s1, b1), (n2, s2, b2) in itertools.combinations(bbs, 2):
        disp = n1.startswith(DISPLAY_ONLY) or n2.startswith(DISPLAY_ONLY)
        if b1.max.X < b2.min.X or b2.max.X < b1.min.X or b1.max.Y < b2.min.Y or b2.max.Y < b1.min.Y or b1.max.Z < b2.min.Z or b2.max.Z < b1.min.Z:
            continue
        try:
            v = b.inter(s1, s2)[0] or 0.0
        except Exception as e:
            v = -1.0; print("loi", n1, n2, repr(e)[:80])
        if v != v: v = -1.0
        if (v > tol or v < 0) and disp: DISPLAY_HITS.append((n1, n2, round(v, 2))); continue
        if v > tol or v < 0: res.append((n1, n2, round(v, 2), intended(n1, n2)))
    return res

if __name__ == "__main__":
    if "--pl" in sys.argv:      # v0.5 cấu hình P-L (23/11/2026): lắp ráp ở assembly_pl.py; file này giữ bản v0.4.6 làm lịch sử
        sys.argv.remove("--pl"); import runpy; runpy.run_path(os.path.join(HERE, "assembly_pl.py"), run_name="__main__"); sys.exit()
    ap = argparse.ArgumentParser(); ap.add_argument("--lift", type=float, default=200.0); ap.add_argument("--pose", default="ready"); ap.add_argument("--check", action="store_true")
    ap.add_argument("--mesh"); ap.add_argument("--step"); ap.add_argument("--parts", action="store_true"); ap.add_argument("--clear", action="store_true"); ap.add_argument("--d0", action="store_true")
    ap.add_argument("--grip", type=float, default=GRIP_OPEN); ap.add_argument("--roll", type=float, default=GRIP_ROLL)   # G2B: khe ngón mm, cuộn độ (kẹp cũ)
    a = ap.parse_args(); GRIP_OPEN, GRIP_ROLL = a.grip, a.roll
    if a.parts:     # v0.4.4: xuất chi tiết mới (khung cẳng tay tay trái: gốc trục khuỷu, x dọc cẳng tay; giá camera đầu: hệ thế giới) + thể tích
        S = step_to_fore(); hm = {n: s for n, _, s in cam_head()}
        P = {"M1-AR-021_forearm_v044": forearm_021(), "M1-AR-025_yaw_lock": S * yaw_lock_025(), "M1-AR-026_flex_seat": seat_026(),
             "M1-CM-002_head_mount": hm["M1-CM-002_head_mount"]}
        out = {}
        for n, s in P.items():
            export_step(s, os.path.join(EXP, n + ".step")); bb = s.bounding_box()
            out[n] = dict(the_tich_cm3=round(s.volume / 1000, 2), hop_bao_mm=[round(v, 1) for v in (bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z)])
        R = wrist_ref(); F = R["fore"]
        out["moc_khung_cang_tay_mm"] = dict(gap=list(np.round(F(R["flex"]), 2)), chot_J5=list(np.round(F(R["yaw"]), 2)), truc_cuon=list(np.round(F(R["roll"]), 2)),
                                           khuyu_URDF_trong_STEP=list(np.round(R["elbow"], 2)), t_URDF_den_STEP=list(np.round(R["t"], 3)))
        json.dump(out, open(os.path.join(EXP, "M1-AR-v044_parts.json"), "w"), indent=1, ensure_ascii=False, default=float); print(json.dumps(out, ensure_ascii=False, default=float)); sys.exit()
    q = POSES[a.pose]; assert valid(*q), q
    parts = build(a.lift, q)
    print("n parts", len(parts), "pose", q, "lift", a.lift, flush=True)
    if a.step: export_step(Compound([s for _, _, s in parts]), a.step)
    if a.mesh:
        os.makedirs(a.mesh, exist_ok=True); man = []
        for n, m, s in parts:
            dm = servo_display(n, "full")      # v0.4.6: STS3215 xuất mesh Menagerie đầy đủ (render), không phải khối bao
            if dm is not None:
                import trimesh; trimesh.Trimesh(*dm, process=False).export(os.path.join(a.mesh, n + ".stl"))
            else:
                export_stl(s, os.path.join(a.mesh, n + ".stl"), tolerance=0.08, angular_tolerance=0.2)
            man.append((n, m))
        json.dump(man, open(os.path.join(a.mesh, "manifest.json"), "w"))
    if a.clear:   # khe 3D (mm) bảng đi dây ↔ vỏ pin, thân, K1
        P = {n: sh_ for n, _, sh_ in parts}
        wb = Compound([P["M1-WB-017_wiring_plate"], P["M1-WB_wiring_parts"]])
        box = Compound([P[n] for n in P if n.startswith("M1-PW-01")])
        zf = base_shell.Z0 + base_shell.WALL      # thành thân (vành trụ trên mặt đáy; đáy là mặt tựa của bảng, khe 0 chủ ý)
        shell = P["M1-BS-001_base_shell"] & Pos(0, 0, (zf + 0.5 + base_shell.CAP_Z0) / 2) * Box(500, 500, base_shell.CAP_Z0 - zf - 0.5); other = Compound([sh_ for n, _, sh_ in base_shell.parts() if n != "M1-BS-001_base_shell"])
        res = dict(bang_vo_pin=round(wb.distance_to(box), 2), bang_thanh_than=round(wb.distance_to(shell), 2), bang_banh_nap_cua=round(wb.distance_to(other), 2),
                   bang_K1=round(wb.distance_to(P["M1-PW-004_K1"]), 2), vo_pin_thanh_than=round(box.distance_to(shell), 2), WB_FIT=WB_FIT)
        print("KHE", json.dumps(res), flush=True)
    if a.d0:      # v0.4.5: phương án truy cập D0: (a) cần ngang theo trục D0 (~82 mm) và (b) trục đứng qua nắp thân
        import battery_box as pw
        fit = json.load(open(os.path.join(EXP, "M1-PW-016_battery_box_check.json")))["fit"]
        T = Pos(0, 0, PW_Z - pw.BASE_Z0 - pw.T_DECK) * pw.to_base(tuple(fit["pack"])) * Pos(-pw.OUT[0] / 2, -pw.OUT[1] / 2, 0)
        horiz = T * pw.cyl_y(pw.X_D0, pw.ZC, -130.0, 0.0, 10.0)
        P = {n: sh_ for n, _, sh_ in parts}
        bs = {n: P[n] for n in P if n.startswith("M1-BS-")}
        hit = {n: round(float((horiz & o).volume), 1) for n, o in bs.items() if (horiz & o) is not None and float((horiz & o).volume) > 0.5}
        pend = (T * Pos(pw.X_D0, -130.0, pw.ZC)).position; ph = math.degrees(math.atan2(pend.Y, pend.X))
        vert = Compound([P[n] for n in P if n.startswith("M1-PW-018")])
        wheels = Compound([P[n] for n in P if n.startswith("M1-BS-010")])
        others = Compound([P[n] for n in P if not n.startswith(("M1-PW-018", "M1-BS-002", "M1-BS-001", "M1-PW-01")) and not n.endswith(("_L", "_R")) and not n.startswith("M1-LF")])
        sx, sy = base_shell.D0_SHAFT_XY
        res = dict(a_can_ngang=dict(giao_than_banh_mm3=hit, phi_huong_deg=round(ph, 1)),
                   b_truc_dung=dict(xy=[sx, sy], r_mm=round(math.hypot(sx, sy), 1), phi_deg=round(math.degrees(math.atan2(sy, sx)), 1),
                                    khe_toi_banh=round(vert.distance_to(wheels), 2), khe_toi_thanh_than=round(vert.distance_to(P["M1-BS-001_base_shell"]), 2),
                                    khe_toi_vo_pin=round(vert.distance_to(Compound([P[n] for n in P if n.startswith(("M1-PW-013", "M1-PW-014", "M1-PW-015"))])), 2),
                                    khe_toi_chi_tiet_khac=round(vert.distance_to(others), 2)))
        print("D0", json.dumps(res, ensure_ascii=False), flush=True)
        json.dump(res, open(os.path.join(EXP, "d0_access_v045.json"), "w"), ensure_ascii=False, indent=1)
    if a.check:
        r = interference(parts); bad = [x for x in r if not x[3]]; print("KHONG CHU Y:", json.dumps(bad, ensure_ascii=False), "| chu y:", len(r) - len(bad), "| hien thi:", json.dumps(DISPLAY_HITS), flush=True); json.dump(dict(pose=q, lift=a.lift, kep=GRIPPER, khe_ngon_mm=GRIP_OPEN, cuon_deg=GRIP_ROLL, giao_cat=r, giao_hien_thi=DISPLAY_HITS, ghi_chu_hien_thi="servo cuộn/kẹp (DISPLAY_ONLY): không tính lỗi, xem assembly_full.DISPLAY_ONLY"), open(os.path.join(EXP, f"M1-full_assembly_check_{a.pose}_{int(a.lift)}" + (f"_g{int(GRIP_OPEN)}_r{int(GRIP_ROLL)}" if GRIPPER == "robonine" else "") + ".json"), "w"), ensure_ascii=False, indent=1)  # public-scan: ok
