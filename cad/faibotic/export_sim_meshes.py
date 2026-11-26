# SPDX-License-Identifier: CC-BY-NC-SA-4.0
# Copyright (c) 2026 Leo Dao (Faibotic)
"""Xuất mesh rút gọn cho sim MuJoCo (sim/assets/) từ lắp ráp CAD v0.4.x (assembly_full.py), gom theo body MJCF và vật liệu.
 Chi tiết tay từ AM-ARM (Apache-2.0): xem sim/assets/NOTICE.
Cách làm giống luồng web (nay là archive/v0.4/web-assets/legacy-web-sim/src/01_export_parts.py, 02_build_glb.py) nhưng không import các file đó:
  1. dựng lắp ráp ở tư thế 0 (tay duỗi ngang), lift 0; tessellate từng chi tiết;
  2. gán chi tiết -> body MJCF (base, carriage, <bên>_shoulder/upper/fore/wrist/hand/finger_a), trừ gốc body theo spec.py (mét);
  3. gộp theo (body, vật liệu) -> một STL nhị phân; ghi meshes.json (danh sách geom hiển thị cho sdk/faibotic/model.py);
  4. dựng lại ở tư thế ready, lift 200 và ghi cad_check.json: tâm từng chi tiết (hệ body ở tư thế 0, và hệ thế giới ở tư thế ready)
     để tests/test_sim_meshes.py so FK của MJCF với CAD.
Gốc khớp theo SDK (spec.py); trục cuộn cổ tay trên trục cổ tay, kẹp và ngón giữ đúng vị trí CAD so với cổ tay.
v0.4.4: cụm cổ tay CAD đặt theo URDF upstream (assembly_full.wrist_ref) nên model.CAD_WRIST_SHIFT = 0 với mesh xuất từ bản này.
Thêm: khoá yaw M1-AR-025 -> wrist, ngàm M1-AR-026 + servo gập -> fore, camera cổ tay -> hand, vỏ pin/bảng đi dây/K1/camera đầu + giá -> base.
v0.4.5: thêm giá camera cổ tay M1-AR-009 -> hand (camera ở vị trí mới WCAM_DX/DY/DZ), ốc tab M1-HW-008 -> upper.
v0.4.6: thân servo thật, vật liệu servo_body: STS3095 (vai, khuỷu, trục nâng) tessellate từ khối sạch servos.sts3095(); STS3215 (pan -> carriage,
gập -> fore, cuộn -> wrist, kẹp -> hand, 3 bánh -> base) lấy mesh Menagerie rút gọn (assembly_full.servo_display) thay cho khối bao CAD.
Bỏ: lò xo cân bằng (hình đổi theo góc vai, sim vẽ bằng tendon).
Khung body (frames) : gốc wrist/hand trùng pos body trong sim/m1.xml (gồm WF_DZ, ROLL_DY, ROLL_DZ).
Chạy (venv build123d 0.13 + trimesh): python cad/faibotic/export_sim_meshes.py [--out sim/assets] [--tol 0.35] [--labels parts_zero.npz]
S1C (P-L): `--pl` xuất tay AM-ARM200 GỐC từ STEP follower upstream (không qua assembly_full.build: bản đó còn tay ghép đôi P-A) + kẹp
(kẹp gốc AM-ARM200 gripper_am.parts_base, hàm ở góc khép -> body <bên>_jaw gốc tại trục kẹp; trước kẹp cũ) theo khung body của sdk/faibotic/model.py ở tư thế 0 SDK; đặt chi tiết bằng FK URDF upstream (STEP = URDF + t,
t của assembly_full.wrist_ref), kiểm tâm khối từng link với inertial URDF. Tay phải = gương y của tay trái. Ghi meshes.json với source.variant = "P-L" (model.load_meshes chỉ nhận bản này).
S1G: thêm thân P-L từ assembly_pl.py ở lift SDK 0 (= lift CAD spec.LIFT_CAD_OFFSET) vào body base/carriage, đế J1 (carriage) và khung xoay J2
(shoulder) đặt như assembly_pl.arm_parts_local; kiểm gốc tay CAD (khớp URDF AlohaMini2) trùng gốc tay SDK; ghi frames + nhãn (--labels) cho web-assets."""
import os, sys, re, json, struct, argparse, time, hashlib
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(REPO, "sdk"))
import assembly_full as A
A.GRIPPER = "am"   # G2B: công cụ này dựng cho kẹp J7 AM-ARM200 (v0.4.x); kẹp cũ v0.5 chưa vào sim/web (C1, C2)
from faibotic import spec

READY = ("ready", 200.0)          # tư thế so khớp: assembly_full POSES["ready"], lift 200 mm

def body_of(n):
    """Tên chi tiết assembly_full -> body MJCF (model.py). None = bỏ."""
    if n.startswith("spring"): return None
    if n.startswith(("M1-LF-block", "M1-LF-carriage_", "M1-LF-024_pan_hub")): return "carriage"      # gồm servo pan M1-LF-carriage_pan_servo* (v0.4.6)
    m = re.match(r"^(.*)_(L|R)$", n)
    if not m or n.startswith("M1-LF-"): return "base"          # thân đế, bánh, cột, ray, vitme, hộp số
    b, s = m.group(1), "left" if m.group(2) == "L" else "right"
    if b.startswith("spring"): return None
    if b.startswith(("M1-SH-003", "servo_", "M1-SH-016", "M1-SH-015", "M1-SH-017", "M1-SH-018", "M1-SH-014")): return s + "_shoulder"
    if b.startswith(("M1-SH-004", "M1-SH-012", "M1-AR-023", "M1-AR-022", "M1-AR-024", "M1-HW-008")): return s + "_upper"
    if b.startswith(("M1-AR-020", "elbow_servo", "M1-AR-021", "M1-AR-026", "wrist_flex_servo", "OB_F_J5_")): return s + "_fore"
    if b.startswith(("OB_F_J56_", "M1-AR-025", "wrist_roll_servo")): return s + "_wrist"
    if b.startswith(("OB_F_J7_Gripper", "M1-CM-001_wrist_cam", "M1-AR-009", "gripper_servo")): return s + "_hand"
    if b.startswith("OB_F_J7_Jaw"): return s + "_finger_a"
    raise SystemExit("chua gan body: " + n)

def frames():
    """Gốc body MJCF ở tư thế 0, lift 0 (m, hệ thế giới; mọi body cùng hướng thế giới ở tư thế 0)."""
    z0 = spec.ARM_BASE_Z0
    F = {"base": (0.0, 0.0, 0.0), "carriage": (0.0, 0.0, z0)}
    for s, k in (("left", 1), ("right", -1)):
        x, y = spec.SHOULDER_X, k * spec.SHOULDER_Y
        F[s + "_shoulder"] = (x, y, z0); F[s + "_upper"] = (x, y, z0 + spec.D0)
        # trục gập thấp hơn đường trục khuỷu WF_DZ; gốc khớp cuộn lệch (LW, ±ROLL_DY, ROLL_DZ) so với trục gập (tay phải lấy gương y),
        # trùng pos body trong sim/m1.xml (model._frame_in_fore) để web-assets/src/10_build_m1_glb.py kiểm khung đạt.
        zf = z0 + spec.D0 + spec.WF_DZ
        F[s + "_fore"] = (x + spec.L1, y, z0 + spec.D0); F[s + "_wrist"] = (x + spec.L1 + spec.L2, y, zf)
        F[s + "_hand"] = (x + spec.L1 + spec.L2 + spec.LW, y + k * spec.ROLL_DY, zf + spec.ROLL_DZ); F[s + "_finger_a"] = F[s + "_hand"]
    return {k: np.array(v) for k, v in F.items()}

def tess(s, tol, ang):
    for k in (1.0, 0.5, 0.25, 0.1):        # vỏ J56 upstream lỗi ở dung sai thô: thử mịn dần (như 01_export_parts.py)
        try:
            v, f = s.tessellate(tol * k, ang); return np.array([[p.X, p.Y, p.Z] for p in v]), np.array(f, dtype=np.int64)
        except Exception:
            pass
    V, Fc = [], []
    for fc in s.faces():                    # solid hỏng: tessellate từng mặt, bỏ mặt lỗi
        try:
            fv, ff = fc.tessellate(tol, ang)
        except Exception:
            continue
        Fc += [(a + len(V), b + len(V), c + len(V)) for a, b, c in ff]; V += [[p.X, p.Y, p.Z] for p in fv]
    return np.array(V, float).reshape(-1, 3), np.array(Fc, dtype=np.int64).reshape(-1, 3)

def write_stl(path, v, f):
    tri = v[f].astype(np.float32)
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); L = np.linalg.norm(n, axis=1, keepdims=True); n = (n / np.maximum(L, 1e-12)).astype(np.float32)
    rec = np.zeros(len(f), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]); rec["n"] = n; rec["v"] = tri
    with open(path, "wb") as fh:
        fh.write(b"Faibotic M1 sim mesh (m, body frame)".ljust(80, b" ")); fh.write(struct.pack("<I", len(f))); fh.write(rec.tobytes())

def clean(v, f):
    """Gộp đỉnh trùng, bỏ tam giác suy biến (MuJoCo báo lỗi mặt diện tích 0)."""
    import trimesh
    m = trimesh.Trimesh(v, f, process=True); m.update_faces(m.nondegenerate_faces(height=1e-6)); m.remove_unreferenced_vertices()
    return np.asarray(m.vertices), np.asarray(m.faces)

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", default=os.path.join(REPO, "sim", "assets")); ap.add_argument("--tol", type=float, default=0.35)
    ap.add_argument("--ang", type=float, default=0.4)
    ap.add_argument("--labels", default=None, help="ghi thêm npz (names, mats, bbs mm, thế giới, tư thế 0) để web-assets gán nhãn cụm/mã M1; không ghi vào sim/assets")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True); F = frames(); t0 = time.time()
    parts0 = A.build(0.0, A.POSES["straight"])
    print("build tu the 0:", len(parts0), "chi tiet", round(time.time() - t0, 1), "s", flush=True)
    groups, check, lab = {}, {}, []
    for n, mat, s in parts0:
        b = body_of(n)
        if b is None: continue
        dm = A.servo_display(n, "sim")
        v, f = dm if dm is not None else tess(s, a.tol, a.ang)
        if not len(f): print("bo (khong tessellate duoc):", n); continue
        lab.append((n, mat, np.concatenate([v.min(0), v.max(0)])))
        v = v * 1e-3 - F[b]
        check[n] = dict(body=b, local=v.mean(0).tolist())
        g = groups.setdefault((b, mat), [[], [], 0]); g[0].append(v); g[1].append(f + g[2]); g[2] += len(v)
    for fn in os.listdir(a.out):
        if fn.endswith(".stl"): os.remove(os.path.join(a.out, fn))
    meshes, tri = [], 0
    for (b, mat), (vs, fs, _) in sorted(groups.items()):
        v, f = clean(np.concatenate(vs), np.concatenate(fs)); name = f"{b}__{mat}"
        write_stl(os.path.join(a.out, name + ".stl"), v, f); tri += len(f)
        h = hashlib.sha256(open(os.path.join(a.out, name + ".stl"), "rb").read()).hexdigest()
        meshes.append(dict(name=name, file=name + ".stl", body=b, material=mat, tri=int(len(f)), sha256=h))
    # tư thế ready để so khớp (tâm đỉnh tessellate, cùng cách tính với tư thế 0)
    pose, lift_mm = READY
    for n, mat, s in A.build(lift_mm, A.POSES[pose]):
        if n in check:
            dm = A.servo_display(n, "sim")
            v, _ = dm if dm is not None else tess(s, a.tol, a.ang); check[n]["ready_world"] = (v * 1e-3).mean(0).tolist()
    src = dict(cad="cad/faibotic/assembly_full.py", tol_mm=a.tol, ang_rad=a.ang, frames={k: v.tolist() for k, v in F.items()})
    json.dump(dict(source=src, meshes=meshes), open(os.path.join(a.out, "meshes.json"), "w"), indent=1)
    qd = A.POSES[pose]
    json.dump(dict(pose=pose, lift_m=lift_mm / 1000, q_deg=dict(shoulder=qd[0], elbow=qd[1], wrist_flex=qd[2]), parts=check),
              open(os.path.join(a.out, "cad_check.json"), "w"), indent=0)
    if a.labels:
        np.savez(a.labels, names=np.array([x[0] for x in lab]), mats=np.array([x[1] for x in lab]), bbs=np.array([x[2] for x in lab]))
    size = sum(os.path.getsize(os.path.join(a.out, m["file"])) for m in meshes)
    print("mesh", len(meshes), "tri", tri, "MB", round(size / 1e6, 2), round(time.time() - t0, 1), "s")

# ---------------- S1C: tay AM-ARM200 gốc + kẹp (P-L: kẹp gốc AM-ARM200, trước đó kẹp cũ) ----------------
# O_F_J1_Base, O_F_J2_Rotation: STEP không cùng hệ lắp với OB_F_*; S1G đặt bằng đăng ký vào mesh URDF (cad/export/AlohaMini2_placement.json), như assembly_pl.arm_parts_local.
PL_PARTS = {   # chi tiết STEP follower -> (link URDF, body sim, vật liệu)
    "OB_F_J3_Shoulder": ("left_Upper_Arm", "upper", "ivory"), "OB_F_J4_Elbow": ("left_Lower_Arm", "fore", "navy"),
    "OB_F_J5_Elbow_Wrist_Bracket": ("left_Wrist_Pitch_Roll", "wrist", "ivory"), "OB_F_J5_Pin_Bearing": ("left_wrist_yaw", "yaw", "steel"),
    "OB_F_J56_Wrist_Housing": ("left_wrist_yaw", "yaw", "navy"), "OB_F_J56_Bottom": ("left_wrist_yaw", "yaw", "navy"),
    "OB_F_J56_Top": ("left_wrist_yaw", "yaw", "navy")}
# (6+1): chi tiết link URDF wrist_yaw (vỏ J56, chốt J5, servo cuộn, servo yaw) -> body sim <bên>_yaw (quay theo khớp wrist_yaw); giá J5 ở <bên>_wrist
# kẹp AM-ARM200: hàm J7 -> body jaw (gốc = trục kẹp spec.JAW_PIVOT), còn lại -> hand
PL_GRIP_BODY = {"M1-AR-008_OB_F_J7_Jaw": "jaw"}

def pl_frames_mm():
    """Gốc body tay trái trong khung cẳng tay (mm, tư thế 0 SDK: mọi khâu thẳng hàng). carriage: gốc tay (đáy J1) dời theo vai."""
    k = 1000.0; up = np.array([-spec.L1, 0, 0]) * k; sh = up - np.array([spec.SHOULDER_DR, 0, spec.D0]) * k
    wr = np.array([spec.L2, 0, spec.WF_DZ]) * k; ha = wr + np.array([spec.LW, spec.ROLL_DY, spec.ROLL_DZ]) * k
    yw = wr + np.array([spec.YAW_DX, spec.ROLL_DY, spec.YAW_DZ]) * k
    return {"carriage": sh - np.array([spec.SHOULDER_X, spec.SHOULDER_Y, 0]) * k, "shoulder": sh, "upper": up, "fore": np.zeros(3),
            "wrist": wr, "yaw": yw, "hand": ha, "jaw": ha + np.array(spec.JAW_PIVOT) * k}

def pl_urdf_to_fore():
    """Hàm (link URDF, điểm mm hệ thế giới URDF ở tư thế 0 URDF) -> khung cẳng tay ở tư thế 0 SDK (vai -162,25 độ, khuỷu +162,25 độ URDF)."""
    sys.path.insert(0, os.path.join(REPO, "sim", "tools")); import am_arm200_pl as U
    a = np.radians(spec.UPPER_URDF_DEG); T0 = U.fk({}); T1 = U.fk({"left_shoulder_lift": -a, "left_elbow_flex": a})
    el1 = U.R_SDK @ T1["left_Lower_Arm"][1] * 1000
    def f(link, P):
        R0, p0 = T0[link]; R1, p1 = T1[link]
        loc = (np.asarray(P) / 1000 - p0) @ R0          # = R0^T (P - p0)
        w = loc @ R1.T + p1
        return (w @ U.R_SDK.T) * 1000 - el1
    return f, U

# S1G/S2: thân P-L (assembly_pl.py): đế, khay, đối trọng, trụ + thanh răng -> base; khung T giữa, bánh răng, chốt hãm, đế ngàm, trạm hai tay + camera đầu, servo trục nâng -> carriage
TRUNK_CARRIAGE = ("M1-TR-", "M1-LF-057", "M1-LF-004", "M1-CM-", "M1-LF-002", "M1-LF-lift_servo", "M1-LF-041_latch", "M1-LF-05",
                  "M1-LF-060", "M1-LF-061", "M1-LF-041_switch")
TRUNK_BASE_ONLY = ("M1-LF-051", "M1-LF-052", "M1-LF-053", "M1-LF-054", "M1-LF-055")
TRUNK_MAX_TRI = 1200         # mỗi chi tiết thân tối đa (trụ có răng, khung T): decimate cho sim và GLB; con lăn omni 60

def trunk_body(n):
    if n.startswith(TRUNK_BASE_ONLY): return "base"
    return "carriage" if n.startswith(TRUNK_CARRIAGE) else "base"

def decimate(v, f, n):
    """Quadric (fast-simplification); chi tiết có răng (trụ thanh răng, bánh răng) hay dừng sớm: khi còn > 1,3 n tam giác thì gộp đỉnh theo lưới
    (ô 1..4 mm, chọn ô nhỏ nhất đạt n) rồi bỏ tam giác suy biến. Chỉ cho hình hiển thị."""
    if len(f) <= n: return v, f
    import fast_simplification, trimesh
    v2, f2 = fast_simplification.simplify(v, f, target_reduction=1 - n / len(f), agg=6)
    if len(f2) < 4: v2, f2 = v, f
    if len(f2) <= 1.3 * n: return v2, f2
    for cell in (1.0, 1.5, 2.0, 3.0, 4.0):
        m = trimesh.Trimesh(np.round(v / cell) * cell, f, process=True); m.update_faces(m.nondegenerate_faces()); m.remove_unreferenced_vertices()
        if len(m.faces) <= 1.3 * n: break
    return np.asarray(m.vertices), np.asarray(m.faces)

def sdk_frames_world():
    """Gốc body MJCF ở tư thế 0, lift 0 (m, hệ thế giới; mọi body cùng hướng thế giới ở tư thế 0). Tay phải gương y."""
    z0 = spec.ARM_BASE_Z0; F = {"base": (0.0, 0.0, 0.0), "carriage": (0.0, 0.0, z0)}
    for s, k in (("left", 1), ("right", -1)):
        sh = np.array([spec.SHOULDER_X, k * spec.SHOULDER_Y, z0]); up = sh + [spec.SHOULDER_DR, 0, spec.D0]; fo = up + [spec.L1, 0, 0]
        wr = fo + [spec.L2, 0, spec.WF_DZ]; ha = wr + [spec.LW, k * spec.ROLL_DY, spec.ROLL_DZ]; yw = wr + [spec.YAW_DX, k * spec.ROLL_DY, spec.YAW_DZ]
        jw = ha + np.array(spec.JAW_PIVOT) * [1, k, 1]
        F.update({s + "_shoulder": sh, s + "_upper": up, s + "_fore": fo, s + "_wrist": wr, s + "_yaw": yw, s + "_hand": ha, s + "_jaw": jw})
    return {k: np.array(v, float) for k, v in F.items()}

def main_pl(a):
    import gripper_am as GA
    import servos as SV
    import assembly_pl as P
    import trimesh
    os.makedirs(a.out, exist_ok=True); t0 = time.time()
    R = A.wrist_ref(); f, U = pl_urdf_to_fore(); Fm = pl_frames_mm(); FW = sdk_frames_world()
    joints, links = U.parse(); T0 = U.fk({})
    L0 = spec.LIFT_CAD_OFFSET * 1000.0             # lift CAD (mm) ứng với lift SDK 0
    # cùng một nguồn: gốc tay CAD (khớp URDF AlohaMini2 left_base_joint) phải trùng gốc tay SDK
    cad_mount = P.U2W(P.ARM_BASE_U["L"]) + (0, 0, L0); sdk_mount = FW["left_shoulder"] * 1000
    assert np.abs(cad_mount - sdk_mount).max() < 0.1, ("gốc tay CAD khác SDK", cad_mount, sdk_mount)
    groups, chk, labels = {}, {}, []
    def add(key, body_w, mat, v, F, name):
        g = groups.setdefault(key, [[], [], 0, body_w, mat]); g[0].append(v); g[1].append(F + g[2]); g[2] += len(v)
        labels.append((name, mat, body_w, v.min(0) + FW[body_w] * 1000, v.max(0) + FW[body_w] * 1000))
    CODE = {"O_F_J1_Base": "M1-SH-005", "O_F_J2_Rotation": "M1-SH-019", "OB_F_J3_Shoulder": "M1-SH-006", "OB_F_J4_Elbow": "M1-AR-001",
            "OB_F_J5_Elbow_Wrist_Bracket": "M1-AR-002", "OB_F_J5_Pin_Bearing": "M1-AR-011", "OB_F_J56_Wrist_Housing": "M1-AR-003", "OB_F_J56_Bottom": "M1-AR-004", "OB_F_J56_Top": "M1-AR-005"}
    arm = []          # (tên, body ngắn, vật liệu, V cục bộ mm, F) của tay trái
    for n, (link, sol) in P.arm_parts_local().items():      # STEP follower trong khung left_Base (J1, J2 đặt bằng đăng ký vào mesh URDF, như assembly_pl)
        lk, body, mat = PL_PARTS[n] if n in PL_PARTS else {"O_F_J1_Base": ("left_Base", "carriage", "ivory"), "O_F_J2_Rotation": ("left_Rotation_Pitch", "shoulder", "navy")}[n]
        link = lk          # chốt J5: link URDF wrist_yaw (S1C): đặt ở yaw 0 (tư thế 0), đi theo body yaw
        v, F = tess(sol, a.tol, a.ang)
        try:   # kiểm: tâm khối chi tiết so tâm URDF (cả link) ở tư thế 0 URDF
            c = trimesh.Trimesh(v, F, process=True).center_mass
            R0, p0 = T0[link]; cu = (p0 + R0 @ links[link]["com"]) * 1000
            chk[n] = dict(link=link, com_step_minus_t=np.round(c, 2).tolist(), com_urdf_link=np.round(cu, 2).tolist())
        except Exception as e:
            chk[n] = dict(link=link, err=str(e))
        arm.append((f"{CODE[n]}_{n}", body, mat, f(link, v) - Fm[body], F))
        print("  ", n, len(F), "tam", round(time.time() - t0, 1), "s", flush=True)
    for M, body, nm in ((A.flex_mesh_pose(), "fore", "M1-SV-001_wrist_flex"), (A.roll_servo_pose(), "yaw", "M1-SV-001_wrist_roll")):
        v, F = SV.mesh("sts3215", M, "sim"); arm.append((nm, body, "servo_body", v - Fm[body], F))
    # (6+1): khoá yaw M1-AR-025 bỏ; servo yaw STS3215 lấy nguyên mesh URDF upstream (chồng servo y < 20 mm của link wrist_yaw,
    # cad/upstream/AM-ARM/am-arm200/urdf/derived/left_wrist_yaw__servo__left_wrist_yaw.stl, cad/faibotic/extract_urdf_servos.py), đặt theo link wrist_yaw
    import trimesh as _tm
    _m = _tm.load(os.path.join(REPO, "cad", "upstream", "AM-ARM", "am-arm200", "urdf", "derived", "left_wrist_yaw__servo__left_wrist_yaw.stl"), process=True)
    R0y, p0y = T0["left_wrist_yaw"]; vy = (np.asarray(_m.vertices) @ R0y.T + p0y) * 1000
    arm.append(("M1-SV-001_wrist_yaw", "yaw", "servo_body", f("left_wrist_yaw", vy) - Fm["yaw"], np.asarray(_m.faces)))
    # kẹp AM-ARM200 (khung left_Base ở tư thế 0 URDF, hàm quay tới góc khép): cùng phép đặt link left_Fixed_Jaw (hàm đi cứng theo nó khi góc kẹp không đổi)
    for name, mat, link, sol in GA.parts_base(spec.GRIPPER_CLOSE_URDF):
        body = PL_GRIP_BODY.get(name, "hand")
        v, F = tess(sol, a.tol, a.ang); arm.append((name, body, mat, f("left_Fixed_Jaw", v) - Fm[body], F))
    v, F = SV.mesh("sts3215", GA.grip_servo_pose_base(), "sim"); arm.append(("M1-SV-001_gripper", "hand", "servo_body", f("left_Fixed_Jaw", v) - Fm["hand"], F))
    for name, b, mat, v, F in arm:
        for side, k in (("left", 1), ("right", -1)):
            v2, F2 = (v, F) if k == 1 else (v * [1, -1, 1], F[:, ::-1])
            add((f"{side}_{b}", mat), "carriage" if b == "carriage" else f"{side}_{b}", mat, v2, F2, f"{name}_{'L' if k == 1 else 'R'}")
    # thân P-L ở lift SDK 0
    trunk = P.base_parts() + P.tray_parts(display=True) + P.column() + P.T_frame(L0) + P.latch_parts(L0) + P.switches(L0) + P.dock(L0) + P.station(L0)
    nt = 0
    for n, mat, sol in trunk:
        body = trunk_body(n); v, F = tess(sol, a.trunk_tol, a.ang)
        if not len(F): print("bo:", n); continue
        v, F = clean(v, F); v, F = decimate(v, F, 60 if "roller" in n else TRUNK_MAX_TRI); nt += len(F)
        add((body, mat), body, mat, v - FW[body] * 1000, F, n)
    for n, model, M in P.wheel_servo_poses() + [("M1-LF-lift_servo", "sts3095", P.lift_servo_pose(L0))]:
        body = trunk_body(n); m = SV.mesh(model, M, "sim")
        v, F = m if m is not None else tess(SV.solid(model, M), a.trunk_tol, a.ang)
        add((body, "servo_body"), body, "servo_body", v - FW[body] * 1000, F, n)
    print("than:", len(trunk), "chi tiet, tam giac", nt, round(time.time() - t0, 1), "s", flush=True)
    for fn in os.listdir(a.out):
        if fn.endswith(".stl"): os.remove(os.path.join(a.out, fn))
    meshes, tri = [], 0
    for (gk, mat), (vs, fs, _, body, _m) in sorted(groups.items()):
        v, F = clean(np.concatenate(vs) * 1e-3, np.concatenate(fs))
        name = f"{gk}__{mat}"; fn = name + ".stl"
        write_stl(os.path.join(a.out, fn), v, F); tri += len(F)
        h = hashlib.sha256(open(os.path.join(a.out, fn), "rb").read()).hexdigest()
        meshes.append(dict(name=name, file=fn, body=body, material=mat, tri=int(len(F)), sha256=h))
    src = dict(variant="P-L", cad="cad/faibotic/assembly_pl.py (thân, lift SDK 0 = lift CAD %g mm) + cad/upstream/AM-ARM/am-arm200/stp/follower + cad/faibotic/gripper_am.py (kẹp AM-ARM200, hàm khép)" % L0,
               tol_mm=a.tol, trunk_tol_mm=a.trunk_tol, trunk_max_tri=TRUNK_MAX_TRI, ang_rad=a.ang, frames={k: v.tolist() for k, v in FW.items()},
               urdf="cad/upstream/AM-ARM/am-arm200/urdf/am-arm200.urdf", t_step_minus_urdf_mm=np.round(R["t"], 3).tolist(), com_check_mm=chk,
               ghi_chu="tay phải = gương y tay trái (như MJCF); CAD lắp hai tay giống nhau. Servo STS3095 của pan/vai/khuỷu chưa có vị trí trong STEP follower: chưa vẽ")
    json.dump(dict(source=src, meshes=meshes), open(os.path.join(a.out, "meshes.json"), "w"), indent=1)
    if a.labels:
        np.savez(a.labels, names=np.array([x[0] for x in labels]), mats=np.array([x[1] for x in labels]), bodies=np.array([x[2] for x in labels]),
                 bbs=np.array([np.concatenate([x[3], x[4]]) for x in labels]))
    size = sum(os.path.getsize(os.path.join(a.out, m["file"])) for m in meshes)
    print("mesh", len(meshes), "tri", tri, "MB", round(size / 1e6, 2), round(time.time() - t0, 1), "s")

if __name__ == "__main__":
    if "--pl" in sys.argv:
        ap = argparse.ArgumentParser(); ap.add_argument("--pl", action="store_true"); ap.add_argument("--out", default=os.path.join(REPO, "sim", "assets"))
        ap.add_argument("--tol", type=float, default=0.35); ap.add_argument("--ang", type=float, default=0.4); ap.add_argument("--trunk-tol", type=float, default=0.5)
        ap.add_argument("--labels", default=None, help="npz (tên, vật liệu, body, hộp bao mm thế giới ở tư thế 0) cho web-assets gán nhãn cụm/mã")
        main_pl(ap.parse_args())
    else:
        main()
